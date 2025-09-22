close all; clear; clc;

%% ====================== Parámetros nominales ======================
R_1 = 15e3;  R_2 = 82e3;  R_3 = 15e3;  R_4 = 82e3;
C_1 = 0.22e-6;  C_2 = 100e-9;

% Tolerancias de catálogo (especificación)
tolR = 0.05;   % ±5% resistencias
tolC = 0.20;   % ±20% capacitores

% Para Monte Carlo (3σ ≈ tolerancia) -> σ = tol/3
sigR = tolR/3;
sigC = tolC/3;

% Correlación entre resistores "pareja" del mismo valor (más realista)
rhoR = 0;  % entre R1-R3 (15k/15k) y R2-R4 (82k/82k)

% Nominales (alias)
R1n = R_1; R2n = R_2; R3n = R_3; R4n = R_4;
C1n = C_1; C2n = C_2;

%% ====================== Helper de planta continua ======================
% G(s) = [(R3/R1)*(R4/R2)] / [(R3*C2 s + 1)(R4*C1 s + 1)]
makeG = @(R1,R2,R3,R4,C1,C2) tf( (R3/R1)*(R4/R2), conv([R3*C2 1],[R4*C1 1]) );

%% ====================== Cargar datos medidos ======================
data = readmatrix('./imagenes/OpenLoop/mediciones.csv');

% u = ENTRADA medida; y = SALIDA medida
u = data(:, 11);   % ajustá si cambia columna
y = data(:, 17);

Ts = 0.000099999997474;  % tiempo de muestreo (≈100 µs)

% Vector de tiempo alineado a la medición
Ndata = min(length(y), length(u));
u = u(1:Ndata); y = y(1:Ndata);
u = u(:); y = y(:);
tdata = (0:Ndata-1).' * Ts;

%% ====================== Modelo estimado (opcional, referencia) ======================
try
    data_id = iddata(y, u, Ts);
    sys0 = tfest(data_id, 2,0);              % 2 polos por defecto
    if sys0.Ts==0, sys0d = c2d(sys0, Ts, 'zoh'); else, sys0d = d2d(sys0, Ts); end
catch
    sys0d = [];  % si no está el toolbox, seguimos sin el estimado
end

%% ====================== Nominal (discreto) ======================
Gc_nom = makeG(R1n,R2n,R3n,R4n,C1n,C2n);
Gd_nom = c2d(Gc_nom, Ts, 'zoh');

%% ====================== MONTE CARLO realista ======================
Nmc = 1000;           % # de muestras (subí si querés más suavidad)
rng(123);             % reproducible

% Ruido normal correlacionado para (R1,R3) y (R2,R4)
Zc13 = randn(Nmc,1);  % común par (R1,R3)
Z1   = randn(Nmc,1);  % propio R1
Z3   = randn(Nmc,1);  % propio R3

Zc24 = randn(Nmc,1);  % común par (R2,R4)
Z2   = randn(Nmc,1);  % propio R2
Z4   = randn(Nmc,1);  % propio R4

% Construir deltas relativos con correlación rhoR
a = rhoR; b = sqrt(1 - rhoR^2);
dR1 = sigR*(a*Zc13 + b*Z1);
dR3 = sigR*(a*Zc13 + b*Z3);
dR2 = sigR*(a*Zc24 + b*Z2);
dR4 = sigR*(a*Zc24 + b*Z4);

% Capacitores: independientes (distinto valor/naturaleza)
dC1 = sigC*randn(Nmc,1);
dC2 = sigC*randn(Nmc,1);

% Clip a las tolerancias físicas (no exceder ±tol)
clip = @(x,tol) min(max(x, -tol), tol);
dR1 = clip(dR1, tolR); dR3 = clip(dR3, tolR);
dR2 = clip(dR2, tolR); dR4 = clip(dR4, tolR);
dC1 = clip(dC1, tolC); dC2 = clip(dC2, tolC);

% Vectores absolutos
R1s = R1n*(1 + dR1);  R3s = R3n*(1 + dR3);
R2s = R2n*(1 + dR2);  R4s = R4n*(1 + dR4);
C1s = C1n*(1 + dC1);  C2s = C2n*(1 + dC2);

% Simular todas las muestras con la MISMA entrada u
Ymc = zeros(Ndata, Nmc);
for k = 1:Nmc
    Gc  = makeG(R1s(k),R2s(k),R3s(k),R4s(k),C1s(k),C2s(k));
    Gdz = c2d(Gc, Ts, 'zoh');
    yk  = lsim(Gdz, u, tdata);
    if isrow(yk), yk = yk.'; end
    Ymc(:,k) = yk;
end

% Envolvente percentil (más realista) y también min/max por si querés ver extremos
yP05 = prctile(Ymc, 5,  2);
yP95 = prctile(Ymc, 95, 2);
yMin = min(Ymc, [], 2);
yMax = max(Ymc, [], 2);

%% ====================== (Opcional) WORST-CASE 64 corners ======================
plot_worstcase = false;   % ponelo true si querés superponerlo
if plot_worstcase
    R1_lo = (1 - tolR)*R1n;   R1_hi = (1 + tolR)*R1n;
    R2_lo = (1 - tolR)*R2n;   R2_hi = (1 + tolR)*R2n;
    R3_lo = (1 - tolR)*R3n;   R3_hi = (1 + tolR)*R3n;
    R4_lo = (1 - tolR)*R4n;   R4_hi = (1 + tolR)*R4n;
    C1_lo = (1 - tolC)*C1n;   C1_hi = (1 + tolC)*C1n;
    C2_lo = (1 - tolC)*C2n;   C2_hi = (1 + tolC)*C2n;

    pairs = [ R1_lo R1_hi; R2_lo R2_hi; R3_lo R3_hi; R4_lo R4_hi; C1_lo C1_hi; C2_lo C2_hi ];
    comb = dec2bin(0:(2^6-1)) - '0';   % 64x6

    Ywc = zeros(Ndata, size(comb,1));
    for k = 1:size(comb,1)
        sel = comb(k,:)+1;
        R1 = pairs(1,sel(1)); R2 = pairs(2,sel(2));
        R3 = pairs(3,sel(3)); R4 = pairs(4,sel(4));
        C1 = pairs(5,sel(5)); C2 = pairs(6,sel(6));
        Gc  = makeG(R1,R2,R3,R4,C1,C2);
        Gdz = c2d(Gc, Ts, 'zoh');
        Ywc(:,k) = lsim(Gdz, u, tdata);
    end
    yWmin = min(Ywc, [], 2);
    yWmax = max(Ywc, [], 2);
else
    yWmin = []; yWmax = [];
end

%% ====================== Simulación nominal y estimado con misma u ======================
y_nom = lsim(Gd_nom, u, tdata);
if ~isempty(sys0d), y_est = lsim(sys0d, u, tdata); else, y_est = []; end

%% ====================== Plot comparativo ======================
figure; hold on; box on;

% Banda percentil (más realista)
fill([tdata; flipud(tdata)], [yP05; flipud(yP95)], [0.80 0.92 1.00], ...
     'EdgeColor','none','FaceAlpha',0.55);

% (opcional) extremos worst-case en líneas finas
if ~isempty(yWmin)
    plot(tdata, yWmin, 'Color',[0.2 0.4 0.8], 'LineStyle','--', 'LineWidth',0.8);
    plot(tdata, yWmax, 'Color',[0.2 0.4 0.8], 'LineStyle','--', 'LineWidth',0.8);
end

% Medición y modelos
stairs(tdata, y, 'k', 'LineWidth', 1.3);          % medición
plot(tdata, y_nom, 'LineWidth', 1.2);             % nominal
if ~isempty(y_est), plot(tdata, y_est, 'LineWidth', 1.2); end  % estimado

xlabel('t [s]'); ylabel('y');
leg = {'Banda MC (p5–p95)'};
if ~isempty(yWmin), leg{end+1} = 'Worst-case min/max'; end
leg = [leg, {'Medición','Nominal'}];
if ~isempty(y_est), leg{end+1} = 'Estimado (tfest)'; end
legend(leg, 'Location','best');
title('Comparación temporal con la MISMA entrada u[k] — banda de tolerancia realista');
grid on; grid minor;

%% ====================== (Opcional) corner MC más parecido por RMSE ======================
RMSE = sqrt(mean( (Ymc - y).^2, 1 ));
[rmseBest, kBest] = min(RMSE);
fprintf('MC con menor RMSE: #%d  RMSE = %.4g\n', kBest, rmseBest);
best = struct('R1',R1s(kBest),'R2',R2s(kBest),'R3',R3s(kBest),'R4',R4s(kBest),'C1',C1s(kBest),'C2',C2s(kBest));
disp(best);


[rmse_raw, nrmse_raw, rmse_bias, nrmse_bias, bias] = nrmse_with_offset(y(300:2100),y_nom(300:2100))
