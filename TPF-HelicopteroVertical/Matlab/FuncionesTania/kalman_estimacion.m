close all; clear; clc

%% ============================================================
% Estimar R desde Excel (sensor vs real) y estimar Q (qI) usando iddata
% SIN resample() (decimación exacta cuando Ts_target/Ts es entero)
%
% Archivos:
%  - planta (1).mat  -> plantaC o sysC (BJ continuo)
%  - Excel:
%      D:\GitHub\auto\TPF-HelicopteroVertical\sensorMedicionesPostCalibracion.xlsx
%      col1 = Medida Sensor (cm)
%      col2 = Medida Real (cm)
%  - datos.mat:
%      D:\GitHub\auto\TPF-HelicopteroVertical\Matlab\PruebasEmpiricas\datos.mat
%      contiene dato1,dato2,dato3 (iddata)
%
% Salidas:
%  - R (cm^2) desde error sensor-real (sin sesgo)
%  - q_best por dataset y q_global (mediana)
%  - L_kalman final con Q=q_global*I
%% ============================================================

%% ===== rutas =====
mat_planta = 'planta (1).mat';
xlsx_path  = 'D:\GitHub\auto\TPF-HelicopteroVertical\sensorMedicionesPostCalibracion.xlsx';
mat_datos  = 'D:\GitHub\auto\TPF-HelicopteroVertical\Matlab\PruebasEmpiricas\datos.mat';

%% ===== Ts objetivo =====
Ts_target = 1/200;  % 0.005 s
Fs_target = 1/Ts_target;

fprintf('Ts_target = %.9f s\n', Ts_target);
fprintf('Excel     = %s\n', xlsx_path);
fprintf('Datos     = %s\n', mat_datos);
fprintf('Planta    = %s\n\n', mat_planta);

%% ============================================================
% 1) Estimar R desde Excel (sensor vs real)
%% ============================================================
T = readtable(xlsx_path);

if width(T) < 2
    error('El Excel debe tener al menos 2 columnas (Sensor, Real).');
end

y_sensor = T{:,1};
y_real   = T{:,2};

mask = isfinite(y_sensor) & isfinite(y_real);
y_sensor = y_sensor(mask);
y_real   = y_real(mask);

if numel(y_sensor) < 20
    error('Muy pocos datos válidos en el Excel luego de filtrar NaNs.');
end

e    = y_sensor - y_real;     % error total
bias = mean(e);
e0   = e - bias;              % error sin sesgo
R    = var(e0, 1);            % var poblacional (1/N)
sigmaR = sqrt(R);

fprintf('==== R desde Excel ====\n');
fprintf('N_excel = %d\n', numel(e0));
fprintf('bias = mean(sensor-real) = %.6g cm\n', bias);
fprintf('R = var(e-bias) = %.6g cm^2\n', R);
fprintf('sigma_R = %.6g cm\n\n', sigmaR);

figure('Name','Excel: error sensor-real','NumberTitle','off');
subplot(2,1,1); plot(e,'LineWidth',1.0); grid on; grid minor;
ylabel('e (cm)'); title('Error crudo: sensor - real');
subplot(2,1,2); histogram(e0,40); grid on; grid minor;
xlabel('e0 (cm)'); title('Error sin sesgo (para R)');

%% ============================================================
% 2) Cargar BJ y construir planta determinista G=B/F discretizada
%% ============================================================
S = load(mat_planta);

if isfield(S,'plantaC')
    plantaC = S.plantaC;
elseif isfield(S,'sysC')
    plantaC = S.sysC;
else
    error('No encuentro "plantaC" ni "sysC" dentro de planta (1).mat');
end

G  = tf(plantaC.B, plantaC.F);              % planta determinista
Gd = c2d(ss(G), Ts_target, 'zoh');          % a Ts_target
[A,B,C,D] = ssdata(Gd);
n = size(A,1);

fprintf('==== Planta determinista G=B/F discretizada ====\n');
fprintf('orden n = %d\n\n', n);

%% ============================================================
% 3) Cargar iddata dato1,dato2,dato3 y llevar a Ts_target SIN resample()
%% ============================================================
DD = load(mat_datos);
names = {'dato1','dato2','dato3'};
datos = cell(1,3);

for i=1:3
    if ~isfield(DD, names{i})
        error('No encuentro %s en %s', names{i}, mat_datos);
    end
    datos{i} = DD.(names{i});
    if ~isa(datos{i}, 'iddata')
        error('%s no es iddata.', names{i});
    end
end

U = cell(1,3);
Y = cell(1,3);

for i=1:3
    di = datos{i};

    Ts_i = di.Ts;
    if isempty(Ts_i) || Ts_i <= 0
        error('%s: iddata no tiene Ts válido.', names{i});
    end

    % Extraer señales crudas (double columnas)
    yraw = di.OutputData(:);
    uraw = di.InputData(:);

    % limpiar NaNs/infs
    m = isfinite(yraw) & isfinite(uraw);
    yraw = yraw(m);
    uraw = uraw(m);

    if abs(Ts_i - Ts_target) < 1e-12
        % ya está
        yds = yraw;
        uds = uraw;
        fprintf('%s: Ts ya es Ts_target (%.9g)\n', names{i}, Ts_i);

    else
        ratio = Ts_target / Ts_i;  % ej: 0.005 / 0.001 = 5
        if abs(ratio - round(ratio)) > 1e-12
            error('%s: Ts_target/Ts_i no es entero. Ts_i=%.9g Ts_target=%.9g', names{i}, Ts_i, Ts_target);
        end
        M = round(ratio);
        fprintf('%s: decimación exacta M=%d (Ts %.9g -> %.9g)\n', names{i}, M, Ts_i, Ts_target);

        Fs_i = 1/Ts_i;

        % --- anti-alias: lowpass antes de decimar ---
        % banda útil de tu planta ~2 Hz, elegimos 20 Hz sobrado
        fc = 20;  % Hz
        y_f = apply_lowpass(yraw, Fs_i, fc);
        u_f = apply_lowpass(uraw, Fs_i, fc);

        % decimar: tomar 1 cada M
        yds = y_f(1:M:end);
        uds = u_f(1:M:end);
    end

    % quitar media de y para evitar offset en innovación
    yds = yds - mean(yds);

    U{i} = uds(:);
    Y{i} = yds(:);

    fprintf('%s: N=%d\n', names{i}, numel(yds));
end

fprintf('\n');

%% ============================================================
% 4) Estimar Q=qI con cada dataset (tuning por innovación)
%% ============================================================
q_grid = logspace(-14, -2, 80);
maxLag = 30;

q_best = zeros(1,3);
score_best = zeros(1,3);

for i=1:3
    u = U{i}; y = Y{i};

    acfs  = zeros(size(q_grid));
    varn  = zeros(size(q_grid));
    score = zeros(size(q_grid));

    for k=1:numel(q_grid)
        q = q_grid(k);
        Q = q*eye(n);

        nu = innovation_kalman(A,B,C,D,u,y,Q,R);

        nu = nu - mean(nu);
        varn(k) = var(nu, 1);

        acf = xcorr(nu, maxLag, 'coeff');
        mid = maxLag+1;
        acfs(k) = sum(acf([1:mid-1 mid+1:end]).^2); % blancura

        % score: primariamente blancura; penalización suave por var gigantes
        score(k) = acfs(k) + 0.05*(log10(varn(k)+1e-30))^2;
    end

    [score_best(i), ix] = min(score);
    q_best(i) = q_grid(ix);

    fprintf('==== %s ====\n', names{i});
    fprintf('q_best = %.3e\n', q_best(i));
    fprintf('score  = %.6g\n', score_best(i));

    figure('Name',sprintf('%s: tuning Q', names{i}), 'NumberTitle','off');
    subplot(3,1,1);
    semilogx(q_grid, acfs, 'LineWidth',1.1); grid on; grid minor;
    ylabel('ACF sum'); title('Blancura innovación (menor=mejor)');

    subplot(3,1,2);
    semilogx(q_grid, varn, 'LineWidth',1.1); grid on; grid minor;
    ylabel('Var(\nu)'); title('Varianza de innovación');

    subplot(3,1,3);
    semilogx(q_grid, score, 'LineWidth',1.1); grid on; grid minor;
    xlabel('q'); ylabel('score'); title('Score combinado');
end

q_global = median(q_best);
Q_global = q_global*eye(n);

fprintf('\n==== RESULTADO GLOBAL ====\n');
fprintf('q_best: [%.3e  %.3e  %.3e]\n', q_best(1), q_best(2), q_best(3));
fprintf('q_global (mediana) = %.3e\n', q_global);

%% ============================================================
% 5) Ganancia Kalman final L (para estimar estados)
%% ============================================================
Gk = eye(n);  % ruido de proceso en estados
[L, P, ~] = dlqe(A, Gk, C, Q_global, R);

fprintf('\n==== Kalman final (planta sola) ====\n');
fprintf('R = %.6g (sigma=%.4g cm)\n', R, sqrt(R));
fprintf('Q_global = qI, q=%.3e\n', q_global);
fprintf('L size %dx%d:\n', size(L,1), size(L,2));
disp(L);

save('RQ_estimados.mat', 'R','sigmaR','bias','q_best','q_global','Q_global','L','P','Ts_target');
disp('Guardado: RQ_estimados.mat');

%% ============================================================
% === Función: innovación nu[k] = y[k] - ypred[k|k-1]
%% ============================================================
function nu = innovation_kalman(A,B,C,D,u,y,Q,R)
    n = size(A,1);
    N = numel(y);

    G = eye(n);
    [L,~,~] = dlqe(A, G, C, Q, R);

    xhat = zeros(n,1);
    nu   = zeros(N,1);

    for k=1:N
        ypred = C*xhat + D*u(k);
        nu(k) = y(k) - ypred;

        xhat = xhat + L*nu(k);      % update
        xhat = A*xhat + B*u(k);     % predict
    end
end

%% ============================================================
% === Filtro lowpass con fallback si no existe lowpass()
%% ============================================================
function xf = apply_lowpass(x, Fs, fc)
    x = double(x(:));

    % si existe lowpass() (Signal Processing Toolbox)
    if exist('lowpass','file') == 2
        xf = lowpass(x, fc, Fs);
        return;
    end

    % fallback FIR simple (Hamming) si no existe lowpass
    % Diseñamos un FIR con fc y aplicamos filtfilt para fase cero
    if exist('fir1','file') == 2 && exist('filtfilt','file') == 2
        Wn = fc/(Fs/2);
        Wn = min(max(Wn, 1e-6), 0.99);
        Nf = 101; % orden FIR
        b = fir1(Nf, Wn, hamming(Nf+1));
        xf = filtfilt(b, 1, x);
        return;
    end

    % último fallback: sin filtro (te aviso)
    warning('No tengo lowpass/fir1/filtfilt. Decimo sin anti-alias.');
    xf = x;
end
