close all
clear all
clc;
%% modelo
% load('../modelo.mat');Gc = G;
% [F,G,H,J] = tf2ss(Gc.Numerator,Gc.Denominator); sysC = ss(F,G,H,J);
% fn = 2*abs(min(zpk(Gc).P{1})/pi);
% Tn = 1/fn;

% === Parametros fisicos ===
% C1 = 103.07e-9;  C2 = 211.1e-9;
% R1 = 14.878e3;    R2 = 14.760e3;
% R3 = 80.55e3;    R4 = 81.09e3;
C2 = 103.07e-9;  C1 = 211.1e-9;
R3 = 14.878e3;    R4 = 14.760e3;
%R3 = 4.7e3;    R4 = 14.760e3;
%R1 = 27e3;    R2 = 81.09e3;
R1 = 80.55e3;    R2 = 81.09e3;


tau1 = R2*C1;           k1 = -R2/R1;
tau2 = R4*C2;           k2 = -R4/R3;
%tau2 = 1/376.4;
%tau1 = 1/114.2;
F = [ -1/tau1,     0;
       k2/tau2, -1/tau2 ];
G = [ k1/tau1; 0 ];
H = [0 1];     % <- justo lo que querías
J = 0;

sysC = ss(F,G,H,J);     % modelo "práctico" coherente
Gc   = tf(sysC);        % si querés compararlo con el empírico

zpk(Gc)



fn = max(abs(zpk(Gc).P{1}))/pi;
Tn = 1/fn;


%% Parametros
Ts = [Tn/2,Tn/1.5]; %[1e-3,1e-3];%
wn_obj = 4*abs(max(zpk(Gc).P{1}));
zita_obj = 0.7;
poloObj = @(zita,wn,Ts) exp(Ts*wn*(-zita+1j*sqrt(1-zita^2)));
p1 =[poloObj(zita_obj,wn_obj,Ts(1));conj(poloObj(zita_obj,wn_obj,Ts(1)))];
p2 = [0.5+1j*0.5;0.5-1j*0.5];
polos = cell(2);
polos = {p1,p2};
%% Calculo


%step(sysC)
%save('../modeloSS.mat','sysC');
sysD = cell(2);
sysD = {c2d(sysC,Ts(1)),c2d(sysC,Ts(2))};
K = cell(length(polos));
K0 = cell(length(polos));
Kref = cell(length(polos));
sysCL = cell(length(polos));

addpath('..\');
for i = 1:length(polos)
    [A,B,C,D] =ssdata(sysD{i});
    K{i} = acker(A,B,polos{i});
    Ac = A - B*K{i};                  
    
    sysCL{i} = ss(Ac, B, C, D, sysD{i}.Ts);
    [~,~,K0{i}] = refi(Ac,B,C,K{i});
    Kref{i} = 1/dcgain(sysCL{i});
end



%%
A = [1,1];
r = cell(2);
r = { A(1)/2.*[-ones(1,50), ones(1,100)], ...
      A(2)/2.*[-ones(1,100), ones(1,100)] };
VDDA2=2.5;
for i = 1:length(sysCL)
    sysi = sysCL{i};

    % === REFERENCIA de esta iteración, como columna ===
    ri = r{i}(:);

    % === Simulación ===
    [X, Y, U] = ss_sym_digital(sysi, ri, K{i}, K0{i}, [0;0],0);

 

    % === Tiempo consistente con la referencia usada ===
    N  = numel(ri);
    t  = (0:N-1)' * sysi.Ts;
    n  = size(X,2);

    % ----- Figura y paneles -----
    fig = figure('Name', sprintf('CL #%d — señales + polos/ceros', i));
    left = uipanel(fig,'Position',[0.05 0.08 0.58 0.87]);
    rightAx = axes(fig,'Position',[0.68 0.10 0.28 0.82]);

    % ----- Columna izquierda -----
    tl = tiledlayout(left, n+2, 1, 'TileSpacing','compact','Padding','compact');

    % y & r
    ax = nexttile(tl); hold(ax,'on');
    stairs(ax, t, VDDA2+Y(:), 'LineWidth', 1.6);             % y como vector
    stairs(ax, t, VDDA2 + ri, '--', 'LineWidth', 1.2); % r alineada a t
    grid(ax,'on'); grid(ax,'minor'); ylabel(ax,'y, r');
    legend(ax, {'y','r'}, 'Location','best');
    title(ax, sprintf('CL #%d — Salida y referencia', i));

    % estados
    for k = 1:n
        axk = nexttile(tl);
        stairs(axk, t, VDDA2+X(:,k), 'LineWidth', 1.4);
        grid(axk,'on'); grid(axk,'minor'); ylabel(axk, sprintf('x_%d',k));
        if k==1, title(axk, 'Estados'); end
    end

    % u
    axu = nexttile(tl);
    stairs(axu, t, VDDA2+U(:), 'LineWidth', 1.3);
    grid(axu,'on'); grid(axu,'minor'); ylabel(axu,'u'); xlabel(axu,'Tiempo [s]');
    title(axu,'Esfuerzo de control');

    % ----- Columna derecha: polos y ceros -----
    axes(rightAx); %#ok<LAXES>
    cla(rightAx);
    try
        pzplot(sysi); zgrid;
        title(sprintf('Polos y ceros (fs=%.4g Hz)', 1/sysi.Ts));
    catch
        [Z,P,~] = zpkdata(sysi,'v'); hold(rightAx,'on'); grid(rightAx,'on'); axis(rightAx,'equal');
        plot(real(P), imag(P), 'x', 'LineWidth', 1.6, 'Parent', rightAx);
        plot(real(Z), imag(Z), 'o', 'LineWidth', 1.6, 'Parent', rightAx);
        if sysi.Ts>0
            th = linspace(0,2*pi,400);
            plot(cos(th), sin(th), '--', 'Parent', rightAx);
            xlabel(rightAx,'Re(z)'); ylabel(rightAx,'Im(z)');
        else
            xline(rightAx,0,'--'); xlabel(rightAx,'Re(s)'); ylabel(rightAx,'Im(s)');
        end
        legend(rightAx,{'Polos','Ceros','Círc. unidad'},'Location','best');
        title(rightAx, sprintf('Polos/Ceros (manual) — fs=%.4g Hz', 1/sysi.Ts));
    end
end


%% === COMPARACIÓN CON RESULTADOS EXPERIMENTALES ===
exp_files = {'./Exp1.csv', './Exp2.csv'};
Ts_exp = 199.99999495E-6;   
VDDA2 = 2.5;                        % para coherencia con simulación
% Tabla vacía robusta
varNames = {'Caso','Ts_sim','Ts_exp','RMSE_y','RMSE_u','MeanErr_y','MeanErr_u'};
varTypes = repmat({'double'}, 1, numel(varNames));
comparacion = table('Size',[0 numel(varNames)], ...
    'VariableTypes',varTypes, 'VariableNames',varNames);

for i = 1:length(exp_files)
    fprintf('=== Caso %d ===\n', i);

    % --- Cargar CSV (sin depender de nombres de columnas) ---
    % Si tus CSV tienen cabecera, readmatrix igual la ignora y traga los números.
    M = readmatrix(exp_files{i});

    % Columnas: E=5 (U), K=11 (X1), Q=17 (X2=Y), W=23 (ref)
    Uexp  = M(:,5);
    X1exp = M(:,11);          %#ok<NASGU> % lo cargas por si querés guardarlo
    X2exp = M(:,17);          % salida/estado 2
    Rexp  = M(:,23);

    % Limpiar NaN de final si los hay
    good  = all(~isnan([Uexp, X2exp, Rexp]), 2);
    Uexp  = Uexp(good);
    X1exp = X1exp(good);
    X2exp = X2exp(good);
    Rexp  = Rexp(good);
    Uexp = Uexp - mean(Uexp, 'omitnan');
    X1exp = X1exp- mean(X1exp, 'omitnan');
    X2exp = X2exp- mean(X2exp, 'omitnan');
    Rexp  = Rexp- mean(Rexp, 'omitnan');
         % --- Tiempo experimental (grilla original del osciloscopio) ---
    Nexp  = numel(Uexp);
    t_exp = (0:Nexp-1)' * Ts_exp;

    % --- Grilla de simulación (impuesta por el modelo) ---
    sysi   = sysCL{i};
    Tfin   = t_exp(end);
    Ns_sim = floor(Tfin / sysi.Ts) + 1;
    t_sim  = (0:Ns_sim-1)' * sysi.Ts;

        % --- Referencia para el modelo en grilla t_sim ---
    % Ya centrada: arriba hiciste Rexp = Rexp - mean(Rexp)
    r0    = Rexp;  % ya sin DC
    r_sim = interp1(t_exp(:), r0(:), t_sim(:), 'previous', 'extrap');

    % --- Simulación con la MISMA referencia (a Ts = sysi.Ts) ---
    [Xsim, Ysim, Usim] = ss_sym_digital(sysi, r_sim, K{i}, K0{i}, [0;0], 0);

    % === Normalizar formas y longitudes ===
    t_sim = t_sim(:);
    Xsim  = Xsim(:,:);         % Nx2
    Ysim  = Ysim(:);           % Nx1
    Usim  = Usim(:);           % Nx1
    Rsim  = r_sim(:);          % <- DEFINIR Rsim AQUÍ (misma longitud que t_sim)

    % Asegurar longitudes consistentes (recorte al mínimo común)
    Lsim  = min([numel(t_sim), size(Xsim,1), numel(Ysim), numel(Usim), numel(Rsim)]);
    t_sim = t_sim(1:Lsim);
    Xsim  = Xsim(1:Lsim,:);
    Ysim  = Ysim(1:Lsim);
    Usim  = Usim(1:Lsim);
    Rsim  = Rsim(1:Lsim);

    % --- Re-muestrear SIMULADAS a la grilla experimental t_exp ---
    % ZOH: 'previous' para no inventar energía
    X1sim_exp = interp1(t_sim, Xsim(:,1), t_exp, 'previous', 'extrap');
    Ysim_exp  = interp1(t_sim, Ysim,      t_exp, 'previous', 'extrap');
    Usim_exp  = interp1(t_sim, Usim,      t_exp, 'previous', 'extrap');
    Rsim_exp  = interp1(t_sim, Rsim,      t_exp, 'previous', 'extrap');


    % --- Alinear longitudes por seguridad (todo sobre t_exp) ---
    L = min([length(t_exp), length(X1exp), length(X2exp), length(Uexp), ...
             length(X1sim_exp), length(Ysim_exp), length(Usim_exp), length(Rsim_exp), length(Rexp)]);
    t_cmp     = t_exp(1:L);
    X1exp_cmp = X1exp(1:L);   Yexp_cmp = X2exp(1:L);   Uexp_cmp = Uexp(1:L);   Rexp_cmp = Rexp(1:L);
    X1sim_cmp = X1sim_exp(1:L); Ysim_cmp = Ysim_exp(1:L); Usim_cmp = Usim_exp(1:L); Rsim_cmp = Rsim_exp(1:L);

    % ------------ ERRORES (vectores) ------------
    % Absolutos
    err_x1_abs = X1exp_cmp - X1sim_cmp;
    err_y_abs  = Yexp_cmp  - Ysim_cmp;
    err_u_abs  = Uexp_cmp  - Usim_cmp;

    % Porcentuales (normalizamos por max|exp| para no explotar cerca de 0)
    den_x1 = max(abs(X1exp_cmp)); if den_x1 < 1e-9, den_x1 = 1e-9; end
    den_y  = max(abs(Yexp_cmp));  if den_y  < 1e-9, den_y  = 1e-9; end
    den_u  = max(abs(Uexp_cmp));  if den_u  < 1e-9, den_u  = 1e-9; end

    err_x1_pct = 100 * err_x1_abs / den_x1;
    err_y_pct  = 100 * err_y_abs  / den_y;
    err_u_pct  = 100 * err_u_abs  / den_u;

    % Máximos
    err_x1_abs_max = max(abs(err_x1_abs));
    err_y_abs_max  = max(abs(err_y_abs));
    err_u_abs_max  = max(abs(err_u_abs));

    err_x1_pct_max = max(abs(err_x1_pct));
    err_y_pct_max  = max(abs(err_y_pct));
    err_u_pct_max  = max(abs(err_u_pct));

        % ===== RMSE (abs y %) =====
    RMSE_x1_abs = sqrt(mean(err_x1_abs.^2));
    RMSE_y_abs  = sqrt(mean(err_y_abs.^2));
    RMSE_u_abs  = sqrt(mean(err_u_abs.^2));

    RMSE_x1_pct = sqrt(mean(err_x1_pct.^2));
    RMSE_y_pct  = sqrt(mean(err_y_pct.^2));
    RMSE_u_pct  = sqrt(mean(err_u_pct.^2));

    % ===== Tabla 3x4 (filas: X1,X2,U | cols: RMSE_abs, RMSE_pct, ErrMax_abs, ErrMax_pct) =====
    MetricsNames = {'RMSE_abs','RMSE_pct','ErrMax_abs','ErrMax_pct'};
    ResultCase = table( ...
        [RMSE_x1_abs; RMSE_y_abs; RMSE_u_abs], ...
        [RMSE_x1_pct; RMSE_y_pct; RMSE_u_pct], ...
        [err_x1_abs_max; err_y_abs_max; err_u_abs_max], ...
        [err_x1_pct_max; err_y_pct_max; err_u_pct_max], ...
        'VariableNames', MetricsNames, ...
        'RowNames', {'X1','X2','U'} );

    % Guardamos en struct por si querés usar luego:
    resultados_por_caso(i).tabla = ResultCase; %#ok<SAGROW>
    resultados_por_caso(i).t    = t_cmp;
    resultados_por_caso(i).X1exp_cmp = X1exp_cmp; resultados_por_caso(i).X1sim_cmp = X1sim_cmp; 
    resultados_por_caso(i).Yexp_cmp  = Yexp_cmp;  resultados_por_caso(i).Ysim_cmp  = Ysim_cmp;
    resultados_por_caso(i).Uexp_cmp  = Uexp_cmp;  resultados_por_caso(i).Usim_cmp  = Usim_cmp;
    resultados_por_caso(i).Rexp_cmp  = Rexp_cmp;  resultados_por_caso(i).Rsim_cmp  = Rsim_cmp;

    % (opcional) exportar la tabla del caso i a CSV con nombres de fila:
    writetable(ResultCase, sprintf('./Comparacion_Metricas_Caso%d.csv', i), 'WriteRowNames', true);
    disp(ResultCase);

        % ------------ FIGURA: 3 filas x 2 columnas (sobre t_cmp=t_exp recortado) ------------
    figure('Name', sprintf('Caso %d — Señales y errores (grilla experimental)', i), 'Color', 'w');
    tl = tiledlayout(3,2,'TileSpacing','compact','Padding','compact');

    % --- Fila 1: X1 ---
    nexttile; hold on;
    plot(t_cmp, X1exp_cmp, 'k', 'LineWidth', 1.3);
    plot(t_cmp, X1sim_cmp, '--', 'LineWidth', 1.2);
    legend('X1_{exp}','X1_{sim}','Location','best'); grid on; ylabel('X1 [V]');
    title(sprintf('X1 (exp vs sim) — RMSE=%.3e, RMSE%%=%.2f%%', RMSE_x1_abs, RMSE_x1_pct));

    nexttile;
    plot(t_cmp, err_x1_abs, 'LineWidth', 1); grid on; ylabel('e_{X1} [V]');
    title(sprintf('Error X1 — max|e|=%.3e, max|e|%%=%.2f%%', err_x1_abs_max, err_x1_pct_max));

    % --- Fila 2: X2 = Y (con referencia) ---
    nexttile; hold on;
    plot(t_cmp, Yexp_cmp, 'k', 'LineWidth', 1.3);
    plot(t_cmp, Ysim_cmp, '--', 'LineWidth', 1.2);
    plot(t_cmp, Rexp_cmp, ':',  'LineWidth', 1.0);
    legend('Y_{exp}','Y_{sim}','r','Location','best'); grid on; ylabel('Y [V]');
    title(sprintf('X2/Y (exp vs sim) — RMSE=%.3e, RMSE%%=%.2f%%', RMSE_y_abs, RMSE_y_pct));

    nexttile;
    plot(t_cmp, err_y_abs, 'LineWidth', 1); grid on; ylabel('e_{Y} [V]');
    title(sprintf('Error Y — max|e|=%.3e, max|e|%%=%.2f%%', err_y_abs_max, err_y_pct_max));

    % --- Fila 3: U ---
    nexttile; hold on;
    plot(t_cmp, Uexp_cmp, 'k', 'LineWidth', 1.3);
    plot(t_cmp, Usim_cmp, '--', 'LineWidth', 1.2);
    legend('U_{exp}','U_{sim}','Location','best'); grid on; ylabel('U [V]'); xlabel('Tiempo [s]');
    title(sprintf('U (exp vs sim) — RMSE=%.3e, RMSE%%=%.2f%%', RMSE_u_abs, RMSE_u_pct));

    nexttile;
    plot(t_cmp, err_u_abs, 'LineWidth', 1); grid on; ylabel('e_{U} [V]'); xlabel('Tiempo [s]');
    title(sprintf('Error U — max|e|=%.3e, max|e|%%=%.2f%%', err_u_abs_max, err_u_pct_max));

end

writetable(comparacion, './Comparacion_Resultados.csv');
