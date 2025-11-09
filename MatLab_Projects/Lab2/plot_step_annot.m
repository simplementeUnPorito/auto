function info_ext = plot_step_annot(sys, nameStr, Ts_override)
% STEP discreto con STAIRS; anota 10-90% (en s y en T), OS, zeta y omega_n.
% Grafica en milisegundos y marca el OS en el pico.

if nargin < 2 || isempty(nameStr), nameStr = inputname(1); end
if isempty(nameStr), nameStr = 'sys'; end

% ----- Respuesta y métricas básicas -----
[y, t] = step(sys); y = y(:); t = t(:);
t_ms = t * 1000;   % graficamos en milisegundos

% Estimar yss por promedio en cola
Ntail = max(10, round(0.05*length(y)));
yss = mean(y(end-Ntail+1:end));

% Límites 10-90
if abs(yss) < 1e-12
    t10 = NaN; t90 = NaN; tr_10_90 = NaN; y10 = 0; y90 = 0;
else
    y10 = 0.10*yss; y90 = 0.90*yss;
    sgn = sign(yss);
    idx10 = find(sgn*y >= sgn*y10, 1, 'first');
    idx90 = find(sgn*y >= sgn*y90, 1, 'first');
    t10 = tern(~isempty(idx10), t(idx10), NaN);
    t90 = tern(~isempty(idx90), t(idx90), NaN);
    tr_10_90 = tern(~isnan(t10)&&~isnan(t90)&&t90>=t10, t90 - t10, NaN);
end
t10_ms = t10 * 1000; t90_ms = t90 * 1000;

% Pico y %OS (para zeta y omega_n)
[ypeak_raw, idxpk] = max(sign(yss).*y);
ypeak = sign(yss)*ypeak_raw;
tpeak = t(idxpk); tpeak_ms = tpeak * 1000;
OS_percent = tern(yss ~= 0, max(0, (ypeak - yss)/abs(yss)*100), NaN);

% zeta desde %OS:  OS% = 100*exp(-zeta*pi/sqrt(1-zeta^2))
if isnan(OS_percent) || OS_percent <= 0
    zeta_est = NaN;
else
    logterm = log(OS_percent/100);
    zeta_est = -logterm / sqrt(pi^2 + logterm^2);
end

% ----- Ts y rise en T -----
Ts = NaN;
try
    if isprop(sys,'Ts') && ~isempty(sys.Ts) && sys.Ts > 0, Ts = sys.Ts; end
catch, end
if nargin >= 3 && ~isempty(Ts_override) && Ts_override > 0, Ts = Ts_override; end
tr_10_90_T = tern(~isnan(Ts) && ~isnan(tr_10_90), tr_10_90/Ts, NaN);

% ----- Estimacion de omega_n -----
omega_n = NaN;
if ~isnan(zeta_est) && zeta_est < 1 && OS_percent > 0 && tpeak > 0
    omega_n = pi / ( tpeak * sqrt(1 - zeta_est^2) );
end
if isnan(omega_n)
    S = stepinfo(y, t);   % Ts(2%) ~ 4/(zeta*omega_n)
    if isfield(S,'SettlingTime') && ~isempty(S.SettlingTime) && S.SettlingTime > 0 && ~isnan(zeta_est) && zeta_est > 0
        omega_n = 4 / ( zeta_est * S.SettlingTime );
    end
end
if isnan(omega_n) && ~isnan(tr_10_90) && tr_10_90 > 0
    k_rise = 1.4;  % aprox para 10-90% en 2do orden subamortiguado
    omega_n = k_rise / tr_10_90;
end

% ----- Grafico -----
figure;
stairs(t_ms, y, 'LineWidth', 1.5); grid on;grid minor; hold on;
xlabel('Tiempo [ms]'); ylabel('Salida');
title(sprintf('Respuesta al escalon de %s', nameStr));

% Líneas guía 10% y 90% y marcadores (en ms)
if ~isnan(tr_10_90)
    yline(y10, '--', '10% yss', 'LabelHorizontalAlignment','left');
    yline(y90, '--', '90% yss', 'LabelHorizontalAlignment','left');
    xline(t10_ms, ':', 't10%');
    xline(t90_ms, ':', 't90%');
    plot([t10_ms t90_ms], [y10 y90], 'o', 'MarkerSize', 6, 'LineWidth', 1.2);
end
% Línea de yss
yline(yss, '-', sprintf('yss=%.3g', yss), 'LabelHorizontalAlignment','left');

% ----- Marca del OS en el pico -----
if ~isnan(OS_percent) && OS_percent > 0
    % línea vertical desde yss a ypeak en tpeak
    plot([tpeak_ms tpeak_ms], [yss ypeak], '-', 'LineWidth', 1.2);
    % texto al costado derecho de la línea
    dx = 0.02 * (t_ms(end) - t_ms(1));
    text(tpeak_ms + dx, yss + 0.5*(ypeak - yss), ...
        sprintf('OS=%.3g%%', OS_percent), ...
        'Interpreter','none', 'Margin',2);
    % marcar el pico
    plot(tpeak_ms, ypeak, 's', 'MarkerSize', 6, 'LineWidth', 1.2);
end

% ----- Cuadro de metricas (abajo-derecha; chico) -----
w = 0.15; h = 0.12; rmargin = 0.1; x = 1 - rmargin - w; y0 = 0.30;
txtLines = {
    sprintf('t_r(10%%-90%%) = %.4g ms  (~%.4g Ts)', safeNum(tr_10_90*1000), safeNum(tr_10_90_T))
    sprintf('OS = %.4g%%',          safeNum(OS_percent))
    sprintf('zeta = %.4g',          safeNum(zeta_est))
    sprintf('omega_n = %.4g rad/s', safeNum(omega_n))
    };
annotation('textbox', [x y0 w h], ...
    'String', txtLines, 'Interpreter','none', ...
    'FontName','Consolas', 'FontSize',9, ...
    'EdgeColor',[0.5 0.5 0.5], 'FaceAlpha',0.88, 'BackgroundColor','w');

hold off;

% ----- Salida -----
info_ext = struct( ...
    'yss', yss, ...
    't10', t10, ...
    't90', t90, ...
    'tr_10_90', tr_10_90, ...
    'tr_10_90_T', tr_10_90_T, ...
    'OS_percent', OS_percent, ...
    'zeta_est', zeta_est, ...
    'omega_n', omega_n, ...
    'Ts', Ts);
end

% --------- helpers ---------
function y = tern(cond, a, b)
if cond, y = a; else, y = b; end
end

function v = safeNum(x)
if isnan(x) || isinf(x), v = NaN; else, v = x; end
end
