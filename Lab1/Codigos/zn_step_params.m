function [L, T, S, t_max] = zn_step_params(G, ts, wn)
% ZN_STEP_PARAMS  (Ziegler–Nichols, método de respuesta al escalón)
% [L, T, S, t_max] = zn_step_params(G, ts, wn)
%  G   : sistema (tf/ss/zpk)
%  ts  : tiempo de establecimiento estimado para fijar horizonte
%  wn  : frecuencia natural estimada (rad/s) para fijar resolución
% Devuelve:
%  L     : tiempo muerto (s)
%  T     : constante aparente (s)
%  S     : pendiente máxima (dy/dt) en el punto de inflexión
%  t_max : instante del máximo slope

    % --- Horizonte y muestreo (como pediste) ---
    t_end = 1.1*ts;
    if ~isfinite(t_end) || t_end <= 0
        t_end = 5*(1/wn); % fallback
    end
    N = max(1000, round(5000*(2*pi/wn)));  % densidad ligada a wn
    t = linspace(0, t_end, N);

    % --- Respuesta al escalón ---
    [y, tout] = step(G, t);

    % Valor final (ganancia estática); si no es finita, usar y(end)
    K = dcgain(G);
    if ~isfinite(K)
        K = y(end);
    end

    % --- Derivada numérica y punto de máxima pendiente ---
    dy = gradient(y, tout);
    [S, idx] = max(dy);   % pendiente máxima
    idx = idx(1);         % por si hay empate
    t_max = tout(idx);
    y_max = y(idx);

    % --- Recta tangente: y_tan(t) = y_max + S*(t - t_max) ---
    % Intersección con eje tiempo (y=0) -> L
    L = t_max - y_max/S;

    % Donde la tangente alcanza el valor final K: y_tan = K
    % t_f = t_max + (K - y_max)/S  ->  T = t_f - L = K/S
    T = K / S;

    % --- Gráfica con anotaciones ---
    figure; clf; hold on; grid on;
    plot(tout, y, 'b', 'LineWidth', 1.6);                      % step
    % Tangente sobre todo el rango
    y_tan = y_max + S*(tout - t_max);
    plot(tout, y_tan, 'm--', 'LineWidth', 1.2);                % tangente
    % Marcas L y L+T
    xline(L,    ':', 'L',   'LabelOrientation','horizontal');
    xline(L+T,  ':', 'L+T', 'LabelOrientation','horizontal');
    % Puntos destacados
    plot(t_max, y_max, 'ro', 'MarkerFaceColor','r');           % punto de máx. slope
    yline(K, 'k--', 'K (final)');

    xlabel('Tiempo [s]'); ylabel('Salida');
    title(sprintf('ZN step: L=%.4g s, T=%.4g s, S=%.4g, t_{max}=%.4g s', L, T, S, t_max));
    legend('step(G)', 'tangente en máx. pendiente', 'Location','best');

    % (Opcional) impresión en consola
    fprintf('ZN: L=%.6g s, T=%.6g s, S=%.6g, t_max=%.6g s (K=%.6g)\n', L, T, S, t_max, K);

    
end
