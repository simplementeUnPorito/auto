function S = ver_intersample_desde_u(G, Ts, ud, M, N)
% VER_INTERSAMPLE_DESDE_U - Simula inter-muestra con G(s) y un esfuerzo u[k] dado.
% Uso:
%   S = ver_intersample_desde_u(G, Ts, ud, M, N)
% Entradas:
%   G  : planta continua (tf/ss continuo)
%   Ts : periodo de muestreo del esfuerzo u[k]
%   ud : vector de esfuerzo discreto u[k] (fila o columna)
%   M  : (opcional) nº de muestras a usar
%        0 o [] => usar todo ud
%        >0     => usar las primeras M muestras
%   N  : (opcional) sobremuestreo del ZOH (submuestras por periodo, ej. 30; default 30)
%
% Salida (struct):
%   S.Ts, S.tk, S.uk, S.yk       : señales discretas (k*Ts)
%   S.tc, S.uc, S.yc             : señales continuas (ZOH + lsim(G))
%   (y figura con y_c(t) y y[k], y u[k])

    if nargin < 5 || isempty(N), N = 30; end
    if nargin < 4 || isempty(M), M = 0;  end
    if Ts <= 0, error('Ts debe ser > 0.'); end

    % --- Acomodar u[k] y recorte opcional ---
    uk = ud(:);                      % columna
    if M > 0
        M = min(M, numel(uk));
        uk = uk(1:M);
    end

    K  = numel(uk) - 1;              % última muestra = k=K
    tk = (0:K).' * Ts;

    % --- Reconstrucción inter-muestra por ZOH explícito ---
    [tc, uc] = zoh_stretch(uk, Ts, N);    % uc(t) pieza-constante
    yc       = lsim(G, uc, tc);           % salida continua

    % Muestras de la salida continua exactamente en k*Ts
    yk = yc(1:N:end);   % cada N submuestras corresponde a un instante k*Ts
    % (Por construcción, length(yk) == length(uk) == length(tk))

    % --- Plots (eje en ms) ---
    figure('Name','Intersample desde u[k]');
    tiledlayout(2,1);

    % y_c(t) y y[k]
    nexttile; hold on; grid on; grid minor;
    plot(tc*1e3, yc, 'LineWidth', 1.2);
    stairs(tk*1e3, yk, 'k.', 'LineWidth', 1.0);
    xlabel('t [ms]'); ylabel('y');
    title(sprintf('Salida continua y_c(t) y muestras y[k]  (T_s = %.4g ms)', Ts*1e3));
    legend('y_c(t)','y[k]','Location','best');

    % u[k]
    nexttile; hold on; grid on; grid minor;
    stairs(tk*1e3, uk, 'LineWidth', 1.2);
    xlabel('t [ms]'); ylabel('u[k]');
    title('Esfuerzo aplicado');

    % --- Salida estructurada ---
    S = struct('Ts', Ts, ...
               'tk', tk, 'uk', uk, 'yk', yk, ...
               'tc', tc, 'uc', uc, 'yc', yc);
end

% ===== Helper: ZOH explícito =====
function [t_hi, u_hi] = zoh_stretch(u_k, T, M)
    % Repite cada muestra M veces (ZOH) y arma el vector de tiempo continuo
    if size(u_k,2) > 1, u_k = u_k(:); end
    u_hi = repelem(u_k, M);
    t_hi = (0:numel(u_hi)-1).' * (T/M);
end
