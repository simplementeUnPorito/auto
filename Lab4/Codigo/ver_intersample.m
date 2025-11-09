function S = ver_intersample(G, C, M, N)
% VER_INTERSAMPLE - Visualiza oscilaciones entre-muestras con ZOH explícito
% Uso:
%   S = ver_intersample(G, C, M, N)
% Entradas:
%   G : planta continua (tf/ss continuo)
%   C : controlador digital (tf/ss discreto, con Ts > 0)
%   M : 0 => simular hasta t = SettlingTime(G)
%       >0 => simular M*Ts
%   N : factor de sobremuestreo del ZOH (submuestras por periodo, ej. 30)

    Ts = getTsStrict(C);
    if Ts <= 0
        error('El controlador C debe ser discreto con Ts > 0.');
    end

    % Discretizar la planta
    Gd = c2d(G, Ts, 'zoh');

    % Lazos cerrados
    Gcl = feedback(C*Gd, 1);  % r->y[k]
    Ucl = feedback(C, Gd);    % r->u[k]

    Tfinal = M*Ts;

    K = max(1, floor(Tfinal/Ts));
    tk = (0:K).' * Ts;

    % Respuestas discretas
    [yk, ~] = step(Gcl, tk);
    rk = ones(size(yk));
    ek = rk - yk;
    uk = lsim(C, ek, tk);

    % ZOH explícito
    [tc, uc] = zoh_stretch(uk, Ts, N);
    yc = lsim(G, uc, tc);

    % ====== SUBPLOTS (x en ms) ======
    figure('Name','Intersample analysis');
    tiledlayout(2,1);

    % y_c(t) y y[k]
    nexttile;
    plot(tc*1e3, yc, 'LineWidth',1.2); grid on; hold on; grid minor;
    stairs(tk*1e3, yk, 'k.');
    xlabel('milisegundos'); ylabel('Voltios');
    title(sprintf('Respuesta de la planta controlada al escalón T_s = %.4g ms',Ts*1e3));
    legend('y_c(t)','y[k]','Location','best');

    % u[k]
    nexttile;
    stairs(tk*1e3, uk, 'LineWidth',1.5); grid on; grid minor;
    xlabel('milisegundos'); ylabel('Voltios');
    title('Esfuerzo del controlador para el escalón');
    legend('u[k]','Location','best');

    % Salida estructurada
    S = struct('Ts', Ts, 'Tfinal', tk(end), ...
               'tk', tk, 'yk', yk, 'uk', uk, 'ek', ek, ...
               'tc', tc, 'uc', uc, 'yc', yc, ...
               'Gd', Gd, 'Gcl', Gcl, 'Ucl', Ucl);
end

% Helpers
function Ts = getTsStrict(sys)
    Ts = 0;
    try, Ts = sys.Ts; catch, end
    if isempty(Ts), Ts = 0; end
end

function [t_hi, u_hi] = zoh_stretch(u_k, T, M)
    if size(u_k,2) > 1, u_k = u_k(:); end
    u_hi = repelem(u_k, M);
    t_hi = (0:numel(u_hi)-1).' * (T/M);
end
