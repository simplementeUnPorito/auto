function [yd, ud, ucd, ed, t] = sim_lazo_discreto_sat(Gd, Cd, refd, umin, umax)
% SIM_LAZO_DISCRETO_SAT
% Simula un lazo discreto con controlador Cd y planta Gd (ambos tf discretos),
% separando la señal de control antes de saturación (uc) y la aplicada (u).
%
% Entradas:
%   Gd   : planta discreta (tf) en z, causal
%   Cd   : controlador discreto (tf) en z, causal
%   refd : referencia discreta r[k] (vector fila o columna)
%   umin, umax : límites de saturación del esfuerzo
%
% Salidas:
%   yd  : salida de la planta y[k] (mismo largo que refd)
%   ud  : esfuerzo aplicado u[k] (saturado)
%   ucd : esfuerzo del controlador sin saturar u_c[k]
%   ed  : error e[k] = r[k] - y[k]
%   t   : tiempo discreto (k*Ts)

    Ts = 0; try, Ts = Cd.Ts; catch, end
    if isempty(Ts) || Ts <= 0, error('Cd debe tener Ts > 0.'); end

    % ===== Coeficientes en z^{-1} y normalización a(0)=1 =====
    [Nc, Dc] = tfdata(Cd, 'v');  Nc = Nc(:).'; Dc = Dc(:).';
    [Nb, Db] = tfdata(Gd, 'v');  Nb = Nb(:).'; Db = Db(:).';

    if abs(Dc(1)) < 1e-12, error('Controlador impropio/no causal: Dc(1)=0'); end
    if abs(Db(1)) < 1e-12, error('Planta discreta impropia/no causal: Db(1)=0'); end

    % Normalizar para que los denominadores empiecen en 1
    Nc = Nc / Dc(1);   Dc = Dc / Dc(1);
    Nb = Nb / Db(1);   Db = Db / Db(1);

    % ===== Preparar señales =====
    refd = refd(:);           % columna
    N = numel(refd);
    t = (0:N-1).' * Ts;

    % y con un paso extra para causalidad explícita (y[k+1])
    y  = zeros(N+1,1);
    ed = zeros(N,1);
    ucd = zeros(N,1);   % señal del controlador (sin saturación)
    ud  = zeros(N,1);   % señal aplicada (saturada)

    % ===== Bucle causal =====
    % Ecuaciones:
    %   Dc(q^-1) * uc[k] = Nc(q^-1) * e[k]
    %   Db(q^-1) * y[k]  = Nb(q^-1) * u[k]
    %
    % Ojo: acá actualizamos y(k+1) para respetar el retardo de ZOH.

    for k = 1:N
        % error con la salida disponible del mismo índice (y[k])
        ed(k) = refd(k) - y(k);

        % ---- Controlador (sin saturación): uc[k] ----
        uc_k = 0.0;

        % término por e[k-i]
        for i = 0:length(Nc)-1
            if k-i >= 1
                uc_k = uc_k + Nc(i+1) * ed(k-i);
            end
        end

        % realimentación por uc[k-i], i >= 1
        for i = 1:length(Dc)-1
            if k-i >= 1
                uc_k = uc_k - Dc(i+1) * ucd(k-i);
            end
        end

        ucd(k) = uc_k;

        % ---- Saturación y esfuerzo aplicado ----
        ud(k) = min(max(uc_k, umin), umax);

        % ---- Planta: y[k+1] con u[k-i] e y[k-i] ----
        y_next = 0.0;

        % parte directa por u[k-i]
        for i = 0:length(Nb)-1
            if k-i >= 1
                y_next = y_next + Nb(i+1) * ud(k-i);
            end
        end

        % realimentación por y[k-i], i >= 1
        for i = 1:length(Db)-1
            if k-i >= 1
                y_next = y_next - Db(i+1) * y(k-i+1);  % nota: y es (k+1)
            end
        end

        % avanzar estado de la planta
        y(k+1) = y_next;
    end

    % Salida alineada con refd
    yd = y(1:N);
end
