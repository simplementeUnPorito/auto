function [yd, ud, ed, t] = sim_compensador_n_order(G, C, refd, umin, umax)
% SIM_COMPENSADOR_N_ORDER
% Simula el lazo con controlador C(z) y planta G(s) (discretizada por ZOH)
% usando ecuaciones recursivas (posición) y saturación en u.
%
% Entradas:
%   G     : tf/ss continuo (planta)
%   C     : tf discreto (controlador), con Ts>0
%   refd  : vector columna o fila con la referencia discreta r[k]
%   umin, umax : saturación del esfuerzo
%
% Salidas:
%   yd : salida discreta y[k] (tamaño = length(refd))
%   ud : esfuerzo u[k]
%   ed : error e[k] = r[k] - y[k]
%   t  : tiempo (k*Ts)

    % ===== Ts y discretización de la planta =====
    Ts = getTsStrict(C);
    if Ts <= 0
        error('C debe ser discreto con Ts>0.');
    end
    Gd = c2d(G, Ts, 'zoh');

    % ===== Coefs en z^{-1} y normalización a(0)=1 =====
    [Nc, Dc] = tfdata(tf(C),  'v');   Nc = Nc(:).'; Dc = Dc(:).';
    [Nb, Db] = tfdata(tf(Gd), 'v');   Nb = Nb(:).'; Db = Db(:).';

    if abs(Dc(1)) < 1e-12, error('Controlador impropio/no causal: Dc(1)=0'); end
    if abs(Db(1)) < 1e-12, error('Planta discreta no causal: Db(1)=0'); end

    Nc = Nc / Dc(1);  Dc = Dc / Dc(1);
    Nb = Nb / Db(1);  Db = Db / Db(1);

    % Órdenes efectivos (en z^{-1})
    mc = find(abs(Nc) > 1e-12, 1, 'last') - 1;   % orden numerador C
    ac = find(abs(Dc) > 1e-12, 1, 'last') - 1;   % orden denominador C (sin a0)
    mb = find(abs(Nb) > 1e-12, 1, 'last') - 1;   % orden numerador Gd
    ab = find(abs(Db) > 1e-12, 1, 'last') - 1;   % orden denominador Gd (sin a0)

    % ===== Señales y tiempo =====
    refd = refd(:);
    N  = numel(refd);
    t  = (0:N-1)' * Ts;

    yd = zeros(N,1);
    ud = zeros(N,1);
    ed = zeros(N,1);

    % ===== Bucle discreto causal =====
    % Estructura:
    %   Dc(q^-1) u[k] = Nc(q^-1) e[k]
    %      -> u[k] = sum_{i=0..mc} Nc_i e[k-i] - sum_{i=1..ac} Dc_i u[k-i]
    %   Db(q^-1) y[k] = Nb(q^-1) u[k]
    %      -> y[k] = sum_{i=0..mb} Nb_i u[k-i] - sum_{i=1..ab} Db_i y[k-i]
    %
    % Para alinear con tu estilo anterior (y(k+1) con u(k)), acá actualizo y(k)
    % en el mismo instante, que es la forma estándar con z^{-1}. Es consistente
    % con Gd = c2d(G,Ts,'zoh').

    for k = 1:N
        % error con la y disponible de este mismo índice
        ed(k) = refd(k) - yd(k);

        % --- Controlador ---
        % parte con e[k-i]
        mE = min(mc, k-1);
        e_vec  = [ed(k); flipud(ed(k-1:-1:k-mE))];
        Nc_vec = Nc(1:mE+1).';         % Nc0..NcmE

        % parte de realimentación con u[k-i], i>=1
        mU = min(ac, k-1);
        if mU > 0
            u_fb  = flipud(ud(k-1:-1:k-mU));
            Dc_fb = Dc(2:mU+1).';      % a1..a_mU
            u_unsat = dot(Nc_vec, e_vec) - dot(Dc_fb, u_fb);
        else
            u_unsat = dot(Nc_vec, e_vec);
        end

        % saturación del esfuerzo
        ud(k) = min(max(u_unsat, umin), umax);

        % --- Planta discreta ---
        % y[k] con u[k-i]
        mUb = min(mb, k-1);
        u_vec  = [ud(k); flipud(ud(k-1:-1:k-mUb))];
        Nb_vec = Nb(1:mUb+1).';

        mYb = min(ab, k-1);
        if mYb > 0
            y_fb  = flipud(yd(k-1:-1:k-mYb));
            Db_fb = Db(2:mYb+1).';
            yd(k) = dot(Nb_vec, u_vec) - dot(Db_fb, y_fb);
        else
            yd(k) = dot(Nb_vec, u_vec);
        end
    end
end

% ===== Helper =====
function Ts = getTsStrict(sys)
    Ts = 0; try, Ts = sys.Ts; catch, end
    if isempty(Ts), Ts = 0; end
end
