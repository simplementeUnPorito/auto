function [td, refd, yd, ud, ed, coefs] = sim_compensador_first_order( ...
    Gd, z0, p0, Kc, T, umin, umax, refmin, refmax, n_per_seg)

% ===== Igual que tu PID: y(k+1)=b0*u(k)+b1*u(k-1)-a1*y(k)-a2*y(k-1) =====
% Controlador (posición): u(k) = p0*u(k-1) + Kc*ed(k) - Kc*z0*ed(k-1)
% ed(k) = refd(k) - yd(k)

    if nargin < 10 || isempty(n_per_seg), n_per_seg = 600; end
    if ~isa(Gd,'tf'), error('Gd debe ser tf discreto.'); end

    % --- Coefs planta exactamente como en tu ejemplo ---
    [numD, denD] = tfdata(Gd, 'v');
    b0 = numD(2);
    b1 = numD(3);
    a1 = denD(2);
    a2 = denD(3);
    coefs = struct('b0',b0,'b1',b1,'a1',a1,'a2',a2);

    % --- Tiempo y referencia (refmin -> refmax -> refmin) ---
    N  = 3*n_per_seg;
    td = (0:N-1)' * T;
    refd = [refmin*ones(n_per_seg,1);
            refmax*ones(n_per_seg,1);
            refmin*ones(n_per_seg,1)];

    % --- Inicialización idéntica a tu estilo ---
    yd = zeros(N,1);
    ed = zeros(N,1);
    ud = zeros(N,1);

    % --- Loop (misma estructura y orden que tu PID) ---
    for k = 3:N-1
        ed(k) = refd(k) - yd(k);

        % Compensador: u(k) = p0*u(k-1) + Kc*ed(k) - Kc*z0*ed(k-1)
        u_k = p0*ud(k-1) + Kc*ed(k) - Kc*z0*ed(k-1);

        
        if u_k>umax
            ud(k) = umax;
        elseif u_k<umin
            ud(k) = umin;
        else 
            ud(k) = u_k;
        end
    

        % Planta discreta (dividido por a0 implícito como en tu ejemplo)
        yd(k+1) = b0*ud(k) + b1*ud(k-1) - a1*yd(k) - a2*yd(k-1);
    end
end
