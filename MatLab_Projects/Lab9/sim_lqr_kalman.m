function [X, Xhat, U, y_true, y_meas] = ...
    sim_lqr_kalman(A,B,C,K1,K2,L,Ts,N,r,off,sigma_w,sigma_v)
% SIM_LQR_KALMAN  Simula lazo con LQR con integrador + estimador (L puede ser Kalman o Luenberger)
%
% Entradas:
%   A,B,C    : modelo discreto
%   K1,K2    : ganancias del LQR aumentado (sobre integrador y estados)
%   L        : ganancia del estimador (Kalman o Luenberger)
%   Ts       : tiempo de muestreo
%   N        : número de muestras
%   r        : referencia (vector 1xN)
%   off      : offset DC (para sumar al final)
%   sigma_w  : std del ruido de proceso
%   sigma_v  : std del ruido de medición
%
% Salidas:
%   X        : estados reales (con offset aplicado al final)
%   Xhat     : estados estimados (con offset aplicado al final)
%   U        : esfuerzo de control (con offset aplicado al final)
%   y_true   : salida "ideal" Cx (con offset)
%   y_meas   : salida medida con ruido (con offset)

    n = size(A,1);

    % ==== Generación de ruidos en TODAS las muestras ====
    w_proc =sigma_w'.* randn(n,N);      % ruido de proceso para todos los estados
    v_meas = sigma_v*randn(1,N);      % ruido de medición en toda la simulación

    % ==== Variables de estado ====
    X     = zeros(n,N);               % estados reales
    Xhat  = zeros(n,N);               % estados estimados
    Vint  = zeros(1,N);               % integrador del error
    U     = zeros(1,N);               % control
    y_true = zeros(1,N);
    y_meas = zeros(1,N);

    % condición inicial estimador (podés cambiarla si querés)
    %Xhat(:,1) = [0.1; 0.1];

    for k = 1:N-1
        rk = r(k);

        % --- salida real y medida en k ---
        y_true(k) = C*X(:,k);
        y_meas(k) = y_true(k) + v_meas(k);

        % --- integrador usando la MEDIDA (lo que pasa en la vida real) ---
        Vint(k+1) = Vint(k) + (rk - y_meas(k));

        % --- control con estimación ---
        u_k  = K1*Vint(k+1) - K2*Xhat(:,k);
        U(k) = u_k;

        % --- planta real con ruido de proceso ---
        X(:,k+1) = A*X(:,k) + B*u_k + w_proc(:,k);

        % --- estimador ("actual"): usa y(k+1) ---
        % predicción
        xhat_pred = A*Xhat(:,k) + B*u_k;
        yhat_pred = C*xhat_pred;

        % medida k+1
        y_true(k+1) = C*X(:,k+1);
        y_meas(k+1) = y_true(k+1) + v_meas(k+1);

        % corrección
        Xhat(:,k+1) = xhat_pred + L*(y_meas(k+1) - yhat_pred);
    end

    % Última muestra: control constante
    U(N) = U(N-1);

    % ==== Aplicar offset (AC) ====
    X      = X      + off;
    Xhat   = Xhat   + off;
    y_true = y_true + off;
    y_meas = y_meas + off;
    U      = U      + off;

end
