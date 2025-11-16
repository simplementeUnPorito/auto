function L = disenar_kalman_simple(A,C,G,sigma_w,sigma_v)
% DISENAR_KALMAN_SIMPLE  Ganancia L de Kalman (estado estacionario)
%   L = disenar_kalman_simple(A,C,sigma_w,sigma_v)
%   Asume:
%       x[k+1] = A x[k] + B u[k] + w[k],   cov(w) = sigma_w^2 * I
%       y[k]   = C x[k] + v[k],           cov(v) = sigma_v^2 * I

    n  = size(A,1);
    ny = size(C,1);

    Q = (sigma_w^2)*eye(n);            % covarianza proceso
    R = (sigma_v^2)*eye(ny);           % covarianza medición

    [L,~,~] = dlqe(A,G,C,Q,R);         % LQE discreto -> ganancia de Kalman

end
