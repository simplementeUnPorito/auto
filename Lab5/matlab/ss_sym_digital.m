function [X, Y, U] = ss_sym_digital(sys, ref, K, K0, x0,sigma)
% Discreto, simple:
%   u(k) = K0*ref(k) - K*x(k)
%   x+   = A*x + B*u
%   y(k) = C*x + D*u   (convención típica)
    [A,B,C,D] = ssdata(sys);
    n = size(A,1);
    N = numel(ref);

    if nargin < 5 || isempty(x0), x0 = zeros(n,1); end

    X = zeros(n,N);
    U = zeros(1,N);
    Y = zeros(1,N);
    ruido = sigma*randn(N);
    X(:,1) = x0;
    for k = 1:N-1
        U(k)     = K0*ref(k) - K*X(:,k)+ruido(k);
        X(:,k+1) = A*X(:,k) + B*U(k);
        Y(k)     = C*X(:,k) + D*U(k);
    end
    % último sample
    U(N) = K0*ref(N) - K*X(:,N);
    Y(N) = C*X(:,N) + D*U(N);

    % para plot cómodo (Nxn)
    X = X.'; Y = Y(:); U = U(:);
end
