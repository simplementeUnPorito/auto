function[X,Xhat,U] = sim_pred_int(A,B,C,D,K1,K2,L,N,Xini)
n = size(A,1);
X=zeros(n,N);Xhat = zeros(n,N);Yhat=zeros(1,N);U=zeros(1,N);V=zeros(1,N);Z = zeros(2,N); Y = zeros(1,N);Yhat = zeros(1,N);
Xhat(:,1) = Xini;
r = ones(1,N);
for k = 1:N-1
    % --- (a) Medición ---
    Y(k) = C * X(:,k)+D*V(k);
   
    % --- (b) Integrador del error ---
    if k>1
        V(k) = V(k-1) + (r(k) - Y(k));
    else
        V(k) = (r(k) - Y(k));
    end
    % --- (c) Control (igual que ISR) ---
    U(k) = K1 * V(k) - K2 * Xhat(:,k);
    
    % --- (d) Evolucion de la planta Planta ---
    X(:,k+1) = A * X(:,k) + B * U(k);
    
    % --- (e) Predicción de la evolucion---
    Yhat(k) = C*Xhat(:,k);
    Xhat(:,k+1) = A*Xhat(:,k) + B * U(k)+ L * (Y(k) - Yhat(k));
 
    
end
% --- (a) Medición ---
Y(N) = C * X(:,N); % planta real
% --- (b) Corrección actual ---
Xhat(:,N) = Z(:,N) + L * (Y(N) - C * Z(:,N));

% --- (c) Integrador del error ---
V(k) = V(k-1) + (r(k) - Y(k));
% --- (d) Control (igual que ISR) ---
U(N) = K1 * V(N) - K2 * Xhat(:,N);

end