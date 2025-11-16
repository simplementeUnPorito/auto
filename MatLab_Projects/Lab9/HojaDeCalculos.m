%% ====================== DISEÑO CON POLOS OBJETIVO ÚNICOS ======================
clear all; clc; close all;
addpath('..\');

%% --- Planta continua (tu modelo) ---
C2 = 103.07e-9;  C1 = 211.1e-9;
R3 = 6.74e3;     R4 = 14.760e3;
R1 = 46.4e3;     R2 = 81.09e3;

[sysC,fn,Tn] = plantaLabs(R1,R2,R3,R4,C1,C2);
Ts = 1/1e3;   % 1 kHz

sysD = c2d(sysC, Ts, 'zoh');
[A,B,C,D] = ssdata(sysD);

%% Añadimos el integrador
n = size(A,1);
m = 1;
Ahat = [A B; zeros(m,n+m)];
Bhat = [zeros(n,m); eye(m)];
Chat = [C zeros(1,m)];
sysDI = ss(Ahat,Bhat,Chat,D);

%% Pesos LQR (para el sistema aumentado)
Q1 = {diag([0.01 0.01 0.001]), ...
      diag([0.01 0.01 0.1]),   ...
      diag([0.01 0.01 0.001])};
Q2 = {1, 0.1, 0.01};

K  = cell(size(Q2)); 
K1 = K; 
K2 = K; 
P  = K;

%% Parámetros de simulación y ruido (ACÁ TOCÁS VOS)
N   = 200;
t   = 0:Ts:(N-1)*Ts;
r   = ones(1,N);      
r(1:30) = 0;

off      = 2.024;   % offset DC de tu planta
sigma_w  = 1e-3;    % std ruido de proceso (w)   <-- CAMBIÁS ACÁ
sigma_v  = 10e-3;    % std ruido de medición (v)  <-- CAMBIÁS ACÁ
G = eye(n);                        % ruido de proceso en todos los estados

% Ganancia de Kalman (misma para todos los LQR en este ejemplo)
L_kal = disenar_kalman_simple(A,C,G,sigma_w,sigma_v);

% (Opcional) polos "5 veces más rápidos" para comparador de Luenberger
p_obs = [0.2+1i*0.2, 0.2-1i*0.2];
L_rapido = acker(A', (C*A)', p_obs).';   % segundo estimador para comparar


%% =================== Bucle sobre los diseños LQR ===================
for i = 1:length(Q2)
    % ========= LQR DISCRETO sobre el sistema AUMENTADO =========
    [Ki,~,Pi] = dlqr(Ahat, Bhat, Q1{i}, Q2{i});  % dlqr con matrices
    K{i}  = Ki; 
    P{i}  = Pi;
    K2{i} = Ki(1,1:n);        % sobre x
    K1{i} = Ki(1,n+1:end);    % sobre integrador (z)
    
    % Dinámica cerrada del sistema AUMENTADO (para análisis de control)
    Acl = [A B;
          (K2{i}-K2{i}*A-K1{i}*C*A) (eye(m)-K2{i}*B-K1{i}*C*B)];
    Bcl = [zeros(n,1); K1{i}];   % generalizado para n estados
    
    % ======== SIMULACIÓN CON KALMAN (ruido en TODO el horizonte) ========
    [Xi_kal, Xhi_kal, Ui_kal, y_true_kal, y_meas_kal] = ...
        sim_lqr_kalman(A,B,C,K1{i},K2{i},L_kal,Ts,N,r,off,sigma_w,sigma_v);
    
    % ======== SIMULACIÓN CON OBSERVADOR "RÁPIDO" (ACKER) ========
    [Xi_lu, Xhi_lu, Ui_lu, y_true_lu, y_meas_lu] = ...
        sim_lqr_kalman(A,B,C,K1{i},K2{i},L_rapido,Ts,N,r,off,sigma_w,sigma_v);

    p_kal = eig(A - L_kal*C);
    % ====== Graficar comparación ======
    plot_comparacion_estimadores( ...
        t, r+off, ...
        Xi_kal, Xhi_kal, Ui_kal, y_true_kal, y_meas_kal, ...
        Xi_lu,  Xhi_lu,  Ui_lu,  y_true_lu,  y_meas_lu, ...
        C, Acl, Bcl, p_obs, p_kal, Ts, i);
end
