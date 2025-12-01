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

k1 = abs(R2/R1);
k2 = abs(R4/R3);
Sx = [1/k1, 1/(k1*k2), 1];   % [x1; x2; z] -> [x1_norm; x2_norm; z]

% caso (a): rápido
Q_fast = [ 100, 100, 1];   % 

% caso (b): ahorro de esfuerzo
Q_soft = [ 2, 4,  1.5];

Q1{1} = diag(Q_fast.*Sx);
Q1{2} = diag(Q_soft.*Sx);


Q2 = {20, 4};

K  = cell(size(Q2)); 
K1 = K; 
K2 = K; 
P  = K;

%% Parámetros de simulación y ruido (ACÁ TOCÁS VOS)

Vpp_adc = 4.8;    % ±2.4 V
Nbits_adc = 12;


Vpp_dac = 4.048;
off      = 2.024;   % offset DC de tu planta
Nbits_dac = 8;

[lsb_adc, sigmaq_adc, snr_adc] = quanti_noise_from_bits(Nbits_adc, Vpp_adc);
[lsb_dac, sigmaq_dac, snr_dac] = quanti_noise_from_bits(Nbits_dac, Vpp_dac);



f_pts  = [0.01 0.1 2 10 100 1000]*1e3;      % Hz, ejemplo
S_nV   = [700 300 100 60 50 45];     % nV/sqrt(Hz), ejemplo
S_pts  = S_nV * 1e-9;                       % V/sqrt(Hz)
BW = 500;
G1 = R2/R1;G2=R4/R3;

poles_plant = [-1/(C1*R2)];           % rad/s

[vrms_unf, vrms_filt] = noise_from_psd_with_poles(f_pts, S_pts, poles_plant);
vrms_filt1 = G1*vrms_filt;

poles_plant = [-1/(C2*R4)];
[vrms_unf, vrms_filt] = noise_from_psd_with_poles(f_pts, S_pts, poles_plant);
vrms_filt2 = G2*vrms_filt;


[vrms_R1, en_R1] = resistor_noise(R1, BW, G1);
[vrms_R2, en_R2] = resistor_noise(R2, BW, 1);
[vrms_R3, en_R3] = resistor_noise(R3, BW, G2);
[vrms_R4, en_R4] = resistor_noise(R4, BW, 1);

Vpp_noise_source = 10e-3;
vrms_noise_source = Vpp_noise_source/6;
sigma_x1 = sqrt(vrms_filt1^2 + vrms_R1^2 + vrms_R2^2 + sigmaq_dac^2+vrms_noise_source^2);
sigma_x2 = sqrt(vrms_filt2^2 + vrms_R3^2 + vrms_R4^2);





sigma_w  = [sigma_x1,sigma_x2];    % std ruido de proceso (w)  
sigma_v = sigmaq_adc;              % std ruido de medicion (v)

B1 = eye(n);                        % ruido de proceso en todos los estados

% Ganancia de Kalman (misma para todos los LQR en este ejemplo)
L_kal = disenar_kalman_simple(A,C,B1,sigma_w,sigma_v);

% (Opcional) polos "5 veces más rápidos" para comparador de Luenberger
L_obs =K;
Aux  = [A-eye(size(A))  B;
        C*A             C*B];
Aux = inv(Aux);
%% =================== Bucle sobre los diseños LQR ===================
for i = 1:1:length(Q2)
    
    % ========= LQR DISCRETO sobre el sistema AUMENTADO =========
    [Ki,~,Pi] = dlqr(Ahat, Bhat, Q1{i}, Q2{i});  % dlqr con matrices
    Ki    = (Ki+ [zeros(1,n) eye(m)])*Aux;
    K{i}  = Ki; 
    P{i}  = Pi;
    K2{i} = Ki(1,1:n);        % sobre x
    K1{i} = Ki(1,n+1:end);    % sobre integrador (z)
    
    % Dinámica cerrada del sistema AUMENTADO (para análisis de control)
    Acl = [A B;
      (K2{i}-K2{i}*A-K1{i}*C*A) (eye(m)-K2{i}*B-K1{i}*C*B)];

    Bcl = [zeros(n,1); K1{i}];   % generalizado para n estados
    Ccl = [C 0];
    sysCl = ss(Acl,Bcl,Ccl,D,Ts);
    N = ceil(2*ceil(stepinfo(sysCl).SettlingTime/Ts));
    
    r   = ones(1,N);      
    r(1:ceil(N/4)) = 0;
    t   = (0:Ts:(N-1)*Ts)*1000;

    Pcl = eig(Acl);           % columna, 3x1

    % Elegir los n polos dominantes (más cercanos al círculo unidad)
    [~, idx] = sort(abs(Pcl), 'descend');  % de mayor |z| a menor
    Pdom = Pcl(idx(1:n));                  % n polos “importantes”
    
    % Hacerlos 5 veces más rápidos (equivalente a escalar en s-plane)
    p_obs = Pdom.^5;   % columna 2x1

    L_rapido = acker(A', (C*A)',p_obs' ).';   % segundo estimador para comparar
    L_obs{i}=L_rapido;
    p_kal = eig(A - L_kal*C);


    % ======== SIMULACIÓN CON KALMAN (ruido) ========
    [Xi_kal, Xhi_kal, Ui_kal, y_true_kal, y_meas_kal] = ...
        sim_lqr_kalman(A,B,C,K1{i},K2{i},L_kal,Ts,N,r,off,[0,0],0);
    
    % ======== SIMULACIÓN CON OBSERVADOR "RÁPIDO" (ACKER) ========
    [Xi_lu, Xhi_lu, Ui_lu, y_true_lu, y_meas_lu] = ...
        sim_lqr_kalman(A,B,C,K1{i},K2{i},L_rapido,Ts,N,r,off,[0,0],0);

    % ====== Graficar comparación ======
    plot_comparacion_estimadores( ...
        t, r+off,[0,0],0, ...
        Xi_kal, Xhi_kal, Ui_kal, y_true_kal, y_meas_kal, ...
        Xi_lu,  Xhi_lu,  Ui_lu,  y_true_lu,  y_meas_lu, ...
        C, Acl, Bcl, p_obs, p_kal, Ts, i,sysC);


    % ======== SIMULACIÓN CON KALMAN (ruido) ========
    [Xi_kal, Xhi_kal, Ui_kal, y_true_kal, y_meas_kal] = ...
        sim_lqr_kalman(A,B,C,K1{i},K2{i},L_kal,Ts,N,r,off,sigma_w,sigma_v);
    

    % ======== SIMULACIÓN CON OBSERVADOR "RÁPIDO" (ACKER) ========
    [Xi_lu, Xhi_lu, Ui_lu, y_true_lu, y_meas_lu] = ...
        sim_lqr_kalman(A,B,C,K1{i},K2{i},L_rapido,Ts,N,r,off,sigma_w,sigma_v);



    % ====== Graficar comparación ======
    plot_comparacion_estimadores( ...
        t, r+off,sigma_w,sigma_v, ...
        Xi_kal, Xhi_kal, Ui_kal, y_true_kal, y_meas_kal, ...
        Xi_lu,  Xhi_lu,  Ui_lu,  y_true_lu,  y_meas_lu, ...
        C, Acl, Bcl, p_obs, p_kal, Ts, i,sysC);
end
