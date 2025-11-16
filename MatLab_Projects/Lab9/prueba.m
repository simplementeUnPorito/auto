%% ====================== DISEÑO CON POLOS OBJETIVO ÚNICOS ======================
clear all; clc; close all;
addpath('..\');

%% --- Planta continua (tu modelo) ---
C2 = 103.07e-9;  C1 = 211.1e-9;
R3 = 6.74e3;      R4 = 14.760e3;
R1 = 46.4e3;       R2 = 81.09e3;

tau1 = R2*C1;           k1 = -R2/R1;
tau2 = R4*C2;           k2 = -R4/R3;

F = [ -1/tau1,     0;
       k2/tau2, -1/tau2 ];
G = [ k1/tau1; 0 ];
H = [0 1];
J = 0;

sysC = ss(F,G,H,J);
Gc   = tf(sysC);

% Ts vía polo más rápido (aunque después fijás Ts a 1 ms)
fn = max(abs(zpk(Gc).P{1}))/pi;
Tn = 1/fn;
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

%% Simulación
N  = 200;
t  = 0:Ts:(N-1)*Ts;
r  = ones(1,N);      
r(1:30) = 0;

% ===== Definición de ruidos =====

% "Envolvente" de ruido de medición (empieza en la muestra 150)
w_env = 0.01*rand;   % desvío típico aprox.
v     = w_env .* randn(1,N);               % ruido de medición v(k)

% Ruido de PROCESO en los estados (afecta a todos los estados)
sigma_proc = 1e-3;                         % ajustá según tu S/R real
w_proc     = sigma_proc*randn(n,N);        % w_proc(k) ~ N(0, sigma_proc^2 I)

% polos deseados que usabas para el observador "rápido" (para comparar si querés)
p_obs = [0.2+1i*0.2, 0.2-1i*0.2];

% ===== Diseño del filtro de KALMAN (LQE discreto) =====

% Covarianza aproximada del ruido de medición (a partir de v)
Rk = mean(v.^2);      % escalar, porque tenés una sola salida

% Relación S/R del PROCESO respecto a la medición (Q/R)
% OJO: el enunciado dice "a partir de S/R"; acá lo parametrizo:
S_sobre_R = 0.01;        % <-- ajustá este valor según la S/R que midas en el lab

Qk = S_sobre_R * Rk * eye(n);   % Q: covarianza de proceso
Gk = eye(n);                    % ruido afecta todos los estados

% Ganancia óptima de Kalman (discreta, LQE)
[L_kal,Pk,~] = dlqe(A,Gk,C,Qk,Rk);

% (si querés comparar con el observador "cinco veces más rápido",
% seguirías usando algo tipo:
% L_rapido = acker(A',(C*A)',p_obs).';
% y hacés otra simulación con ese L)

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
    
    %% =================== SIMULACIÓN con ruido de proceso + medición ===================
    Xi_a    = zeros(n,N);          % estados reales
    Xhi_a   = zeros(n,N);          % estimación de estados
    Xhi_a(:,1) = [0.1;0.1];        % condición inicial del estimador
    
    Vi_a    = zeros(1,N);          % integrador del error
    Ui_a    = zeros(1,N);          % esfuerzo de control
    Y_meas  = zeros(1,N);          % salida medida (con ruido)
    
    for k = 1:N-1
        rk = r(k);
        
        % --- salida real y medida en k ---
        y_true_k   = C*Xi_a(:,k);
        y_meas_k   = y_true_k + v(k);
        Y_meas(k)  = y_meas_k;
        
        % --- integrador usando la MEDIDA (como en el experimento) ---
        Vi_a(k+1) = Vi_a(k) + (rk - y_meas_k);
        
        % --- control con estimación: u(k) ---
        ui_a      = K1{i}*Vi_a(k+1) - K2{i}*Xhi_a(:,k);
        
        % --- planta real con ruido de PROCESO ---
        Xi_a(:,k+1) = A*Xi_a(:,k) + B*ui_a + w_proc(:,k);
        
        % --- estimador de Kalman (forma "actual" usando y(k+1)) ---
        % predicción
        z_ai        = A*Xhi_a(:,k) + B*ui_a;
        
        % medida en k+1
        y_true_next  = C*Xi_a(:,k+1);
        y_meas_next  = y_true_next + v(k+1);
        Y_meas(k+1)  = y_meas_next;
        
        % corrección Kalman
        Xhi_a(:,k+1) = z_ai + L_kal*(y_meas_next - C*z_ai);
        
        Ui_a(k)      = ui_a;
    end
    
    % ====== OFFSET (si estás en AC) ======
    off    = 2.024;
    Xi_a   = Xi_a   + off;
    Xhi_a  = Xhi_a  + off;
    Ui_a   = Ui_a   + off;
    r_plot = r      + off;
    Y_meas = Y_meas + off;
    
    % ====== Gráfico resumen SOLO ACTUAL (con Kalman) ======
    plot_resumen_actual( ...
        t, Xi_a, Xhi_a, Ui_a, r_plot, Y_meas, ...
        C, Acl, Bcl, p_obs, Ts);
end


%% ====================== FUNCIÓN DE PLOT ======================
function fig = plot_resumen_actual( ...
    t, Xi_a, Xhi_a, Ui_a, r_plot, Y_meas, ...
    C, Acl, Bcl, p_obs, Ts)

N = numel(t);

% Igualar longitudes de u (si vino con N-1)
if size(Ui_a,2) == N-1
    Ui_a = [Ui_a Ui_a(:,end)];
end

% Salidas real y estimada
y_act    = C*Xi_a;
yhat_act = C*Xhi_a;

% Tr y OS del sistema aumentado en lazo cerrado (CONTROL, sin ruido)
sys_cl = ss(Acl, Bcl, [C 0], 0, Ts);   % [C 0] porque hay integrador
info   = stepinfo(sys_cl);
tr     = info.RiseTime*1000;   % ms
os     = info.Overshoot;       % %

% Polos de la dinámica cerrada (con integrador)
p_aug_cl = eig(Acl);

% Parámetro para dibujar menos marcadores en la estimación

% ---- Figura única ----
fig = figure('Name','Y, Ŷ, Y_{med}, U y Mapa de Polos','Position',[100 100 1100 800]);
tlo = tiledlayout(fig,3,1,'TileSpacing','compact','Padding','compact');

% ===== (1) y real vs ŷ vs y medida vs ref =====
nexttile(tlo);
plot(t, y_act,   'r',  'LineWidth',1.3); hold on; grid on; grid minor;
plot(t, Y_meas,  'ro', 'LineWidth',0.8);
plot(t, r_plot,  'k--','LineWidth',1.1);
xlabel('t [s]'); ylabel('Salida');
title(sprintf('Actual (Kalman) — t_r = %.3f ms, OS = %.1f%%', tr, os));
legend('y real','\hat{y}','y medida','r','Location','best');

% ===== (2) Esfuerzo u =====
nexttile(tlo);
stairs(t, Ui_a, 'r', 'LineWidth',1.3);hold on;
plot(t, r_plot,  'k--','LineWidth',1.1);
grid on; grid minor; xlabel('t [s]'); ylabel('u');
title('Esfuerzo de control');

% ===== (3) Mapa de polos =====
nexttile(tlo);
zgrid on; hold on; axis equal; grid on; box on;
theta = linspace(0,2*pi,400);
plot(cos(theta), sin(theta), 'k--','HandleVisibility','off'); % círculo unidad

plot(real(p_aug_cl), imag(p_aug_cl), 'm^', 'MarkerSize',8, 'LineWidth',1.2, ...
     'DisplayName','Polos cerrados (A_{cl})');
plot(real(p_obs),    imag(p_obs),    'bs', 'MarkerSize',8, 'LineWidth',1.4, ...
     'DisplayName','Polos observador rápido');
xlabel('Re\{z\}'); ylabel('Im\{z\}');
title('Mapa de polos');
legend('Location','bestoutside');

end
