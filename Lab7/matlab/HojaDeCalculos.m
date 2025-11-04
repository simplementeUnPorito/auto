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

% Ts vía polo más rápido
fn = max(abs(zpk(Gc).P{1}))/pi;
Tn = 1/fn;
Ts = Tn/4;

sysD = c2d(sysC, Ts, 'zoh');
[A,B,C,D] = ssdata(sysD);

%% --- Chequeo
n = size(A,1);

%% --- Polos objetivo en z ---
p_ctrl = [0.8 + 1j*0.2; 0.8 - 1j*0.2];   % CONTROL
K = acker(A,B,p_ctrl);

% Prefiltro para el caso sin integrador
[~,~,Nbar] = refi(A,B,C,K);

%% --- Observadores (mismo set rápido) ---
p_obs = [0.2+1j*0.2  0.2-1j*0.2];
L_pred   = acker(A', C',        p_obs).';   % estimador de PREDICCIÓN
L_actual = acker(A',(C*A)',     p_obs).';   % estimador ACTUAL

%% --- Integrador Ogata 6.19 con tus fórmulas ---
m = 1;
Ahat = [A B; zeros(m,n+m)];
Bhat = [zeros(n,m); eye(m)];
Chat = [C zeros(1,m)];

polos_i = [p_ctrl.' 0.6];    % mismos 2 + uno real
Khat = acker(Ahat,Bhat,polos_i);

Aux  = [A-eye(size(A))  B;
        C*A             C*B];
K2K1 = (Khat + [zeros(1,n) eye(m)]) / Aux;
K2   = K2K1(1,1:n);          % sobre x
K1   = K2K1(1,n+1:end);      % sobre integrador

% observadores para el caso con integrador
p_obs_int = [0.2+1j*0.2 0.2-1j*0.2];
L_pred_i   = acker(A', C',        p_obs_int).';
L_actual_i = acker(A',(C*A)',     p_obs_int).';

%% =================== SIMULACIÓN ===================
N  = 200;
t  = 0:Ts:(N-1)*Ts;
r  = ones(1,N);      % escalón 1
r(1:30) = 0;         % arranca en 0

% ruido en medición
w  = [zeros(1,75),0.01*ones(1,125)];    % ajustá el nivel

% ----- 1) PREDICTIVO SIN integrador -----
X_p   = zeros(n,N);
Xh_p  = zeros(n,N);
U_p   = zeros(1,N);

% ----- 2) ACTUAL SIN integrador -----
X_a   = zeros(n,N);
Xh_a  = zeros(n,N);
U_a   = zeros(1,N);

% ----- 3) PREDICTIVO CON integrador -----
Xi_p   = zeros(n,N);
Xhi_p  = zeros(n,N);
Vi_p   = zeros(1,N);
Ui_p   = zeros(1,N);

% ----- 4) ACTUAL CON integrador -----
Xi_a   = zeros(n,N);
Xhi_a  = zeros(n,N);
Vi_a   = zeros(1,N);
Ui_a   = zeros(1,N);

for k = 1:N-1
    rk = r(k);

    %% ===== 1) PREDICTIVO (sin int) =====
    y_p = C*X_p(:,k);
    u_p = Nbar*rk - K*Xh_p(:,k);
    X_p(:,k+1)  = A*X_p(:,k)  + B*u_p+ w(k);
    Xh_p(:,k+1) = A*Xh_p(:,k) + B*u_p + L_pred*(y_p - C*Xh_p(:,k));
    U_p(k)      = u_p;

    %% ===== 2) ACTUAL (sin int) =====
    u_a = Nbar*rk - K*Xh_a(:,k);
    X_a(:,k+1) = A*X_a(:,k) + B*u_a+ w(k);
    y_a        = C*X_a(:,k+1) ;          % medición en k+1
    z_a        = A*Xh_a(:,k) + B*u_a;
    Xh_a(:,k+1)= z_a + L_actual*(y_a - C*z_a);
    U_a(k)     = u_a;

    %% ===== 3) PREDICTIVO + integrador =====
    yi_p          = C*Xi_p(:,k) ;
    Vi_p(k+1)     = Vi_p(k) + (rk - yi_p);       % integrador real
    ui_p          = K1*Vi_p(k+1) - K2*Xhi_p(:,k);
    Xi_p(:,k+1)   = A*Xi_p(:,k) + B*ui_p+ w(k);
    Xhi_p(:,k+1)  = A*Xhi_p(:,k) + B*ui_p + L_pred_i*(yi_p - C*Xhi_p(:,k));
    Ui_p(k)       = ui_p;

    %% ===== 4) ACTUAL + integrador =====
    yi_a          = C*Xi_a(:,k);
    Vi_a(k+1)     = Vi_a(k) + (rk - yi_a);
    ui_a          = K1*Vi_a(k+1) - K2*Xhi_a(:,k);
    Xi_a(:,k+1)   = A*Xi_a(:,k) + B*ui_a + w(k);
    yia_next      = C*Xi_a(:,k+1) + w(k);        % misma muestra de ruido para comparar
    z_ai          = A*Xhi_a(:,k) + B*ui_a;
    Xhi_a(:,k+1)  = z_ai + L_actual_i*(yia_next - C*z_ai);
    Ui_a(k)       = ui_a;
end

%% ====== OFFSET AC ======
off = 2.024;
X_p   = X_p   + off;   Xh_p  = Xh_p  + off;   U_p  = U_p  + off;
X_a   = X_a   + off;   Xh_a  = Xh_a  + off;   U_a  = U_a  + off;
Xi_p  = Xi_p  + off;   Xhi_p = Xhi_p + off;   Ui_p = Ui_p + off;
Xi_a  = Xi_a  + off;   Xhi_a = Xhi_a + off;   Ui_a = Ui_a + off;
r_plot = r + off;

%% =================== GRAFICAS ===================

% 1) PREDICTIVO: sin vs con integrador
figure;
subplot(3,1,1);
plot(t, X_p(1,:), 'b', t, Xi_p(1,:), 'r'); grid on;
ylabel('x_1');
legend('pred','pred+int');
title('Estimador predictivo — sin vs con integrador'); grid minor;

subplot(3,1,2);
plot(t, X_p(2,:), 'b', t, Xi_p(2,:), 'r', t, r_plot, 'k--'); grid on;
ylabel('x_2 / y'); grid minor;

subplot(3,1,3);
plot(t(1:end-1), U_p(1:end-1), 'b', t(1:end-1), Ui_p(1:end-1), 'r'); grid on;
ylabel('u'); xlabel('t'); grid minor;

% 2) ACTUAL: sin vs con integrador
figure;
subplot(3,1,1);
plot(t, X_a(1,:), 'b', t, Xi_a(1,:), 'r'); grid on;
ylabel('x_1'); grid minor;
legend('act','act+int');
title('Estimador actual — sin vs con integrador');

subplot(3,1,2);
plot(t, X_a(2,:), 'b', t, Xi_a(2,:), 'r', t, r_plot, 'k--'); grid on; grid minor;
ylabel('x_2 / y');

subplot(3,1,3);
plot(t(1:end-1), U_a(1:end-1), 'b', t(1:end-1), Ui_a(1:end-1), 'r'); grid on; grid minor;
ylabel('u'); xlabel('t');

% 3) esfuerzos todos
figure;
plot(t(1:end-1), U_p(1:end-1),  'b', ...
     t(1:end-1), Ui_p(1:end-1), '--b', ...
     t(1:end-1), U_a(1:end-1),  'r', ...
     t(1:end-1), Ui_a(1:end-1), '--r'); grid on;
legend('pred','pred+int','act','act+int','Location','Best');
title('Esfuerzos de control');
xlabel('t'); ylabel('u');  grid minor;

%% ========== MAPA DE POLOS RESUMIDO (3 GRAFICOS GRANDES) ==========
figure('Position',[100 100 1000 900]); % figura grande

% === 1) Sistema sin integrador ===
subplot(1,3,1);
zgrid; hold on; 
%axis equal; axis([-1.2 1.2 -1.2 1.2]);

p_planta = eig(A);
p_cl     = eig(A - B*K);

plot(real(p_planta), imag(p_planta), 'ko', 'MarkerSize',8, 'LineWidth',1.4);
plot(real(p_cl),     imag(p_cl),     'rx', 'MarkerSize',10, 'LineWidth',1.6);

title('Sistema sin integrador');
%legend({'Planta A','Lazo cerrado A - BK'},'Location','best');
xlabel('Re\{z\}'); ylabel('Im\{z\}');
grid on; box on;

% === 2) Sistema con integrador ===
subplot(1,3,2);
zgrid; hold on;
%axis equal; axis([-1.2 1.2 -1.2 1.2]);

p_aug    = eig(Ahat);
p_aug_cl = eig(Ahat - Bhat*Khat);

plot(real(p_aug),    imag(p_aug),    'm^', 'MarkerSize',8, 'LineWidth',1.4);
plot(real(p_aug_cl), imag(p_aug_cl), 'c*', 'MarkerSize',10, 'LineWidth',1.6);

title('Sistema con integrador (Ahat y Ahat - BhatKhat)');
%legend({'Aumentado Ahat','Aum. cerrado Ahat - BhatKhat'},'Location','best');
xlabel('Re\{z\}'); ylabel('Im\{z\}');
grid on; box on;

% === 3) Estimadores (uno solo, mismo polos para ambos) ===
subplot(1,3,3);
zgrid; hold on;
%axis equal; axis([-1.2 1.2 -1.2 1.2]);

p_obs = eig(A - L_pred*C);   % mismo para predictivo y actual

plot(real(p_obs), imag(p_obs), 'bs', 'MarkerSize',9, 'LineWidth',1.5);

title('Estimadores de estado (p_{obs})');
%legend({'A - L·C'},'Location','best');
xlabel('Re\{z\}'); ylabel('Im\{z\}');
grid on; box on;

sgtitle('Mapa de polos — Sistemas y estimadores');
