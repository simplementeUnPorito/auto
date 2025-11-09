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
Ts = 1/1e3;

sysD = c2d(sysC, Ts, 'zoh');
[A,B,C,D] = ssdata(sysD);

%% Anhadimos el integrador 
n = size(A,1);
m = 1;
Ahat = [A B; zeros(m,n+m)];
Bhat = [zeros(n,m); eye(m)];
Chat = [C zeros(1,m)];
sysDI = ss(Ahat,Bhat,Chat,D);
%%
Q1 = {diag([0.01 0.01 0.001]), diag([0.01 0.01 0.1]), diag([0.01 0.01 0.001])};
Q2 = {1, 0.1, 0.01};


%% ... (tu código de arriba igual hasta definir Ahat,Bhat,Chat, Q1, Q2) ...

K = cell(size(Q2)); K1 = K; K2 = K; P = K;

N  = 200;
t  = 0:Ts:(N-1)*Ts;
r  = ones(1,N);      r(1:30) = 0;

% ruido de medición (no lo metas al estado)
w  = [zeros(1,150), 0.01*ones(1,50)];

for i = 1:length(Q2)
    % ========= LQR DISCRETO sobre el sistema AUMENTADO =========
    [Ki,~,Pi] = dlqr(Ahat, Bhat, Q1{i}, Q2{i})  % <- dlqr y matrices, no sys
    
    K{i} = Ki; P{i} = Pi;
    K2{i} = Ki(1,1:n);          % sobre x
    K1{i} = Ki(1,n+1:end);      % sobre integrador (z)
    
    Acl = [A B;(K2{i}-K2{i}*A-K1{i}*C*A) (eye(m)-K2{i}*B-K1{i}*C*B)];
    Bcl = [0;0;K1{i}];
    p_obs = [0.2+1i*0.2, 0.2-1i*0.2];
    L_pred_i   = acker(A', C', p_obs).';  % predictivo
    L_actual_i = acker(A', (C*A)', p_obs).'; % actual 

    %% =================== SIMULACIÓN ===================
    Xi_p   = zeros(n,N);  Xhi_p  = zeros(n,N); Xhi_p(:,1) =[0.1;0.1]; 
    Vi_p   = zeros(1,N);  Ui_p   = zeros(1,N);

    Xi_a   = zeros(n,N);  Xhi_a  = zeros(n,N); Xhi_a(:,1) =[0.1;0.1]; 
    Vi_a   = zeros(1,N);  Ui_a   = zeros(1,N);
    
    for k = 1:N-1
        rk = r(k);

        % ===== 1) PREDICTIVO (con integrador) =====
        yi_p          = C*Xi_p(:,k);       % ruido SOLO en medición
        Vi_p(k+1)     = Vi_p(k) + (rk - yi_p);    % integrador de error
        ui_p          = K1{i}*Vi_p(k+1) - K2{i}*Xhi_p(:,k);   % <-- usar {i}
        Xi_p(:,k+1)   = A*Xi_p(:,k) + B*ui_p;
        Xhi_p(:,k+1)  = A*Xhi_p(:,k) + B*ui_p + L_pred_i*(yi_p - C*Xhi_p(:,k));
        Ui_p(k)       = ui_p;

        % ===== 2) ACTUAL (con integrador) =====
        yi_a          = C*Xi_a(:,k);
        Vi_a(k+1)     = Vi_a(k) + (rk - yi_a);
        ui_a          = K1{i}*Vi_a(k+1) - K2{i}*Xhi_a(:,k);   % <-- usar {i}
        Xi_a(:,k+1)   = A*Xi_a(:,k) + B*ui_a;
        % actualización "actual": usa y(k+1)
        z_ai          = A*Xhi_a(:,k) + B*ui_a;
        yia_next      = C*Xi_a(:,k+1);
        Xhi_a(:,k+1)  = z_ai + L_actual_i*(yia_next - C*z_ai);
        Ui_a(k)       = ui_a;
    end

    % ====== OFFSET (si estás en AC) ======
    off = 2.024;
    Xi_p  = Xi_p + off;   Xhi_p = Xhi_p + off;   Ui_p = Ui_p + off;
    Xi_a  = Xi_a + off;   Xhi_a = Xhi_a + off;   Ui_a = Ui_a + off;
    r_plot = r + off;

    % ====== Graficar en UNA figura (estados y, esfuerzos, polos) ======
    % Nota: como NO simulaste "sin integrador", no pasamos X_p/X_a.
    % Para polos sin integrador, si no tenés K_noInt, se omite.
    K_noInt = [];           % si luego querés, llená esto con dlqr(A,B,Q,R)
    K_aug   = K{i};         % K del sistema aumentado actual
    plot_resumen_unico( ...
    t, Xi_p, Xhi_p, Xi_a, Xhi_a, Ui_p, Ui_a, r_plot, ...
    A, B, Ahat, Bhat, [], K{i}, L_pred_i, C,Acl,Bcl,Ts);

end

function fig = plot_resumen_unico( ...
    t, Xi_p, Xhi_p, Xi_a, Xhi_a, Ui_p, Ui_a, r_plot, ...
    A, B, Ahat, Bhat, K_noInt, K_aug, L_pred, C,Acl,Bcl,Ts)

N = numel(t);
% Igualar longitudes de u (si vinieron con N-1)
if size(Ui_p,2)==N-1, Ui_p=[Ui_p Ui_p(:,end)]; end
if size(Ui_a,2)==N-1, Ui_a=[Ui_a Ui_a(:,end)]; end

% Salidas real y estimada
y_pred   = C*Xi_p;          % 1xN
yhat_pred= C*Xhi_p;         % 1xN
y_act    = C*Xi_a;
yhat_act = C*Xhi_a;

% Polos

p_obs    = eig(A - L_pred*C);
tr = stepinfo(ss(Acl,Bcl,[C 0],0,Ts)).RiseTime*1000;
os = stepinfo(ss(Acl,Bcl,[C 0],0,Ts)).Overshoot;

p_aug_cl = eig(Acl);

% ---- Figura única 2x2 ----
fig = figure('Name','Y & Ŷ, U y Mapa de Polos','Position',[100 100 1100 800]);
tlo = tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');

% ===== (1) Estimador PREDICTIVO: y vs ŷ vs ref =====
nexttile(tlo);
plot(t, y_pred, 'b', 'LineWidth',1.3); hold on; grid on; grid minor;
mk = max(1, floor(N/80));              % paso de marcadores para no ensuciar
plot(t(1:mk:end), yhat_pred(1:mk:end), '*', 'Color',[0 0.45 0.74]); % azul
plot(t, r_plot, 'k--', 'LineWidth',1.1);
xlabel('t [s]'); ylabel('Salida'); 
title(sprintf('Predictivo — OS = %.3f%%', os));

% ===== (2) Estimador ACTUAL: y vs ŷ vs ref =====
nexttile(tlo);
plot(t, y_act, 'r', 'LineWidth',1.3); hold on; grid on; grid minor;
plot(t(1:mk:end), yhat_act(1:mk:end), '*', 'Color',[0.85 0.33 0.10]); % rojo
plot(t, r_plot, 'k--', 'LineWidth',1.1);
xlabel('t [s]'); ylabel('Salida'); 
title(sprintf('Actual — ts = %.3f ms', tr));

% ===== (3) Esfuerzos u =====
nexttile(tlo);
plot(t, Ui_p, 'b', 'LineWidth',1.3); hold on;
plot(t, Ui_a, 'r', 'LineWidth',1.3);
grid on; grid minor; xlabel('t [s]'); ylabel('u');
title('Esfuerzo de control');

% ===== (4) Mapa de polos con zgrid =====
nexttile(tlo);
zgrid on; hold on; axis equal; grid on; box on;
theta = linspace(0,2*pi,400);
plot(cos(theta), sin(theta), 'k--','HandleVisibility','off'); % círculo unidad

plot(real(p_aug_cl),    imag(p_aug_cl),    'm^', 'MarkerSize',8, 'LineWidth',1.2, 'DisplayName','\hat{A}');
plot(real(p_obs),    imag(p_obs),    'bs', 'MarkerSize',8, 'LineWidth',1.4, 'DisplayName','A-LC');
xlabel('Re\{z\}'); ylabel('Im\{z\}');
title('Mapa de polos');

end
