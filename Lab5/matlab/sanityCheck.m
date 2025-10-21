close all
clear all
% Verificar que las FT parciales salen exactas:
s = tf('s');
% === Parametros fisicos ===
C1 = 103.07e-9;  C2 = 211.1e-9;
R1 = 14.878e3;    R2 = 14.760e3;
R3 = 80.55e3;    R4 = 81.09e3;
tau1 = R2*C1;           k1 = -R2/R1;
tau2 = R4*C2;           k2 = -R4/R3;
F = [ -1/tau1,     0;
       k2/tau2, -1/tau2 ];
G = [ k1/tau1; 0 ];
H = [0 1];     % <- justo lo que querías
J = 0;

X1_R = (-R2/R1) / (tau1*s + 1);
X2_X1 = (-R4/R3) / (tau2*s + 1);

% Desde el ss:
[num1,den1] = tfdata(ss(F,[k1/tau1;0],[1 0],0),'v');  % X1/R
X1_R_ss = tf(num1,den1);

[num2,den2] = tfdata(ss([ -1/tau2 ], [k2/tau2], 1, 0),'v'); % X2/X1
X2_X1_ss = tf(num2,den2);

% Comparar (deberían ser iguales o numéricamente muy cercanos):
bode(X1_R, X1_R_ss); legend('X1/R (esperado)','X1/R (ss)');
figure; bode(X2_X1, X2_X1_ss); legend('X2/X1 (esperado)','X2/X1 (ss)');




X2_R   = X2_X1 * X1_R;          % cascada completa (verificación)

% Espacio de estados equivalente (x = [x1; x2], y = x2)
F = [ -1/tau1,    0;
       k2/tau2,  -1/tau2 ];
G = [ k1/tau1; 0 ];
H = [0 1]; 
J = 0;
sysC = ss(F,G,H,J);

%% === Excitación: escalón unitario en R ===
tf_end = 6*max(tau1,tau2);      % horizonte suficiente
t  = linspace(0, tf_end, 1500).';
r  = ones(size(t));             % escalón unitario

%% === Respuesta por espacio de estados (x1(t), x2(t)=y(t)) ===
[y_ss, t_ss, x_ss] = lsim(sysC, r, t);  % x_ss(:,1)=x1, x_ss(:,2)=x2

%% === Respuesta por T.F. en cascada como pediste ===
% 1) X1(t) respecto a R(t)
[x1_tf, t1] = step(X1_R, t);    % como R es escalón unitario, step sirve
% 2) Alimentar X1(t) como entrada de la segunda etapa para obtener X2(t)
x2_tf = lsim(X2_X1, x1_tf, t1);

% (opcional) chequeo directo de X2/R:
[y_tfdir, ~] = step(X2_R, t);   % debería coincidir con x2_tf

%% === Gráficas comparativas ===
figure('Name','Estados vs TF en cascada','Color','w');

% x1
subplot(3,1,1); hold on; grid on;
plot(t_ss, x_ss(:,1),'LineWidth',1.6);
plot(t1,   x1_tf,'--','LineWidth',1.4);
ylabel('x_1(t)');
title('x_1: Estado vs. TF X1/R');
legend('x_1 (estado-espacio)','x_1 (TF X1/R)','Location','best');

% x2
subplot(3,1,2); hold on; grid on;
plot(t_ss, x_ss(:,2),'LineWidth',1.6);
plot(t1,   x2_tf,'--','LineWidth',1.4);
ylabel('x_2(t)');
title('x_2: Estado vs. TF en cascada (X1/R → X2/X1)');
legend('x_2 (estado-espacio)','x_2 (TF cascada)','Location','best');

% y = x2, y verificación X2/R directo
subplot(3,1,3); hold on; grid on;
plot(t_ss, y_ss,'LineWidth',1.6);
plot(t,    y_tfdir,'--','LineWidth',1.4);
xlabel('Tiempo [s]'); ylabel('y(t)=x_2(t)');
title('Salida: Estado vs. TF directa (X2/R)');
legend('y (estado-espacio)','y (TF X2/R directa)','Location','best');

%% === Sanity checks cortos en consola ===
fprintf('Ganancia DC esperada X1/R = k1 = %.4f\n', k1);
fprintf('Ganancia DC esperada X2/X1 = k2 = %.4f\n', k2);
fprintf('Ganancia DC esperada X2/R = k1*k2 = %.4f\n', k1*k2);
