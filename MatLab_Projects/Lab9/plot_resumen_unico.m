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