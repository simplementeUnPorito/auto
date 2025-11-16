function plot_comparacion_estimadores( ...
    t, r_plot, ...
    X_kal, Xhat_kal, U_kal, y_true_kal, y_meas_kal, ...
    X_lu,  Xhat_lu,  U_lu,  y_true_lu,  y_meas_lu, ...
    C, Acl, Bcl, p_obs, p_kal, Ts, idx_case)

% Polos sistemas cerrados (control) – IGUAL para ambos estimadores
p_aug_cl = eig(Acl);

% Info de escalones (solo control)
sys_cl = ss(Acl,Bcl,[C 0],0,Ts);
info   = stepinfo(sys_cl);
tr     = info.RiseTime*1000;    % ms
os     = info.Overshoot;        % %

mk = 5;

fig = figure('Name',sprintf('Caso LQR %d — Kalman vs rápido',idx_case), ...
             'Position',[100 100 1200 800]);
tlo = tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');

% ===== (1) Kalman: y vs ŷ vs y_meas =====
nexttile(tlo);
plot(t,y_true_kal,'r','LineWidth',1.3); hold on; grid on; grid minor;
plot(t(1:mk:end), (C*Xhat_kal(:,1:mk:end)),'b*');
plot(t,y_meas_kal,'k:');
plot(t,r_plot,'k--');
xlabel('t [s]'); ylabel('Salida');
title(sprintf('Kalman — t_r = %.2f ms, OS = %.1f%%',tr,os));
legend('y real','\hat{y}','y medida','r','Location','best');

% ===== (2) Rápido: y vs ŷ vs y_meas =====
nexttile(tlo);
plot(t,y_true_lu,'r','LineWidth',1.3); hold on; grid on; grid minor;
plot(t(1:mk:end), (C*Xhat_lu(:,1:mk:end)),'b*');
plot(t,y_meas_lu,'k:');
plot(t,r_plot,'k--');
xlabel('t [s]'); ylabel('Salida');
title('Observador rápido (polos ×5)');
legend('y real','\hat{y}','y medida','r','Location','best');

% ===== (3) U Kalman =====
nexttile(tlo);
plot(t,U_kal,'LineWidth',1.3); grid on; grid minor;
xlabel('t [s]'); ylabel('u');
title('Esfuerzo de control — Kalman');

% ===== (4) U rápido =====
nexttile(tlo);
plot(t,U_lu,'LineWidth',1.3); grid on; grid minor;
xlabel('t [s]'); ylabel('u');
title('Esfuerzo de control — rápido');

% ===== (5) Mapa de polos =====
nexttile(tlo,[1 2]);
zgrid on; hold on; axis equal; grid on; box on;
theta = linspace(0,2*pi,400);
plot(cos(theta), sin(theta), 'k--','HandleVisibility','off');

% Polos de lazo cerrado (control + integrador)
plot(real(p_aug_cl), imag(p_aug_cl), 'm^', 'MarkerSize',8, 'LineWidth',1.2, ...
     'DisplayName','Polos cerrados (control)');

% Polos del observador rápido (Luenberger por acker)
plot(real(p_obs),    imag(p_obs),    'bs', 'MarkerSize',8, 'LineWidth',1.4, ...
     'DisplayName','Polos observador rápido');

% Polos del estimador de Kalman
plot(real(p_kal),    imag(p_kal),    'go', 'MarkerSize',8, 'LineWidth',1.4, ...
     'DisplayName','Polos Kalman');

xlabel('Re\{z\}'); ylabel('Im\{z\}');
title('Mapa de polos');
legend('Location','bestoutside');

end
