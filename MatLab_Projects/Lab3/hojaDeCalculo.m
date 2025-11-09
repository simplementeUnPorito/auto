close all
clear all
close all
clear all

addpath('..\Lab1\')
addpath('..\Lab2\')

%% Definicion de parametros
R_1 = 15e3;
R_3 = 15e3;
C_2 = 100e-9;

R_2 = 82e3;
R_4 = 82e3;
C_1 = 0.22e-6;

%% Generar funcion de transferencia d
numStage = [-R_3/R_1 -R_4/R_2];
denStage = { [C_2*R_3 1], [C_1*R_4 1] };

% Usamos celdas para guardar los tf de cada stage
Gstage = cell(1,2);
G = 1;
for i = 1:2
    Gstage{i} = tf(numStage(i), denStage{i});
    G = G*Gstage{i};
end

%% Analizamos en el tiempo
[tr, ts, wn] = plot_step_info(G);
disp('Planta continua G(s):')
G
zpk(G)



%% Paso 1: Definir periodo de muestreo
% Se busca una mejora de 4 en el tiempo de rising , o sea tr = 10 ms
N = 4; 
T = tr/(8*N);
%Gcl = feedback(G,1);
%figure;
%step(Gcl)
%T = stepinfo(Gcl).SettlingTime/8;

%% Paso 2: Digitalizar con ZOH
Gd = c2d(G, T, 'zoh');
disp('Planta digital G(z):')
Gd
zpk(Gd)

%% === K para PM deseada ===
Gw = d2c(Gd,'tustin');        % continuo equivalente por Tustin
figure;
margin(Gw);grid on;
%% === LOOP INTERACTIVO: ajustar K, agregar LEAD/LAG, o auto-ajustar specs ===
% Requiere tener Gw (continuo por Tustin) ya definido.
s = tf('s');
C_total = tf(1);
K = 1;

fprintf('\n=== Estado inicial (K=1, C=1) ===\n');
L_now = K*C_total*Gw;
[GM, PM, Wcg, Wcp] = margin(L_now);
fprintf('GM = %.2f dB, PM = %.2f°, Wcg = %.3g rad/s, Wcp = %.3g rad/s\n', 20*log10(GM), PM, Wcg, Wcp);
figure; margin(L_now); grid on; title('L(s) = K*C*G_w  (inicio)');

seguir = true;
while seguir
    fprintf('\nOpciones:\n');
    fprintf('  [1] Ajustar K a un GM deseado (dB)\n');
    fprintf('  [2] Agregar etapa LEAD/LAG (normalizada en Wcp)\n');
    fprintf('  [3] Auto-ajustar a (GM_dB, PM)  — o usar K fijo y PM\n');
    fprintf('  [4] Fijar K explícito (sin cálculos)\n');
    fprintf('  [0] Terminar\n');
    op = str2double(input('Elegí opción: ','s'));

    switch op
        case 1
            GM_des_dB = str2double(input(' GM deseado (dB): ','s'));
            L_now = K*C_total*Gw;
            [GM_now, ~, ~, ~] = margin(L_now);
            GM_now_dB = 20*log10(GM_now);
            if ~isfinite(GM_now_dB)
                error('GM no definido; agregá una etapa o cambiá el rango antes de ajustar K.');
            end
            % K_new saca GM a GM_des_dB (aprox directo en dB)
            K = K * 10^((GM_now_dB - GM_des_dB)/20);
            L_now = K*C_total*Gw;
            [GM, PM, Wcg, Wcp] = margin(L_now);
            fprintf('  -> K = %.6g; GM = %.2f dB, PM = %.2f°, Wcg = %.3g, Wcp = %.3g\n', ...
                    K, 20*log10(GM), PM, Wcg, Wcp);
            figure; margin(L_now); grid on; title('Tras ajustar K');

        case 2
            tipo = lower(strtrim(input(' Tipo ("lead" o "lag"): ','s')));
            phi  = str2double(input(' Fase a compensar (grados, positivo): ','s'));
            [C_total, rep] = add_stage_at_Wcp(Gw, C_total, K, tipo, phi, s);
            fprintf('  -> Etapa %s %+g° aplicada en wc=%.3g rad/s (normalizada)\n', rep.type, rep.phi_deg, rep.wc_used);
            L_now = K*C_total*Gw;
            [GM, PM, Wcg, Wcp] = margin(L_now);
            fprintf('  -> GM = %.2f dB, PM = %.2f°, Wcg = %.3g, Wcp = %.3g\n', 20*log10(GM), PM, Wcg, Wcp);
            figure; margin(L_now); grid on; title('Tras agregar etapa');

        case 3
            usa_k_fijo = lower(strtrim(input(' ¿Usar K fijo? [y/n]: ','s')));
            if strcmpi(usa_k_fijo,'y')
                K_fixed = str2double(input(' Valor de K fijo: ','s'));
                PM_des  = str2double(input(' PM deseada (grados): ','s'));
                [K, C_total, rep] = auto_hit_specs_fixedK(Gw, C_total, K_fixed, PM_des, s);
                fprintf('  -> AUTO (K fijo=%.6g) listo: PM=%.2f°, GM=%.2f dB, Wcg=%.3g, Wcp=%.3g\n', ...
                        K, rep.PM, rep.GMdB, rep.Wcg, rep.Wcp);
            else
                GM_des_dB = str2double(input(' GM deseado (dB): ','s'));
                PM_des    = str2double(input(' PM deseada (grados): ','s'));
                [K, C_total, rep] = auto_hit_specs(Gw, C_total, K, GM_des_dB, PM_des, s);
                fprintf('  -> AUTO listo (iters=%d): GM=%.2f dB, PM=%.2f°, Wcg=%.3g, Wcp=%.3g\n', ...
                        rep.iters, rep.GMdB, rep.PM, rep.Wcg, rep.Wcp);
            end
            L_now = K*C_total*Gw;
            figure; margin(L_now); grid on; title('Tras auto-ajuste');

        case 4
            K = str2double(input(' K = ','s'));
            L_now = K*C_total*Gw;
            [GM, PM, Wcg, Wcp] = margin(L_now);
            fprintf('  -> K fijado. GM = %.2f dB, PM = %.2f°, Wcg = %.3g, Wcp = %.3g\n', 20*log10(GM), PM, Wcg, Wcp);
            figure; margin(L_now); grid on; title('Tras fijar K');

        case 0
            seguir = false;
            continue;

        otherwise
            fprintf('Opción inválida.\n');
    end
end


%% Resultado final
 K = 7;
C_total = K;
% C_total = 11.7057*d2c(C1,'tustin',T);
L_final = C_total*Gw;
[GMf, PMf, Wcgf, Wcpf] = margin(L_final);
fprintf('Márgenes: GM = %.2f dB, PM = %.2f°, Wcg = %.3g, Wcp = %.3g\n', 20*log10(GMf), PMf, Wcgf, Wcpf);
% % figure; bode(C_total); grid on;
% figure; margin(L_final); grid on; title('L_{final}(s) = K · C_{total}(s) · G_w(s)');
% % C1 = c2d(C_total,T,'tustin');
%  figure;step(feedback(C1*Gd,1));grid on;title('Respuesta al escalon');
% figure;step(feedback(C1,Gd));grid on;title('Esfuerzo al escalon');



umin = 0;
umax =  4.08;
refmin = 1.75;
refmax = 2.25;
n_per_seg = 500;

Gc1f = feedback(Gd*C1,1);
%figure;
%step(Gc1f)
info = stepinfo(Gc1f);


info_ext = plot_step_annot(Gc1f, 'la planta compensada con un lag-filter de 1º orden, anulando el polo más rápido de la planta.');
compensador = zpk(C1);

% [td, refd, yd, ud, ed, coefs] = sim_compensador_first_order( ...
%     Gd, compensador.Z{1}, compensador.P{1}, compensador.K, T, umin, umax, refmin, refmax, n_per_seg);

[td, refd, yd, ud, ed, coefs] = sim_compensador_first_order( ...
    Gd, 0, 0, K, T, umin, umax, refmin, refmax, n_per_seg);

% [td, refd, yd, ud, ed, coefs] = sim_compensador_second_order( ...
%     Gd, compensador.Z{1}(1), ...
%     compensador.Z{1}(2), ...
%     compensador.P{1}(1), ...
%     compensador.P{1}(2), compensador.K, ...
%     T, umin, umax, refmin, refmax, n_per_seg);

Nini = 100;              % descartar al inicio
Nfin = 100;              % descartar al final
idx0 = Nini + 1;         % índice inicial válido
idx1 = length(td) - Nfin; % índice final válido

td   = td(idx0:idx1)-td(idx0);
refd = refd(idx0:idx1);
yd   = yd(idx0:idx1);
ud   = ud(idx0:idx1);
ed   = ed(idx0:idx1);

% Si querés que el tiempo arranque en 0





figure;

% ---------- Subplot 1 ----------
axAbs = subplot(2,1,1);  % eje izquierdo (absoluto)
hold(axAbs,'on'); grid(axAbs,'on');

% y[k] en rojo (abs)
hY = stairs(axAbs, td, yd, 'r-', 'LineWidth', 1.4);

% Armamos eje derecho transparente
axPct = axes('Position', get(axAbs,'Position'), ...
             'Color','none', 'YAxisLocation','right', ...
             'XLim', get(axAbs,'XLim'), 'XTick',[], 'Box','off');
hold(axPct,'on');

% r[k] en negro, graficado en %
ref_pct = 100*(refd - refmin)/(refmax - refmin);
hR = stairs(axPct, td, refd, 'k--', 'LineWidth', 1.2);

% ======= Cálculo de límites con margen =======
yd_pct  = 100*(yd - refmin)/(refmax - refmin);
pctAll  = [yd_pct; ref_pct];

% Valores extremos en % con margen del 5 %
rawMin = min(pctAll);
rawMax = max(pctAll);
span   = rawMax - rawMin;
pctMin = rawMin - 0.05*span;
pctMax = rawMax + 0.05*span;

% Redondeamos a múltiplos de 20 para ticks
stepPct = 20;
tickMin = stepPct*floor(pctMin/stepPct);
tickMax = stepPct*ceil (pctMax/stepPct);
pctTicks = tickMin:stepPct:tickMax;

% Convertimos a absolutos
valTicks = refmin + (pctTicks/100)*(refmax - refmin);

% Aplicamos a ambos ejes
set(axAbs,'YLim',[valTicks(1) valTicks(end)],'YTick',valTicks);
set(axPct,'YLim',[valTicks(1) valTicks(end)],'YTick',valTicks,...
          'YTickLabel',compose('%.0f %%',pctTicks));

% Etiquetas
xlabel(axAbs,'Tiempo [s]');
ylabel(axAbs,'Respuesta [valor absoluto]');
ylabel(axPct,'Escala relativa a ref [%]');
title(axAbs,'Respuesta discreta con compensador 1º orden, anulando el polo más rápido de la planta.');
legend(axAbs, [hY, hR], {'y[k]', 'r[k]'}, 'Location', 'best');

% ---------- Subplot 2: esfuerzo de control ----------
axU = subplot(2,1,2);
stairs(axU, td, ud, 'LineWidth',1.2); grid(axU,'on');
yline(axU, umax,'r:'); yline(axU, umin,'r:');
xlabel(axU,'Tiempo [s]'); ylabel(axU,'u[k]');
title(axU,'Esfuerzo de control con saturación');

% --- Margen de 5% en eje Y ---
uMin = min(ud);
uMax = max(ud);
span = uMax - uMin;
ylim(axU, [uMin - 0.05*span, uMax + 0.05*span]);


grid(axAbs,'on');    % grilla principal
grid(axAbs,'minor'); % grilla secundaria

grid(axPct,'on');
grid(axPct,'minor');

grid(axU,'on');
grid(axU,'minor');
%% ================== SETUP INICIAL ==================
% Asumimos que ya construiste G (continuo), calculaste T, y obtuviste:
%   Gd = c2d(G, T, 'zoh');
%   Gw = d2c(Gd, 'tustin');   % "plano w" para diseñar
%
% Si no, descomenta y ajustá a tu caso:
% R_1 = 15e3; R_3 = 15e3; C_2 = 100e-9; R_2 = 82e3; R_4 = 82e3; C_1 = 0.22e-6;
% numStage = [-R_3/R_1 -R_4/R_2];
% denStage = { [C_2*R_3 1], [C_1*R_4 1] };
% Gstage = cell(1,2); G = 1;
% for i = 1:2, Gstage{i} = tf(numStage(i), denStage{i}); G = G*Gstage{i}; end
% [tr, ts, wn] = plot_step_info(G); N = 4; T = tr/(8*N);
% Gd = c2d(G, T, 'zoh'); Gw = d2c(Gd, 'tustin');

assert(exist('Gw','var')==1 && exist('Gd','var')==1 && exist('T','var')==1, ...
    'Definí primero Gw, Gd y T antes de correr este script.');

s = tf('s');

%% ================== COMPENSADOR INICIAL ==================
% Tu elección: integrador por defecto (podés cambiar a tf(1) si no querés I)
C_total = 1/s;     % <-- SI NO QUERÉS integrador, poné: C_total = tf(1);
K = 1;

%% ================== ESTADO INICIAL (CONTINUO, PLANO w) ==================
fprintf('\n=== Estado inicial (K=1, C inicial) ===\n');
L_now = K*C_total*Gw;
[GM, PM, Wcg, Wcp] = margin(L_now);
fprintf('GM = %.2f dB, PM = %.2f°, Wcg = %.3g rad/s, Wcp = %.3g rad/s\n', 20*log10(GM), PM, Wcg, Wcp);
figure; margin(L_now); grid on; title('L(s) = K*C(s)*G_w(s)  (inicio)');

%% ================== LOOP INTERACTIVO ==================
seguir = true;
while seguir
    fprintf('\nOpciones:\n');
    fprintf('  [1] Ajustar K a un GM deseado (dB)\n');
    fprintf('  [2] Agregar etapa LEAD/LAG (normalizada en Wcp)\n');
    fprintf('  [3] Auto-ajustar a (GM_dB, PM)  — o usar K fijo y PM\n');
    fprintf('  [4] Fijar K explícito (sin cálculos)\n');
    fprintf('  [5] Ir a DISCRETO (c2d con prewarp) y simular\n');
    fprintf('  [0] Terminar\n');
    op = str2double(input('Elegí opción: ','s'));

    switch op
        case 1  % Ajustar K a GM deseado
            GM_des_dB = str2double(input(' GM deseado (dB): ','s'));
            L_now = K*C_total*Gw;
            [GM_now, ~, ~, ~] = margin(L_now);
            GM_now_dB = 20*log10(GM_now);
            if ~isfinite(GM_now_dB)
                error('GM no definido; agregá una etapa o cambiá el rango antes de ajustar K.');
            end
            K = K * 10^((GM_now_dB - GM_des_dB)/20);
            L_now = K*C_total*Gw;
            [GM, PM, Wcg, Wcp] = margin(L_now);
            fprintf('  -> K = %.6g; GM = %.2f dB, PM = %.2f°, Wcg = %.3g, Wcp = %.3g\n', ...
                    K, 20*log10(GM), PM, Wcg, Wcp);
            figure; margin(L_now); grid on; title('Tras ajustar K');

        case 2  % Agregar etapa lead/lag normalizada en Wcp
            tipo = lower(strtrim(input(' Tipo ("lead" o "lag"): ','s')));
            phi  = str2double(input(' Fase a compensar (grados, positivo): ','s'));
            [C_total, rep] = add_stage_at_Wcp(Gw, C_total, K, tipo, phi, s);
            fprintf('  -> Etapa %s %+g° aplicada en wc=%.3g rad/s (normalizada)\n', rep.type, rep.phi_deg, rep.wc_used);
            L_now = K*C_total*Gw;
            [GM, PM, Wcg, Wcp] = margin(L_now);
            fprintf('  -> GM = %.2f dB, PM = %.2f°, Wcg = %.3g, Wcp = %.3g\n', 20*log10(GM), PM, Wcg, Wcp);
            figure; margin(L_now); grid on; title('Tras agregar etapa');

        case 3  % Auto-ajuste
            usa_k_fijo = lower(strtrim(input(' ¿Usar K fijo? [y/n]: ','s')));
            if strcmpi(usa_k_fijo,'y')
                K_fixed = str2double(input(' Valor de K fijo: ','s'));
                PM_des  = str2double(input(' PM deseada (grados): ','s'));
                [K, C_total, rep] = auto_hit_specs_fixedK(Gw, C_total, K_fixed, PM_des, s);
                fprintf('  -> AUTO (K fijo=%.6g) listo: PM=%.2f°, GM=%.2f dB, Wcg=%.3g, Wcp=%.3g\n', ...
                        K, rep.PM, rep.GMdB, rep.Wcg, rep.Wcp);
            else
                GM_des_dB = str2double(input(' GM deseado (dB): ','s'));
                PM_des    = str2double(input(' PM deseada (grados): ','s'));
                [K, C_total, rep] = auto_hit_specs(Gw, C_total, K, GM_des_dB, PM_des, s);
                fprintf('  -> AUTO listo (iters=%d): GM=%.2f dB, PM=%.2f°, Wcg=%.3g, Wcp=%.3g\n', ...
                        rep.iters, rep.GMdB, rep.PM, rep.Wcg, rep.Wcp);
            end
            L_now = K*C_total*Gw;
            figure; margin(L_now); grid on; title('Tras auto-ajuste');

        case 4  % Fijar K
            K = str2double(input(' K = ','s'));
            L_now = K*C_total*Gw;
            [GM, PM, Wcg, Wcp] = margin(L_now);
            fprintf('  -> K fijado. GM = %.2f dB, PM = %.2f°, Wcg = %.3g, Wcp = %.3g\n', 20*log10(GM), PM, Wcg, Wcp);
            figure; margin(L_now); grid on; title('Tras fijar K');

        case 5  % Discretizar C(s) -> C(z) con prewarp y simular
            Lc = K*C_total*Gw;
            [~, ~, ~, Wcp_c] = margin(Lc);
            if ~(isfinite(Wcp_c) && Wcp_c>0)
                warning('No hay Wcp continuo definido; uso prewarp=0 (Tustin simple).');
                opt = c2dOptions('tustin');
            else
                opt = c2dOptions('tustin','PrewarpFrequency', Wcp_c);
            end
            C2 = c2d(C_total, T, opt);

            % Renormalizar en la frecuencia digital equivalente
            if isfinite(Wcp_c) && Wcp_c>0
                Omega_c = 2*atan(Wcp_c*T/2);                 % Ωc
                Cz_ejO  = evalfr(C2, exp(1j*Omega_c));
                gfix    = 1/abs(Cz_ejO);
                C2      = gfix * C2;
            end

            % Lazo discreto y márgenes
            Lz = K * C2 * Gd;
            figure; margin(Lz); grid on; title('Discreto: Lz = K*C(z)*Gd(z)');
            [GMz, PMz, Wcgz, Wcpz] = margin(Lz);
            fprintf('DISCRETO: GM=%.2f dB, PM=%.2f°, Wcg=%.3g rad/s, Wcp=%.3g rad/s\n', 20*log10(GMz), PMz, Wcgz, Wcpz);

            % Respuesta al escalón (y[k])
            Ts = Gd.Ts; if Ts<=0, Ts = T; end
            N  = 2000; tvec = (0:N-1)'*Ts;
            CLz_y = feedback(Lz, 1);
            [y, tstep] = step(CLz_y, tvec);
            figure; stairs(tstep, y); grid on; title('Discreto: y[k] ante escalón'); xlabel('t [s]');

            % Esfuerzo u[k] = C(z)/(1+C(z)Gd(z)) * r[k]  (para r=step)
            T_u_r = feedback(C2, Gd*C2);   % C2 / (1 + C2*Gd)
            [u, tstep2] = step(T_u_r, tvec);
            figure; stairs(tstep2, u); grid on; title('Discreto: esfuerzo u[k] ante escalón'); xlabel('t [s]');

        case 0
            seguir = false;
            continue;

        otherwise
            fprintf('Opción inválida.\n');
    end
end

%% ================== RESUMEN FINAL (CONTINUO Y DISCRETO, TUSTIN SIMPLE) ==================
% --- Continuo (diseño sobre Gw) ---
L_final = K*C_total*Gw;
[GMf, PMf, Wcgf, Wcpf] = margin(L_final);

fprintf('\n=== RESULTADO FINAL (CONTINUO, DISEÑO) ===\n');
fprintf('K = %.6g\n', K);
disp('C_total(s) ='); disp(zpk(C_total));
fprintf('Márgenes: GM = %.2f dB, PM = %.2f°, Wcg = %.3g, Wcp = %.3g\n', 20*log10(GMf), PMf, Wcgf, Wcpf);

figure; bode(C_total); grid on; title('Compensador total C_{total}(s)');
figure; margin(L_final); grid on; title('L_{final}(s) = K · C_{total}(s) · G_w(s)');

% --- Discretizar el compensador (Tustin “normal”, sin prewarp) ---
C1  = K*c2d(C_total, T, 'tustin');

% --- Lazo discreto y márgenes ---
Lz = C1*Gd;
figure; margin(Lz); grid on; title('Final discreto: Lz = K·C(z)·G_d(z)');
[GMz, PMz, Wcgz, Wcpz] = margin(Lz);
fprintf('=== DISCRETO FINAL (Tustin simple) ===\nGM=%.2f dB, PM=%.2f°, Wcg=%.3g, Wcp=%.3g\n', ...
        20*log10(GMz), PMz, Wcgz, Wcpz);


% Salida y[k] ante escalón (CL: (K*C2*Gd)/(1+K*C2*Gd))
CLz_y = feedback(Lz, 1);
[y, tstep] = step(CLz_y);
figure; stairs(tstep, y); grid on; xlabel('t [s]'); ylabel('y[k]');
title(' y[k] ante escalón');



% Esfuerzo u[k] ante escalón: T_{u<-r}(z) = (K*C2)/(1+K*C2*Gd)
T_u_r = feedback(C1, Gd);   % equivalente a (K*C2) / (1 + K*C2*Gd)
[u, tstep2] = step(T_u_r);
figure; stairs(tstep2, u); grid on; xlabel('t [s]'); ylabel('u[k]');
title('Final discreto: esfuerzo u[k] ante escalón');


%% ================== COMPENSADOR INICIAL ==================
% Tu elección: integrador por defecto (podés cambiar a tf(1) si no querés I)
C_total = 1/s;     % <-- SI NO QUERÉS integrador, poné: C_total = tf(1);
K = 1;

fprintf('\n=== Estado inicial (K=1, C inicial) ===\n');
L_now = K*C_total*Gw;
[GM, PM, Wcg, Wcp] = margin(L_now);
fprintf('GM = %.2f dB, PM = %.2f°, Wcg = %.3g rad/s, Wcp = %.3g rad/s\n', 20*log10(GM), PM, Wcg, Wcp);
figure; margin(L_now); grid on; title('L(s) = K*C(s)*G_w(s)  (inicio)');

seguir = true;
while seguir
    fprintf('\nOpciones:\n');
    fprintf('  [1] Ajustar K a un GM deseado (dB)\n');
    fprintf('  [2] Agregar etapa LEAD/LAG (normalizada en Wcp)\n');
    fprintf('  [3] Auto-ajustar a (GM_dB, PM)  — o usar K fijo y PM\n');
    fprintf('  [4] Fijar K explícito (sin cálculos)\n');
    fprintf('  [5] Ir a DISCRETO (c2d con prewarp) y simular\n');
    fprintf('  [0] Terminar\n');
    op = str2double(input('Elegí opción: ','s'));

    switch op
        case 1  % Ajustar K a GM deseado
            GM_des_dB = str2double(input(' GM deseado (dB): ','s'));
            L_now = K*C_total*Gw;
            [GM_now, ~, ~, ~] = margin(L_now);
            GM_now_dB = 20*log10(GM_now);
            if ~isfinite(GM_now_dB)
                error('GM no definido; agregá una etapa o cambiá el rango antes de ajustar K.');
            end
            K = K * 10^((GM_now_dB - GM_des_dB)/20);
            L_now = K*C_total*Gw;
            [GM, PM, Wcg, Wcp] = margin(L_now);
            fprintf('  -> K = %.6g; GM = %.2f dB, PM = %.2f°, Wcg = %.3g, Wcp = %.3g\n', ...
                    K, 20*log10(GM), PM, Wcg, Wcp);
            figure; margin(L_now); grid on; title('Tras ajustar K');

        case 2  % Agregar etapa lead/lag normalizada en Wcp
            tipo = lower(strtrim(input(' Tipo ("lead" o "lag"): ','s')));
            phi  = str2double(input(' Fase a compensar (grados, positivo): ','s'));
            [C_total, rep] = add_stage_at_Wcp(Gw, C_total, K, tipo, phi, s);
            fprintf('  -> Etapa %s %+g° aplicada en wc=%.3g rad/s (normalizada)\n', rep.type, rep.phi_deg, rep.wc_used);
            L_now = K*C_total*Gw;
            [GM, PM, Wcg, Wcp] = margin(L_now);
            fprintf('  -> GM = %.2f dB, PM = %.2f°, Wcg = %.3g, Wcp = %.3g\n', 20*log10(GM), PM, Wcg, Wcp);
            figure; margin(L_now); grid on; title('Tras agregar etapa');

        case 3  % Auto-ajuste
            usa_k_fijo = lower(strtrim(input(' ¿Usar K fijo? [y/n]: ','s')));
            if strcmpi(usa_k_fijo,'y')
                K_fixed = str2double(input(' Valor de K fijo: ','s'));
                PM_des  = str2double(input(' PM deseada (grados): ','s'));
                [K, C_total, rep] = auto_hit_specs_fixedK(Gw, C_total, K_fixed, PM_des, s);
                fprintf('  -> AUTO (K fijo=%.6g) listo: PM=%.2f°, GM=%.2f dB, Wcg=%.3g, Wcp=%.3g\n', ...
                        K, rep.PM, rep.GMdB, rep.Wcg, rep.Wcp);
            else
                GM_des_dB = str2double(input(' GM deseado (dB): ','s'));
                PM_des    = str2double(input(' PM deseada (grados): ','s'));
                [K, C_total, rep] = auto_hit_specs(Gw, C_total, K, GM_des_dB, PM_des, s);
                fprintf('  -> AUTO listo (iters=%d): GM=%.2f dB, PM=%.2f°, Wcg=%.3g, Wcp=%.3g\n', ...
                        rep.iters, rep.GMdB, rep.PM, rep.Wcg, rep.Wcp);
            end
            L_now = K*C_total*Gw;
            figure; margin(L_now); grid on; title('Tras auto-ajuste');

        case 4  % Fijar K
            K = str2double(input(' K = ','s'));
            L_now = K*C_total*Gw;
            [GM, PM, Wcg, Wcp] = margin(L_now);
            fprintf('  -> K fijado. GM = %.2f dB, PM = %.2f°, Wcg = %.3g, Wcp = %.3g\n', 20*log10(GM), PM, Wcg, Wcp);
            figure; margin(L_now); grid on; title('Tras fijar K');

        case 5  % Discretizar C(s) -> C(z) con prewarp y simular
            Lc = K*C_total*Gw;
            [~, ~, ~, Wcp_c] = margin(Lc);
            if ~(isfinite(Wcp_c) && Wcp_c>0)
                warning('No hay Wcp continuo definido; uso prewarp=0 (Tustin simple).');
                opt = c2dOptions('tustin');
            else
                opt = c2dOptions('tustin','PrewarpFrequency', Wcp_c);
            end
            C2 = c2d(C_total, T, opt);

            % Renormalizar en la frecuencia digital equivalente
            if isfinite(Wcp_c) && Wcp_c>0
                Omega_c = 2*atan(Wcp_c*T/2);                 % Ωc
                Cz_ejO  = evalfr(C2, exp(1j*Omega_c));
                gfix    = 1/abs(Cz_ejO);
                C2      = gfix * C2;
            end

            % Lazo discreto y márgenes
            Lz = K * C2 * Gd;
            figure; margin(Lz); grid on; title('Discreto: Lz = K*C(z)*Gd(z)');
            [GMz, PMz, Wcgz, Wcpz] = margin(Lz);
            fprintf('DISCRETO: GM=%.2f dB, PM=%.2f°, Wcg=%.3g rad/s, Wcp=%.3g rad/s\n', 20*log10(GMz), PMz, Wcgz, Wcpz);

            % Respuesta al escalón (y[k])
            Ts = Gd.Ts; if Ts<=0, Ts = T; end
            N  = 2000; tvec = (0:N-1)'*Ts;
            CLz_y = feedback(Lz, 1);
            [y, tstep] = step(CLz_y, tvec);
            figure; stairs(tstep, y); grid on; title('Discreto: y[k] ante escalón'); xlabel('t [s]');

            % Esfuerzo u[k] = C(z)/(1+C(z)Gd(z)) * r[k]  (para r=step)
            T_u_r = feedback(C2, Gd*C2);   % C2 / (1 + C2*Gd)
            [u, tstep2] = step(T_u_r, tvec);
            figure; stairs(tstep2, u); grid on; title('Discreto: esfuerzo u[k] ante escalón'); xlabel('t [s]');

        case 0
            seguir = false;
            continue;

        otherwise
            fprintf('Opción inválida.\n');
    end
end


%% Resultado final
C_total = K*C_total;
L_final = C_total*Gw;
[GMf, PMf, Wcgf, Wcpf] = margin(L_final);
fprintf('Márgenes: GM = %.2f dB, PM = %.2f°, Wcg = %.3g, Wcp = %.3g\n', 20*log10(GMf), PMf, Wcgf, Wcpf);
figure; bode(C_total); grid on;
figure; margin(L_final); grid on; title('L_{final}(s) = K · C_{total}(s) · G_w(s)');
C2 = c2d(C_total,T,'tustin');
 figure;step(feedback(C2*Gd,1));grid on;title('Respuesta al escalon');
figure;step(feedback(C2,Gd));grid on;title('Esfuerzo al escalon');


Lz = Gd*C2;
% Salida y[k] ante la rampa (CL: (K*C2*Gd)/(1+K*C2*Gd))
CLz_y = feedback(Lz, 1);
N = 100*T;            % duración total
t = 0:T:N;            % vector de tiempo
rampa = t;            % rampa pendiente 1

[y, tstep] = lsim(CLz_y, rampa, t);

figure;
stairs(tstep, y, 'LineWidth',1.5); hold on;
plot(tstep, rampa, 'r--','LineWidth',1.2);

xlabel('t [s]');
ylabel('y[k]');
title('Respuesta y[k] ante rampa');
grid on; grid minor;

% Forzar relación 1:1 en ejes
axis equal




umin = 0;
umax =  4.08;
refmin = 1.75;
refmax = 2.25;
n_per_seg = 500;

Gc1f = feedback(Gd*C2,1);
%figure;
%step(Gc1f)
info = stepinfo(Gc1f);


info_ext = plot_step_annot(Gc1f, 'la planta compensada con un lag-filter de 1º orden, anulando el polo más rápido de la planta.');
compensador = zpk(C2);


[td, refd, yd, ud, ed, coefs] = sim_compensador_second_order( ...
    Gd, compensador.Z{1}(1), ...
    compensador.Z{1}(2), ...
    compensador.P{1}(1), ...
    compensador.P{1}(2), compensador.K, ...
    T, umin, umax, refmin, refmax, n_per_seg);

figure;
[y,td]=lsim(Gc1f,td);
stairs(td,y);
hold on;
grid on;
plot(td,td);


Nini = 100;              % descartar al inicio
Nfin = 100;              % descartar al final
idx0 = Nini + 1;         % índice inicial válido
idx1 = length(td) - Nfin; % índice final válido

td   = td(idx0:idx1)-td(idx0);
refd = refd(idx0:idx1);
yd   = yd(idx0:idx1);
ud   = ud(idx0:idx1);
ed   = ed(idx0:idx1);

% Si querés que el tiempo arranque en 0





figure;

% ---------- Subplot 1 ----------
axAbs = subplot(2,1,1);  % eje izquierdo (absoluto)
hold(axAbs,'on'); grid(axAbs,'on');

% y[k] en rojo (abs)
hY = stairs(axAbs, td, yd, 'r-', 'LineWidth', 1.4);

% Armamos eje derecho transparente
axPct = axes('Position', get(axAbs,'Position'), ...
             'Color','none', 'YAxisLocation','right', ...
             'XLim', get(axAbs,'XLim'), 'XTick',[], 'Box','off');
hold(axPct,'on');

% r[k] en negro, graficado en %
ref_pct = 100*(refd - refmin)/(refmax - refmin);
hR = stairs(axPct, td, refd, 'k--', 'LineWidth', 1.2);

% ======= Cálculo de límites con margen =======
yd_pct  = 100*(yd - refmin)/(refmax - refmin);
pctAll  = [yd_pct; ref_pct];

% Valores extremos en % con margen del 5 %
rawMin = min(pctAll);
rawMax = max(pctAll);
span   = rawMax - rawMin;
pctMin = rawMin - 0.05*span;
pctMax = rawMax + 0.05*span;

% Redondeamos a múltiplos de 20 para ticks
stepPct = 20;
tickMin = stepPct*floor(pctMin/stepPct);
tickMax = stepPct*ceil (pctMax/stepPct);
pctTicks = tickMin:stepPct:tickMax;

% Convertimos a absolutos
valTicks = refmin + (pctTicks/100)*(refmax - refmin);

% Aplicamos a ambos ejes
set(axAbs,'YLim',[valTicks(1) valTicks(end)],'YTick',valTicks);
set(axPct,'YLim',[valTicks(1) valTicks(end)],'YTick',valTicks,...
          'YTickLabel',compose('%.0f %%',pctTicks));

% Etiquetas
xlabel(axAbs,'Tiempo [s]');
ylabel(axAbs,'Respuesta [valor absoluto]');
ylabel(axPct,'Escala relativa a ref [%]');
title(axAbs,'Respuesta discreta con compensador 1º orden, anulando el polo más rápido de la planta.');
legend(axAbs, [hY, hR], {'y[k]', 'r[k]'}, 'Location', 'best');

% ---------- Subplot 2: esfuerzo de control ----------
axU = subplot(2,1,2);
stairs(axU, td, ud, 'LineWidth',1.2); grid(axU,'on');
yline(axU, umax,'r:'); yline(axU, umin,'r:');
xlabel(axU,'Tiempo [s]'); ylabel(axU,'u[k]');
title(axU,'Esfuerzo de control con saturación');

% --- Margen de 5% en eje Y ---
uMin = min(ud);
uMax = max(ud);
span = uMax - uMin;
ylim(axU, [uMin - 0.05*span, uMax + 0.05*span]);


grid(axAbs,'on');    % grilla principal
grid(axAbs,'minor'); % grilla secundaria

grid(axPct,'on');
grid(axPct,'minor');

grid(axU,'on');
grid(axU,'minor');
