close all
clear all

addpath('..\Lab1\')

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
figure;
pzmap(Gd);
title('Lugar de raíces de G(z).')

zgrid;
z = tf([1 0],1,T);

%% Paso 3: Análisis en tiempo discreto lazo abierto
figure;

% Obtenemos salida y tiempo de la función step
[y, t] = step(Gd);

% Graficamos con stairs (propio de señales discretas)
stairs(t*1000, y, 'LineWidth',1.4);

title('Respuesta al escalón de la planta digitalizad')
xlabel('Tiempo [ms]');
ylabel('Salida');
grid on;
ax = gca;
ax.XMinorTick = 'on';  % activamos minor ticks
ax.YMinorTick = 'on';
grid minor;

%% Paso 4: Definimimos los valores de la simulación
umin = 0;
umax =  4.08;
refmin = 1.5;
refmax = 2.5;
n_per_seg = 500;
%% Paso 5: Lugar de raíces para zita = 0.7, compensador P
%figure

%rlocus(Gd)
%zgrid     % agrega la grilla en el plano-z
z0=0.436;
p0=0.769221;%0.7705;
C_1 = (z-z0)/(z-p0);
figure;
Gc1 = Gd*C_1;
rlocus(Gc1);
title('Lugar de raíces de G(z) compensado con un lag-filter de 1º orden, anulando el polo más rápido de la planta.')

zgrid     % agrega la grilla en el plano-z
%pause;
%[K, ~] = rlocfind(Gc1);   % hacés click donde querés los polos
%K = 0.73139;
%K = 0.82543;
K = 0.743;
Gc1f = feedback(K*Gc1,1);
%figure;
%step(Gc1f)
info = stepinfo(Gc1f);
info

info_ext = plot_step_annot(Gc1f, 'la planta compensada con un lag-filter de 1º orden, anulando el polo más rápido de la planta.');
%step(Gc3f);
info = stepinfo(Gc1f);
info
info_ext

[td, refd, yd, ud, ed, coefs] = sim_compensador_first_order( ...
    Gd, z0, p0, K, T, umin, umax, refmin, refmax, n_per_seg);

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

%% Paso 6: Lugar de raices para zita = 0.7 y y ESS = 0, compensador PI
z0 = 0.857419558001;
p0 = 1;
C_2 = (z-z0)/(z-p0); %z=0.8433 %8433
Gc2 =Gd*C_2;
figure;
rlocus(Gc2);
title('Lugar de raíces de G(z) compensado con un lag-filter de 1º orden , fijando un polo en z = 1.')

zgrid
%[K, ~] = rlocfind(Gc2);   % hacés click donde querés los polos
%K = 3.2779;
K2 = 3.09;
%figure;
Gc2f= feedback(K2*Gc2,1);
%step(Gc2f);
info = stepinfo(Gc2f);
info

info_ext = plot_step_annot(Gc2f, 'la planta compensada con un lag-filter de 1º orden , fijando un polo en z = 1.');
%step(Gc3f);
info = stepinfo(Gc2f);
info
info_ext


[td, refd, yd, ud, ed, coefs] = sim_compensador_first_order( ...
    Gd, z0, p0, K, T, umin, umax, refmin, refmax, n_per_seg);


Nini = 100;              % descartar al inicio
Nfin = 100;              % descartar al final
idx0 = Nini + 1;         % índice inicial válido
idx1 = length(td) - Nfin; % índice final válido

td   = td(idx0:idx1)-td(idx0);
refd = refd(idx0:idx1);
yd   = yd(idx0:idx1);
ud   = ud(idx0:idx1);
ed   = ed(idx0:idx1);




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
title(axAbs,'Respuesta discreta con compensador 1º orden, fijando un polo en z = 1.');
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



%% Paso 7: Lugar de raices para zita = 0.7 y y ESS = 0, compensador PI de segundo orden
z0_0 = 0.9333;
p0_0 = 1;
z0_1 = 0.436;
p0_1 = 0.71135;
C_3 = ((z-z0_0)*(z-z0_1))/((z-p0_0)*(z-p0_1));
Gc3 =Gd*C_3; %z=0.8433 %8433
figure;
rlocus(Gc3)
title('Lugar de raíces de G(z) compensado con un lag-filter de 2º orden, fijando un polo en z = 1 y anulando ambos polos de la planta.');

zgrid
%pause;
%[K, ~] = rlocfind(Gc3);   % hacés click donde querés los polos
K = 1.14;
%figure;
Gc3f= feedback(K*Gc3,1);

info_ext = plot_step_annot(Gc3f, 'la planta compensada con un lag-filter de 2º orden, fijando un polo en z = 1 y anulando ambos polos de la planta.');
%step(Gc3f);
info = stepinfo(Gc3f);
info
info_ext


% --- Llamada a tu simulador (misma interfaz que tu first_order) ---
[td, refd, yd, ud, ed, coefs] = sim_compensador_second_order( ...
    Gd, z0_0, z0_1, p0_0, p0_1, K, T, umin, umax, refmin, refmax, n_per_seg);


Nini = 100;              % descartar al inicio
Nfin = 100;              % descartar al final
idx0 = Nini + 1;         % índice inicial válido
idx1 = length(td) - Nfin; % índice final válido

td   = td(idx0:idx1)-td(idx0);
refd = refd(idx0:idx1);
yd   = yd(idx0:idx1);
ud   = ud(idx0:idx1);
ed   = ed(idx0:idx1);




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
title(axAbs,'Respuesta discreta con compensador 2º orden, fijando un polo en z = 1 y anulando ambos polos de la planta');
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


%% impresion antigua
% % --- Gráficas estilo informe ---
% figure; 
% subplot(2,1,1);
% stairs(td, refd, 'k--','LineWidth',1.0); hold on;
% stairs(td, yd,   'LineWidth',1.4);
% grid on; xlabel('Tiempo [s]'); ylabel('Respuesta de la planta/Referencia');
% title('Respuesta discreta con compensador 2º orden'); legend('r[k]','y[k]','Location','best');
% 
% subplot(2,1,2);
% stairs(td, ud, 'LineWidth',1.2);
% yline(umax,'r:'); yline(umin,'r:');
% grid on; xlabel('Tiempo [s]'); ylabel('u[k]');
% title('Esfuerzo de control con saturación');