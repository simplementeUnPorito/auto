close all
clear all
clc;
%% modelo
% load('../modelo.mat');Gc = G;
% [F,G,H,J] = tf2ss(Gc.Numerator,Gc.Denominator); sysC = ss(F,G,H,J);
% fn = 2*abs(min(zpk(Gc).P{1})/pi);
% Tn = 1/fn;

% === Parametros fisicos ===
% C1 = 103.07e-9;  C2 = 211.1e-9;
% R1 = 14.878e3;    R2 = 14.760e3;
% R3 = 80.55e3;    R4 = 81.09e3;
C2 = 103.07e-9;  C1 = 211.1e-9;
R3 = 14.878e3;    R4 = 14.760e3;
R1 = 80.55e3;    R2 = 81.09e3;


tau1 = R2*C1;           k1 = -R2/R1;
tau2 = R4*C2;           k2 = -R4/R3;
%tau2 = 1/376.4;
%tau1 = 1/114.2;
F = [ -1/tau1,     0;
       k2/tau2, -1/tau2 ];
G = [ k1/tau1; 0 ];
H = [0 1];     % <- justo lo que querías
J = 0;

sysC = ss(F,G,H,J);     % modelo "práctico" coherente
Gc   = tf(sysC);        % si querés compararlo con el empírico

zpk(Gc)



fn = max(abs(zpk(Gc).P{1}))/pi;
Tn = 1/fn;


%% Parametros
Ts = [Tn/2,Tn/1.5]; 
wn_obj = 4*abs(max(zpk(Gc).P{1}));
zita_obj = 0.7;
poloObj = @(zita,wn,Ts) exp(Ts*wn*(-zita+1j*sqrt(1-zita^2)));
p1 =[poloObj(zita_obj,wn_obj,Ts(1));conj(poloObj(zita_obj,wn_obj,Ts(1)))];
p2 = [0.5+1j*0.5;0.5-1j*0.5];
polos = cell(2);
polos = {p1,p2};
%% Calculo


%step(sysC)
%save('../modeloSS.mat','sysC');
sysD = cell(2);
sysD = {c2d(sysC,Ts(1)),c2d(sysC,Ts(2))};
K = cell(length(polos));
K0 = cell(length(polos));
Kref = cell(length(polos));
sysCL = cell(length(polos));

addpath('..\');
for i = 1:length(polos)
    [A,B,C,D] =ssdata(sysD{i});
    K{i} = acker(A,B,polos{i});
    Ac = A - B*K{i};                  
    
    sysCL{i} = ss(Ac, B, C, D, sysD{i}.Ts);
    [~,~,K0{i}] = refi(Ac,B,C,K{i});
    Kref{i} = 1/dcgain(sysCL{i});
end



%%
A = [0.5,0.4];
r = cell(2);
r = { A(1)/2.*[-ones(1,50), ones(1,50)], ...
      A(2)/2.*[-ones(1,100), ones(1,100)] };

for i = 1:length(sysCL)
    sysi = ss(sysCL{i});

    % === REFERENCIA de esta iteración, como columna ===
    ri = r{i}(:);

    % === Simulación ===
    [X, Y, U] = ss_sym_digital(sysi, ri, K{i}, K0{i}, [0;0],0);

    % (si querés offset DC)
    X = 2.024 + X;
    Y = 2.024 + Y;
    U = 2.024 + U;

    % === Tiempo consistente con la referencia usada ===
    N  = numel(ri);
    t  = (0:N-1)' * sysi.Ts;
    n  = size(X,2);

    % ----- Figura y paneles -----
    fig = figure('Name', sprintf('CL #%d — señales + polos/ceros', i));
    left = uipanel(fig,'Position',[0.05 0.08 0.58 0.87]);
    rightAx = axes(fig,'Position',[0.68 0.10 0.28 0.82]);

    % ----- Columna izquierda -----
    tl = tiledlayout(left, n+2, 1, 'TileSpacing','compact','Padding','compact');

    % y & r
    ax = nexttile(tl); hold(ax,'on');
    stairs(ax, t, Y(:), 'LineWidth', 1.6);             % y como vector
    stairs(ax, t, 2.024 + ri, '--', 'LineWidth', 1.2); % r alineada a t
    grid(ax,'on'); grid(ax,'minor'); ylabel(ax,'y, r');
    legend(ax, {'y','r'}, 'Location','best');
    title(ax, sprintf('CL #%d — Salida y referencia', i));

    % estados
    for k = 1:n
        axk = nexttile(tl);
        stairs(axk, t, X(:,k), 'LineWidth', 1.4);
        grid(axk,'on'); grid(axk,'minor'); ylabel(axk, sprintf('x_%d',k));
        if k==1, title(axk, 'Estados'); end
    end

    % u
    axu = nexttile(tl);
    stairs(axu, t, U(:), 'LineWidth', 1.3);
    grid(axu,'on'); grid(axu,'minor'); ylabel(axu,'u'); xlabel(axu,'Tiempo [s]');
    title(axu,'Esfuerzo de control');

    % ----- Columna derecha: polos y ceros -----
    axes(rightAx); %#ok<LAXES>
    cla(rightAx);
    try
        pzplot(sysi); zgrid;
        title(sprintf('Polos y ceros (fs=%.4g Hz)', 1/sysi.Ts));
    catch
        [Z,P,~] = zpkdata(sysi,'v'); hold(rightAx,'on'); grid(rightAx,'on'); axis(rightAx,'equal');
        plot(real(P), imag(P), 'x', 'LineWidth', 1.6, 'Parent', rightAx);
        plot(real(Z), imag(Z), 'o', 'LineWidth', 1.6, 'Parent', rightAx);
        if sysi.Ts>0
            th = linspace(0,2*pi,400);
            plot(cos(th), sin(th), '--', 'Parent', rightAx);
            xlabel(rightAx,'Re(z)'); ylabel(rightAx,'Im(z)');
        else
            xline(rightAx,0,'--'); xlabel(rightAx,'Re(s)'); ylabel(rightAx,'Im(s)');
        end
        legend(rightAx,{'Polos','Ceros','Círc. unidad'},'Location','best');
        title(rightAx, sprintf('Polos/Ceros (manual) — fs=%.4g Hz', 1/sysi.Ts));
    end
end
