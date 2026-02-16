close all; clear; clc

DATA_MAT_PATH = 'D:\auto\TPF-HelicopteroVertical\Matlab\Compensadores\rlocus_teorico.mat';
S = load(DATA_MAT_PATH);

% Elegí qué planta usar (prioridad: sysD > plantaD > plantaC > planta)
if isfield(S,'sysD')
    P = S.sysD;
elseif isfield(S,'plantaD')
    P = S.plantaD;
elseif isfield(S,'plantaC')
    P = S.plantaC;
elseif isfield(S,'planta')
    P = S.planta;
else
    error('No encuentro sysD/plantaD/plantaC/planta en el .mat');
end

fprintf('Planta elegida: %s | Ts = %g\n', class(P), getfield(P,'Ts',{1}));

% Si es idpoly/idss lo convierto a tf/ss para graficar bien
try
    Ptf = tf(P);
catch
    Ptf = ss(P);
end

figure; pzmap(Ptf); grid on; title('Polos y ceros de la planta');

figure; bode(Ptf); grid on; title('Bode de la planta');

figure; step(Ptf); grid on; title('Step de la planta');

% Si existe un controlador "Cd" (como se ve en tu workspace)
if isfield(S,'Cd')
    Cd = S.Cd;
    L = series(Cd, Ptf);
    figure; rlocus(L); grid on; title('Root Locus de L(z)=C(z)P(z)');
    
    % cerrado (si querés ver step CL)
    Tcl = feedback(L, 1);
    figure; step(Tcl); grid on; title('Step del lazo cerrado (feedback unity)');
end
