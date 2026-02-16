close all; clear; clc

%% ===== 1) Cargar planta continua desde .mat =====
S = load('D:\auto\TPF-HelicopteroVertical\Matlab\planta (1).mat');

if isfield(S,'plantaC')
    plantaC = S.plantaC;      % <- tu variable típica
elseif isfield(S,'sysC')
    plantaC = S.sysC;
else
    error('No encuentro "plantaC" ni "sysC" en el .mat');
end

%% ===== 2) A,B,C,D CONTINUO =====
sysC = ss(plantaC);           % asegura forma espacio de estados
[Ac,Bc,Cc,Dc] = ssdata(sysC);

disp('=== CONTINUO ===');
disp('Ac ='); disp(Ac);
disp('Bc ='); disp(Bc);
disp('Cc ='); disp(Cc);
disp('Dc ='); disp(Dc);

%% ===== 3) A,B,C,D DISCRETO (Ts = 1 ms) =====
Ts = 1/1000;
sysD = c2d(sysC, Ts, 'zoh');  % ZOH recomendado para planta con hold

[Ad,Bd,Cd,Dd] = ssdata(sysD);

disp('=== DISCRETO (ZOH) ===');
fprintf('Ts = %.9f s\n', Ts);
disp('Ad ='); disp(Ad);
disp('Bd ='); disp(Bd);
disp('Cd ='); disp(Cd);
disp('Dd ='); disp(Dd);

%% ===== 4) (Opcional) Exportar a .mat para usar después =====
save('ABCD_cont_y_disc.mat','Ac','Bc','Cc','Dc','Ad','Bd','Cd','Dd','Ts');


