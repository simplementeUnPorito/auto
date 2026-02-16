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

%% ===== 2.1) Controlabilidad/Observabilidad (CONTINUO) =====
n = size(Ac,1);

CtrbC = ctrb(Ac,Bc);
ObsvC = obsv(Ac,Cc);

% ranks (con y sin tolerancia explícita)
tolC = 1e-9;   % podés ajustar si querés
rankCtrbC = rank(CtrbC);
rankObsvC = rank(ObsvC);
rankCtrbC_tol = rank(CtrbC, tolC);
rankObsvC_tol = rank(ObsvC, tolC);

disp('--- CONTINUO: Controlabilidad / Observabilidad ---');
disp('CtrbC = ctrb(Ac,Bc) ='); disp(CtrbC);
disp('ObsvC = obsv(Ac,Cc) ='); disp(ObsvC);
fprintf('rank(CtrbC) = %d (de %d) | con tol=%.1e -> %d\n', rankCtrbC, n, tolC, rankCtrbC_tol);
fprintf('rank(ObsvC) = %d (de %d) | con tol=%.1e -> %d\n', rankObsvC, n, tolC, rankObsvC_tol);

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

%% ===== 3.1) Controlabilidad/Observabilidad (DISCRETO) =====
CtrbD = ctrb(Ad,Bd);
ObsvD = obsv(Ad,Cd);

tolD = 1e-9;
rankCtrbD = rank(CtrbD);
rankObsvD = rank(ObsvD);
rankCtrbD_tol = rank(CtrbD, tolD);
rankObsvD_tol = rank(ObsvD, tolD);

disp('--- DISCRETO: Controlabilidad / Observabilidad ---');
disp('CtrbD = ctrb(Ad,Bd) ='); disp(CtrbD);
disp('ObsvD = obsv(Ad,Cd) ='); disp(ObsvD);
fprintf('rank(CtrbD) = %d (de %d) | con tol=%.1e -> %d\n', rankCtrbD, n, tolD, rankCtrbD_tol);
fprintf('rank(ObsvD) = %d (de %d) | con tol=%.1e -> %d\n', rankObsvD, n, tolD, rankObsvD_tol);

%% ===== 4) (Opcional) Exportar a .mat para usar después =====
save('ABCD_cont_y_disc.mat', ...
     'Ac','Bc','Cc','Dc','Ad','Bd','Cd','Dd','Ts', ...
     'CtrbC','ObsvC','rankCtrbC','rankObsvC','rankCtrbC_tol','rankObsvC_tol', ...
     'CtrbD','ObsvD','rankCtrbD','rankObsvD','rankCtrbD_tol','rankObsvD_tol', ...
     'tolC','tolD');
