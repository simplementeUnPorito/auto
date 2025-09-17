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

disp('Planta continua G(s):');
%% Paso 1: Definir periodo de muestreo
% Se busca una mejora de 2 en el tiempo de rising =, o sea tr = 10 ms
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





%% C1_K

data = readmatrix('.\C1_K\tablas.csv');  % lee todo el CSV en una matriz

t_meas   = data(:,4);   % columna 1 = tiempo
ref_meas = data(:,5);   % columna 2 = referencia
u_meas   = data(:,11);   % columna 3 = esfuerzo
y_meas   = data(:,17);   % columna 4 = salida

% Suponiendo que tenés vectores del osciloscopio:
% t_meas, ref_meas, u_meas, y_meas  (todos Nx1, en segundos y unidades reales)
opts = struct('Nini', 1, 'Nfin', 1, 'plot', true,'demean_errors',false);

S = sim_compensador_first_order_meas( ...
    Gd, 0, 0, 7, T, 0, 4.08, ...
    t_meas, ref_meas, u_meas, y_meas, ...
    opts);

% Accesos:
S.errors.u    % métricas esfuerzo sim vs real
S.errors.y    % métricas salida  sim vs real

opts = struct('Nini', 1, 'Nfin', 1, 'plot', true);

S = sim_compensador_first_order_meas( ...
    Gd, 0, 0, 7, T, 0, 4.08, ...
    t_meas, ref_meas, u_meas, y_meas, ...
    opts);

% Accesos:
S.errors.u    % métricas esfuerzo sim vs real
S.errors.y    % métricas salida  sim vs real


%% C1_Lead

data = readmatrix('.\C1_lead\tablas.csv');  % lee todo el CSV en una matriz

t_meas   = data(:,4);   % columna 1 = tiempo
ref_meas = data(:,5);   % columna 2 = referencia
u_meas   = data(:,11);   % columna 3 = esfuerzo
y_meas   = data(:,17);   % columna 4 = salida

% Suponiendo que tenés vectores del osciloscopio:
% t_meas, ref_meas, u_meas, y_meas  (todos Nx1, en segundos y unidades reales)
opts = struct('Nini', 1, 'Nfin', 1, 'plot', true);

S = sim_compensador_first_order_meas( ...
    Gd, 0.622387144541200, 0.374311732736374,  1.184734400491065, T, 0, 4.08, ...
    t_meas, ref_meas, u_meas, y_meas, ...
    opts);

% Accesos:
S.errors.u    % métricas esfuerzo sim vs real
S.errors.y    % métricas salida  sim vs real

opts = struct('Nini', 1, 'Nfin', 1, 'plot', true,'demean_errors',true);

S = sim_compensador_first_order_meas( ...
    Gd, 0.622387144541200, 0.374311732736374,  1.184734400491065, T, 0, 4.08, ...
    t_meas, ref_meas, u_meas, y_meas, ...
    opts);

% Accesos:
S.errors.u    % métricas esfuerzo sim vs real
S.errors.y    % métricas salida  sim vs real

%% C2

data = readmatrix('.\C2\tablas.csv');  % lee todo el CSV en una matriz

t_meas   = data(:,4);   % columna 1 = tiempo
ref_meas = data(:,5);   % columna 2 = referencia
u_meas   = data(:,11);   % columna 3 = esfuerzo
y_meas   = data(:,17);   % columna 4 = salida

% Suponiendo que tenés vectores del osciloscopio:
% t_meas, ref_meas, u_meas, y_meas  (todos Nx1, en segundos y unidades reales)
opts = struct('Nini', 1, 'Nfin', 1, 'plot', true,'demean_errors',false);

S = sim_compensador_second_order_meas( ...
    Gd,-1,0.948680356607820,...
    1,0.805044751405714,...
    0.185620357009816,...
 T, 0, 4.08, ...
    t_meas, ref_meas, u_meas, y_meas, ...
    opts);


opts = struct('Nini', 1, 'Nfin', 1, 'plot', true,'demean_errors',true);

S = sim_compensador_second_order_meas( ...
    Gd,-1,0.948680356607820,...
    1,0.805044751405714,...
    0.185620357009816,...
 T, 0, 4.08, ...
    t_meas, ref_meas, u_meas, y_meas, ...
    opts);

% Accesos:
S.errors.u    % métricas esfuerzo sim vs real
S.errors.y    % métricas salida  sim vs real


