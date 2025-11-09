close all; clear; clc;


%% ====================== Cargar datos medidos ======================
data = readmatrix('./openLoop.csv');

% u = ENTRADA medida; y = SALIDA medida
u = data(:,5 );   % ajustá si cambia columna
y = data(:, 11);

Ts = 199.99999495E-6;  % tiempo de muestreo (≈100 µs)

 data_id = iddata(y, u, Ts);
 G = tfest(data_id, 2,0); 

compare(data_id, G);

save('modelo.mat','G')