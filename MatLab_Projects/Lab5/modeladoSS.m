%% ====================== Cargar datos medidos ======================
data = readmatrix('mediciones.csv');

% u = ENTRADA medida; y = SALIDA medida
y = data(:, 11);   % ajustá si cambia columna
u = data(:, 17);

Ts = 0.000099999997474;  % tiempo de muestreo (≈100 µs)
data_id = iddata(y, u, Ts);
sysC = ssest(data_id, 2);
sysC = ss(sysC);            % A,B,C,D numéricos; sin K ni NoiseVariance
step(sysC)