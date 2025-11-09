close all
clear all
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

% Cargar CSV
data = readmatrix('opltab.csv');

% Ignorar cabeceras, quedarte solo con datos numéricos
u = data(1:end, 5); % columna E = salida
y = data(1:end, 11); % columna K = entrada
Ts = 0.000099999997474; % tiempo de muestreo de B3

% Crear objeto de identificación
data_id = iddata(y, u, Ts);

% Estimar una función de transferencia discreta (ejemplo: 2 polos, 1 cero)
sys0 = tfest(data_id,2);

% % Ver resultado
%  step(sys);
% % Crear datos de identificación
% data_id = iddata(y, u, Ts);
% 
% % Probar con distintos órdenes
% sys1 = tfest(data_id, 1, 0); % 1 polo
% sys2 = tfest(data_id, 2, 1); % 2 polos, 1 cero
% 
% % Comparar

%% Tolerancias
tolR = 0.05;   % ±5% resistencias
tolC = 0.20;   % ±20% capacitores

% Nominales (por claridad)
R1n = R_1;  R2n = R_2;  R3n = R_3;  R4n = R_4;
C1n = C_1;  C2n = C_2;

% Extremos
R1_lo = (1 - tolR)*R1n;   R1_hi = (1 + tolR)*R1n;
R2_lo = (1 - tolR)*R2n;   R2_hi = (1 + tolR)*R2n;
R3_lo = (1 - tolR)*R3n;   R3_hi = (1 + tolR)*R3n;
R4_lo = (1 - tolR)*R4n;   R4_hi = (1 + tolR)*R4n;

C1_lo = (1 - tolC)*C1n;   C1_hi = (1 + tolC)*C1n;
C2_lo = (1 - tolC)*C2n;   C2_hi = (1 + tolC)*C2n;

% -----------------------------
% G_min: ganancia mínima y polos más lentos (taus máximos)
%   - Minimizar (R3/R1)*(R4/R2): R3,R4 bajos; R1,R2 altos
%   - Maximizar taus: R3,R4 altos; C1,C2 altos
% Para ser conservadores con ambos objetivos:
K_min  = (R3_lo/R1_lo) * (R4_lo/R2_lo);
tau1_max = R3_hi * C2_hi;
tau2_max = R4_hi * C1_hi;
G_min = tf(K_min, conv([tau1_max 1], [tau2_max 1]));

% -----------------------------
% G_max: ganancia máxima y polos más rápidos (taus mínimos)
%   - Maximizar (R3/R1)*(R4/R2): R3,R4 altos; R1,R2 bajos
%   - Minimizar taus: R3,R4 bajos; C1,C2 bajos
K_max  = (R3_hi/R1_hi) * (R4_hi/R2_hi);
tau1_min = R3_lo * C2_lo;
tau2_min = R4_lo * C1_lo;
G_max = tf(K_max, conv([tau1_min 1], [tau2_min 1]));

%% Ganancia estacionaria experimental
dy = y(end) - y(1);
du = u(end) - u(1);
K_exp = dy / du;

% Ganancias DC de cada modelo
[magG, ~]   = bode(G, 0);     magG   = squeeze(magG);
[mag0, ~]   = bode(sys0, 0);  mag0   = squeeze(mag0);
[magMin, ~] = bode(G_min, 0); magMin = squeeze(magMin);
[magMax, ~] = bode(G_max, 0); magMax = squeeze(magMax);

% Normalizar cada uno a la ganancia experimental
G       = G      * (K_exp / magG);
sys0_n  = sys0   * (K_exp / mag0);
G_min_n = G_min  * (K_exp / magMin);
G_max_n = G_max  * (K_exp / magMax);

%% Comparación
figure;
compare(data_id, sys0_n, G, G_min_n, G_max_n);
legend('Datos','sys0 normalizado','G nominal norm.','G_{min} norm.','G_{max} norm.','Location','best');
grid on;



% G = tf(sys0);
% T = 1.25e-3;
% Gd = c2d(G, T, 'zoh');
% disp('Planta digital G(z):')
% zpk(Gd)
% controlSystemDesigner(Gd)
% 
% save('planta.mat', 'G');
