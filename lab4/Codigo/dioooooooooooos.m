% Ejemplo simple de simulación con saturación
clear all
close all
% Planta continua: G(s) = 1/(s+1)
s = tf('s');
G = 1/((s+1));

% Controlador discreto PI: C(z) = Kp + Ki*T/(z-1)
Kp = 1; Ki = 0.001; T = 0.01;
C = c2d(pid(Kp,Ki), T, 'tustin'); % discreto

% Parámetros simulación
N = 500;
refd = ones(N,1);      % escalón
umin = -2; umax = 2;   % saturación

% Discretizar planta
Gd = c2d(G,T,'zoh');
zpk( feedback(C*Gd,1))
% Obtener coeficientes
[Nc,Dc] = tfdata(C,'v');  Nc = Nc(:)'; Dc = Dc(:)';
[Nb,Db] = tfdata(Gd,'v'); Nb = Nb(:)'; Db = Db(:)';

% Inicializar
yc = zeros(N,1);   % salida de planta
u  = zeros(N,1);   % señal aplicada
uc = zeros(N,1);   % señal de controlador (sin saturación)
e  = zeros(N,1);

for k=1:N
    e(k) = refd(k) - yc(k);
    
    % --- controlador discreto (uc) ---
    uc_k = 0;
    for i=0:length(Nc)-1
        if k-i>=1, uc_k = uc_k + Nc(i+1)*e(k-i); end
    end
    for i=1:length(Dc)-1
        if k-i>=1, uc_k = uc_k - Dc(i+1)*uc(k-i); end
    end
    uc(k) = uc_k;          % señal antes de saturación
    u(k)  = min(max(uc_k,umin), umax);  % saturada

    % --- planta discreta ---
    yk = 0;
    for i=0:length(Nb)-1
        if k-i>=1, yk = yk + Nb(i+1)*u(k-i); end
    end
    for i=1:length(Db)-1
        if k-i>=1, yk = yk - Db(i+1)*yc(k-i); end
    end
    if k<N, yc(k+1)=yk; end
end

% Gráficos
t = (0:N-1)*T;
subplot(3,1,1); plot(t,yc); ylabel('y[k]')
subplot(3,1,2); plot(t,u); ylabel('u[k]')
subplot(3,1,3); plot(t,e); ylabel('e[k]'); xlabel('t [s]')



