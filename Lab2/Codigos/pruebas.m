clc; clear; close all;
syms s t n z
syms k interger

% -------- Parámetros --------
a = 1/(81e3*200e-9);   
b = 1/(15e3*100e-9);  
T = 1.3452e-3;

% -------- Transformada de Laplace --------
Gs_s = 1/(s*(s+a)*(s+b));

% Inversa de Laplace
int_g_t = ilaplace(Gs_s, s, t);
disp('L^-1(G(s)/s) = ')
pretty(int_g_t)

% -------- Señal muestreada --------
int_g_n = subs(int_g_t, t, n*T); % f[n] = f(nT)

disp('L^-1(G(s)/s)*  = ')
pretty(int_g_n)

% -------- Transformada Z --------
Z_Gs_s = ztrans(int_g_n, n, z);
disp('Z{G(s)/s} = ')
pretty(Z_Gs_s)


Gz = simplify(((z - 1)/z) * Z_Gs_s);

disp('G(z) = (z-1)/z * Z{G(s)/s} =')
pretty(Gz)


% 2) Separar numerador y denominador y expandir
[num, den] = numden(Gz);
num = expand(num);
den = expand(den);

disp('Numerador factorado:')
num_fact = factor(num)

disp('Denominador factorado:')
den_fact = factor(den)


z0=solve_zero_syms(43,1,0.84327,0.1366771);
z0

