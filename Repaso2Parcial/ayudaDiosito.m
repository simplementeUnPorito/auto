clc;close all;clear all;


s = tf([1,0],1);
Gc = 1/s^2
T = 1;

Gd = zpk(c2d(Gc,T,'zoh'))

z = tf([1,0],1,T);

%% Analitico con oscilaciones entre muestras
Gcl_objetivo = z^-1;
C1 = 1/Gd*(Gcl_objetivo/(1-Gcl_objetivo))

Gcl = zpk(feedback(C1*Gd,1))
[Yd,td] = step(Gcl);
figure;stairs(td,Yd);
Ud = step(feedback(C1,Gd));
Uc = repelem(Ud,40);
tc = linspace(0,T*length(Ud),length(Uc));
[Y,t] = lsim(Gc,Uc,tc);
figure;plot(t,Y)

%% Analitico sin oscilaciones entre muestras
a1 = T^2/2;
a0 = a1;
K = 1/(a1+a0);
b0 = K*a0;
C = K*(z-1)/(z+b0);
Gcl = zpk(feedback(C*Gd,1))
[Yd,td] = step(Gcl);
figure;stairs(td,Yd);
Ud = step(feedback(C,Gd));
Uc = repelem(Ud,40);
tc = linspace(0,T*length(Ud),length(Uc));
[Y,t] = lsim(Gc,Uc,tc);
figure;plot(t,Y);
%% 