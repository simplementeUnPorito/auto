%Script para comparar el rendimiento del algoritmo PID discretizado por el
%método de Euler y el continuo

%Realizado por: Enrique A. Vargas
%Fecha: 04/09/05

close all;
clear all;
%Se define la planta. Se utiliza la planta del ejemplo 3.4 del Franklin
numG = 360000;
denG = conv([1 60],[1 600]);
motor = tf(numG,denG);

%Implementación del control analogico
% C(s) = Kp*(Ti*Td*S^2 + Ti*S + 1)/(Ti*S)
Kp = 5;
Td = .0008;
Ti = .003;
numC=Kp*[Ti*Td Ti 1];
denC=[Ti 0];
controlador = tf(numC,denC);
cloop_c =feedback(controlador*motor,1);
t=0:.0002:.01;
yc=step(cloop_c,t);
figure(1)
plot(t*1000,yc,'-');
hold on
title('Respuesta a un escalon unitario');
xlabel('Tiempo (msec)');
ylabel('Velocidad angular (rad/sec)');
%pause

%Se define el tiempo de muestreo el cual es función del ancho de banda 
%tr = 1.8/Wn
T = 0.03e-3;
%El tiempo de simulacion
td = [0:T:0.01];
%Se discretiza la planta 
motorD = c2d(motor,T,'zoh');
%Se obtienen las matrices de la plantaD y los coeficientes de los polinomios
[numD denD]= tfdata(motorD,'v');
%Se obtienen los coeficientes del numerador y del denominador de la planta digital
b0 = numD(2);
b1 = numD(3);
a0 = denD(1);
a1 = denD(2);
a2 = denD(3);

%Se redefinen los parametros del controlador PID 



%Se inicializan los vectores
yd = zeros(size(td));
ed = zeros(size(td));
ud = zeros(size(td));
refd = ones(size(td));

Pt = zeros(size(td));
It = zeros(size(td));
Dt = zeros(size(td));

for(k=3:max(size(td))-1)
    %Se "lee" el valor de la referencia y se calcula la señal de error
    ed(k) = refd(k) - yd(k);
     %Se calcula el esfuerzo de control
%     Pt(k) = Kp*ed(k);
%     It(k) = It(k-1) + (Kp*T/Ti)*ed(k-1);
%     Dt(k) = (Kp*Td/T)*(ed(k) - ed(k-1));
%     ud(k) = Pt(k) + It(k) + Dt(k);
    %Se calcula el esfuerzo de control
    ud_sinsaturar = ud(k-1)+Kp*((1+T/Ti+Td/T)*ed(k)-(1+2*Td/T)*ed(k-1)+(Td/T)*ed(k-2));
     if ud_sinsaturar >5
            ud(k)=5;
        elseif ud_sinsaturar<0
            ud(k) = 0;
        else
             ud(k) = ud_sinsaturar;
        end
    %Se actualiza la salida de la planta
    yd(k+1) = b0*ud(k) + b1*ud(k-1) - a1*yd(k)-a2*yd(k-1);
end
figure;
stairs(1000*td,yd)
figure;
plot(1000*td,ud);

N = 8;
alpha2 = Td*Ti/(T^2);
alpha1 = N*Ti/T;
beta2  = Kp*(N*Td*Ti + Td*Ti)/(T^2);
beta1  = Kp*(N*Ti + Td)/T;
beta0  = Kp*N;

U0 = alpha2 + alpha1;
U1 = -(2*alpha2 + alpha1);
U2 = alpha2;

E0 = beta2 + beta1 + beta0;
E1 = -(2*beta2 + beta1);
E2 = beta2;

for k = 3:numel(td)-1
        ed(k) = refd(k) - yd(k);
        ud(k) = (-U1*ud(k-1) + -U2*ud(k-2) + E0*ed(k) + E1*ed(k-1) + E2*ed(k-2))/U0;
        yd(k+1) = b0*ud(k) + b1*ud(k-1) - a1*yd(k) - a2*yd(k-1);
end
% === Gráfico combinado con doble eje ===
figure;
stairs(1000*td,yd)
figure;
plot(1000*td,ud)

