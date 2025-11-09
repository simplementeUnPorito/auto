%% lab 7 - integrador con estiamdor de prediccion 
addpath('..\');

%refi leer seccion 8.4.1 del franklin

close all
clear all

R1 = 47000;
R2 = 150000;
R3 = 4700;
R4 = 15000;
C1 = 100e-9;
C2 = 100e-9;

F = [-1/(R2*C1) 0;-1/(R3*C2) -1/(R4*C2)];
G = [-1/(R1*C1); 0];
H = [0 1];
J = 0;

%Montamos la planta continua
planta = ss(F,G,H,J);

%Discretizamos la planta
T = 1e-3;
plantaD = c2d(planta,T);

%Discretizamos las matrices de estado --> x(k+1) = Ax(k) + Bu(k)
%                                     --> y(k) = Cx(k) + Du(k)
%Hallamos A,B,C,D
A = plantaD.a;
B = plantaD.b;
C = plantaD.c;
D = plantaD.d;

polos = [0.8+i*0.25 0.8-i*0.25];

K = acker(A,B,polos);

N = 100;
t = [0:T:N*T];

%se agrega el estimador de prediccion:
xep = zeros(2,N);
yep = zeros(2,N);
polos_ep = [0.4+i*0.4 0.4-i*0.4];
Kep = acker(A',C',polos_ep')'; %los polos del estimador deben de presentar siempren una respuesta mas rapida que el sistema, por ley del control

Xp= zeros(2,max(size(t)));
yp = zeros(1,max(size(t)));

[Nx,Nu,Nbar] = refi(A,B,C,K);

W = 0.5*Nbar.*ones(size(yp));
c = 50;
%c = max(size(W));
W(1,1:c) = zeros(1,c);
ref = Nbar.*ones(size(yp));

for k = 1: max(size(t))-1,
    %Se calcula el esfuerzo de control
    u(k) = ref(k)-K*xep(:,k)+ W(1,k);
    %Se hace evolucionar la planta 
    Xp(:,k+1)= A*Xp(:,k) + B*u(k);    
    %se hace evolucionar el el estimador de prediccion
    xep(:,k+1) = A*xep(:,k) + B*u(k) + Kep*(C*Xp(:,k) - C*xep(:,k));
end

%% SE AGREGA EL INTEGRADOR
%Se tiene que definir el nuevo sistema (Ahat, Bhat, Chat, Dhat)
n = max(size(A)); %El orden del sistema
m = 1;            %La cantidad de salidas  
Ahat = [A B;zeros(m,n+m)];
Bhat = [zeros(n,m); eye(m)];
Chat = [C zeros(1,m)];

polos_i = [0.8+i*0.25 0.8-i*0.25 0.6];

Khat = acker(Ahat,Bhat,polos_i);

Aux = [A-eye(size(A)) B; C*A C*B];
K2K1 = (Khat + [zeros(1,max(size(A))) eye(m)])*inv(Aux);
K2 = K2K1(1,1:n);
K1 = K2K1(1,n+1:n+m);

%se define el estimador de prediccion para el sistema con integrador
xepi= zeros(2,N);
yepi= zeros(2,N);
polos_epi = [0.4+i*0.4 0.4-i*0.4];
Kepi = acker(A',C',polos_epi')'; %los polos del estimador deben de presentar siempren una respuesta mas rapida que el sistema, por ley del control


%A seguir se determina la salida, para ello se utiliza el diagrama de bloques de 
%la figura 6.19 del Ogata
%Se inicializan las variables
%Se determina un valor inicial de la planta;
Xi = zeros(n,length(t));
V = zeros(m,length(t));
ui = zeros(size(t));
V(m,1) = 0;
ref = ones(size(t));

%[Nx,Nu,Nbar] = refi(A,B,C,K2); % no se usa pq hay integrador

for k = 2: max(size(t))-1,5
  %Se calcula la salida actual
   yi(m,k) = C*Xi(:,k);
   %Ecuacionamiento del integrador 
   V(m,k) = V(m,k-1) + ref(m,k) - yi(m,k);
   ui(m,k) = K1*V(m,k)- K2*xepi(:,k) + W(m,k); % + Nx.*ref(m,k);
   %Se calcula el siguiente valor del vector de estado
   Xi(:,k+1) = A*Xi(:,k) + B*ui(k);
   %se calcula la estimacion
   xepi(:,k+1) = A*xepi(:,k) + B*ui(k) + Kepi*(C*Xi(:,k) - C*xepi(:,k))
   
end

figure
subplot(211)
plot(t,Xp(1,:),'b', t,Xi(1,:),'r');
ylabel('Estado 1')
subplot(212)
plot(t,Xp(2,:),'b',t,Xi(2,:),'r');
ylabel('Estado 2 (salida)')

figure
plot(t(1,1:max(size(u))),u,'b',t,ui,'r')
title('Esfuerzo de control con y sin integrador, estimador de predicción.')