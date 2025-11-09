%% lab 6 comparacion entre estimadores
clear all
close all

N =100;
T = 1e-3;
t = [0:T:(N-1)*T];

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
plantaD = c2d(planta,T);

%Discretizamos las matrices de estado --> x(k+1) = Ax(k) + Bu(k)
% --> y(k) = Cx(k) + Du(k)
%Hallamos A,B,C,D
A = plantaD.a;
B = plantaD.b;
C = plantaD.c;
D = plantaD.d;
%% estimador de prediccion

%variables de la planta
xp = zeros(2,N);
up = zeros(1,N);
%K = [10 3.5];
Kp = acker(A,B,[0.8+i*0.25, 0.8-i*0.25]);
%ref = ones(1,N);
%condicion inicial
xp(:,1) = [pi/6; 0.5] ;
%variables a estimar:
xep = zeros(2,N);
yep = zeros(2,N);
polos_ep = [0.4+i*0.4 0.4-i*0.4];
Kep = acker(A',C',polos_ep')'; %los polos del estimador deben de presentar siempren una respuesta mas rapida que el sistema, por ley del control

%% estimador actual
xa = zeros(2,N);
ua = zeros(1,N);
zea = zeros(2,N);
%Kea = [10 3.5];
Ka = acker(A,B,[0.8+i*0.25, 0.8-i*0.25]);

%condicion inicial
xa(:,1) = [pi/6; 0.5] ;

%variables a estimar:
xea = zeros(2,N);
polos_ea = [0.4+i*0.4 0.4-i*0.4];
%polos_e = [0 0];
%para el estimador actual cambia la definicion del error de estiamacion
%e(k+1) = (A-KeCA)e(k), la ganancia en ackerman en A' y CA'
Kea = acker(A',(C*A)',polos_ea')'; %los polos del estimador deben de presentar siempren una respuesta mas rapida que el sistema, por ley del control

ruido = 0.03*randn(size(xep)); %el 0.03 es la varianza ? 

ref = ones(1,N);
%% se evolucionan ambos sistemas
for j=1:N-1
   
    %estimador de prediccion:
    %se calcula el esfuerzo de control con el estado estimado
    up(j) = ref(j)-Kp*xep(:,j);
    
    %se hace evolucionar la planta
    xp(:,j+1) = A*xp(:,j) + B*up(j);
    
    %se hace evolucionar el el estimador de prediccion
    xep(:,j+1) = A*xep(:,j) + B*up(j) + Kep*(ruido(j) + C*xp(:,j) - C*xep(:,j));
    
    %estimador actual:
    ua(j) = ref(j)-Ka*xea(:,j);
    
    %se hace evolucionar la planta
    xa(:,j+1) = A*xa(:,j) + B*ua(j);

    %se hace evolucionar el estimador actual:
    zea(:,j+1) = A*xea(:,j) + B*ua(j);
    xea(:,j+1) = zea(:,j+1) + Kea*(ruido(j)+ C*xa(:,j+1) - C*zea(:,j+1));
    
end

figure
subplot(211)
plot(t,xp(1,:),'b',t,xep(1,:),'b*',t,xa(1,:),'r',t,xea(1,:),'r*')
subplot(212)
plot(t,xp(2,:),'b',t,xep(2,:),'b*',t,xa(2,:),'r',t,xea(2,:),'r*')

% figure
% subplot(211)
% plot(t,xa(1,:),t,xea(1,:))
% title('X1 estimador actual')
% subplot(212)
% plot(t,xa(2,:),t,xea(2,:))
% title('X2 estimador actual')