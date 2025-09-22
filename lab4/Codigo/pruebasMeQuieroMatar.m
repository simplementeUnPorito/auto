clear all
close all

% Test rápido
s = tf('s');
G = 1/(0.05*s+1);  % planta continua
Ts = 1e-3;
Gd = c2d(G, Ts, 'zoh');

% PI discreto simple (ejemplo)
Kp=10; Ki=50;
Cd = c2d(pid(Kp,Ki), Ts, 'tustin');

[t, refd] = gen_ref_pulso_blocks(Ts,3*50e-3,2,0.5,1/50e-3); umin=-5; umax=5;
figure;
plot(t,refd)
[yd,ud,ucd,ed,t] = sim_lazo_discreto_sat(Gd,Cd,refd,umin,umax);

figure;
subplot(3,1,1); plot(t,yd); grid on; ylabel('y[k]')
subplot(3,1,2); plot(t,ud,'-', t,ucd,'--'); grid on; ylabel('u[k]'); legend('u','uc')
subplot(3,1,3); plot(t,ed); grid on; ylabel('e[k]'); xlabel('t [s]')


Nups = 40;           % submuestras por periodo (p.ej. 40)
Muse = 0;            % 0 => usar todo u[k]
S = ver_intersample_desde_u(G, Ts, ud, Muse, Nups);
