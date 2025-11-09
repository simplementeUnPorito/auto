s = tf([1 0],1);
G = 1/s^2;
sys = ss(G)

close all
clear all

F = [-2 2;0.5 -0.75];
G = [0;0.5];

sys = ss(F,G,[1 0],0);

sysd = c2d(sys,0.25,'zoh');