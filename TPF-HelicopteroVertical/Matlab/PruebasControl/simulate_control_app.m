clear all
close all
clc
tau =1e3*5e-9;   % rad/s
s  = tf('s');
G  = 1/(tau*s + 1); % DC gain = 1, pole = -wp

Ts = 1/100e3;
Gd = c2d(G, Ts, 'tustin');

controlSystemDesigner(Gd)
