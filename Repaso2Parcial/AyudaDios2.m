clear all;close all

sys_tf = tf(1,[1,10,0]);

[F,G,H,J] = tf2ss(sys_tf.Numerator{1},sys_tf.Denominator{1});
sys_ssc = ss(F,G,H,J);
Ts = 3e-3;
sys_ssd =c2d(sys_ssc,Ts);
[A,B,C,D] = ssdata(sys_ssd);

wn = 40;zita=0.7;
s = -wn*zita+i*wn*sqrt(1-zita^2);
p = exp(s*Ts);

p_obs = p^5;
p_obs = [p_obs,conj(p_obs)]';

L = acker(A',(C*A)',p_obs).';

phat = [p,conj(p),0.5]'

n = length(A);m=size(C);m= m(1);
Ahat = [A,B;zeros(m,n+m)]
Bhat = [zeros(n,m);eye(m)]

khat = acker(Ahat,Bhat,phat);

Aux = [A-eye(n),B;...
        C*A,C*B];

K2K1 = (khat+[zeros(m,n) eye(m)])/(Aux);
K2 = K2K1(:,1:n);
K1 = K2K1(:,n+1:end);


Acl = [A,B;...
      K2-K2*A-K1*C*A,eye(m)-K2*B-K1*C*B];
Bcl = [zeros(n,m);K1*eye(m)];

Ccl = [C zeros(m,m);
       0 0 1];
Dcl = D;

sys_ssd_cl = ss(Acl,Bcl,Ccl,Dcl,Ts);
%figure;step(sys_tf);figure;step(sys_ssd_cl);figure;pzmap(sys_ssd_cl,sys_ssd);zgrid;
%%
N = 50;
[X,Xhat,U] = sim_current_int(A,B,C,D,K1,K2,L,N,[0.5;0.5]);

t = 0:Ts:(N-1)*Ts;
figure(1);
subplot(211);
plot(t,X(2,:));hold on; grid on; grid minor; stairs(t,Xhat(2,:));
subplot(212);
stairs(t,U); grid on; grid minor; hold on;

figure(2);
L = acker(A',(C)',p_obs).';
[X,Xhat,U] = sim_current_int(A,B,C,D,K1,K2,L,N,[0.5;0.5]);
subplot(211);
plot(t,X(2,:));hold on; grid on; grid minor; stairs(t,Xhat(2,:));
subplot(212);
stairs(t,U); grid on; grid minor;
