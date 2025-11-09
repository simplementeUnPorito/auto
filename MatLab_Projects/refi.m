function [Nx,Nu,Nbar] = refi(phi,gam,Hr,K)
% REFI.M  computes command input matrices
% # of contols must be equal to # of outputs
% i.e., if gam = n x m, then H must be m x n
% [Nx,Nu,Nbar] = refi(phi,gam,Hr,K)

I=eye(size(phi));
[m,n]=size(Hr);
np=inv([phi-I gam;Hr zeros(m)])*([zeros(n,m);eye(m)]);
Nx=np(1:n,:);
Nu=np(n+1:n+m,:);
Nbar=Nu+K*Nx;