close all; clear; clc

DATA_MAT_PATH = 'D:\auto\TPF-HelicopteroVertical\Matlab\Compensadores\PID_teorico.mat';
S = load(DATA_MAT_PATH);

u = S.u(:);
y = S.y(:);

N = min(numel(u), numel(y));
u = u(1:N); y = y(1:N);

if isfield(S,'x')
    x = S.x(:);
    if numel(x) ~= N, x = (1:N).'; end
    xlab = 'x';
elseif isfield(S,'n')
    x = S.n(:);
    if numel(x) ~= N, x = (1:N).'; end
    xlab = 'n [muestras]';
else
    x = (1:N).';
    xlab = 'n [muestras]';
end

if isfield(S,'xlab'), xlab = S.xlab; end
if isfield(S,'titulo'), titulo = S.titulo; else, titulo = 'PID (log)'; end

figure('Name','PID: u/y');
subplot(2,1,1); plot(x,y); grid on; grid minor; title([titulo ' - y']); ylabel('y');
subplot(2,1,2); plot(x,u); grid on; grid minor; title([titulo ' - u']); ylabel('u'); xlabel(xlab);
