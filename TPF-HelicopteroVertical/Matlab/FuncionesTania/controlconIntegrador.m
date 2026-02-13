close all; clear; clc

%% =========================
% 1) CARGA + DISCRETIZACIÓN
%% =========================
S = load('planta (1).mat');

if isfield(S,'plantaC')
    plantaC = S.plantaC;
elseif isfield(S,'sysC')
    plantaC = S.sysC;
else
    error('No encuentro "plantaC" ni "sysC" dentro de planta (1).mat');
end

Ts   = 1/50;                 % <-- tu Fs real
sysD = c2d(plantaC, Ts, 'zoh');
[A,B,C,D] = ssdata(ss(sysD));

% SISO: si hay varias salidas, elegimos la primera
if size(C,1) > 1
    warning('C tiene %d salidas. Usando la primera (y1).', size(C,1));
    C = C(1,:);
    D = D(1,:);
end

n = size(A,1);
m = size(B,2);

fprintf('Ts=%.9f | n=%d | m=%d\n', Ts, n, m);
if m ~= 1
    error('Este script asume SISO (m=1). Tu m=%d', m);
end

%% =========================
% 2) POLOS DESEADOS
%% =========================
p_ctrl = [0.9 + 0.15i, 0.9 - 0.15i, 0.90];   % n polos
p_obs  = [0.4 + 0.25i, 0.4 - 0.25i, 0.60];   % n polos

% Polo extra para el integrador (n+1 polos total)
p_int  = 0.80;

p_ctrl = fit_poles_to_n(p_ctrl, n, 'p_ctrl');
p_obs  = fit_poles_to_n(p_obs,  n, 'p_obs');

%% =========================
% 3) GANANCIAS: Kx, Ki (integral) + observadores
%% =========================
% --------- Diseño con integrador de error ----------
% Definición (discreta, causal):
% y(k) = C x(k) + D u(k-1)   (medís antes de aplicar u(k))
% e(k) = r(k) - y(k)
% xi(k+1) = xi(k) + e(k)     (o +Ts*e(k) si activás)
%
% Para el diseño servo integral (sin referencia):
% x(k+1)  = A x(k) + B u(k)
% xi(k+1) = xi(k) - C x(k) - D u(k)   (porque e = r - y)
Aint = [A          zeros(n,1);
       -C          eye(1)];

Bint = [B;
       -D];

p_ctrl_i = [p_ctrl(:).' p_int];

[Kint, precKi] = place_prec(Aint, Bint, p_ctrl_i);
Kx = Kint(1,1:n);
Ki = Kint(1,end);

% --------- Observadores (tu formato) ----------
[KeP, precKeP] = place_prec(A', C', p_obs);      Ke_pred = KeP.';          % predictor
[KeA, precKeA] = place_prec(A', (C*A)', p_obs);  Ke_act  = KeA.';          % "actual"

fprintf('precKint=%.2f dig | precKe_pred=%.2f dig | precKe_act=%.2f dig\n', precKi, precKeP, precKeA);
disp('Kx ='); disp(Kx);
disp('Ki ='); disp(Ki);

%% =========================
% 4) SIMULACIÓN: IDEAL vs REALISTA + float32
%% =========================
N        = 200;
ulim_sat = 300;     % sat ±300 (ajustá según tu "u": Δu o PWM)
ulim_inf = Inf;

% step 0 -> 20
r = zeros(1,N);
r(2:end) = 20;

use_Ts_in_integrator = false;  % xi += e  (false)  |  xi += Ts*e (true)
use_antiwindup       = true;   % recomendado en sat

% ----- DOUBLE ideal (sin saturación) -----
out_noobs_d_ideal = sim_noobs_loop_int(A,B,C,D,Kx,Ki,r,Ts,ulim_inf,use_Ts_in_integrator,false,false);
out_pred_d_ideal  = sim_obs_loop_int  (A,B,C,D,Kx,Ki,Ke_pred,Ke_act,r,Ts,ulim_inf,false,use_Ts_in_integrator,false,false);
out_act_d_ideal   = sim_obs_loop_int  (A,B,C,D,Kx,Ki,Ke_pred,Ke_act,r,Ts,ulim_inf,true ,use_Ts_in_integrator,false,false);

% ----- DOUBLE realista (con saturación) -----
out_noobs_d_sat = sim_noobs_loop_int(A,B,C,D,Kx,Ki,r,Ts,ulim_sat,use_Ts_in_integrator,use_antiwindup,false);
out_pred_d_sat  = sim_obs_loop_int  (A,B,C,D,Kx,Ki,Ke_pred,Ke_act,r,Ts,ulim_sat,false,use_Ts_in_integrator,use_antiwindup,false);
out_act_d_sat   = sim_obs_loop_int  (A,B,C,D,Kx,Ki,Ke_pred,Ke_act,r,Ts,ulim_sat,true ,use_Ts_in_integrator,use_antiwindup,false);

% ----- FLOAT32 realista (con saturación) -----
out_noobs_f = sim_noobs_loop_int(A,B,C,D,Kx,Ki,r,Ts,ulim_sat,use_Ts_in_integrator,use_antiwindup,true);
out_pred_f  = sim_obs_loop_int  (A,B,C,D,Kx,Ki,Ke_pred,Ke_act,r,Ts,ulim_sat,false,use_Ts_in_integrator,use_antiwindup,true);
out_act_f   = sim_obs_loop_int  (A,B,C,D,Kx,Ki,Ke_pred,Ke_act,r,Ts,ulim_sat,true ,use_Ts_in_integrator,use_antiwindup,true);

t = (0:N-1)*Ts;

%% =========================
% 5) PLOTS (ideal vs real)
%% =========================
figure('Name','IDEAL: y (double, sin sat) - con integral');
plot(t, out_noobs_d_ideal.y, t, out_pred_d_ideal.y, '--', t, out_act_d_ideal.y, '-.');
grid on; xlabel('t [s]'); ylabel('y'); legend('NoObs','Pred','Act','Location','best');
title('IDEAL: y(t) sin saturación (con integral, sin Nbar)');

figure('Name','IDEAL: u (double, sin sat) - con integral');
plot(t, out_noobs_d_ideal.u, t, out_pred_d_ideal.u, '--', t, out_act_d_ideal.u, '-.');
grid on; xlabel('t [s]'); ylabel('u'); legend('NoObs','Pred','Act','Location','best');
title('IDEAL: u(t) sin saturación (con integral, sin Nbar)');

figure('Name','REAL: y (double, sat ±ulim) - con integral');
plot(t, out_noobs_d_sat.y, t, out_pred_d_sat.y, '--', t, out_act_d_sat.y, '-.');
grid on; xlabel('t [s]'); ylabel('y'); legend('NoObs','Pred','Act','Location','best');
title('REAL: y(t) con saturación (con integral + antiwindup, sin Nbar)');

figure('Name','REAL: u (double, sat ±ulim) - con integral');
plot(t, out_noobs_d_sat.u, t, out_pred_d_sat.u, '--', t, out_act_d_sat.u, '-.');
yline(+ulim_sat,'k--'); yline(-ulim_sat,'k--');
grid on; xlabel('t [s]'); ylabel('u'); legend('NoObs','Pred','Act','Location','best');
title('REAL: u(t) con saturación (con integral + antiwindup, sin Nbar)');

figure('Name','Error float32 vs double (REAL, y) - integral');
plot(t, out_pred_d_sat.y - out_pred_f.y, t, out_act_d_sat.y - out_act_f.y);
grid on; xlabel('t [s]'); ylabel('y_{double} - y_{float32}');
legend('Pred','Act','Location','best');
title('REAL: diferencia por usar float32 (single) - con integral');

fprintf('\nMax |u| ideal (double): NoObs=%.3g  Pred=%.3g  Act=%.3g\n', ...
    max(abs(out_noobs_d_ideal.u)), max(abs(out_pred_d_ideal.u)), max(abs(out_act_d_ideal.u)));

fprintf('Muestras saturadas (double realista ±ulim): NoObs=%d  Pred=%d  Act=%d\n', ...
    sum(abs(out_noobs_d_sat.u) >= ulim_sat-1e-9), ...
    sum(abs(out_pred_d_sat.u)  >= ulim_sat-1e-9), ...
    sum(abs(out_act_d_sat.u)   >= ulim_sat-1e-9));

%% =========================
% 6) Z-PLANE: planta + CL aumentado + obs
%% =========================
p_ol   = eig(A);
p_cl_i = eig(Aint - Bint*Kint);

p_op = eig(A - Ke_pred*C);
p_oa = eig(A - Ke_act*C*A);

try
    z_plant = tzero(ss(A,B,C,D,Ts));
catch
    z_plant = [];
end

plot_zplane(figName('Z-plane: Integral'), Ts, p_ol, p_cl_i, p_op, z_plant);


fprintf('\nPolos planta (A):\n');              disp(p_ol.');
fprintf('Polos CL aumentado (Aint-Bint*Kint):\n'); disp(p_cl_i.');
fprintf('Polos obs pred (A-KeC):\n');        disp(p_op.');
fprintf('Polos obs act  (A-KeCA):\n');       disp(p_oa.');

%% ============================================================
% ===================== FUNCIONES LOCALES ======================

function p = fit_poles_to_n(p, n, name)
    p = p(:).';
    if numel(p) ~= n
        warning('%s tiene %d polos pero n=%d. Ajustando...', name, numel(p), n);
        if numel(p) < n
            p = [p repmat(p(end), 1, n-numel(p))];
        else
            p = p(1:n);
        end
    end
end

function [K, prec] = place_prec(A,B,p)
    K = place(A,B,p);
    pa = eig(A-B*K);
    err = norm(poly(pa) - poly(p(:)), inf);
    if err <= 0, prec = 99; else, prec = max(0, -log10(err)); end
end

function out = sim_noobs_loop_int(A,B,C,D,Kx,Ki,r,Ts,ulim,useTsInt,useAW,use_single)
    if use_single
        A=single(A); B=single(B); C=single(C); D=single(D);
        Kx=single(Kx); Ki=single(Ki); r=single(r);
        Ts=single(Ts); ulim=single(ulim);
        useTsInt = logical(useTsInt); useAW = logical(useAW);
    end
    n = size(A,1); N = numel(r);
    x  = zeros(n,N,'like',A);
    y  = zeros(1,N,'like',A);
    u  = zeros(1,N,'like',A);
    xi = zeros(1,N,'like',A);

    u_prev = zeros(1,1,'like',A);
    y(1) = C*x(:,1) + D*u_prev;

    for k=1:N-1
        yk = C*x(:,k) + D*u_prev;   % medición causal
        e  = r(k) - yk;

        % integrador
        if useTsInt, xi_prop = xi(k) + Ts*e;
        else,       xi_prop = xi(k) + e;
        end

        % control SIN Nbar: u = -Kx x - Ki xi
        u_unsat = -Kx*x(:,k) - Ki*xi_prop;
        u_sat   = sat(u_unsat, ulim);

        % anti-windup: freeze si saturó y el error empuja hacia saturación
        if useAW && ~isinf(ulim) && (u_sat ~= u_unsat)
            if ( (u_unsat > u_sat && e > 0) || (u_unsat < u_sat && e < 0) )
                xi_prop = xi(k);
                u_unsat = -Kx*x(:,k) - Ki*xi_prop;
                u_sat   = sat(u_unsat, ulim);
            end
        end

        xi(k+1) = xi_prop;
        u(k)    = u_sat;

        % planta
        x(:,k+1) = A*x(:,k) + B*u(k);

        y(k) = yk;
        u_prev = u(k);
    end

    y(N) = C*x(:,N) + D*u_prev;
    u(N) = sat(-Kx*x(:,N) - Ki*xi(N), ulim);

    out.x = x; out.y = y; out.u = u; out.xi = xi;
end

function out = sim_obs_loop_int(A,B,C,D,Kx,Ki,Ke_pred,Ke_act,r,Ts,ulim,use_actual,useTsInt,useAW,use_single)
    if use_single
        A=single(A); B=single(B); C=single(C); D=single(D);
        Kx=single(Kx); Ki=single(Ki);
        Ke_pred=single(Ke_pred); Ke_act=single(Ke_act);
        r=single(r); Ts=single(Ts); ulim=single(ulim);
        use_actual = logical(use_actual); useTsInt = logical(useTsInt); useAW = logical(useAW);
    end
    n = size(A,1); N = numel(r);
    x  = zeros(n,N,'like',A);
    xh = zeros(n,N,'like',A);
    y  = zeros(1,N,'like',A);
    u  = zeros(1,N,'like',A);
    xi = zeros(1,N,'like',A);

    u_prev = zeros(1,1,'like',A);
    y(1) = C*x(:,1) + D*u_prev;

    for k=1:N-1
        yk = C*x(:,k) + D*u_prev;
        e  = r(k) - yk;

        if useTsInt, xi_prop = xi(k) + Ts*e;
        else,       xi_prop = xi(k) + e;
        end

        u_unsat = -Kx*xh(:,k) - Ki*xi_prop;   % SIN Nbar
        u_sat   = sat(u_unsat, ulim);

        if useAW && ~isinf(ulim) && (u_sat ~= u_unsat)
            if ( (u_unsat > u_sat && e > 0) || (u_unsat < u_sat && e < 0) )
                xi_prop = xi(k);
                u_unsat = -Kx*xh(:,k) - Ki*xi_prop;
                u_sat   = sat(u_unsat, ulim);
            end
        end

        xi(k+1) = xi_prop;
        u(k)    = u_sat;

        % planta
        x(:,k+1) = A*x(:,k) + B*u(k);

        % observador
        if ~use_actual
            % predictor: usa y(k)
            yhat = C*xh(:,k) + D*u_prev;
            xh(:,k+1) = A*xh(:,k) + B*u(k) + Ke_pred*( yk - yhat );
        else
            % actual: usa y(k+1)
            z   = A*xh(:,k) + B*u(k);
            yk1 = C*x(:,k+1) + D*u(k);   % medida en k+1 (post-aplicación)
            yzh = C*z + D*u(k);
            xh(:,k+1) = z + Ke_act*( yk1 - yzh );
        end

        y(k) = yk;
        u_prev = u(k);
    end

    y(N) = C*x(:,N) + D*u_prev;
    u(N) = sat(-Kx*xh(:,N) - Ki*xi(N), ulim);

    out.x = x; out.xh = xh; out.y = y; out.u = u; out.xi = xi;
end

function y = sat(u,lim)
    if isinf(lim), y = u;
    else,          y = min(max(u, -lim), lim);
    end
end

function plot_zplane(figName, Ts, p_ol, p_cl, p_obs, z_plant)
    figure('Name',figName,'NumberTitle','off');
    ax = gca; hold(ax,'on'); box(ax,'on');

    % --- círculo unitario (discreto) ---
    th = linspace(0,2*pi,600);
    plot(ax, cos(th), sin(th), 'k:', 'LineWidth', 1.0);

    % --- grilla z (ζ y ωn) ---
    % zgrid sin args suele estar OK. Si se ve “cargado”, te dejo abajo la versión custom.
    zgrid(ax,'on');                 % <- lo que pediste: zgrid
    grid(ax,'on'); grid(ax,'minor');

    % --- datos ---
    h1 = plot(ax, real(p_ol),  imag(p_ol),  'o', 'MarkerSize', 7, 'LineWidth', 1.6);
    h2 = plot(ax, real(p_cl),  imag(p_cl),  'x', 'MarkerSize', 8, 'LineWidth', 2.0);
    h3 = plot(ax, real(p_obs), imag(p_obs), '^', 'MarkerSize', 7, 'LineWidth', 1.8);

    hz = [];
    if ~isempty(z_plant)
        hz = plot(ax, real(z_plant), imag(z_plant), 's', 'MarkerSize', 7, 'LineWidth', 1.6);
    end

    % --- estética ---
    axis(ax,'equal');
    xlim(ax, [-1.2 1.2]);
    ylim(ax, [-1.2 1.2]);
    xlabel(ax,'Re\{z\}');
    ylabel(ax,'Im\{z\}');
    title(ax, sprintf('%s  (Ts=%.4g s)', figName, Ts));

    if ~isempty(hz)
        legend(ax,[h1 h2 h3 hz], ...
            {'Polos planta (A)','Polos CL (aug)','Polos obs','Ceros planta'}, ...
            'Location','bestoutside');
    else
        legend(ax,[h1 h2 h3], ...
            {'Polos planta (A)','Polos CL (aug)','Polos obs'}, ...
            'Location','bestoutside');
    end
end


function s = figName(base)
    s = base;
end
