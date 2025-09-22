% function sim_pid_euler_astrom(G, T, Kp, Ti, Td, N, stepSize, umin, umax, var, k, seed)
% % PID incremental (Euler) con derivador filtrado N (Åström).
% % - MISMA referencia ruidosa para continuo y discreto:
% %   refd[k] = stepSize + N(0,var)  -->  refc(t) = ZOH(refd)
% % - Saturación del actuador (DAC) en [umin,umax]
% % - Anti-windup simple por "clamping" (como tu versión anterior)
% %
% % G         : planta continua (tf)
% % T         : tiempo de muestreo
% % Kp,Ti,Td  : parámetros PID
% % N         : parámetro del filtro derivativo
% % stepSize  : amplitud del escalón
% % umin,umax : saturación del actuador (discreto)
% % var       : varianza del ruido gaussiano en la referencia
% % k         : factor: tend = k * SettlingTime
% % seed      : (opcional) semilla RNG
% 
%     if nargin >= 12 && ~isempty(seed), rng(seed,'twister'); end
%     if nargin < 11 || isempty(k), k = 1.5; end
% 
%     % ===== Control analógico (para comparación) =====
%     numC = Kp*[Ti*Td*(1+1/N), Ti+Td/N, 1];
%     denC = [Ti*Td/N, Ti, 0];
%     C        = tf(numC, denC);
%     Tcls     = feedback(C*G, 1);   % referencia -> salida
% 
%     info = stepinfo(Tcls);
%     if isfinite(info.SettlingTime) && info.SettlingTime > 0
%         tend  = max(5*T, k*info.SettlingTime);
%         wn    = max(1e-6, 1.8/max(info.RiseTime, eps));
%         Tstep = (2*pi/wn)/3000;    % malla fina para lsim
%     else
%         Tstep = T/20; tend = 10*T;
%     end
%     t  = 0:Tstep:tend;     % continuo
%     td = 0:T:tend;         % discreto
% 
%     % ===== Referencia CONTINUA con ruido y su versión muestreada =====
%     sigma = sqrt(max(var,0));
%     refc  = stepSize*ones(size(t)) + sigma*randn(size(t));   % ref continua
%     % "Digitalización" por muestreo ideal en kT:
%     refd  = interp1(t, refc, td, 'linear', 'extrap');        % r[k] = r_c(kT)
% 
%     % Salida continua con la MISMA referencia (ZOH)
%     yc = lsim(Tcls, refc, t);
% 
%     % ===== Planta discreta (ZOH) =====
%     Gd = c2d(G, T, 'zoh');
%     [numD, denD] = tfdata(Gd, 'v');
%     if numel(numD) < 3 || numel(denD) < 3
%         error('Se espera una planta discreta al menos de 2º orden.');
%     end
%     % y[n+1] = b0*u[n] + b1*u[n-1] - a1*y[n] - a2*y[n-1]
%     b0 = numD(2); b1 = numD(3);
%     a1 = denD(2); a2 = denD(3);
% 
%     % ===== Coeficientes PID incremental (Euler + filtro derivativo N) =====
%     alpha2 = Td*Ti/(T^2);
%     alpha1 = N*Ti/T;
%     beta2  = Kp*(N*Td*Ti + Td*Ti)/(T^2);
%     beta1  = Kp*(N*Ti + Td)/T;
%     beta0  = Kp*N;
% 
%     U0 = alpha2 + alpha1;
%     U1 = -(2*alpha2 + alpha1);
%     U2 = alpha2;
% 
%     E0 = beta2 + beta1 + beta0;
%     E1 = -(2*beta2 + beta1);
%     E2 = beta2;
% 
%     % ===== Simulación discreta =====
%     yd = zeros(size(td));   % salida
%     ud = zeros(size(td));   % control (saturado)
%     ed = zeros(size(td));   % error
% 
%     for n = 3:numel(td)-1
%         % error (misma referencia que el continuo, pero en kT)
%         ed(n) = refd(n) - yd(n);
% 
%         % (1) control sin saturación
%         u_unsat = (-U1*ud(n-1) + -U2*ud(n-2) + E0*ed(n) + E1*ed(n-1) + E2*ed(n-2))/U0;
% 
%         % (2) saturación del actuador
%         u_sat = min(max(u_unsat, umin), umax);
% 
%         % (3) anti-windup por "clamping" (opcional: comentar si NO querés AW)
%         push_up   = (u_sat >= umax) && (ed(n) > 0);
%         push_down = (u_sat <= umin) && (ed(n) < 0);
%         if push_up || push_down
%             ed_eff = 0;   % congelar contribución integral (aprox)
%         else
%             ed_eff = ed(n);
%         end
% 
%         % (4) recomputar u[n] con error efectivo
%         ud(n) = (-U1*ud(n-1) + -U2*ud(n-2) + E0*ed_eff + E1*ed(n-1) + E2*ed(n-2))/U0;
% 
%         % (5) planta discreta
%         yd(n+1) = b0*ud(n) + b1*ud(n-1) - a1*yd(n) - a2*yd(n-1);
%     end
% 
%     % Alineación visual (shift)
%     yshift = circshift(yd, -2);
% 
%     % ===== Gráficos =====
%     figure;
%     yyaxis left
%     plot(t*1000, refc, ':','Color', '#B0B0B0', 'LineWidth', 1.0); hold on;       % r_ZOH(t)
%     plot(t*1000, yc,  '--','LineWidth',1.4);                  % y_c(t)
%     stairs(1000*td, yshift, '-','LineWidth',1.2);             % y_d[n]
%     ylabel('Salida / Referencia');
% 
%     yyaxis right
%     stairs(1000*td, ud, '-','LineWidth',1.2);                 % u_d[n]
%     ylabel('u[n]');
% 
%     title(sprintf(['Escalón con ref muestreada ruidosa (var=%.3g) | ' ...
%                    'Kp=%.3g, Ti=%.3g, Td=%.3g, N=%.3g | f_m=%.3g Hz'], ...
%                    var, Kp, Ti, Td, N, 1/T));
%     xlabel('Tiempo (ms)');
%     grid on;
%     legend('r_{ZOH}(t)','y_c(t) (lsim)','y_d[n] (shift)','u_d[n]', 'Location','best');
% end



function sim_pid_euler_astrom(G, T, Kp, Ti, Td,N,stepSize,sigma,umin,umax)
% Replica el script ejem3_4.m usando:
%   - G   : planta continua (tf)
%   - T   : tiempo de muestreo (s)
%   - Kp, Ti, Td : parámetros del PID
%
% PID continuo: C(s) = Kp*(Ti*Td*s^2 + Ti*s + 1)/(Ti*s)
% PID discreto: forma incremental (Euler), planta con ZOH, ecuación en diferencias.
    % ===== Coeficientes PID incremental (Euler + filtro derivativo N) =====
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
    % --- Control analógico ---
    numC = Kp*[Ti*Td, Ti, 1];
    denC = [Ti, 0];
    controlador = tf(numC, denC);
    cloop_c = feedback(controlador*G, 1);
    info = stepinfo(cloop_c);
    tend = info.SettlingTime*3;
    wn = (1.8/ info.RiseTime);
    Tstep = (2*pi/wn)/3000;
    t = 0:Tstep:tend;
    td = 0:T:tend;
    %yc = step(cloop_c, t);
    refc  = stepSize*ones(size(t)) + sigma*randn(size(t));   % ref continua
    % Salida continua con la MISMA referencia (ZOH)
    yc = lsim(cloop_c, refc, t);
    % "Digitalización" por muestreo ideal en kT:
    refd  = interp1(t, refc, td, 'linear', 'extrap');        % r[k] = r_c(kT)
    % --- Discretización y lazo discreto ---
    td = 0:T:tend;
    Gd = c2d(G, T, 'zoh');
    [numD, denD] = tfdata(Gd, 'v');

    b0 = numD(2);
    b1 = numD(3);
    a1 = denD(2);
    a2 = denD(3);

    yd   = zeros(size(td));
    ed   = zeros(size(td));
    ud   = zeros(size(td));
    %refd = ones(size(td));
    

    % Mostrar
    fprintf('=== Coeficientes PID MATLAB ===\n');
    fprintf('alpha1 = %.10f\n', alpha1);
    fprintf('alpha2 = %.10f\n', alpha2);
    fprintf('beta0  = %.10f\n', beta0);
    fprintf('beta1  = %.10f\n', beta1);
    fprintf('beta2  = %.10f\n', beta2);
    fprintf('U0     = %.10f\n', U0);
    fprintf('U1     = %.10f\n', U1);
    fprintf('U2     = %.10f\n', U2);
    fprintf('E0     = %.10f\n', E0);
    fprintf('E1     = %.10f\n', E1);
    fprintf('E2     = %.10f\n', E2);
    for k = 3:numel(td)-1
        ed(k) = refd(k) - yd(k);
        ud_sinsaturar = (-U1*ud(k-1) + -U2*ud(k-2) + E0*ed(k) + E1*ed(k-1) + E2*ed(k-2))/U0;
         if ud_sinsaturar >umax
            ud(k)=umax;
        elseif ud_sinsaturar<umin
            ud(k) = umin;
        else
             ud(k) = ud_sinsaturar;
        end
        yd(k+1) = b0*ud(k) + b1*ud(k-1) - a1*yd(k) - a2*yd(k-1);
    end

    % === Graficar ===
    yshift = circshift(yd, -2);

    figure;
    yyaxis left
    plot(t*1000, refc, ':','Color','#B0B0B0'); hold on;
    plot(t*1000, yc, '--','LineWidth',1.3);
    stairs(td*1000, yshift,'-','LineWidth',1.2);
    ylabel('Salida');

    yyaxis right
    plot(td*1000, ud, 'r--','LineWidth',1.2);
    ylabel('u_d(k)');

    xlabel('Tiempo [ms]');
    title(sprintf('PID Aström: Kp=%.3g, Ti=%.3g, Td=%.3g, N=%.3g', Kp, Ti, Td,N));
    legend(sprintf('Escalon Ruidoso (A=%.3g, $\\sigma$=%.3g)', stepSize, sigma), ...
           'Respuesta al Escalon Continua sin Wind-Up', ...
           'Respuesta al Escalon Discreta con Wind-Up', ...
           sprintf('Esfuerzo de Control (Rango:%.3g-%.3g)',umin,umax), ...
           'Location', 'best', 'Interpreter','latex');
    grid on;

end

