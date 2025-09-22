function sim_pid_euler_ideal(G, T, Kp, Ti, Td, stepSize)

    % --- Control analógico (para comparar) ---
    numC = Kp*[Ti*Td, Ti, 1];
    denC = [Ti, 0];
    controlador = tf(numC, denC);
    cloop_c = feedback(controlador*G, 1);

    info  = stepinfo(cloop_c);
    tend  = 3*info.SettlingTime;
    wn    = 1.8/info.RiseTime;
    Tstep = (2*pi/wn)/3000;
    t  = 0:Tstep:tend;
    td = 0:T:tend;

    % Referencia con ruido
    refc = stepSize*ones(size(t)); %+ sigma*randn(size(t));
    yc   = lsim(cloop_c, refc, t);
    refd = interp1(t, refc, td, 'linear', 'extrap');

    % --- Planta discreta ---
    Gd = c2d(G, T, 'zoh');
    [numD, denD] = tfdata(Gd, 'v');
    
    b0 = numD(2);
    b1 = numD(3);
    a1 = denD(2);
    a2 = denD(3);

    
    % Inicialización
    yd = zeros(size(td));
    ed = zeros(size(td));
    ud = zeros(size(td));

    % --- PID discreto incremental ---
    for k = 3:numel(td)-1
        ed(k) = refd(k) - yd(k);

        ud(k) = ud(k-1) + Kp*((1+T/Ti+Td/T)*ed(k) ...
                             -(1+2*Td/T)*ed(k-1) ...
                             +(Td/T)*ed(k-2));

        

        % Planta discreta (dividido por a0 por seguridad)
        yd(k+1) = b0*ud(k) + b1*ud(k-1) - a1*yd(k)-a2*yd(k-1);
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
    title(sprintf('PID Euler: Kp=%.3g, Ti=%.3g, Td=%.3g, f_m=%.3g', Kp, Ti, Td,1/T));
    legend('Referencia','Continuo','Discreto','u_d','Location','best');
    grid on;
end
