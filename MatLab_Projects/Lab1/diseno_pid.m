function results = diseno_pid(G, filename)
% diseno_pid - Bucle interactivo para sintonía PID y selección del mejor ensayo
%
% Uso:
%   [Kp, Ti, Td] = diseno_pid(G, 'log_pid.mat', OSd, trd, tsd)
%   Los últimos 3 argumentos son opcionales.
%
% Registra todos los intentos en filename y retorna el mejor setpoint.

    if nargin < 2
        filename = 'log_pid.mat';
    end
    if nargin < 3, OSd = []; end
    if nargin < 4, trd = []; end
    if nargin < 5, tsd = []; end

    % Cargar historial si existe
    results = struct([]);
    if exist(filename,'file')
        S = load(filename);
        if isfield(S,'results'), results = S.results; end
    end
    k_iter = numel(results) + 1;

    % Loop interactivo
    while true
        fprintf('\n--- Parámetros PID ---\n');
        Kp = input('Kp: ');
        Td = input('Td: ');
        Ti = input('Ti: ');

        % Controlador PID
        numC = Kp*[Ti*Td, Ti, 1];
        denC = [Ti, 0];
        C = tf(numC, denC);

        % Lazo cerrado
        cl = feedback(C*G, 1);

        % Métricas
        info = stepinfo(cl);
        OS = info.Overshoot;
        tr = info.RiseTime;
        ts = info.SettlingTime;

        % Simulación
        if isfinite(ts) && ts > 0
            t_end = 1.1*ts;
        else
            t_end = 1;
        end
        t = linspace(0, t_end, 1200);
        [y, tout] = step(cl, t);

        % Plot
        figure(10); clf;
        plot(tout, y, 'LineWidth', 1.5); grid on; hold on;
        yline(1,'k:');
        if isfinite(tr) && tr > 0, xline(tr,'--','t_r'); end
        if isfinite(ts) && ts > 0, xline(ts,':','t_s'); end
        title(sprintf('PID: Kp=%.4g, Ti=%.4g, Td=%.4g | OS=%.2f%%, t_r=%.4gs, t_s=%.4gs', ...
                      Kp, Ti, Td, OS, tr, ts));
        xlabel('Tiempo [s]'); ylabel('Salida');

        % Consola
        fprintf('>> Kp=%.6g, Ti=%.6g, Td=%.6g | OS=%.2f%%, t_r=%.4gs, t_s=%.4gs\n', ...
                Kp, Ti, Td, OS, tr, ts);

        % Guardar log
        results(k_iter).Kp = Kp;
        results(k_iter).Ti = Ti;
        results(k_iter).Td = Td;
        results(k_iter).Overshoot = OS;
        results(k_iter).RiseTime = tr;
        results(k_iter).SettlingTime = ts;
        save(filename,'results');
        k_iter = k_iter + 1;

        % Continuar o salir
        resp = input('¿Continuar? [s/n]: ','s');
        if isempty(resp) || any(lower(resp(1)) == ['n','q'])
            disp('Finalizando loop.');
            break;
        end
    end


end
