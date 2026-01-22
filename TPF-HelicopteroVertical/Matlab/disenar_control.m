function S = disenar_control(G, fs, metodo, varargin)
%DISENAR_CONTROL  Diseña y evalúa controladores digitales SISO (simple y robusto).
%
%   S = disenar_control(G, fs, metodo, Name,Value,...)
%
% Entradas:
%   G      : planta continua SISO (tf/ss/zpk). Si viene discreta, se convierte con d2c(zoh) (warning).
%   fs     : frecuencia de muestreo [Hz]. Ts = 1/fs.
%   metodo : 'zoh' o 'tustin' para discretizar la planta usada en diseño.
%
% Name-Value:
%   'tecnica'          : 'menu' (default) | 'designer' | 'opt_intra' | 'pp_obs' | 'pp_obs_int' | 'dlqr' | 'lqg'
%   'observer'      : 'actual' (default) | 'predictor'        (aplica en pp_obs/pp_obs_int/lqg)
%   'N'             : muestras de simulación (default 400)
%   'subN'          : submuestras por Ts para intersample (default 30)
%   'u_sat'         : [umin umax] saturación (default [-Inf Inf])
%   'designer_mode' : 'rlocus' (default) | 'default'
%   'polesK'        : polos deseados K (si aplica)
%   'polesL'        : polos deseados L (si aplica)
%   'x0'            : valor inicial de la planta
% Salida struct S:
%   .Ts,.fs,.G,.Gd,.Gd_zoh
%   .Cd (si TF) o .ctrl (si estado-espacio)
%   .step, .ramp  (señales, métricas, intersample)
%
% Notas:
% - Simulación inter-muestra siempre usa ZOH real (independiente de 'metodo').
% - Para state feedback (PP/DLQR/LQG) se calcula Kr para tracking unitario.

% =================== Parse ===================
ip = inputParser;

ip.addParameter('polesK',[],@(v)isnumeric(v) && (isempty(v) || isvector(v)));
ip.addParameter('polesL',[],@(v)isnumeric(v) && (isempty(v) || isvector(v)));

ip.addParameter('tecnica','menu',@(s)ischar(s)||isstring(s));
ip.addParameter('observer','actual',@(s)ischar(s)||isstring(s));
ip.addParameter('N',400,@(x)isnumeric(x)&&isscalar(x)&&x>=10);
ip.addParameter('subN',30,@(x)isnumeric(x)&&isscalar(x)&&x>=2);
ip.addParameter('u_sat',[-Inf Inf],@(v)isnumeric(v)&&numel(v)==2&&v(1)<v(2));
ip.addParameter('designer_mode','rlocus',@(s)ischar(s)||isstring(s));
% --- LQR / LQG weights (opcionales) ---
ip.addParameter('Q',[],@(v)isnumeric(v) && (isempty(v) || ismatrix(v)));
ip.addParameter('R',[],@(v)isnumeric(v) && (isempty(v) || (isscalar(v) || ismatrix(v))));
ip.addParameter('Qn',[],@(v)isnumeric(v) && (isempty(v) || ismatrix(v)));
ip.addParameter('Rn',[],@(v)isnumeric(v) && (isempty(v) || (isscalar(v) || ismatrix(v))));
ip.addParameter('Gk',[],@(v)isnumeric(v) && (isempty(v) || ismatrix(v)));

ip.addParameter('x0',[],@(v)isnumeric(v) && (isempty(v) || isvector(v)));

ip.parse(varargin{:});
P = ip.Results;

tecnica = lower(string(P.tecnica));
tecnica = validatestring(tecnica, {'menu','designer','opt_intra','pp_obs','pp_obs_int','dlqr','dlqr_int','lqg','lqg_int'});

obsType = lower(string(P.observer));
obsType = validatestring(obsType, {'actual','predictor'});

designer_mode = lower(string(P.designer_mode));
designer_mode = validatestring(designer_mode, {'rlocus','default'});

polesK_user   = P.polesK;
polesL_user   = P.polesL;

Q_user  = P.Q;
R_user  = P.R;
Qn_user = P.Qn;
Rn_user = P.Rn;
Gk_user = P.Gk;

x0_user = P.x0;

N     = P.N;
subN  = P.subN;
u_sat = P.u_sat(:).';

if fs <= 0, error('fs debe ser > 0 (Hz).'); end
Ts = 1/fs;

% =================== Normalizar planta ===================
G = normalize_plant_local(G);

if ~is_siso_local(G)
    error('G debe ser SISO.');
end

% Discretizaciones
metodo = lower(string(metodo));
metodo = validatestring(metodo, {'zoh','tustin'});
Gd     = c2d(G, Ts, char(metodo));
Gd_zoh = c2d(G, Ts, 'zoh');   % útil para RLocus y para "actuador real"

% struct salida base
S = struct();
S.fs    = fs;
S.Ts    = Ts;
S.G     = G;
S.Gd    = Gd;
S.Gd_zoh= Gd_zoh;

S.Cd    = [];     % si controlador TF
S.ctrl  = [];     % si controlador estado-espacio
S.CL    = [];     % lazo cerrado lineal (cuando se arma)

S.step  = [];
S.ramp  = [];
S.note  = "";

% =================== Menú si corresponde ===================
if tecnica == "menu"
    k = menu('Elegí acción',...
        'Abrir Control System Designer (planta discreta)',...
        'Óptimo con oscilaciones intramuestra: C = inv(Gd) * 1/(z-1)',...
        'Polos + observador (sin integrador, con Kr)',...
        'Polos + observador (integrador clásico, con Kr opcional)',...
        'DLQR (estado medido, con Kr)',...
        'LQG (DLQR + Kalman, con Kr)');
    map = ["designer","opt_intra","pp_obs","pp_obs_int","dlqr","lqg"];
    tecnica = map(k);
end

% =================== Ejecutar opción ===================
switch tecnica
    case "designer"
        sys_open = Gd_zoh; % para RLocus suele tener más sentido con ZOH
        if strcmp(designer_mode, "default")
            controlSystemDesigner(sys_open);
        else
            controlSystemDesigner('rlocus', sys_open);
        end
        S.note = "Control System Designer abierto con Gd_zoh. Exportá el compensador si querés analizarlo luego.";
        return;

    case "opt_intra"
        % C = inv(Gd) * 1/(z-1)
        I  = tf([0 1],[1 -1],Ts);  % z^-1/(1-z^-1) equivalente a 1/(z-1)
        Cd = inv(tf(Gd)) * I;

        if ~isproper(Cd)
            warning('Cd salió impropio/no causal. Puede explotar u[k]. Sigo igual (como pediste).');
        end

        S.Cd = Cd;
        S = analizar_y_simular_tf(S, Cd, u_sat, N, subN);

    case "pp_obs"
        % Polos + observador (sin integrador) con Kr
        [Ad,Bd,Cd_,Dd_] = ssdata(ss(Gd));
        n = size(Ad,1);

        polesK = get_poles_param(polesK_user, n, default_poles(n,0.8,0.25), 'polesK');
        polesL = get_poles_param(polesL_user, n, default_poles(n,0.4,0.4), 'polesL');

        K  = acker(Ad,Bd,polesK(:).');
        if strcmp(obsType,'predictor')
            L  = acker(Ad',Cd_',polesL(:).')';
        else
            L  = acker(Ad',(Cd_*Ad)',polesL(:).')';
        end
        Kr = calcular_Kr(Ad,Bd,Cd_,Dd_,K,Ts);

        S.ctrl = struct('type','pp_obs','K',K,'L',L,'Kr',Kr,'observer',obsType,'integrator',false);
        S = sim_estado_espacio(S, Ad,Bd,Cd_,Dd_, K,L,Kr, obsType, false, [],x0_user ,u_sat, N, subN);

    case "pp_obs_int"
        % Integrador clásico v[k+1] = v[k] + (r - y)
        % Control: u = Kr*r - K*xhat - Ki*v
        [Ad,Bd,Cd_,Dd_] = ssdata(ss(Gd));
        n = size(Ad,1);

        polesAug = get_poles_param(polesK_user, n+1, [default_poles(n,0.8,0.25); 0.6], 'polesAug');
        polesL   = get_poles_param(polesL_user,   n,   default_poles(n,0.4,0.4), 'polesL');

        % Aaug, Baug para integrador de error (SISO)
        Aaug = [Ad zeros(n,1);
               -Cd_ 1];
        Baug = [Bd; 0];

        Kaug = acker(Aaug,Baug,polesAug(:).');
        K  = Kaug(1:n);
        Ki = Kaug(n+1);

        if strcmp(obsType,'predictor')
            L  = acker(Ad',Cd_',polesL(:).')';
        else
            L  = acker(Ad',(Cd_*Ad)',polesL(:).')';
        end

        % Kr "opcional": por default lo calculo para ayudar al tracking/ganancia
        Kr = 1;

        S.ctrl = struct('type','pp_obs_int','K',K,'Ki',Ki,'L',L,'Kr',Kr,'observer',obsType,'integrator',true);
        S = sim_estado_espacio(S, Ad,Bd,Cd_,Dd_, K,L,Kr, obsType, true, Ki,x0_user, u_sat, N, subN);

    case "dlqr"
        [Ad,Bd,Cd_,Dd_] = ssdata(ss(Gd));
        n = size(Ad,1);

        m = size(Bd,2); % en SISO es 1
        Q = get_Q_param(Q_user, n, 'Q');
        R = get_R_param(R_user, m, 'R');


        K  = dlqr(Ad,Bd,Q,R);
        Kr = calcular_Kr(Ad,Bd,Cd_,Dd_,K,Ts);

        S.ctrl = struct('type','dlqr','K',K,'Kr',Kr,'Q',Q,'R',R,'observer',"measured",'integrator',false);
        S = sim_estado_espacio_puro(S, Ad,Bd,Cd_,Dd_, K,Kr,x0_user, u_sat, N, subN);

   case "lqg"
        [Ad,Bd,Cd_,Dd_] = ssdata(ss(Gd));
        n = size(Ad,1);
        m = size(Bd,2);
    
        % --- pesos DLQR (como ya habíamos parametrizado) ---
        Q  = get_Q_param(Q_user,  n, 'Q');
        R  = get_R_param(R_user,  m, 'R');
    
        K  = dlqr(Ad,Bd,Q,R);
        Kr = calcular_Kr(Ad,Bd,Cd_,Dd_,K,Ts);
    
        % --- covarianzas del estimador (DISCRETAS) ---
        if isempty(Qn_user), Qn = 1e-3*eye(n); else, Qn = Qn_user; end
        if isempty(Rn_user), Rn = 1e-3;        else, Rn = Rn_user; end
    
        % validar tamaños
        if ~isequal(size(Qn), [n n])
            error('Qn debe ser %dx%d (recibí %dx%d).', n, n, size(Qn,1), size(Qn,2));
        end
    
        ny = size(Cd_,1);
        if isscalar(Rn)
            Rn = Rn * eye(ny);
        end
        if ~isequal(size(Rn), [ny ny])
            error('Rn debe ser %dx%d o escalar (recibí %dx%d).', ny, ny, size(Rn,1), size(Rn,2));
        end
    
        % --- Gk (matriz de ruido de proceso) ---
        if isempty(Gk_user)
            Gk = eye(n);
        else
            Gk = Gk_user;
        end
        if ~isequal(size(Gk), [n n])
            error('Gk debe ser %dx%d (recibí %dx%d).', n, n, size(Gk,1), size(Gk,2));
        end
    
        % --- estimador Kalman estacionario via DLQE ---
        % dlqe devuelve L tal que: xhat[k+1] = A xhat + B u + L (y - C xhat - D u)
        [Lk,~,~] = dlqe(Ad, Gk, Cd_, Qn, Rn);
    
        S.ctrl = struct('type','lqg','K',K,'Kr',Kr,'L',Lk,'Q',Q,'R',R,'Qn',Qn,'Rn',Rn,'Gk',Gk, ...
                        'observer',obsType,'integrator',false);
    
        S = sim_estado_espacio(S, Ad,Bd,Cd_,Dd_, K,Lk,Kr, obsType, false, [],x0_user, u_sat, N, subN);
    case "dlqr_int"
        % DLQR con integrador clásico: v[k+1] = v[k] + (r - y)
        % u = Kr*r - K*x - Ki*v   (acá asumimos estado medido)
        [Ad,Bd,Cd_,Dd_] = ssdata(ss(Gd));
        n = size(Ad,1);
        m = size(Bd,2); % SISO => 1
    
        % Sistema aumentado
        Aaug = [Ad zeros(n,1);
               -Cd_ 1];
        Baug = [Bd; 0];
    
        % Pesos (notá que Q ahora puede ser nxn o (n+1)x(n+1))
        Qaug = get_Qaug_param(Q_user, n, 'Q');
        R    = get_R_param(R_user,  m, 'R');
    
        Kaug = dlqr(Aaug, Baug, Qaug, R);
    
        K  = Kaug(1:n);
        Ki = Kaug(n+1);
    
        % Con integrador, Kr no es necesario (tracking lo fuerza v)
        Kr = 1;
    
        S.ctrl = struct('type','dlqr_int','K',K,'Ki',Ki,'Kr',Kr,'Q',Qaug,'R',R, ...
                        'observer',"measured",'integrator',true);
    
        % usamos sim_estado_espacio_puro? No, porque acá hay integrador en la ley.
        % Reutilizamos sim_estado_espacio con L=[], obsType ignorado, useInt=true.
        S = sim_estado_espacio(S, Ad,Bd,Cd_,Dd_, K,[],Kr, obsType, true, Ki,x0_user, u_sat, N, subN);
    

    case "lqg_int"
        % LQG con integrador:
        % - Control: DLQR sobre (Aaug,Baug) -> K y Ki
        % - Estimador: Kalman sobre (Ad,Cd) (no estimamos v; v es conocido por r-y)
        [Ad,Bd,Cd_,Dd_] = ssdata(ss(Gd));
        n = size(Ad,1);
        m = size(Bd,2);
    
        % Aumentado para control
        Aaug = [Ad zeros(n,1);
               -Cd_ 1];
        Baug = [Bd; 0];
    
        Qaug = get_Qaug_param(Q_user, n, 'Q');
        R    = get_R_param(R_user,  m, 'R');
    
        Kaug = dlqr(Aaug, Baug, Qaug, R);
        K  = Kaug(1:n);
        Ki = Kaug(n+1);
    
        Kr = 1;
    
        % --- Kalman (DISCRETO) sobre la planta ---
        if isempty(Qn_user), Qn = 1e-3*eye(n); else, Qn = Qn_user; end
        if isempty(Rn_user), Rn = 1e-3;        else, Rn = Rn_user; end
    
        if ~isequal(size(Qn), [n n])
            error('Qn debe ser %dx%d (recibí %dx%d).', n, n, size(Qn,1), size(Qn,2));
        end
    
        ny = size(Cd_,1);
        if isscalar(Rn)
            Rn = Rn * eye(ny);
        end
        if ~isequal(size(Rn), [ny ny])
            error('Rn debe ser %dx%d o escalar (recibí %dx%d).', ny, ny, size(Rn,1), size(Rn,2));
        end
    
        if isempty(Gk_user)
            Gk = eye(n);
        else
            Gk = Gk_user;
        end
        if ~isequal(size(Gk), [n n])
            error('Gk debe ser %dx%d (recibí %dx%d).', n, n, size(Gk,1), size(Gk,2));
        end
    
        [Lk,~,~] = dlqe(Ad, Gk, Cd_, Qn, Rn);
    
        S.ctrl = struct('type','lqg_int','K',K,'Ki',Ki,'Kr',Kr,'L',Lk, ...
                        'Q',Qaug,'R',R,'Qn',Qn,'Rn',Rn,'Gk',Gk, ...
                        'observer',obsType,'integrator',true);
    
        S = sim_estado_espacio(S, Ad,Bd,Cd_,Dd_, K,Lk,Kr, obsType, true, Ki,x0_user, u_sat, N, subN);

        otherwise
            error('tecnica no reconocido: %s', tecnica);
end

end % ===== end main =====


% =====================================================================
%                             Helpers
% =====================================================================

function G = normalize_plant_local(G)
    if ~(isa(G,'tf') || isa(G,'ss') || isa(G,'zpk'))
        error('G debe ser tf/ss/zpk.');
    end

    % Si es discreta, la convierto a continua (ZOH) para mantener el contrato "G continua"
    if is_dt_local(G)
        warning('G parece discreta (Ts ~= 0). La convierto a continua con d2c(...,''zoh'').');
        G = d2c(G,'zoh');
    end

    % Forzar representación ss continua
    G = ss(G);
end

function tf_siso = is_siso_local(sys)
    [ny,nu] = size(sys);
    tf_siso = (ny==1)&&(nu==1);
end

function tf_dt = is_dt_local(sys)
    % Usa isdt si existe; si no, cae a Ts
    if exist('isdt','file') == 2
        tf_dt = isdt(sys);
    else
        tf_dt = isprop(sys,'Ts') && (sys.Ts ~= 0);
    end
end



function poles = default_poles(n, a, b)
    poles = [];
    while numel(poles) < n
        if numel(poles) <= n-2
            poles = [poles; a+1i*b; a-1i*b]; %#ok<AGROW>
        else
            poles = [poles; a]; %#ok<AGROW>
        end
    end
    poles = poles(1:n);
end

function Kr = calcular_Kr(A,B,C,D,K,Ts)
    % Kr = 1 / dcgain( y/u ) del sistema: x+ = (A-BK)x + B u, y = Cx + Du
    sys_u2y = ss(A - B*K, B, C, D, Ts);
    g0 = dcgain(sys_u2y);  % DC gain en z=1
    if isempty(g0) || ~isfinite(g0) || abs(g0) < 1e-12
        warning('dcgain(sys_u2y) inválido o ~0. Pongo Kr=1.');
        Kr = 1;
    else
        Kr = 1/g0;
    end
end

function poles = get_poles_param(userVal, n, defaultVal, name)
    if isempty(userVal)
        poles = defaultVal(:);
    else
        poles = userVal(:);
    end
    if numel(poles) ~= n
        error('%s debe tener %d polos (recibí %d).', name, n, numel(poles));
    end
end

function print_pz(sys, name)
    try
        sysz = zpk(sys);
        p = pole(sysz);
        z = zero(sysz);
    catch
        p = pole(sys);
        z = zero(sys);
    end

    fprintf('\n--- %s ---\n', name);
    fprintf('Polos (%d):\n', numel(p)); disp(p);
    fprintf('Ceros (%d):\n', numel(z)); disp(z);
end



% =====================================================================
%                         TF controller sim
% =====================================================================

function S = analizar_y_simular_tf(S, Cd, u_sat, N, subN)
    Ts = S.Ts;

    % asegurar Cd como ss discreto
    try
        Cd_ss = ss(Cd);
    catch
        error('No se pudo convertir Cd a ss.');
    end
    if Cd_ss.Ts == 0
        Cd_ss = c2d(Cd_ss, Ts, 'tustin');
    end

    % CL lineal (sin saturación)
    try
        S.CL = feedback(series(tf(Cd), tf(S.Gd)), 1);
    catch
        S.CL = [];
    end
    if ~isempty(S.CL), print_pz(S.CL, 'Lazo cerrado lineal (TF)'); end

    % Root locus (solo si Cd es TF usable)
    try
        figure('Name','Root Locus (TF Controller): L(z)=C(z)G(z)'); grid on;
        rlocus(series(tf(Cd), tf(S.Gd_zoh)));
        title('Lugar de raíces usando Gd\_zoh');
    catch
    end

    % STEP
    r_step = ones(N,1);
    R = sim_loop_tf(ss(S.Gd), Cd_ss, Ts, r_step, u_sat);
    S.step = paquete_step(S.G, Ts, r_step, R, subN);

    % RAMP
    r_ramp = (0:N-1)'*Ts;
    R2 = sim_loop_tf(ss(S.Gd), Cd_ss, Ts, r_ramp, u_sat);
    S.ramp = paquete_ramp(S.G, Ts, r_ramp, R2, subN);

    print_pz(S.Gd_zoh, 'Planta discreta Gd\_zoh');
end


% =====================================================================
%                   State-space controllers sim
% =====================================================================

function S = sim_estado_espacio(S, A,B,C,D, K,L,Kr, obsType, useInt, Ki,x0_user, u_sat, N, subN)
    Ts = S.Ts;

        % Polos (lazo cerrado y observador) - plano z
    try
        figure('Name','Plano-z: polos lazo cerrado y observador'); clf;
        hold on; grid on; axis equal;
        % --- zgrid (si existe) ---
        try
            zgrid;  % líneas de zeta y wn mapeadas al plano-z
        catch
        end
        % Circulo unidad (estabilidad discreta)
        %th = linspace(0, 2*pi, 600);
        %plot(cos(th), sin(th), 'k--', 'LineWidth', 1.0);

        n = size(A,1);

        % --- Polos del lazo cerrado ---
        if useInt
            % Sistema aumentado por integrador (coherente con tu diseño en pp_obs_int)
            % x_aug = [x; v],   v[k+1] = v[k] + (r - y)  -> dinámica interna para pole placement
            Aaug = [A zeros(n,1);
                   -C 1];
            Baug = [B; 0];

            if isempty(Ki), Ki = 0; end
            Kaug = [K(:).' Ki];

            p_cl = eig(Aaug - Baug*Kaug);

            plot(real(p_cl), imag(p_cl), 'x', 'LineWidth', 1.8, 'MarkerSize', 9);
            ttl = 'Polos lazo cerrado: eig(A_{aug}-B_{aug}K_{aug})';
        else
            p_cl = eig(A - B*K);

            plot(real(p_cl), imag(p_cl), 'x', 'LineWidth', 1.8, 'MarkerSize', 9);
            ttl = 'Polos lazo cerrado: eig(A-BK)';
        end

        % --- Polos del observador ---
        if ~isempty(L)
            if strcmp(obsType,'predictor')
                p_obs = eig(A - L*C);
            else
                p_obs = eig(A - L*C*A);
            end
                
            plot(real(p_obs), imag(p_obs), 'o', 'LineWidth', 1.6, 'MarkerSize', 7);
            %legend({'Círculo unidad', 'Polos lazo cerrado', 'Polos observador (A-LC)'}, ...
            %       'Location','best');
        else
            %legend({'Círculo unidad', 'Polos lazo cerrado'}, 'Location','best');
        end
        if strcmp(obsType,'predictor')
            title({ttl, 'y polos del observador: eig(A-LC)'});
        else
            title({ttl, 'y polos del observador: eig(A-LAC)'});
        end
        
        xlabel('Re\{z\}');
        ylabel('Im\{z\}');

    catch
    end


    % STEP
    r_step = ones(N,1);
    R = sim_obs(A,B,C,D,K,L,Kr,obsType,useInt,Ki,Ts,r_step,u_sat,x0_user);
    S.step = paquete_step(S.G, Ts, r_step, R, subN);

    % RAMP
    r_ramp = (0:N-1)'*Ts;
    R2 = sim_obs(A,B,C,D,K,L,Kr,obsType,useInt,Ki,Ts,r_ramp,u_sat,x0_user);
    S.ramp = paquete_ramp(S.G, Ts, r_ramp, R2, subN);

    % CL lineal aproximado (sin saturación, ignorando dinámica del estimador)
    try
        S.CL = ss(A - B*K, B*Kr, C, D, Ts);
    catch
        S.CL = [];
    end
    if ~isempty(S.CL), print_pz(S.CL, 'Lazo cerrado lineal (SS aprox)'); end
end

function S = sim_estado_espacio_puro(S, A,B,C,D, K,Kr,x0_user, u_sat, N, subN)
    Ts = S.Ts;

    % STEP
    r_step = ones(N,1);
    R = sim_puro(A,B,C,D,K,Kr,Ts,r_step,u_sat,x0_user);
    S.step = paquete_step(S.G, Ts, r_step, R, subN);

    % RAMP
    r_ramp = (0:N-1)'*Ts;
    R2 = sim_puro(A,B,C,D,K,Kr,Ts,r_ramp,u_sat,x0_user);
    S.ramp = paquete_ramp(S.G, Ts, r_ramp, R2, subN);

    try
        S.CL = ss(A - B*K, B*Kr, C, D, Ts);
    catch
        S.CL = [];
    end
end


% =====================================================================
%                         Result packaging
% =====================================================================

function P = paquete_step(Gc, Ts, r, R, subN)
    t = (0:numel(r)-1)'*Ts;

    y   = R.y;
    ess = abs(1 - y(end));

    info = [];
    try
        info = stepinfo(y, t, 1);
    catch
    end

    figure('Name','Step (discreto)'); tiledlayout(4,1);


    nexttile; grid on; hold on;grid minor;
    stairs(t, r, 'LineWidth',1.0);
    stairs(t, y, 'LineWidth',1.2);
    legend('r[k]','y[k]','Location','best');
    title('Step: salida discreta');

    nexttile; grid on; hold on;grid minor;
    stairs(t, R.u, 'LineWidth',1.2);
    title('u[k] (con saturación)'); ylabel('u');
    
    nexttile; grid on; hold on; grid minor;
    e = r - y;
    stairs(t, e, 'LineWidth',1.2);
    title('Error e[k]=r[k]-y[k]'); ylabel('e');

    nexttile; grid on; hold on;grid minor;
    if ~isempty(R.x)
        stairs(t, R.x.', 'LineWidth',1.0);
        if ~isempty(R.xhat)
            stairs(t, R.xhat.','--','LineWidth',1.0);
            title('x (línea) y x\_hat (punteada)');
        else
            title('x');
        end
    else
        plot(t, y, 'LineWidth',1.0);
        title('y[k]');
    end

    % intersample desde u[k] (ZOH real)
    Si = ver_intersample_desde_u(Gc,y,r, Ts, R.u, 0, subN);

    P = struct();
    P.t          = t;
    P.r          = r;
    P.y          = y;
    P.u          = R.u;
    P.x          = R.x;
    P.xhat       = R.xhat;
    P.v          = R.v;
    P.stepinfo   = info;
    P.ess        = ess;
    P.intersample= Si;
end

function P = paquete_ramp(Gc, Ts, r, R, subN)
    t = (0:numel(r)-1)'*Ts;

    y = R.y;
    e = r - y;
    ess = abs(e(end));

    figure('Name','Rampa (discreto)'); tiledlayout(4,1);


    nexttile; grid on; hold on;grid minor;
    stairs(t, r, 'LineWidth',1.0);
    stairs(t, y, 'LineWidth',1.2);
    legend('r[k]','y[k]','Location','best');
    title('Rampa: salida discreta');

    nexttile; grid on; hold on;grid minor;
    stairs(t, R.u, 'LineWidth',1.2);
    title('u[k] (con saturación)'); ylabel('u');

    nexttile; grid on; hold on;grid minor;
    stairs(t, e, 'LineWidth',1.2);
    title('Error e[k]=r[k]-y[k]'); ylabel('e');
    

    nexttile; grid on; hold on; grid minor;
    if ~isempty(R.x)
        stairs(t, R.x.', 'LineWidth',1.0);
        title('Estados x[k] (discreto)'); ylabel('x');
    else
        stairs(t, y, 'LineWidth',1.0);
        title('y[k]'); ylabel('y');
    end
    xlabel('t [s]');

    % intersample desde u[k] (ZOH real)
    Si = ver_intersample_desde_u(Gc,y,r, Ts, R.u, 0, subN);

    P = struct();
    P.t          = t;
    P.r          = r;
    P.y          = y;
    P.u          = R.u;
    P.x          = R.x;
    P.xhat       = R.xhat;
    P.v          = R.v;
    P.stepinfo   = [];   % stepinfo no aplica bien en rampa
    P.ess        = ess;
    P.intersample= Si;
end


% =====================================================================
%                           Sim cores
% =====================================================================

function R = sim_loop_tf(Gd_ss, Cd_ss, Ts, r, u_sat) %#ok<INUSD>
    [Ap,Bp,Cp,Dp] = ssdata(Gd_ss);
    [Ac,Bc,Cc,Dc] = ssdata(Cd_ss);

    N  = numel(r);
    nxp = size(Ap,1);
    nxc = size(Ac,1);

    xp = zeros(nxp, N);
    xc = zeros(nxc, N);

    y = zeros(N,1);
    u = zeros(N,1);

    for k=1:N-1
        y(k) = Cp*xp(:,k) + Dp*u(k);
        e    = r(k) - y(k);

        uc = Cc*xc(:,k) + Dc*e;
        xc(:,k+1) = Ac*xc(:,k) + Bc*e;

        u(k) = sat(uc, u_sat);
        xp(:,k+1) = Ap*xp(:,k) + Bp*u(k);
    end
    y(N) = Cp*xp(:,N) + Dp*u(N);

    R = struct('u',u,'y',y,'x',xp,'xhat',[],'v',[]);
end

function R = sim_puro(A,B,C,D,K,Kr,Ts,r,u_sat,x0_user) %#ok<INUSD>
    N  = numel(r);
    nx = size(A,1);
    nx = size(A,1);

    if isempty(x0_user)
        x0 = zeros(nx,1);
    else
        x0 = x0_user(:);
        if numel(x0) ~= nx
            error('x0 debe tener %d elementos (recibí %d).', nx, numel(x0));
        end
    end


    x = zeros(nx,N);
    x(:,1)    = x0;

    y = zeros(N,1);
    u = zeros(N,1);

    for k=1:N-1
        y(k) = C*x(:,k) + D*u(k);

        uc   = Kr*r(k) - K*x(:,k);
        u(k) = sat(uc,u_sat);

        x(:,k+1) = A*x(:,k) + B*u(k);
    end
    y(N) = C*x(:,N) + D*u(N);

    R = struct('u',u,'y',y,'x',x,'xhat',x,'v',[]);
end

function R = sim_obs(A,B,C,D,K,L,Kr,obsType,useInt,Ki,Ts,r,u_sat,x0_user) %#ok<INUSD>
    N  = numel(r);
    nx = size(A,1);
    if isempty(x0_user)
        x0 = zeros(nx,1);
    else
        x0 = x0_user(:);
        if numel(x0) ~= nx
            error('x0 debe tener %d elementos (recibí %d).', nx, numel(x0));
        end
    end
    useObs = ~isempty(L);   % <-- NUEVO: si no hay L, estado medido

    x    = zeros(nx,N);
    x(:,1)    = x0;

    xhat = zeros(nx,N);

    y    = zeros(N,1);
    u    = zeros(N,1);
    v    = zeros(1,N); % integrador

    for k=1:N-1
        % salida actual
        y(k) = C*x(:,k) + D*u(k);
        % --- NUEVO: si no hay observador, usar estado medido ---
        if ~useObs
            xhat(:,k) = x(:,k);
        end
        % integrador clásico: v[k+1] = v[k] + (r - y)
        if useInt
            v(k+1) = v(k) + (r(k) - y(k));
        end

        % ley de control
        if ~useInt
            uc = Kr*r(k) - K*xhat(:,k);
        else
            uc = Kr*r(k) - K*xhat(:,k) - Ki*v(k);
        end

        u(k) = sat(uc, u_sat);

        % evoluciona planta
        x(:,k+1) = A*x(:,k) + B*u(k);

        if useObs
            if strcmp(obsType,"predictor")
                innov = y(k) - (C*xhat(:,k) + D*u(k));
                xhat(:,k+1) = A*xhat(:,k) + B*u(k) + L*innov;
            else
                xpred = A*xhat(:,k) + B*u(k);
                ykp1  = C*x(:,k+1) + D*u(k);
                innov = ykp1 - (C*xpred + D*u(k));
                xhat(:,k+1) = xpred + L*innov;
            end
        else
            % estado medido
            xhat(:,k+1) = x(:,k+1);
        end

    end
    y(N) = C*x(:,N) + D*u(N);

    R = struct('u',u,'y',y,'x',x,'xhat',xhat,'v',v);
end

function u = sat(u, lim)
    u = min(max(u, lim(1)), lim(2));
end


% =====================================================================
%                    Intersample (ZOH real)
% =====================================================================

function S = ver_intersample_desde_u(G, yk_disc, rk_disc, Ts, ud, M, N)
    % VER_INTERSAMPLE_DESDE_U
    % Reconstruye y_c(t) aplicando ZOH sobre u[k] y simula la planta continua G.
    % Además grafica y[k] (discreta) y r[k] sobre la misma figura.
    %
    % Uso:
    %   S = ver_intersample_desde_u(G, yk, rk, Ts, u, M, N)
    %
    % Entradas:
    %   G       : planta continua
    %   yk_disc : y[k] discreta (del simulador)
    %   rk_disc : r[k] discreta (referencia usada)
    %   Ts      : periodo de muestreo
    %   ud      : u[k] discreta aplicada
    %   M       : si >0, recorta a M muestras (incluye hasta ese k)
    %   N       : submuestras por Ts para reconstrucción ZOH
    %
    % Nota importante:
    %   Para evitar el "drop" artificial, NO usamos el último u(N) (a menudo queda
    %   sin asignar en simulaciones que corren k=1:N-1). Se usa u(1:end-1).

    if nargin < 7 || isempty(N), N = 30; end
    if nargin < 6 || isempty(M), M = 0;  end
    if Ts <= 0, error('Ts debe ser > 0.'); end

    % --- asegurar vectores columna ---
    uk = ud(:);
    yk = yk_disc(:);
    rk = rk_disc(:);

    % --- recorte coherente ---
    L = min([numel(uk), numel(yk), numel(rk)]);
    uk = uk(1:L);
    yk = yk(1:L);
    rk = rk(1:L);

    if M > 0
        M = min(M, L);
        uk = uk(1:M);
        yk = yk(1:M);
        rk = rk(1:M);
        L  = M;
    end

    % --- EVITAR DROP: usar solo intervalos reales (u[1..L-1]) ---
    if L < 2
        error('Necesito al menos 2 muestras para intersample.');
    end
    uk_int = uk(1:end-1);          % u[k] que define los intervalos
    K      = numel(uk_int) - 1;    % K intervalos -> (K+1) muestras en tk
    tk     = (0:K).' * Ts;         % tiempos de las muestras y[k]

    % --- ZOH (alta resolución) ---
    [tc, uc] = zoh_stretch(uk_int, Ts, N);
    yc       = lsim(G, uc, tc);

    % y[k] "muestrado" de la reconstrucción continua (para referencia)
    yk_from_cont = yc(1:N:end);

    figure('Name','Intersample desde u[k]');
    tiledlayout(2,1);

    % ---- salida ----
    nexttile; hold on; grid on; grid minor;
    plot(tc*1e3, yc, 'LineWidth', 1.2);                 % y_c(t)
    stairs(tk*1e3, yk(1:numel(tk)), 'k.', 'LineWidth', 1.0); % y[k] discreta real
    stairs(tk*1e3, rk(1:numel(tk)), '--', 'LineWidth', 1.0); % r[k]
    xlabel('t [ms]'); ylabel('y');
    title(sprintf('Salida continua y_c(t) + y[k] + r[k]  (T_s = %.4g ms)', Ts*1e3));
    legend('y_c(t)','y[k]','r[k]','Location','best');

    % ---- entrada ----
    nexttile; hold on; grid on; grid minor;
    stairs(tk*1e3, uk_int, 'LineWidth', 1.2);
    xlabel('t [ms]'); ylabel('u[k]');
    title('Esfuerzo aplicado (ZOH)');

    S = struct('Ts', Ts, ...
               'tk', tk, ...
               'uk', uk_int, ...
               'yk_disc', yk(1:numel(tk)), ...
               'rk_disc', rk(1:numel(tk)), ...
               'yk_from_cont', yk_from_cont, ...
               'tc', tc, 'uc', uc, 'yc', yc);
end


function [t_hi, u_hi] = zoh_stretch(u_k, T, M)
    if size(u_k,2) > 1, u_k = u_k(:); end
    u_hi = repelem(u_k, M);
    t_hi = (0:numel(u_hi)-1).' * (T/M);
end
function Q = get_Q_param(userVal, n, name)
    if isempty(userVal)
        Q = eye(n);
    else
        Q = userVal;
    end
    if ~isequal(size(Q), [n n])
        error('%s debe ser %dx%d (recibí %dx%d).', name, n, n, size(Q,1), size(Q,2));
    end
end

function R = get_R_param(userVal, m, name)
    if isempty(userVal)
        R = eye(m); % para SISO -> 1
    else
        R = userVal;
    end
    % aceptar escalar para SISO
    if isscalar(R)
        if m ~= 1
            R = R * eye(m);
        end
    end
    if ~isequal(size(R), [m m])
        error('%s debe ser %dx%d o escalar (recibí %dx%d).', name, m, m, size(R,1), size(R,2));
    end
end
function Qaug = get_Qaug_param(Q_user, n, name)
    % Acepta:
    % - [] => default: blkdiag(eye(n), 1)
    % - Q nxn => blkdiag(Q, qI=1)
    % - Q (n+1)x(n+1) => usar directo

    if isempty(Q_user)
        Qaug = blkdiag(eye(n), 1);
        return;
    end

    Q = Q_user;

    if isequal(size(Q), [n n])
        qI = 1; % peso default del integrador
        Qaug = blkdiag(Q, qI);
    elseif isequal(size(Q), [n+1 n+1])
        Qaug = Q;
    else
        error('%s debe ser %dx%d o %dx%d (recibí %dx%d).', ...
              name, n, n, n+1, n+1, size(Q,1), size(Q,2));
    end
end
