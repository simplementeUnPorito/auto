function S = sim_compensador_first_order_meas( ...
    Gd, z0, p0, Kc, T, umin, umax, ...
    t_meas, ref_meas, u_meas, y_meas, ...
    opts)
% SIM_COMPENSADOR_FIRST_ORDER_MEAS
% Simula compensador 1er orden + planta discreta usando la referencia medida
% y compara contra esfuerzo/salida medidos del osciloscopio.
%
% Controlador (posición):
%   e(k)    = ref(k) - y(k)
%   u(k)    = p0*u(k-1) + Kc*e(k) - Kc*z0*e(k-1)
%   saturación: u in [umin, umax]
%
% Planta discreta (2º orden):
%   y(k+1) = b0*u(k) + b1*u(k-1) - a1*y(k) - a2*y(k-1)
%
% I/O
%  - Gd: tf discreta (2º orden típico)
%  - z0, p0, Kc, T, umin, umax: parámetros del compensador y muestreo
%  - t_meas: vector tiempo de medición (segundos) [Nx1]
%  - ref_meas, u_meas, y_meas: señales medidas [Nx1] (misma longitud que t_meas)
%  - opts (struct, opcional):
%       .Nini = muestras a descartar al inicio (default 0)
%       .Nfin = muestras a descartar al final  (default 0)
%       .plot = true/false para graficar (default true)
%
% Return: struct S con campos:
%   .td             - tiempo uniforme (0:T:t_end)
%   .refd           - referencia en grilla uniforme
%   .ud_sim, .yd_sim- esfuerzo y salida simulados
%   .ud_meas, .yd_meas - esfuerzo y salida medidos re-sampleados a td
%   .errors.u / .errors.y - métricas (RMSE, MAE, MaxAbs, NRMSE_pct)
%   .coefs          - b0,b1,a1,a2 usados
%
% Nota: Se hace re-sample (interp1) de señales medidas a grilla td uniforme.
%       Se aplica recorte común si opts.Nini/Nfin > 0.

    if nargin < 12
        error('Faltan argumentos. Usa: Gd,z0,p0,Kc,T,umin,umax,t_meas,ref_meas,u_meas,y_meas[,opts]');
    end
    if nargin < 13 || isempty(opts), opts = struct; end
    if ~isfield(opts,'Nini'), opts.Nini = 0; end
    if ~isfield(opts,'Nfin'), opts.Nfin = 0; end
    if ~isfield(opts,'plot'), opts.plot = true; end
    if ~isfield(opts,'demean_errors')
     opts.demean_errors = true;   % valor por defecto
    end

    % --- Validaciones básicas ---
    assert(isa(Gd,'tf') && Gd.Ts > 0, 'Gd debe ser tf discreta con Ts>0.');
    assert(isvector(t_meas) && isvector(ref_meas) && isvector(u_meas) && isvector(y_meas), ...
        't_meas, ref_meas, u_meas, y_meas deben ser vectores.');
    t_meas = t_meas(:); ref_meas = ref_meas(:); u_meas = u_meas(:); y_meas = y_meas(:);
    Nraw = numel(t_meas);
    assert(numel(ref_meas)==Nraw && numel(u_meas)==Nraw && numel(y_meas)==Nraw, ...
        'Todas las señales medidas deben tener igual longitud que t_meas.');

    % --- Normalizar tiempo medido a t0=0 y asegurar monotonía ---
    t_meas = t_meas - t_meas(1);
    [t_meas, uniq_idx] = unique(t_meas, 'stable'); % evitar puntos repetidos
    ref_meas = ref_meas(uniq_idx);
    u_meas   = u_meas(uniq_idx);
    y_meas   = y_meas(uniq_idx);

    % --- Grilla uniforme a paso T desde 0 hasta el tiempo medido final ---
    t_end = t_meas(end);
    N = floor(t_end/T) + 1;
    td = (0:N-1)' * T;

    % --- Re-sample/interpolación a grilla td ---
    refd_m = interp1(t_meas, ref_meas, td, 'linear', 'extrap');
    ud_m   = interp1(t_meas, u_meas,   td, 'linear', 'extrap');
    yd_m   = interp1(t_meas, y_meas,   td, 'linear', 'extrap');

    % --- Coeficientes de la planta discreta (2º orden estándar) ---
    [numD, denD] = tfdata(Gd, 'v');
    % Asegurar longitud 3 (b0,b1,b2) y 3 (a0,a1,a2); normalizar por a0:
    numD = numD(:).';
    denD = denD(:).';
    if numel(numD) < 3, numD = [zeros(1, 3-numel(numD)), numD]; end
    if numel(denD) < 3, denD = [zeros(1, 3-numel(denD)), denD]; end
    a0 = denD(1);
    if abs(a0) < eps, error('Denominador inválido (a0 ~ 0).'); end
    numD = numD / a0;
    denD = denD / a0;

    % Convención del bucle: y(k+1) = b0*u(k) + b1*u(k-1) - a1*y(k) - a2*y(k-1)
    % donde denD = [1 a1 a2], numD = [b? b0 b1] (ajustamos a eso):
    % Aceptamos numD como [b?, b0, b1] (coincide con tu ejemplo original)
    b0 = numD(end-1);
    b1 = numD(end);
    a1 = denD(2);
    a2 = denD(3);

    coefs = struct('b0',b0,'b1',b1,'a1',a1,'a2',a2);

    % --- Recortes opcionales (descartar transitorios de borde) ---
    idx0 = 1 + max(0, opts.Nini);
    idx1 = numel(td) - max(0, opts.Nfin);
    if idx1 <= idx0+2
        error('Recortes Nini/Nfin demasiado agresivos para la longitud de datos.');
    end
    td     = td(idx0:idx1);
    refd_m = refd_m(idx0:idx1);
    ud_m   = ud_m(idx0:idx1);
    yd_m   = yd_m(idx0:idx1);

    % --- Simulación (mismo orden que tu PID), usando ref medida ---
    Nsim = numel(td);
    yd = zeros(Nsim,1);
    ed = zeros(Nsim,1);
    ud = zeros(Nsim,1);

    for k = 3:Nsim-1
        ed(k) = refd_m(k) - yd(k);

        % Controlador (posición)
        u_k = p0*ud(k-1) + Kc*ed(k) - Kc*z0*ed(k-1);

        % Saturación
        if u_k > umax
            ud(k) = umax;
        elseif u_k < umin
            ud(k) = umin;
        else
            ud(k) = u_k;
        end

        % Planta discreta
        yd(k+1) = b0*ud(k) + b1*ud(k-1) - a1*yd(k) - a2*yd(k-1);
    end


if opts.demean_errors
    refd_m = refd_m - mean(refd_m)+2.04;
    ud  = ud  - mean(ud)+2.04;
    yd = yd - mean(yd)+2.04;
    ud_m  = ud_m  - mean(ud_m)+2.04;
    yd_m = yd_m - mean(yd_m)+2.04;
end


    xd = 50;
    L   = numel(td);
    idx = xd:L;
    % --- Errores (sim vs medidos en la misma grilla) ---
    % Alinear longitudes (por si el último punto de y queda sin predecir):
    L = min([numel(ud), numel(ud_m), numel(yd), numel(yd_m)]);
    td      = td(1:L);
    refd    = refd_m(1:L);
    ud_sim  = ud(1:L);
    yd_sim  = yd(1:L);
    ud_meas = ud_m(1:L);
    yd_meas = yd_m(1:L);

    e_u = ud_sim(idx) - ud_meas(idx);
    e_y = yd_sim(idx) - yd_meas(idx);

    errors.u = summarize_errors(e_u, ud_meas);
    errors.y = summarize_errors(e_y, yd_meas);

    % --- Salida ---
    S = struct();
    S.td      = td;
    S.refd    = refd;
    S.ud_sim  = ud_sim;
    S.yd_sim  = yd_sim;
    S.ud_meas = ud_meas;
    S.yd_meas = yd_meas;
    S.errors  = errors;
    S.coefs   = coefs;
    
    % --- Plots comparativos (si se pide) ---
    if opts.plot
    % --- índice de recorte visual (xd debe existir; si no, cae a 1) ---
    if ~exist('xd','var') || isempty(xd) || xd < 1, xd = 1; end
    

    % --- layout con ejes vinculados ---
    figure('Name','Comparación esfuerzo y salida','NumberTitle','off');
    tl = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

    % ===== Subplot 1: esfuerzo =====
    ax1 = nexttile(tl,1);
    plot(td(idx), ud_meas(idx), '-', ...
         td(idx), ud_sim(idx),  '--', 'LineWidth', 1.2);
    grid(ax1,'on'); grid(ax1,'minor');
    ylabel(ax1,'u');
    legend(ax1, 'u_{medida}','u_{sim}','Location','best');
    title(ax1, sprintf('Esfuerzo  |  RMSE=%.4g,  NRMSE=%.2f%%,  Max|e|=%.4g', ...
        errors.u.RMSE, errors.u.NRMSE_pct, errors.u.MaxAbs));

    % ===== Subplot 2: salida + referencia =====
    ax2 = nexttile(tl,2);
    plot(td(idx), yd_meas(idx), '-', ...
         td(idx), yd_sim(idx),  '--', ...
         td(idx), refd(idx),    ':',  'LineWidth', 1.2);
    grid(ax2,'on'); grid(ax2,'minor');
    xlabel(ax2,'t [s]'); ylabel(ax2,'y');
    legend(ax2, 'y_{medida}','y_{sim}','ref','Location','best');
    title(ax2, sprintf('Salida  |  RMSE=%.4g,  NRMSE=%.2f%%,  Max|e|=%.4g', ...
        errors.y.RMSE, errors.y.NRMSE_pct, errors.y.MaxAbs));

    % Título general
    sgtitle(tl,'Comparación medición vs. simulación');
    % Ejes X sincronizados
    linkaxes([ax1 ax2],'x');
end

end

% ==== Helper para métricas ====
function E = summarize_errors(e, yref)
    e = e(:); yref = yref(:);
    RMSE = sqrt(mean(e.^2));
    MAE  = mean(abs(e));
    MaxAbs = max(abs(e));

    % NRMSE respecto al rango de la señal medida (si rango ~0, usar 1 para evitar NaN)
    rng = max(yref) - min(yref);
    if rng < eps, rng = 1; end
    NRMSE_pct = 100 * RMSE / rng;

    E = struct('RMSE', RMSE, 'MAE', MAE, 'MaxAbs', MaxAbs, 'NRMSE_pct', NRMSE_pct);
end
