function [K_out, C_out, report] = auto_hit_specs(Gw, C_in, K_in, GM_des_dB, PM_des, s)
% AUTO que prioriza: (1) ajustar K ~ GM, (2) sumar LEAD (normalizado) para PM,
% (3) si GM < objetivo, sumar LAG "de ganancia" (NO normalizado) para subir GM,
% (4) si el LAG te bajó PM, agrega un LEAD chiquito para recuperar.
%
% Devuelve K_out, C_out y reporte (GM, PM, Wcg, Wcp, iters).

    tolGM = 0.5;    % dB
    tolPM = 1.0;    % deg
    maxit = 4;

    K = K_in;
    C = C_in;

    for it = 1:maxit
        % --- (A) Ajustar K para GM deseado (aprox directo en dB)
        L = K*C*Gw;
        [GM, PM, ~, ~] = margin(L); GMdB = 20*log10(GM);
        if ~isfinite(GMdB), error('GM no definido; añadí una etapa antes.'); end
        K = K * 10^((GMdB - GM_des_dB)/20);

        % --- (B) LEAD normalizado para alcanzar PM deseada en Wcp actual
        L = K*C*Gw;
        [~, PM, ~, Wcp] = margin(L);
        if ~(isfinite(Wcp) && Wcp>0), error('No hay Wcp tras ajustar K.'); end

        phi_need = PM_des - PM;            % fase a sumar
        if phi_need > 0.5
            phi_apply = min(max(phi_need, 1), 60);   % limitar 1..60°
            C = lead_norm_at_wc(phi_apply, Wcp, s) * C;
        end

        % --- Recalculamos márgenes
        L = K*C*Gw;
        [GM, PM, ~, Wcp] = margin(L); GMdB = 20*log10(GM);

        % --- (C) Si GM quedó por debajo, metemos un LAG de ganancia (NO normalizado)
        if isfinite(GMdB) && (GMdB < GM_des_dB - tolGM)
            dB_needed = GM_des_dB - GMdB;                 % cuánto GM falta
            C = lag_gain_at_wc_db(dB_needed, Wcp, s, 10) * C;  % kfactor=10 => esquinas bien por debajo

            % recuperar PM si el lag bajó demasiado
            L2 = K*C*Gw;
            [~, PM2, ~, Wcp2] = margin(L2);
            if isfinite(PM2) && PM2 < PM_des - tolPM
                phi_fix = min(max(PM_des - PM2, 1), 30);         % LEAD suave de corrección
                C = lead_norm_at_wc(phi_fix, Wcp2, s) * C;
            end
        end

        % --- (D) Chequeo de convergencia
        Lf = K*C*Gw;
        [GMf, PMf, ~, ~] = margin(Lf); GMf_dB = 20*log10(GMf);
        doneGM = isfinite(GMf_dB) && abs(GMf_dB - GM_des_dB) <= tolGM;
        donePM = isfinite(PMf)    && abs(PMf    - PM_des)    <= tolPM;
        if doneGM && donePM, break; end
    end

    K_out = K; C_out = C;
    Lf = K_out*C_out*Gw; [GMf, PMf, Wcgf, Wcpf] = margin(Lf);
    report = struct('GMdB',20*log10(GMf), 'PM',PMf, 'Wcg',Wcgf, 'Wcp',Wcpf, 'iters',it);
end
