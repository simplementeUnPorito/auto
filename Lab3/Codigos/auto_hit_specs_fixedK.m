function [K_out, C_out, report] = auto_hit_specs_fixedK(Gw, C_in, K_fixed, PM_des, s)
% No toca K. Ajusta PM usando:
%  (1) Lead normalizado en el Wcp actual para sumar fase hasta PM_des
%  (2) Si por el lead se movió GM y querés recuperar un poquito de fase,
%      mete un lead suavito extra (<=20°). (Lag normalizado opcional si querés bajar fase)
    tolPM = 1.0;    % deg
    maxit = 4;

    K = K_fixed;
    C = C_in;

    for it = 1:maxit
        L = K*C*Gw;
        [~, PM, ~, Wcp] = margin(L);
        if ~(isfinite(Wcp) && Wcp > 0)
            error('No hay Wcp con K fijo; probá agregar una etapa o cambiar K.');
        end

        phi_need = PM_des - PM;
        if phi_need > 0.5
            phi_apply = min(max(phi_need, 1), 60);
            C = lead_norm_at_wc(phi_apply, Wcp, s) * C;
        elseif phi_need < -0.5
            % Si te pasaste de PM, podés usar lag NORMALIZADO suave para restar fase
            phi_lag = min(max(abs(phi_need), 1), 30);
            C = lag_norm_at_wc(phi_lag, Wcp, s) * C;
        else
            break;  % dentro de tolerancia
        end

        % ¿ya estamos dentro de tolerancia?
        L2 = K*C*Gw;
        [~, PM2] = margin(L2);
        if isfinite(PM2) && abs(PM2 - PM_des) <= tolPM
            break;
        end
    end

    K_out = K;
    C_out = C;

    Lf = K_out*C_out*Gw;
    [GMf, PMf, Wcgf, Wcpf] = margin(Lf);
    report = struct('GMdB',20*log10(GMf), 'PM',PMf, 'Wcg',Wcgf, 'Wcp',Wcpf, 'iters',it);
end
