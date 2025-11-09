function [C_out, report] = add_stage_at_Wcp(Gw, C_in, K, comp_type, phi_deg, s)
% Agrega UNA etapa (lead o lag) normalizada en el Wcp del lazo actual L = K*C*G.
% No modifica K. Devuelve C_out y un pequeño reporte con los nuevos márgenes.

    % 1) Wcp del lazo actual
    L_now = K * C_in * Gw;
    [GM_now, PM_now, Wcg_now, Wcp_now] = margin(L_now);
    if ~(isfinite(Wcp_now) && Wcp_now > 0)
        error('No hay cruce de ganancia (Wcp) con el lazo actual. Ajustá K o añadí otra etapa antes.');
    end
    wc = Wcp_now;

    % 2) Construir la etapa normalizada en wc
    comp_type = lower(strtrim(comp_type));
    if strcmpi(comp_type,'lead')
        C_stage = lead_norm_at_wc(phi_deg, wc, s);     % |C(j wc)| = 1
    elseif strcmpi(comp_type,'lag')
        C_stage = lag_norm_at_wc(phi_deg, wc, s);      % |C(j wc)| = 1
    else
        error('comp_type debe ser "lead" o "lag".');
    end

    % 3) Encadenar etapa (K NO cambia)
    C_out = C_stage * C_in;

    % 4) Reporte (con K fijo)
    L_new = K * C_out * Gw;
    [GMf, PMf, Wcgf, Wcpf] = margin(L_new);
    report = struct('GMdB', 20*log10(GMf), 'PM', PMf, 'Wcg', Wcgf, 'Wcp', Wcpf, ...
                    'wc_used', wc, 'type', comp_type, 'phi_deg', phi_deg);
end
