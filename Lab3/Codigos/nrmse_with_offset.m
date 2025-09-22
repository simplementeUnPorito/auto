function [rmse_raw, nrmse_raw, rmse_bias, nrmse_bias, bias] = nrmse_with_offset(y_exp, y_sim)
% Calcula RMSE/NRMSE (en %) con la señal tal cual y corrigiendo un offset constante.
% y_exp, y_sim: vectores (misma longitud o se recortan al mínimo)
% NRMSE normaliza por el rango de y_exp.

    y_exp = y_exp(:); y_sim = y_sim(:);
    N = min(numel(y_exp), numel(y_sim));
    y_exp = y_exp(1:N); y_sim = y_sim(1:N);

    scale = max(y_exp) - min(y_exp);
    if scale <= 0, scale = 1; end  % evita división por cero

    % Tal cual (con offset)
    err  = y_exp - y_sim;
    rmse_raw  = sqrt(mean(err.^2));
    nrmse_raw = 100 * rmse_raw / scale;

    % Corrigiendo offset (bias constante, robusto con mediana)
    bias = median(y_exp - y_sim);       % también podés usar mean(...)
    y_sim_bc = y_sim + bias;

    err_bc  = y_exp - y_sim_bc;
    rmse_bias  = sqrt(mean(err_bc.^2));
    nrmse_bias = 100 * rmse_bias / scale;
end
