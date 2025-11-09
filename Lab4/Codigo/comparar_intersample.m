function comparar_intersample(S_all, T)
% COMPARAR_INTERSAMPLE - Compara esfuerzos y salidas continuas
% Uso:
%   comparar_intersample(S_all, T)
% Entradas:
%   S_all : cell array de structs devueltos por ver_intersample
%   T     : vector de Ts correspondientes

    estilos = {'-','--',':','-.'}; % distintos estilos
    colores = lines(numel(T));     % paleta distinta para cada curva
    leg = arrayfun(@(Ts) sprintf('T_s = %.4g ms', Ts*1e3), T, 'UniformOutput', false);

    figure('Name','Comparativa esfuerzos y respuestas continuas');
    tiledlayout(2,1);

    % --- Subplot 1: esfuerzos ---
    ax1 = nexttile; hold on; grid on; grid minor;
    for k = 1:numel(T)
        est = estilos{mod(k-1,numel(estilos))+1};
        stairs(S_all{k}.tk*1e3, S_all{k}.uk, est, ...
            'Color', colores(k,:), 'LineWidth', 1);
    end
    xlabel('milisegundos');
    ylabel('Voltios');
    title('Esfuerzos del controlador para distintos T_s');
    legend(leg, 'Location', 'best');

    % --- Subplot 2: salidas continuas ---
    ax2 = nexttile; hold on; grid on; grid minor;
    for k = 1:numel(T)
        est = estilos{mod(k-1,numel(estilos))+1};
        plot(S_all{k}.tc*1e3, S_all{k}.yc, est, ...
            'Color', colores(k,:), 'LineWidth', 1);
    end
    xlabel('milisegundos');
    ylabel('Voltio');
    title('Respuesta continua de G(s) con u_{ZOH}(t)');
    legend(leg, 'Location', 'best');

    linkaxes([ax1 ax2],'x'); % sincroniza zoom en X
end
