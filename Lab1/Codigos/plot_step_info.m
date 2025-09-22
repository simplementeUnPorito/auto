function [tr, ts, wn] = plot_step_info(G)
    info = stepinfo(G);
    tr = info.RiseTime; 
    ts = info.SettlingTime; 
    wn = 1.8 / tr;

    t_end = 1.1 * ts;
    if ~isfinite(t_end) || t_end <= 0, t_end = 5 * max(tr, 1e-3); end
    t = linspace(0, t_end, 2*pi/wn);

    [y, tout] = step(G, t);
    figure; plot(tout*1000, y, 'LineWidth', 1.4); grid on;grid minor; hold on;
    xlabel('Tiempo [ms]'); ylabel('Salida');
    title(sprintf('Escalón: tr=%.4gms, ts=%.4gms, \\omega_n=%.4g rad/s', tr*1000, ts*1000, wn));
    xline(tr*1000, '--', sprintf('  t_r=%.3g ms', tr*1000), 'LabelOrientation','horizontal');
    xline(ts*1000,  '--', sprintf('  t_s=%.3g ms', ts*1000), 'LabelOrientation','horizontal');
    legend('G(t)', 'Location', 'best');
end
