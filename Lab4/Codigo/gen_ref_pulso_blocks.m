function [t, refd] = gen_ref_pulso_blocks(Ts_ref, T_total, ref_amp, pulse_amp, f_hz)
% Alterna bloques de alto/bajo alrededor de ref_amp con frecuencia f_hz.
% Alto = ref_amp + pulse_amp
% Bajo = ref_amp - pulse_amp

    N      = max(1, round(T_total / Ts_ref));     % total de muestras
    t      = (0:N-1)' * Ts_ref;
    Nblk   = max(1, round((1/(2*f_hz)) / Ts_ref));% muestras por medio periodo

    hi = ref_amp + pulse_amp;
    lo = ref_amp - pulse_amp;

    pattern = [repmat(hi, Nblk, 1); repmat(lo, Nblk, 1)];
    refd = repmat(pattern, ceil(N/(2*Nblk)), 1);
    refd = refd(1:N);
end
