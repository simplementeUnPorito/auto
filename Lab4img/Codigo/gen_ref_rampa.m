function [t, refd] = gen_ref_rampa(Ts_ref, T_total, ref_min, ref_max, f_ramp, phase)
% GEN_REF_RAMPA: rampa periódica (sawtooth) de ref_min a ref_max
% Ts_ref  : periodo de muestreo de la referencia (s)
% T_total : duración total (s)
% ref_min : valor mínimo
% ref_max : valor máximo
% f_ramp  : frecuencia de la rampa (Hz)  -> periodo = 1/f_ramp
% phase   : fase en rad. Default 0
    if nargin < 6 || isempty(phase), phase = 0; end

    N = max(1, round(T_total / Ts_ref));
    t = (0:N-1)' * Ts_ref;

    % diente de sierra en [0,1): fase normalizada
    s = mod(f_ramp * t + phase/(2*pi), 1);

    % escalar a [ref_min, ref_max]
    refd = ref_min + (ref_max - ref_min) * s;
end
