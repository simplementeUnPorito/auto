
function C = lag_gain_at_wc_db(drop_dB, wc, s, kfactor)
% Lag que REDUCE el módulo ~drop_dB en torno a wc sin tocar K.
% NO normalizado (justamente para cambiar GM). Ubica esquinas a wc/kfactor.
% Aproxima |C(jω)| ~ 1/β para ω >> 1/T (efecto lag clásico).
    if nargin<4 || isempty(kfactor), kfactor = 10; end
    beta = max(1.05, 10^(drop_dB/20));     % objetivo de caída de |C|≈1/beta (dB ~ -drop_dB)
    % Colocar esquinas bien por debajo: ωz=1/T, ωp=1/(βT); con β>1 => polo más bajo.
    % Elegimos ωz = wc/kfactor => T = kfactor/wc
    T = kfactor / wc;
    C = (1 + T*s) / (1 + beta*T*s);        % lag correcto (módulo ~1/β a altas frecuencias)
end