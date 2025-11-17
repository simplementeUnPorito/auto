

% ============================================================
% 3) Ruido térmico de una resistencia con ganancia
% ============================================================
function [vrms, en] = resistor_noise(R, B, gain, T)
% RESISTOR_NOISE  Ruido térmico de una resistencia.
%
%   [vrms, en] = resistor_noise(R, B, gain, T)
%
%   R     : resistencia en ohm
%   B     : ancho de banda en Hz (banda efectiva)
%   gain  : ganancia (en tensión) con la que se amplifica ese ruido
%   T     : temperatura en Kelvin (opcional, default 300 K)
%
%   en    : densidad espectral de ruido: sqrt(4 k T R)  [V/sqrt(Hz)]
%   vrms  : ruido RMS en la salida: gain * sqrt(4 k T R B)
%
%   Fórmula clásica de ruido térmico: en = sqrt(4 k T R).

    if nargin < 4 || isempty(T)
        T = 300;   % ~27°C
    end

    kB = 1.38064852e-23;        % J/K

    en   = sqrt(4 * kB * T * R);     % V/sqrt(Hz)
    vrms = gain * sqrt(4 * kB * T * R * B);
end
