



function noise_utils_demo()
% NOISE_UTILS_DEMO
%   Archivo “contenedor” para las funciones de cálculo de ruido.
%   Guardá esto como noise_utils.m y usá directamente las funciones
%   de más abajo desde tu script:
%
%   [lsb, sigma_q, snr_dB] = quant_noise_from_bits(Nbits, Vpp);
%   [vrms_unf, vrms_filt]  = noise_from_psd_with_poles(f_pts, S_pts, poles);
%   vrms_res               = resistor_noise(R, B, gain, T);
%
%   Esta función demo no hace nada por defecto.

disp('noise_utils cargado. Usá las funciones:');
disp('  quant_noise_from_bits, noise_from_psd_with_poles, resistor_noise');

end

% ============================================================
% 1) Ruido de cuantización a partir de bits del digitalizador
% ============================================================
function [lsb, sigma_q, snr_theor_dB] = quant_noise_from_bits(Nbits, Vpp)
% QUANT_NOISE_FROM_BITS  Ruido de cuantización de un ADC/DAC ideal.
%
%   [lsb, sigma_q, snr_theor_dB] = quant_noise_from_bits(Nbits, Vpp)
%
%   Nbits : número de bits (por ejemplo 8 ó 12)
%   Vpp   : rango pico-a-pico analógico (por ejemplo 4.8 para ±2.4 V)
%
%   lsb       : tamaño del LSB en voltios
%   sigma_q   : ruido RMS de cuantización (V)
%   snr_theor_dB : SNR teórico de un seno a FS (6.02N+1.76 dB)

    % tamaño de paso
    lsb = Vpp / (2^Nbits);

    % ruido RMS de cuantización de una señal uniforme en [-lsb/2, lsb/2]
    sigma_q = lsb / sqrt(12);

    % SNR teórico para un seno a fondo de escala
    snr_theor_dB = 6.02 * Nbits + 1.76;
end

% ===================================================================
% 2) Ruido a partir de PSD tabulada, con filtrado por una lista de polos
% ===================================================================
function [vrms_unf, vrms_filt, f_dense, S_dense] = ...
         noise_from_psd_with_poles(f_pts, S_pts, poles, Ngrid)
% NOISE_FROM_PSD_WITH_POLES
%   Integra una PSD (curva de ruido de datasheet) y la filtra por
%   un sistema definido solo por sus polos continuos.
%
%   [vrms_unf, vrms_filt, f_dense, S_dense] = ...
%       noise_from_psd_with_poles(f_pts, S_pts, poles, Ngrid)
%
%   f_pts  : frecuencias de los puntos de la curva (Hz)      [vector]
%   S_pts  : densidad de ruido en V/sqrt(Hz) en esas frecs   [vector]
%            (si tu curva está en nV/sqrt(Hz), hace:
%             S_pts = curva_nV * 1e-9;)
%   poles  : vector de polos continuos (en rad/s), por ej.:
%            poles = [-1/tau1, -1/tau2];   % para 2 polos reales
%   Ngrid  : (opcional) número de puntos para la interpolación log;
%            por defecto 2000.
%
%   vrms_unf  : ruido RMS integrado SIN filtro (solo la curva)
%   vrms_filt : ruido RMS integrado LUEGO de filtrar por H(s)
%               donde H(jw) = prod( 1 / (j w - p_k) ), k = 1..Npolos
%   f_dense   : frecuencias del grid denso (Hz)
%   S_dense   : PSD interpolada (V/sqrt(Hz)) sobre f_dense

    if nargin < 4 || isempty(Ngrid)
        Ngrid = 2000;
    end

    f_pts  = f_pts(:);   % asegurar columnas
    S_pts  = S_pts(:);

    % --- Grid denso logarítmico entre min y max de la curva ---
    fmin = min(f_pts);
    fmax = max(f_pts);
    f_dense = logspace(log10(fmin), log10(fmax), Ngrid).';

    % --- Interpolación de la PSD en escala log-f ---
    S_dense = interp1(f_pts, S_pts, f_dense, 'pchip', 'extrap');

    % --- Ruido integrado SIN filtro ---
    S2 = S_dense.^2;           % V^2/Hz
    vrms_unf = sqrt( trapz(f_dense, S2) );

    % --- Filtro definido por sus polos continuos ---
    if isempty(poles)
        vrms_filt = vrms_unf;
        return;
    end

    poles = poles(:).';       % fila

    w = 2*pi*f_dense;         % rad/s

    % H(jw) = prod_k 1 / (j w - p_k)
    H = ones(size(w));
    for k = 1:numel(poles)
        H = H .* (1 ./ (1j*w - poles(k)));
    end

    % Si quisieras forzar ganancia DC = 1:
    % H0 = prod(1 ./ (-poles));    % H(s) en s=0
    % H = H / H0;

    S2_filt = S2 .* (abs(H).^2);
    vrms_filt = sqrt( trapz(f_dense, S2_filt) );
end

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
