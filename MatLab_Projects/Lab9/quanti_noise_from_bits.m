
% ============================================================
% 1) Ruido de cuantización a partir de bits del digitalizador
% ============================================================
function [lsb, sigma_q, snr_theor_dB] = quanti_noise_from_bits(Nbits, Vpp)
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
