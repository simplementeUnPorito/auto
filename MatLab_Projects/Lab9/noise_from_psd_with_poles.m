
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

