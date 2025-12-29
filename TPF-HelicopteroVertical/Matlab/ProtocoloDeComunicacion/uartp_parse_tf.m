function [num6, den6, reserved] = uartp_parse_tf(coeffs)
%UARTP_PARSE_TF  Extrae num/den (6+6) desde coeffs(16) en formato TF.
%
% Entradas:
%   coeffs : vector (single/double) de 16 elementos (o más; toma los primeros 16)
%
% Salidas:
%   num6     : single(1x6) numerador
%   den6     : single(1x6) denominador
%   reserved : single(1x4) coeffs(13..16) reservados (por si los usás después)

c = single(coeffs(:));
assert(numel(c) >= 16, "uartp_parse_tf: coeffs debe tener al menos 16 floats");

c = c(1:16);

num6 = c(1:6).';
den6 = c(7:12).';
reserved = c(13:16).';
end
