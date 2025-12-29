function [A,B,C,D,L,K,Ki,reserved] = uartp_parse_ss(coeffs)
%UARTP_PARSE_SS  Extrae A,B,C,D,L,K,Ki desde coeffs(16) en formato SS (2 estados).
%
% Entradas:
%   coeffs : vector (single/double) de 16 elementos (o más; toma los primeros 16)
%
% Salidas:
%   A        : single(2x2)
%   B        : single(2x1)
%   C        : single(1x2)
%   D        : single(1x1)
%   L        : single(2x1)  (ganancias observador)
%   K        : single(2x1)  (realimentación de estado)
%   Ki       : single(1x1)  (integrador; 0 => sin integrador)
%   reserved : single(1x2)  coeffs(15..16)

c = single(coeffs(:));
assert(numel(c) >= 16, "uartp_parse_ss: coeffs debe tener al menos 16 floats");

c = c(1:16);

A = [c(1) c(2);
     c(3) c(4)];

B = [c(5); c(6)];

C = [c(7) c(8)];

D = c(9);

L = [c(10); c(11)];

K = [c(12); c(13)];

Ki = c(14);

reserved = c(15:16).';
end
