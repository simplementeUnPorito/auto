function [A,B,C,D,L,K,Ki,meta] = uartp_parse_ss(coeffs16)
c = single(coeffs16(:));
assert(numel(c)==16, "coeffs16 debe tener 16 floats");

A  = [c(1) c(2); c(3) c(4)];
B  = [c(5); c(6)];
C  = [c(7) c(8)];
D  = c(9);
L  = [c(10); c(11)];
K  = [c(12); c(13)];
Ki = c(14);

meta.N      = c(15);
meta.Period = c(16);
meta.res15  = c(15); % por si querés mantener naming
meta.res16  = c(16);
end
