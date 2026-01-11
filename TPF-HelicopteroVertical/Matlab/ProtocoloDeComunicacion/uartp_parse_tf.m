function [num6, den6, meta] = uartp_parse_tf(coeffs16)
c = single(coeffs16(:));
assert(numel(c)==16, "coeffs16 debe tener 16 floats");

num6 = c(1:6).';
den6 = c(7:12).';

meta.res13  = c(13);
meta.res14  = c(14);
meta.N      = c(15);
meta.Period = c(16);
end
