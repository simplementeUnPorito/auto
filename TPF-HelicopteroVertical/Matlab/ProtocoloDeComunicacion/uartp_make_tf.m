function coeffs = uartp_make_tf(num6, den6)
num6 = single(num6(:)); den6 = single(den6(:));
assert(numel(num6)==6 && numel(den6)==6, "num6 y den6 deben tener 6 coeficientes");
coeffs = single(zeros(16,1));
coeffs(1:6)  = num6;
coeffs(7:12) = den6;
% 13..16 reservados
end
