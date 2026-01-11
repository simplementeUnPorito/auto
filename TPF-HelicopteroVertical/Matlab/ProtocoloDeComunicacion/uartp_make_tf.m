function coeffs = uartp_make_tf(num6, den6, N, period)
% coeffs(1..6)  = num6
% coeffs(7..12) = den6
% coeffs(13..14)= reservados (0)
% coeffs(15)    = N (float, pero representa entero)
% coeffs(16)    = period (float, pero representa entero)

if nargin < 3 || isempty(N),      N = 0; end
if nargin < 4 || isempty(period), period = 0; end

num6 = single(num6(:)); 
den6 = single(den6(:));
assert(numel(num6)==6 && numel(den6)==6, "num6 y den6 deben tener 6 coeficientes");

coeffs = single(zeros(16,1));
coeffs(1:6)   = num6;
coeffs(7:12)  = den6;
coeffs(13:14) = single([0;0]);

% c[14] y c[15] => coeffs(15) y coeffs(16)
coeffs(15) = single(N);
coeffs(16) = single(period);
end
