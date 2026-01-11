function coeffs = uartp_make_ss(A,B,C,D,L,K,Ki,N,period)
% UARTP SS layout (16 floats, 2 estados):
%  1..4   = A11 A12 A21 A22
%  5..6   = B1  B2
%  7..8   = C1  C2
%  9      = D
%  10..11 = L1 L2
%  12..13 = K1 K2
%  14     = Ki
%  15     = N      (c[14])
%  16     = period (c[15])

if nargin < 8 || isempty(N),      N = 0;      end
if nargin < 9 || isempty(period), period = 0; end

A  = single(A);
B  = single(B(:));
C  = single(C(:));
D  = single(D);
L  = single(L(:));
K  = single(K(:));
Ki = single(Ki);

assert(isequal(size(A),[2 2]), "A debe ser 2x2");
assert(numel(B)==2, "B debe tener 2 elementos");
assert(numel(C)==2, "C debe tener 2 elementos (1x2 o 2x1)");
assert(numel(L)==2, "L debe tener 2 elementos");
assert(numel(K)==2, "K debe tener 2 elementos");

assert(isfinite(N) && N>=0, "N inválido");
assert(isfinite(period) && period>=0, "period inválido");

coeffs = single(zeros(16,1));

% A
coeffs(1) = A(1,1);
coeffs(2) = A(1,2);
coeffs(3) = A(2,1);
coeffs(4) = A(2,2);

% B
coeffs(5:6) = B;

% C
coeffs(7:8) = C(:).';

% D
coeffs(9) = D;

% L
coeffs(10:11) = L;

% K
coeffs(12:13) = K;

% Ki
coeffs(14) = Ki;

% N y period (c[14], c[15])
coeffs(15) = single(N);
coeffs(16) = single(period);
end
