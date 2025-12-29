function coeffs = uartp_make_ss(A,B,C,D,L,K,Ki)
A = single(A); B = single(B(:)); C = single(C(:));
D = single(D); L = single(L(:)); K = single(K(:)); Ki = single(Ki);

assert(isequal(size(A),[2 2]), "A debe ser 2x2");
assert(numel(B)==2, "B debe tener 2 elementos");
assert(numel(C)==2, "C debe tener 2 elementos (1x2 o 2x1)");
assert(numel(L)==2, "L debe tener 2 elementos");
assert(numel(K)==2, "K debe tener 2 elementos");

coeffs = single(zeros(16,1));

% A11 A12 A21 A22
coeffs(1) = A(1,1);
coeffs(2) = A(1,2);
coeffs(3) = A(2,1);
coeffs(4) = A(2,2);

% B1 B2
coeffs(5:6) = B(:);

% C1 C2
coeffs(7:8) = C(:).';

% D
coeffs(9) = D;

% L1 L2
coeffs(10:11) = L(:);

% K1 K2
coeffs(12:13) = K(:);

% Ki
coeffs(14) = Ki;

% 15..16 reservados
end
