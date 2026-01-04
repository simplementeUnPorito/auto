function gen_sim_export_5ctrl_psoc()
clear; clc; close all;

%% ===================== 0) POLOS =====================
p_ctrl = 0.60;
p_obs  = 0.30;
p_int  = 0.50;

%% ===================== 1) PLANTA =====================
tau = 1e3*5e-9;      % 5e-6 s
Ts  = 1/100e3;       % 10 us

a  = exp(-Ts/tau);
A1 = a;
B1 = 1;
C1 = 1 - a;
D1 = 0;

fprintf("A1=%.9f  B1=%.9f  C1=%.9f  D1=%.9f\n", A1,B1,C1,D1);

%% ===================== 2) SS SIN integrador =====================
K = acker(A1, B1, p_ctrl);

sys_cl = ss(A1 - B1*K, B1, C1, D1, Ts);   % r -> y si Kr=1
gdc = dcgain(sys_cl);
Nbar = 1/gdc;

% Verificación con refi
[~,~,Nbar_refi] = refi(A1, B1, C1, K);

fprintf("\nNOI: K=%.9f\n", K);
fprintf("NOI: Nbar(dcgain)=%.9f   Nbar(refi)=%.9f\n", Nbar, Nbar_refi);

L_pred   = acker(A1', C1',      p_obs).';
L_actual = acker(A1', (C1*A1)', p_obs).';

COEFF_SS_PRED_NOI = pack_ss16_1state(A1,B1,C1,D1, L_pred,   K, Nbar);
COEFF_SS_ACT_NOI  = pack_ss16_1state(A1,B1,C1,D1, L_actual, K, Nbar);

%% ===================== 3) SS CON integrador (Ogata) =====================
m = 1; n = 1;
Ahat = [A1 B1; zeros(m,n+m)];
Bhat = [zeros(n,m); eye(m)];
polos_i = [p_ctrl p_int];

Khat = acker(Ahat, Bhat, polos_i);

Aux  = [A1-1   B1;
        C1*A1  C1*B1];
K2K1 = (Khat + [zeros(1,n) eye(m)]) / Aux;
K2   = K2K1(1,1:n);
K1   = K2K1(1,n+1:end);

% IMPORTANTE: firmware integra vint += e*Ts  -> Ki_pSoC = K1/Ts
Ki_psoc = K1 / Ts;

fprintf("\nI: K2=%.9f  K1(Ogata)=%.9f  Ki_psoc=%.9f\n", K2, K1, Ki_psoc);

L_pred_i   = acker(A1', C1',      p_obs).';
L_actual_i = acker(A1', (C1*A1)', p_obs).';

COEFF_SS_PRED_I = pack_ss16_1state(A1,B1,C1,D1, L_pred_i,   K2, Ki_psoc);
COEFF_SS_ACT_I  = pack_ss16_1state(A1,B1,C1,D1, L_actual_i, K2, Ki_psoc);

%% ===================== 4) IMPRIMIR PARA PEGAR EN C =====================
fprintf("\nKx (NOI) esperado ~= Nbar  = %.9f\n", Nbar);
fprintf("Kx (I)   esperado ~= Ki_psoc= %.9f\n", Ki_psoc);

print_c16("COEFF_SS_PRED_NOI", COEFF_SS_PRED_NOI);
print_c16("COEFF_SS_ACT_NOI",  COEFF_SS_ACT_NOI);
print_c16("COEFF_SS_PRED_I",   COEFF_SS_PRED_I);
print_c16("COEFF_SS_ACT_I",    COEFF_SS_ACT_I);

end

%% ===================== FUNCIONES =====================

function c16 = pack_ss16_1state(A1,B1,C1,D1,L1,K_states,Kx)
% Layout UART (16 floats):
% 0..3   A11 A12 A21 A22
% 4..5   B1  B2
% 6..7   C1  C2
% 8      D
% 9..10  L1  L2
% 11..12 K1  K2
% 13     Kx  (si modo *_I => Ki, si modo *_NOI => Kr)
% 14..15 reservados

    c16 = single(zeros(1,16));
    c16(1)  = single(A1);   c16(2)  = 0;  c16(3)  = 0;  c16(4)  = 0;
    c16(5)  = single(B1);   c16(6)  = 0;
    c16(7)  = single(C1);   c16(8)  = 0;
    c16(9)  = single(D1);
    c16(10) = single(L1);   c16(11) = 0;
    c16(12) = single(K_states); c16(13) = 0;
    c16(14) = single(Kx);
    c16(15) = 0;
    c16(16) = 0;
end

function [Nx,Nu,Nbar] = refi(phi,gam,Hr,K)
    I = eye(size(phi));
    [m,n] = size(Hr); %#ok<ASGLU>
    np = inv([phi-I gam; Hr zeros(m)]) * ([zeros(n,m); eye(m)]);
    Nx = np(1:n,:);
    Nu = np(n+1:n+m,:);
    Nbar = Nu + K*Nx;
end

function print_c16(name, c16)
    fprintf("\nstatic const float %s[16] = {\n", name);
    for i=1:16
        if i < 16
            fprintf("    %.9ff,\n", double(c16(i)));
        else
            fprintf("    %.9ff\n",  double(c16(i)));
        end
    end
    fprintf("};\n");
end
