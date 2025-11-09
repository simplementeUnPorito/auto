function C = lead_norm_at_wc(phi_deg, wc, s)
    phi_deg = max(1, min(60, phi_deg));
    sphi = sind(phi_deg);
    alpha = (1 - sphi) / (1 + sphi);      % 0<alpha<1
    T = 1 / (wc * sqrt(alpha));
    C0 = (1 + T*s) / (1 + alpha*T*s);
    g = 1 / abs(freqresp(C0, 1j*wc));     % |C(jwc)|=1
    C = g * C0;
end
