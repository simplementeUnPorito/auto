
function C = lag_norm_at_wc(phi_deg, wc, s)
    phi_deg = max(1, min(60, phi_deg));
    sphi = sind(phi_deg);
    beta = (1 + sphi) / (1 - sphi);       % >1
    T = 1 / (wc * sqrt(beta));
    C0 = (1 + T*s) / (1 + (T/beta)*s);
    g = 1 / abs(freqresp(C0, 1j*wc));     % |C(jwc)|=1
    C = g * C0;
end
