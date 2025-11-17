Vpp_adc = 4.8;    % ±2.4 V
Nbits_adc = 12;


Vpp_dac = 4.024;
Nbits_dac = 8;

[lsb_adc, sigmaq_adc, snr_adc] = quanti_noise_from_bits(Nbits_adc, Vpp_adc);
[lsb_dac, sigmaq_dac, snr_dac] = quanti_noise_from_bits(Nbits_dac, Vpp_dac);



f_pts  = [0.01 0.1 2 10 100 1000]*1e3;      % Hz, ejemplo
S_nV   = [700 300 100 60 50 45];     % nV/sqrt(Hz), ejemplo
S_pts  = S_nV * 1e-9;                       % V/sqrt(Hz)

poles_plant = [-1/(C1*R2), -1/(C2*R4)];           % rad/s

[vrms_unf, vrms_filt] = noise_from_psd_with_poles(f_pts, S_pts, poles_plant);

B = 500;
K1 = R2/R1;K2=R4/R3;

[vrms_R1, en_R1] = resistor_noise(R1, B, K1);
[vrms_R2, en_R2] = resistor_noise(R2, B, 1);
[vrms_R3, en_R3] = resistor_noise(R3, B, K2);
[vrms_R4, en_R4] = resistor_noise(R4, B, 1);

sigma_x1 = sqrt(vrms_unf^2 + vrms_R1^2 + vrms_R2^2 + sigmaq_dac^2);
sigma_x2 = sqrt(vrms_unf^2 + vrms_R3^2 + vrms_R4^2);

sigma_v = sigmaq_adc;   




