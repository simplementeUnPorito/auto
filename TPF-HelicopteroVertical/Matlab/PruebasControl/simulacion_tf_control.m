close all
clear all

addpath("D:\auto\TPF-HelicopteroVertical\Matlab\ProtocoloDeComunicacion")

sp = uartp_open("COM9",115200);

uartp_reset(sp);
uartp_setmode(sp, 0); % TF

num6 = [0 0.73536 0 0 0 0];
den6 = [1 -0.7814 0 0 0 0];

coeffs = uartp_make_tf(num6, den6);
uartp_send_coeffs(sp, coeffs, true);

ref0 = 1.0;               % <-- referencia que querés aplicar al arrancar
uartp_init(sp, single(ref0));
