function rx = uartp_get_coeffs_raw(sp, timeout_s)
if nargin < 2, timeout_s = 2.0; end

rsp = uartp_ll_cmd_wait(sp, 't', timeout_s);
assert(rsp == uint8('S'), "t no devolvió S (rsp=%c)", char(rsp));

rx = uartp_ll_recv_words4(sp, 64, timeout_s);

final = uartp_ll_readexact(sp, 1, timeout_s);
assert(final == uint8('K'), "t no terminó con K (rsp=%c)", char(final));
end
