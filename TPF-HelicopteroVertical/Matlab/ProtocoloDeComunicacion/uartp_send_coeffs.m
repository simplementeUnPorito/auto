function uartp_send_coeffs(sp, coeffs, verify_roundtrip, timeout_s)
% coeffs:
%   - single(16,1) / double(16,1) -> se envían como 64 bytes
%   - uint8(64,1)                 -> se envían tal cual
if nargin < 3 || isempty(verify_roundtrip), verify_roundtrip = true; end
if nargin < 4, timeout_s = 2.0; end

payload = uartp_ll_to_payload64(coeffs);

rsp = uartp_ll_cmd_wait(sp, 'c', timeout_s);
assert(rsp == uint8('R'), "c no devolvió R (rsp=%c)", char(rsp));

uartp_ll_send_words4(sp, payload, timeout_s);

final = uartp_ll_readexact(sp, 1, timeout_s);
assert(final == uint8('K'), "c no terminó con K (rsp=%c)", char(final));

if verify_roundtrip
    rx = uartp_get_coeffs_raw(sp, timeout_s);  % uint8(64,1)
    assert(isequal(rx(:), payload(:)), "Roundtrip 't' devolvió bytes distintos");
end
end
