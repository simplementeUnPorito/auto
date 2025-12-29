function payload = uartp_ll_recv_words4(sp, nbytes, timeout_s)
A = uint8('A'); N = uint8('N');

payload = zeros(nbytes,1,'uint8');
idx = 1;

while idx <= nbytes
    w = zeros(4,1,'uint8');
    for i=1:4, w(i)=uartp_ll_readexact(sp,1,timeout_s); end

    write(sp, w, "uint8");

    ctl = uartp_ll_readexact(sp, 1, timeout_s);
    if ctl ~= A && ctl ~= N
        write(sp, N, "uint8");
        continue;
    end

    write(sp, ctl, "uint8");

    if ctl == A
        take = min(4, nbytes-idx+1);
        payload(idx:idx+take-1) = w(1:take);
        idx = idx + take;
    end
end
end
