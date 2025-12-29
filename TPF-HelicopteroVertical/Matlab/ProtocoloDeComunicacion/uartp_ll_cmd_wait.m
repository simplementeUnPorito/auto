function rsp = uartp_ll_cmd_wait(sp, cmd_char, timeout_s)
valid = uint8(['R','S','K','!']);
write(sp, uint8(cmd_char), "uint8");

t0 = tic;
while true
    if toc(t0) > timeout_s
        error("Timeout esperando respuesta a '%s'", cmd_char);
    end
    if sp.NumBytesAvailable > 0
        b = read(sp, 1, "uint8"); b = b(1);
        if any(b == valid)
            rsp = b;
            return;
        end
    else
        pause(0.001);
    end
end
end
