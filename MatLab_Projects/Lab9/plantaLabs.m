function [sysC,fn,Tn] = plantaLabs(R1,R2,R3,R4,C1,C2)
    tau1 = R2*C1;           k1 = -R2/R1;
    tau2 = R4*C2;           k2 = -R4/R3;
    
    F = [ -1/tau1,     0;
           k2/tau2, -1/tau2 ];
    G = [ k1/tau1; 0 ];
    H = [0 1];
    J = 0;
    
    sysC = ss(F,G,H,J);
    Gc   = tf(sysC);
    
    % Ts vía polo más rápido (aunque después fijás Ts a 1 ms)
    fn = max(abs(zpk(Gc).P{1}))/pi;
    Tn = 1/fn;


end