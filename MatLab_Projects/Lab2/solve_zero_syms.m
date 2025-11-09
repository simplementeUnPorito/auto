function z0 = solve_zero_syms(phi_deg, z0, x,y)
    % phi_deg: ángulo deseado en grados
    % p0: polo elegido
    % omega: frecuencia de diseño (rad/muestra)

    syms p0_sym real
    
    phi = deg2rad(phi_deg);

    % Ecuación simbólica
    eq = atan(y,(x - z0)) - atan2(y,(x - p0_sym)) == phi;

    % Resolver
    sol = solve(eq, p0_sym, 'Real', true);

    % Devolver solución real (puede haber varias, nos quedamos con la real simple)
    p0 = double(sol);
end
