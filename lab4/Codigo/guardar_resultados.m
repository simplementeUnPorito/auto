%% ================== Guardar figuras e informe ==================
% Carpeta de salida
outDir = fullfile(pwd, 'resultados');
figDir = fullfile(outDir, 'figuras');
if ~exist(outDir, 'dir'), mkdir(outDir); end
if ~exist(figDir, 'dir'), mkdir(figDir); end

% --- 1) Guardar TODAS las figuras abiertas (las que genera comparar_intersample y otras)
figs = findall(0, 'Type', 'figure');
tsStamp = datestr(now, 'yyyymmdd_HHMMSS');  % timestamp único
for i = 1:numel(figs)
    try
        set(figs(i), 'PaperPositionMode', 'auto');
        fname = sprintf('fig_%02d_%s.png', i, tsStamp);
        print(figs(i), fullfile(figDir, fname), '-dpng', '-r300');
    catch ME
        warning('No pude exportar la figura %d: %s', i, ME.message);
    end
end
%%

% --- 2) Armar archivo .txt con: Ts | Planta(G) | C_m1 | C_m2 (solo FT)
txtPath = fullfile(outDir, sprintf('resumen_control_%s.txt', tsStamp));
fid = fopen(txtPath, 'w');
if fid < 0
    error('No pude crear el archivo de resumen en %s', txtPath);
end

fprintf(fid, 'Resumen de control por periodo de muestreo\n');
fprintf(fid, 'Formato: Ts [s] | Planta G | C_m1 | C_m2\n');
fprintf(fid, '---------------------------------------------------------------\n');

% String "limpio" de G continua (única para todas las filas)

Nrows = min([numel(T), numel(C_all_m1), numel(C_all_m2)]);
for k = 1:Nrows
    Ts_k   = T(k);
    G_show = tf2str_z(Gd_all_m2{k});
    C1_show = tf2str_z(C_all_m1{k});
    C2_show = tf2str_z(C_all_m2{k});
    fprintf(fid, '%.12g | %s | %s | %s\n', Ts_k, G_show, C1_show, C2_show);
end
fclose(fid);

% Guarda también el workspace por si querés reusar
save(fullfile(outDir, sprintf('workspace_%s.mat', tsStamp)), ...
     'T','G','C_all_m1','C_all_m2','Gd_all_m1','Gd_all_m2');
fprintf('Listo.\nResumen: %s\n', txtPath);

function s = tf2str_simple(sys)
% Devuelve "(num)/(den)" en una línea.
% - Si es continuo: variable 's'
% - Si es discreto: variable 'z^-1' + sample time
    [num, den, Ts] = tfdata(sys, 'v');
    if isct(sys)
        s = sprintf('(%s)/(%s)', poly2str(num,'s'), poly2str(den,'s'));
    else
        s = sprintf('(%s)/(%s); Ts=%.10g', poly2str(num,'z^-1'), poly2str(den,'z^-1'), Ts);
    end
    s = regexprep(s, '\s+', ' '); % compactar espacios
end
function s = tf2str_z(sys)
% Escribe la FT discreta en variable 'z' (no z^-1)
    assert(~isct(sys),'Solo discreto');
    [num, den, ~] = tfdata(sys,'v');  % coef. en z^-1 descendentes
    % Convertimos a polinomios en z multiplicando por z^-N y revirtiendo:
    num_z = fliplr(num); den_z = fliplr(den);
    s = sprintf('(%s)/(%s);', poly2str(num_z,'z'), poly2str(den_z,'z'));
    s = regexprep(s, '\s+', ' ');
end