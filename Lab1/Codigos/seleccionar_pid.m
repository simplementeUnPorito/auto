function best = seleccionar_pid(results, OSd, trd, tsd)
% Selecciona el mejor PID de acuerdo a criterios de OS, tr y ts
% - Si el parámetro es escalar: busca lo más cercano por debajo.
% - Si el parámetro es [min max]: busca dentro de ese rango.
% - Si está vacío ([]): no se usa como filtro, pero sí como desempate.

if isempty(results)
    error('No hay resultados registrados.');
end

% Filtrar solo estables
valid_idx = ~isnan([results.Overshoot]) & ~isnan([results.RiseTime]) & ~isnan([results.SettlingTime]);
res_valid = results(valid_idx);
if isempty(res_valid)
    error('No hay ensayos estables.');
end

OS_vals = [res_valid.Overshoot];
tr_vals = [res_valid.RiseTime];
ts_vals = [res_valid.SettlingTime];

candidatos = 1:numel(res_valid);

% --- criterio Overshoot ---
if ~isempty(OSd)
    if isscalar(OSd)
        under = candidatos(OS_vals(candidatos) <= OSd);
        if ~isempty(under)
            [~, idx_local] = min(OSd - OS_vals(under)); % más cercano por debajo
            candidatos = under(idx_local);
        end
    elseif numel(OSd) == 2
        candidatos = candidatos(OS_vals(candidatos) >= OSd(1) & OS_vals(candidatos) <= OSd(2));
    end
end

% --- criterio RiseTime ---
if ~isempty(trd) && numel(candidatos) > 1
    if isscalar(trd)
        under = candidatos(tr_vals(candidatos) <= trd);
        if ~isempty(under)
            [~, idx_local] = min(trd - tr_vals(under));
            candidatos = under(idx_local);
        end
    elseif numel(trd) == 2
        candidatos = candidatos(tr_vals(candidatos) >= trd(1) & tr_vals(candidatos) <= trd(2));
    end
end

% --- criterio SettlingTime ---
if ~isempty(tsd) && numel(candidatos) > 1
    if isscalar(tsd)
        under = candidatos(ts_vals(candidatos) <= tsd);
        if ~isempty(under)
            [~, idx_local] = min(tsd - ts_vals(under));
            candidatos = under(idx_local);
        end
    elseif numel(tsd) == 2
        candidatos = candidatos(ts_vals(candidatos) >= tsd(1) & ts_vals(candidatos) <= tsd(2));
    end
end

% --- Desempate con criterios NO especificados ---
if numel(candidatos) > 1
    if isempty(OSd) % menor overshoot
        [~, idx_local] = min(OS_vals(candidatos));
        candidatos = candidatos(idx_local);
    end
end
if numel(candidatos) > 1
    if isempty(tsd) % menor settling time
        [~, idx_local] = min(ts_vals(candidatos));
        candidatos = candidatos(idx_local);
    end
end
if numel(candidatos) > 1
    if isempty(trd) % menor rise time
        [~, idx_local] = min(tr_vals(candidatos));
        candidatos = candidatos(idx_local);
    end
end

best = res_valid(candidatos);

fprintf('>> Seleccionado: Kp=%.4g, Ti=%.4g, Td=%.4g | OS=%.2f%%, tr=%.4gs, ts=%.4gs\n', ...
        best.Kp, best.Ti, best.Td, best.Overshoot, best.RiseTime, best.SettlingTime);
end
