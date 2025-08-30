function results = diseno_pid(G, filename)
	% diseno_pid - Bucle interactivo para sintonia PID y selección del mejor ensayo
	%
	% Uso:
	%   [Kp, Ti, Td] = diseno_pid(G, 'log_pid.mat', OSd, trd, tsd)

	if nargin < 2
		filename = 'log_pid.mat';
	end
	if nargin < 3, OSd = []; end
	if nargin < 4, trd = []; end
	if nargin < 5, tsd = []; end

	results = struct([]);
	if exist(filename,'file')
		S = load(filename);
		if isfield(S,'results'), results = S.results; end
	end
	k_iter = numel(results) + 1;
	while true
		fprintf('\n--- Parametros PID ---\n');
		Kp = input('Kp: ');
		Td = input('Td: ');
		Ti = input('Ti: ');
		numC = Kp*[Ti*Td, Ti, 1];
		denC = [Ti, 0];
		C = tf(numC, denC);
		cl = feedback(C*G, 1);
		
		info = stepinfo(cl);
		OS = info.Overshoot;
		tr = info.RiseTime;
		ts = info.SettlingTime;
		
		% guardar en struct
		results(k_iter).Kp = Kp;
		results(k_iter).Ti = Ti;
		results(k_iter).Td = Td;
		results(k_iter).Overshoot = OS;
		results(k_iter).RiseTime = tr;
		results(k_iter).SettlingTime = ts;
		save(filename,'results');
		k_iter = k_iter + 1;
		
		resp = input('Continuar? [s/n]: ','s');
		if isempty(resp) || any(lower(resp(1)) == ['n','q'])
			disp('Finalizando loop.');
			break;
		end
	end
end
