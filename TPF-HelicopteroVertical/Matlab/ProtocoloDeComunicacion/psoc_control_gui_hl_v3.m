function psoc_control_gui_hl_v3()
% GUI basada en UARTP High-Level API.
%
% Requiere en el path:
%   uartp_open, uartp_reset, uartp_setmode, uartp_send_coeffs, uartp_get_coeffs,
%   uartp_init, uartp_stop, uartp_make_ss
%
% Streaming:
%   PSoC manda frames binarios de 8 bytes: [u(float32) y(float32)] little-endian.

    % =========================
    % Estado
    % =========================
    S = struct();
    S.sp = [];
    S.isConnected = false;

    % Streaming
    S.streamTimer  = [];
    S.t0           = [];
    S.nVec         = zeros(0,1);
    S.uVec         = zeros(0,1);
    S.yVec         = zeros(0,1);
    S.maxPoints    = 400000;
    S.streamRxBuf  = uint8([]);
    S.framesTotal  = 0;
    S.streamParseEnabled = true;

    % Auto-stop
    S.autoStopEnabled   = false;
    S.autoStopTarget    = 0;
    S.autoStopArmed     = false;
    S.autoStopStartBase = 0;
    S.autoStopPending   = false;
    S.autoStopReason    = "";

    % Plot scaling (solo visual)
    S.scaleU = 0.01;
    S.scaleY = 1.0;

    % ===== Protocolo: SIEMPRE paquete 25 =====
    NCOEFF = 25;

    % Índices meta (1-based MATLAB)
    IDX_TF_ORDER = 23; % c23
    IDX_META_N   = 24; % c24
    IDX_META_FS  = 25; % c25

    TF_MAX_ORDER = 10; % b0..b10 y a0..a10

    % =========================
    % UI
    % =========================
    fig = uifigure( ...
        'Name', 'PSoC Control GUI (HL) - TF25/SS25', ...
        'Position', [80 80 1280 720] ...
    );
    fig.CloseRequestFcn = @onClose;

    % Plot
    ax = uiaxes(fig, 'Position', [20 220 820 480]);
    ax.XGrid = 'on'; ax.YGrid = 'on';
    title(ax, 'Streaming u,y');
    xlabel(ax, 'frame index (telemetry frames)');
    ylabel(ax, 'value');

    hold(ax, 'on');
    hU = stairs(ax, nan, nan, 'DisplayName', 'u');
    hY = plot(ax,   nan, nan, 'DisplayName', 'y');
    legend(ax, 'show', 'Location', 'best');
    hold(ax, 'off');

    % Log
    txtLog = uitextarea(fig, 'Editable', 'off', 'Position', [20 20 820 180]);
    txtLog.Value = strings(0,1);

    % ===== Layout columna derecha =====
    RX = 860; RW = 400;

    % Connection panel
    pConn = uipanel(fig, 'Title', 'Connection', 'Position', [RX 620 RW 95]);

    uilabel(pConn, 'Text', 'COM:', 'Position', [10 45 35 22]);
    edtCom = uieditfield(pConn, 'text', 'Value', 'COM9', 'Position', [50 45 90 22]);

    edtBaud = uieditfield(pConn, 'numeric', 'Value', 921600, 'Limits', [1200 2000000], ...
        'Position', [150 45 90 22]);
    uilabel(pConn, 'Text', 'baud', 'Position', [245 45 40 22]);

    btnConnect = uibutton(pConn, 'Text', 'Connect', 'Position', [10 10 90 26], ...
        'ButtonPushedFcn', @onConnectToggle);

    lblStat = uilabel(pConn, 'Text', 'DISCONNECTED', 'Position', [110 10 280 26]);

    btnReset = uibutton(pConn, 'Text', 'Reset (r)', 'Position', [310 10 80 26], ...
        'ButtonPushedFcn', @onReset);

    % Mode panel
    pMode = uipanel(fig, 'Title', 'Mode', 'Position', [RX 435 RW 180]);

    ddType = uidropdown(pMode, 'Items', {'TF','SS','Open-loop'}, 'Value', 'Open-loop', ...
        'Position', [10 130 120 24], 'ValueChangedFcn', @(~,~) refreshVisibility());

    ddObserver = uidropdown(pMode, 'Items', {'Predictor','Actual'}, ...
        'Position', [150 130 120 24], 'Tooltip', 'Only applies for SS');

    cbIntegrator = uicheckbox(pMode, 'Text', 'Integrator', 'Position', [290 130 110 24]);

    uilabel(pMode, 'Text', 'N', 'Position', [10 95 20 24]);
    edtN = uieditfield(pMode, 'numeric', 'Limits', [0 1e9], 'RoundFractionalValues', 'on', ...
        'Value', 1, 'Position', [35 95 80 24]);

    uilabel(pMode, 'Text', 'Fs (Hz)', 'Position', [130 95 50 24]);
    edtFs = uieditfield(pMode, 'numeric', 'Limits', [0.001 1e9], 'Value', 1000, ...
        'Position', [180 95 90 24], 'ValueChangedFcn', @onFsChanged);

    lblPeriod = uilabel(pMode, 'Text', 'Fs=?', 'Position', [10 70 360 20]);

    btnSendMode = uibutton(pMode, 'Text', 'Send Mode (m)', 'Position', [280 95 110 24], ...
        'ButtonPushedFcn', @onSendMode);

    btnSendCoeffs = uibutton(pMode, 'Text', 'Send Coeffs (c)', 'Position', [10 25 140 30], ...
        'ButtonPushedFcn', @onSendCoeffs);

    cbVerify = uicheckbox(pMode, 'Text', 'Verify (t)', 'Value', true, 'Position', [160 30 120 24]);

    btnGetCoeffs = uibutton(pMode, 'Text', 'Get Coeffs (t)', 'Position', [280 25 110 30], ...
        'ButtonPushedFcn', @onGetCoeffs);

    % Control panel
    pRef = uipanel(fig, 'Title', 'Control', 'Position', [RX 315 RW 110]);

    uilabel(pRef, 'Text', 'u0 / ref', 'Position', [10 55 60 22]);
    edtU0 = uieditfield(pRef, 'numeric', 'Value', 1000, 'Position', [80 50 80 26]);

    % multiplicador de referencia r
    uilabel(pRef, 'Text', 'r x', 'Position', [165 55 25 22]);
    edtRmult = uieditfield(pRef, 'numeric', 'Value', 1.0, 'Limits', [-1e12 1e12], ...
        'Position', [190 50 55 26], 'Tooltip', 'Start envía r = (u0/ref) * (r mult)');

    btnStart = uibutton(pRef, 'Text', 'Start (i)', 'Position', [250 50 70 26], ...
        'ButtonPushedFcn', @onStart);

    btnStop = uibutton(pRef, 'Text', 'Stop (s)', 'Position', [325 50 65 26], ...
        'ButtonPushedFcn', @onStop);

    cbWaitBack = uicheckbox(pRef, 'Text', 'wait back to COMMAND', 'Value', true, ...
        'Position', [10 15 190 24]);

    cbAutoStop = uicheckbox(pRef, 'Text', 'Auto-stop', 'Value', false, ...
        'Position', [205 15 75 24]);

    uilabel(pRef, 'Text', 'at frames', 'Position', [285 15 55 22]);
    edtAutoStopN = uieditfield(pRef, 'numeric', 'Limits', [0 1e12], ...
        'RoundFractionalValues', 'on', 'Value', 0, 'Position', [345 15 45 24], ...
        'Tooltip', '0=disabled. Counts TELEMETRY FRAMES.');

    % Data panel (abajo)
    pData = uipanel(fig, 'Title', 'Data (Export / Clear)', 'Position', [RX 5 RW 85]);

    btnExportMAT = uibutton(pData, 'Text', 'Export .mat', 'Position', [10 12 120 32], ...
        'Tooltip', 'Save current n,u,y vectors to a MAT-file', 'ButtonPushedFcn', @onExportMAT);

    btnClearData = uibutton(pData, 'Text', 'Clear data', 'Position', [140 12 120 32], ...
        'Tooltip', 'Clear plotted data (does not affect device)', 'ButtonPushedFcn', @onClearData);

    % n=0 MÁS ARRIBA
    lblDataInfo = uilabel(pData, 'Text', 'n=0', 'Position', [70 64 80 22], 'HorizontalAlignment','right');

    uilabel(pData, 'Text', 'u x', 'Position', [270 20 30 18]);
    edtScaleU = uieditfield(pData, 'numeric', 'Value', S.scaleU, 'Limits', [-1e12 1e12], ...
        'Position', [300 17 70 22], 'Tooltip', 'Escala visual para u (solo plot)', ...
        'ValueChangedFcn', @onScaleChanged);

    uilabel(pData, 'Text', 'y x', 'Position', [375 20 30 18]);
    edtScaleY = uieditfield(pData, 'numeric', 'Value', S.scaleY, 'Limits', [-1e12 1e12], ...
        'Position', [405 17 70 22], 'Tooltip', 'Escala visual para y (solo plot)', ...
        'ValueChangedFcn', @onScaleChanged);

    % =========================
    % TF panel (25) -> Tabla b/a k=0..10 + order
    % =========================
    pTF = uipanel(fig, 'Title', 'TF (25): b0..b10, a0..a10, order, N, Fs', 'Position', [RX 90 RW 220]);

    uilabel(pTF, 'Text', 'k', 'Position', [10 175 20 18]);
    uilabel(pTF, 'Text', 'b(k)', 'Position', [60 175 60 18]);
    uilabel(pTF, 'Text', 'a(k)', 'Position', [185 175 60 18]);

    tfData = zeros(TF_MAX_ORDER+1, 2);
    tfData(1,2) = 1; % a0 default
    tfTable = uitable(pTF, ...
        'Data', tfData, ...
        'ColumnName', {'b','a'}, ...
        'RowName', string(0:TF_MAX_ORDER), ...
        'ColumnEditable', [true true], ...
        'Position', [10 50 260 125]);

    uilabel(pTF, 'Text', 'Order (0..10)', 'Position', [285 145 100 18]);
    edtTFOrder = uieditfield(pTF, 'numeric', ...
        'Value', 5, 'Limits', [0 TF_MAX_ORDER], 'RoundFractionalValues', 'on', ...
        'Position', [285 120 80 24], ...
        'Tooltip', 'PSoC calcula solo hasta este orden (evita ops inútiles)');

    uilabel(pTF, 'Text', '', 'Position', [285 85 120 40]);

    % =========================
    % SS panel (compacto) -> BAJADO para no pisar el título
    % =========================
    pSS = uipanel(fig, 'Title', 'SS Coeffs (3 estados / 25)', 'Position', [RX 90 RW 220]);

    % Bajamos todo para que no se superponga con el título
    yLblTop = 170;
    y1 = 145; y2 = 120; y3 = 95;
    yLblMid = 80;
    yMid    = 55;
    yK1 = 55; yK2 = 30; yK3 = 5;

    uilabel(pSS, 'Text', 'A (3x3)', 'Position', [10 yLblTop 60 18]);

    edtA11 = uieditfield(pSS, 'text', 'Value', '1', 'Position', [10  y1 60 22]);
    edtA12 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [75  y1 60 22]);
    edtA13 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [140 y1 60 22]);

    edtA21 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [10  y2 60 22]);
    edtA22 = uieditfield(pSS, 'text', 'Value', '1', 'Position', [75  y2 60 22]);
    edtA23 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [140 y2 60 22]);

    edtA31 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [10  y3 60 22]);
    edtA32 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [75  y3 60 22]);
    edtA33 = uieditfield(pSS, 'text', 'Value', '1', 'Position', [140 y3 60 22]);

    uilabel(pSS, 'Text', 'B (3x1)', 'Position', [210 yLblTop 60 18]);
    edtB1 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [210 y1 60 22]);
    edtB2 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [210 y2 60 22]);
    edtB3 = uieditfield(pSS, 'text', 'Value', '1', 'Position', [210 y3 60 22]);

    uilabel(pSS, 'Text', 'L (3x1)', 'Position', [280 yLblTop 60 18]);
    edtL1 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [280 y1 60 22]);
    edtL2 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [280 y2 60 22]);
    edtL3 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [280 y3 60 22]);

    uilabel(pSS, 'Text', 'C (1x3)', 'Position', [10 yLblMid 60 18]);
    edtC1 = uieditfield(pSS, 'text', 'Value', '1', 'Position', [10  yMid 60 22]);
    edtC2 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [75  yMid 60 22]);
    edtC3 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [140 yMid 60 22]);

    uilabel(pSS, 'Text', 'D', 'Position', [210 yLblMid 30 18]);
    edtD = uieditfield(pSS, 'text', 'Value', '0', 'Position', [210 yMid 60 22]);

    uilabel(pSS, 'Text', 'Ki', 'Position', [210 yK2 30 18]);
    edtKi = uieditfield(pSS, 'text', 'Value', '0', 'Position', [210 yK3 60 22]);

    uilabel(pSS, 'Text', 'K (3x1)', 'Position', [280 yLblMid 60 18]);
    edtK1 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [280 yK1 60 22]);
    edtK2 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [280 yK2 60 22]);
    edtK3 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [280 yK3 60 22]);

    % init
    onFsChanged();
    refreshVisibility();
    updateDataInfo();
    applyPlotScaling();

    % =====================================================================
    % Helpers
    % =====================================================================
    function refreshVisibility()
        typ = ddType.Value;
        pTF.Visible = strcmp(typ,'TF');
        pSS.Visible = strcmp(typ,'SS');

        ddObserver.Enable   = strcmp(typ,'SS');
        cbIntegrator.Enable = strcmp(typ,'SS');
    end

    function onFsChanged(~,~)
        try
            fs = double(edtFs.Value);
            if ~isfinite(fs) || fs <= 0
                lblPeriod.Text = "Fs inválida";
            else
                lblPeriod.Text = sprintf("Fs=%.6g Hz", fs);
            end
        catch
            lblPeriod.Text = "Fs=?";
        end
    end

    function updateDataInfo()
        try
            lblDataInfo.Text = sprintf("n=%d", numel(S.nVec));
        catch
        end
    end

    function onScaleChanged(~,~)
        su = double(edtScaleU.Value);
        sy = double(edtScaleY.Value);
        if ~isfinite(su) || su == 0, su = 1; end
        if ~isfinite(sy) || sy == 0, sy = 1; end
        S.scaleU = su;
        S.scaleY = sy;
        applyPlotScaling();
        drawnow limitrate;
    end

    function applyPlotScaling()
        su = double(S.scaleU); if ~isfinite(su) || su==0, su=1; end
        sy = double(S.scaleY); if ~isfinite(sy) || sy==0, sy=1; end

        if isempty(S.nVec)
            set(hU, 'XData', nan, 'YData', nan);
            set(hY, 'XData', nan, 'YData', nan);
        else
            set(hU, 'XData', S.nVec, 'YData', S.uVec * su);
            set(hY, 'XData', S.nVec, 'YData', S.yVec * sy);
        end
    end

    function ok = requireConn()
        ok = S.isConnected && ~isempty(S.sp);
        if ~ok
            logMsg("Not connected.");
        end
    end

    function logMsg(msg)
        ts = string(datestr(now,'HH:MM:SS.FFF'));
        line = "[" + ts + "] " + string(msg);
        v = txtLog.Value; if ~isstring(v), v = string(v); end
        v(end+1,1) = line;
        if numel(v) > 400, v = v(end-400:end); end
        txtLog.Value = v;
        drawnow limitrate;
    end

    function mode = computeMode()
        typ = ddType.Value;

        if strcmp(typ,'TF'),        mode = 0; return; end
        if strcmp(typ,'Open-loop'), mode = 5; return; end

        isAct = strcmp(ddObserver.Value,'Actual');
        hasI  = cbIntegrator.Value;

        if ~isAct && ~hasI
            mode = 1;
        elseif isAct && ~hasI
            mode = 2;
        elseif ~isAct && hasI
            mode = 3;
        else
            mode = 4;
        end
    end

    function rm = getRmult()
        rm = double(edtRmult.Value);
        if ~isfinite(rm), rm = 1.0; end
    end

    function ord = getTFOrder()
        ord = round(double(edtTFOrder.Value));
        if ~isfinite(ord), ord = 5; end
        ord = max(0, min(TF_MAX_ORDER, ord));
    end

    function [b,a] = readTFTable()
        D = tfTable.Data;

        if iscell(D)
            % convertir celdas a double
            D2 = nan(size(D));
            for ii = 1:size(D,1)
                for jj = 1:size(D,2)
                    v = D{ii,jj};
                    if isstring(v) || ischar(v)
                        x = str2double(strtrim(string(v)));
                    else
                        x = double(v);
                    end
                    D2(ii,jj) = x;
                end
            end
            D = D2;
        else
            D = double(D);
        end

        if any(~isfinite(D), 'all')
            error("TF table: hay NaN/inf o texto inválido.");
        end

        b = D(:,1).'; % b0..b10
        a = D(:,2).'; % a0..a10
    end

    % =====================================================================
    % Acciones UI
    % =====================================================================
    function onExportMAT(~,~)
        try
            if isempty(S.nVec)
                uialert(fig, "No hay datos para exportar.", "Export");
                return;
            end
            [file, path] = uiputfile("*.mat", "Guardar datos (.mat)", "psoc_stream.mat");
            if isequal(file,0) || isequal(path,0)
                logMsg("Export cancelado.");
                return;
            end

            data = struct();
            data.n = S.nVec;
            data.u = S.uVec;
            data.y = S.yVec;
            data.framesTotal = S.framesTotal;
            data.timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS.FFF');

            data.meta = struct();
            data.meta.com   = string(edtCom.Value);
            data.meta.baud  = double(edtBaud.Value);
            data.meta.type  = string(ddType.Value);
            data.meta.mode  = computeMode();
            data.meta.N_ui  = double(edtN.Value);
            data.meta.Fs_ui = double(edtFs.Value);
            data.meta.r_ui  = double(edtU0.Value);
            data.meta.rMult = double(getRmult());
            data.meta.tfOrder = double(getTFOrder());
            data.meta.scaleU = double(S.scaleU);
            data.meta.scaleY = double(S.scaleY);

            save(fullfile(path,file), "-struct", "data");
            logMsg("Export OK: " + string(fullfile(path,file)));
        catch e
            logMsg("Export FAIL: " + string(e.message));
            try uialert(fig, e.message, "Export error"); catch, end
        end
    end

    function onClearData(~,~)
        try
            S.nVec = zeros(0,1);
            S.uVec = zeros(0,1);
            S.yVec = zeros(0,1);
            S.streamRxBuf = uint8([]);
            S.framesTotal = 0;

            S.scaleU = 0.01;
            S.scaleY = 1.0;
            edtScaleU.Value = S.scaleU;
            edtScaleY.Value = S.scaleY;

            S.autoStopPending = false;
            S.autoStopReason  = "";
            S.autoStopStartBase = 0;
            S.autoStopArmed = false;

            applyPlotScaling();
            updateDataInfo();
            logMsg("Data cleared (plot + buffers).");
        catch e
            logMsg("Clear FAIL: " + string(e.message));
        end
    end

    function onConnectToggle(~,~)
        if ~S.isConnected
            com = strtrim(string(edtCom.Value));
            if com == ""
                logMsg("COM vacío.");
                return;
            end
            try
                S.sp = uartp_open(com, edtBaud.Value);
                S.isConnected = true;
                btnConnect.Text = "Disconnect";
                lblStat.Text = "CONNECTED: " + com;

                try flush(S.sp); catch, end

                S.streamParseEnabled = true;
                S.streamRxBuf = uint8([]);
                S.autoStopPending = false;
                S.autoStopArmed = false;

                logMsg("Connected to " + com);
                startStreaming();
            catch e
                logMsg("Connect error: " + string(e.message));
                stopStreaming();
            end
        else
            try
                stopStreaming();
                if ~isempty(S.sp)
                    try flush(S.sp); catch, end
                    clear S.sp;
                end
            catch
            end
            S.sp = [];
            S.isConnected = false;
            btnConnect.Text = "Connect";
            lblStat.Text = "DISCONNECTED";
            logMsg("Disconnected.");
        end
    end

    function onReset(~,~)
        if ~requireConn(); return; end
        try
            stopStreaming();
            try flush(S.sp); catch, end
            uartp_reset(S.sp);
            try flush(S.sp); catch, end

            S.streamParseEnabled = false;
            S.streamRxBuf = uint8([]);
            S.autoStopPending = false;
            S.autoStopArmed = false;

            startStreaming();
            logMsg("reset OK (parsing disabled until Start)");
        catch e
            logMsg("reset FAIL: " + string(e.message));
            startStreaming();
        end
    end

    function onSendMode(~,~)
        if ~requireConn(); return; end
        try
            stopStreaming();
            try flush(S.sp); catch, end

            mode = computeMode();
            uartp_setmode(S.sp, mode);

            try flush(S.sp); catch, end

            S.streamParseEnabled = false;
            S.streamRxBuf = uint8([]);
            S.autoStopPending = false;
            S.autoStopArmed = false;

            startStreaming();
            logMsg(sprintf("setmode OK: mode=%d (%s) (parsing disabled until Start)", mode, ddType.Value));
        catch e
            logMsg("setmode FAIL: " + string(e.message));
            startStreaming();
        end
    end

    function onSendCoeffs(~,~)
        if ~requireConn(); return; end
        typ = ddType.Value;

        try
            stopStreaming();
            try flush(S.sp); catch, end

            Nval = single(round(double(edtN.Value)));
            fs = double(edtFs.Value);
            if ~isfinite(fs) || fs <= 0
                error("Fs inválida (Hz).");
            end

            coeffs = zeros(NCOEFF,1,'single');

            if strcmp(typ,'TF')
                [b,a] = readTFTable();
                ord = getTFOrder();

                % Anula coeficientes > order (para coherencia)
                b(ord+2:end) = 0;
                a(ord+2:end) = 0;

                % Empaquetado TF25:
                % c1..c11 = b0..b10
                % c12..c22 = a0..a10
                coeffs(1:11)   = single(b(:));
                coeffs(12:22)  = single(a(:));

                % meta
                coeffs(IDX_TF_ORDER) = single(ord);
                coeffs(IDX_META_N)   = Nval;
                coeffs(IDX_META_FS)  = single(fs);

            elseif strcmp(typ,'SS')
                A = [ ...
                    str2double(string(edtA11.Value)), str2double(string(edtA12.Value)), str2double(string(edtA13.Value)); ...
                    str2double(string(edtA21.Value)), str2double(string(edtA22.Value)), str2double(string(edtA23.Value)); ...
                    str2double(string(edtA31.Value)), str2double(string(edtA32.Value)), str2double(string(edtA33.Value)) ...
                ];
                B  = [str2double(string(edtB1.Value)); str2double(string(edtB2.Value)); str2double(string(edtB3.Value))];
                Cc = [str2double(string(edtC1.Value)), str2double(string(edtC2.Value)), str2double(string(edtC3.Value))];
                D  = str2double(string(edtD.Value));
                L  = [str2double(string(edtL1.Value)); str2double(string(edtL2.Value)); str2double(string(edtL3.Value))];
                K  = [str2double(string(edtK1.Value)); str2double(string(edtK2.Value)); str2double(string(edtK3.Value))];
                Ki = str2double(string(edtKi.Value));

                coeffs = uartp_make_ss(A, B, Cc, D, L, K, Ki, Nval, fs);
                coeffs = single(coeffs(:));
                if numel(coeffs) ~= NCOEFF
                    error("SS: uartp_make_ss devolvió %d coef (esperaba %d).", numel(coeffs), NCOEFF);
                end

            else
                % Open-loop: solo meta
                coeffs(IDX_META_N)  = Nval;
                coeffs(IDX_META_FS) = single(fs);
            end

            uartp_send_coeffs(S.sp, coeffs, cbVerify.Value);
            try flush(S.sp); catch, end

            S.streamParseEnabled = false;
            S.streamRxBuf = uint8([]);
            S.autoStopPending = false;
            S.autoStopArmed = false;

            startStreaming();
            logMsg(sprintf("coeffs OK (%s) (25 floats). Parsing disabled until Start.", typ));

        catch e
            logMsg("coeffs FAIL: " + string(e.message));
            startStreaming();
        end
    end

    function onGetCoeffs(~,~)
        if ~requireConn(); return; end
        try
            stopStreaming();
            try flush(S.sp); catch, end

            c = uartp_get_coeffs(S.sp);
            c = single(c(:));
            if numel(c) < 25
                error("get_coeffs: llegaron %d floats (<25).", numel(c));
            end

            typ = ddType.Value;

            if strcmp(typ,'TF')
                b = double(c(1:11)).';
                a = double(c(12:22)).';
                ord = double(c(IDX_TF_ORDER));
                N   = double(c(IDX_META_N));
                fs  = double(c(IDX_META_FS));

                tfTable.Data = [b(:), a(:)];
                edtTFOrder.Value = min(max(round(ord),0),TF_MAX_ORDER);

                logMsg("t OK (TF25)");
                logMsg(sprintf("order=%g  N=%g  Fs=%.6g", ord, N, fs));

            elseif strcmp(typ,'SS')
                N  = double(c(IDX_META_N));
                fs = double(c(IDX_META_FS));
                logMsg("t OK (SS25)");
                logMsg(sprintf("meta: N=%g Fs=%.6g", N, fs));

            else
                N  = double(c(IDX_META_N));
                fs = double(c(IDX_META_FS));
                logMsg("t OK (Open-loop / 25)");
                logMsg(sprintf("meta: N=%g Fs=%.6g", N, fs));
            end

            try flush(S.sp); catch, end

            S.streamParseEnabled = false;
            S.streamRxBuf = uint8([]);
            S.autoStopPending = false;
            S.autoStopArmed = false;

            startStreaming();
        catch e
            logMsg("t FAIL: " + string(e.message));
            startStreaming();
        end
    end

    function onStart(~,~)
        if ~requireConn(); return; end
        try
            stopStreaming();
            try flush(S.sp); catch, end

            r_ui   = double(edtU0.Value);
            r_mult = getRmult();
            r_send = r_ui * r_mult;

            uartp_init(S.sp, r_send);

            S.autoStopEnabled = logical(cbAutoStop.Value);
            tgt = round(double(edtAutoStopN.Value));
            if ~isfinite(tgt) || tgt < 0, tgt = 0; end

            S.autoStopTarget    = tgt;
            S.autoStopStartBase = S.framesTotal;
            S.autoStopPending   = false;
            S.autoStopReason    = "";
            S.autoStopArmed = (S.autoStopEnabled && S.autoStopTarget > 0);

            S.streamParseEnabled = true;
            S.streamRxBuf = uint8([]);

            try flush(S.sp); catch, end
            startStreaming();

            logMsg(sprintf("init OK: r_ui=%.6g  r_mult=%.6g  r_sent=%.6g (parsing enabled)", r_ui, r_mult, r_send));
        catch e
            logMsg("init FAIL: " + string(e.message));
            S.streamParseEnabled = true;
            startStreaming();
        end
    end

    function onStop(~,~)
        if ~requireConn(); return; end
        try
            S.autoStopPending = false;
            S.autoStopArmed = false;

            S.streamParseEnabled = false;
            stopStreaming();
            try flush(S.sp); catch, end

            uartp_stop(S.sp, cbWaitBack.Value);

            try flush(S.sp); catch, end
            S.streamRxBuf = uint8([]);

            startStreaming();
            logMsg("stop OK (parsing disabled; UART bytes discarded)");
        catch e
            logMsg("stop FAIL: " + string(e.message));
            S.streamParseEnabled = false;
            stopStreaming();
        end
    end

    % =====================================================================
    % Streaming
    % =====================================================================
    function startStreaming()
        stopStreaming();

        if ~S.isConnected || isempty(S.sp)
            return;
        end

        if isempty(S.t0)
            S.t0 = tic; %#ok<NASGU>
        end

        S.streamTimer = timer( ...
            'ExecutionMode', 'fixedSpacing', ...
            'Period', 0.02, ...
            'TimerFcn', @onStreamTick, ...
            'BusyMode', 'drop' ...
        );
        start(S.streamTimer);

        if S.streamParseEnabled
            logMsg("Streaming reader ON (parsing ENABLED) (frames 8B: u,y)");
        else
            logMsg("Streaming reader ON (parsing DISABLED) (discarding UART bytes)");
        end
    end

    function stopStreaming()
        try
            if ~isempty(S.streamTimer) && isvalid(S.streamTimer)
                stop(S.streamTimer);
                delete(S.streamTimer);
            end
        catch
        end
        S.streamTimer = [];
    end

    function onStreamTick(~,~)
        if ~S.isConnected || isempty(S.sp)
            return;
        end
        sp = S.sp;

        try
            if S.autoStopPending
                S.autoStopPending = false;
                doAutoStopNow();
                return;
            end

            nAvail = sp.NumBytesAvailable;
            if nAvail <= 0, return; end

            raw = read(sp, nAvail, "uint8");
            if isempty(raw), return; end

            if ~S.streamParseEnabled
                S.streamRxBuf = uint8([]);
                return;
            end

            S.streamRxBuf = [S.streamRxBuf; uint8(raw(:))];

            n = numel(S.streamRxBuf);
            nFrames = floor(n/8);
            if nFrames <= 0, return; end

            take = 8*nFrames;
            blk = S.streamRxBuf(1:take);
            S.streamRxBuf = S.streamRxBuf(take+1:end);

            frames = reshape(blk, 8, []).';

            u_u32 = uint32(frames(:,1)) ...
                + bitshift(uint32(frames(:,2)), 8) ...
                + bitshift(uint32(frames(:,3)),16) ...
                + bitshift(uint32(frames(:,4)),24);

            y_u32 = uint32(frames(:,5)) ...
                + bitshift(uint32(frames(:,6)), 8) ...
                + bitshift(uint32(frames(:,7)),16) ...
                + bitshift(uint32(frames(:,8)),24);

            u = typecast(u_u32, 'single');
            y = typecast(y_u32, 'single');

            idx0 = S.framesTotal;
            nIdx = (idx0 + (1:numel(u))).';

            S.nVec = [S.nVec; nIdx];
            S.uVec = [S.uVec; double(u)];
            S.yVec = [S.yVec; double(y)];

            if numel(S.nVec) > S.maxPoints
                k0 = numel(S.nVec) - S.maxPoints + 1;
                S.nVec = S.nVec(k0:end);
                S.uVec = S.uVec(k0:end);
                S.yVec = S.yVec(k0:end);
            end

            S.framesTotal = S.framesTotal + numel(u);
            applyPlotScaling();
            updateDataInfo();

            if S.autoStopArmed
                framesSinceStart = S.framesTotal - S.autoStopStartBase;
                if framesSinceStart >= S.autoStopTarget
                    S.autoStopArmed   = false;
                    S.autoStopPending = true;
                    S.autoStopReason  = sprintf("framesSinceStart=%d >= %d", framesSinceStart, S.autoStopTarget);
                    logMsg("Auto-stop TRIGGER (deferred): " + S.autoStopReason);
                end
            end

            drawnow limitrate;

        catch e
            S.streamRxBuf = uint8([]);
            try flush(sp); catch, end
            logMsg("stream WARN: " + string(e.message));
        end
    end

    function doAutoStopNow()
        if ~requireConn(), return; end
        try
            S.streamParseEnabled = false;
            S.streamRxBuf = uint8([]);

            stopStreaming();
            try flush(S.sp); catch, end

            logMsg("Auto-stop: sending STOP... (" + S.autoStopReason + ")");
            uartp_stop(S.sp, cbWaitBack.Value);

            try flush(S.sp); catch, end
            S.streamRxBuf = uint8([]);

            startStreaming();
            logMsg("Auto-stop: stop OK (parsing disabled; draining UART).");
        catch e
            logMsg("Auto-stop: stop FAIL: " + string(e.message));
            S.streamParseEnabled = false;
            try startStreaming(); catch, end
        end
    end

    function onClose(~,~)
        try
            stopStreaming();
            if S.isConnected && ~isempty(S.sp)
                try flush(S.sp); catch, end
                clear S.sp;
            end
        catch
        end
        delete(fig);
    end

end
