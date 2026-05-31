function results = diagnose_grid_forming_pmsg1_no_aero(cfg)
%DIAGNOSE_GRID_FORMING_PMSG1_NO_AERO 诊断无气动基准模型的无扰动运行状态。
%
% 目标：
%   1. 不修改、不保存 Grid_Forming_PMSG1.mdl；
%   2. 临时接入 To Workspace 观测块，绕过旧模型仅记录前 0.5 s 的限制；
%   3. 记录并网前、并网瞬间和仿真末端的功率、直流电压和 GFM 内部量；
%   4. 判断模型是否真正建立 1 MW 无扰动稳态。
%
% 示例：
%   results = diagnose_grid_forming_pmsg1_no_aero();
%   results = diagnose_grid_forming_pmsg1_no_aero(struct('simStop', 8));

if nargin < 1
    cfg = struct();
end
cfg = local_defaults(cfg);

root = fileparts(mfilename('fullpath'));
oldDir = pwd;
cleanupDir = onCleanup(@() cd(oldDir)); %#ok<NASGU>
cd(root);

mdl = 'Grid_Forming_PMSG1';
bdclose('all');
if cfg.compileMex
    setenv('MW_MINGW64_LOC', 'C:\mingw64');
    clear mex
    mex(sprintf('-DPRESYN_SWITCH_TIME=%.12g', cfg.presynSwitchTime), ...
        sprintf('-DGSI_PLOOP_KP=%.12g', cfg.pLoopKp), ...
        sprintf('-DGSI_PLOOP_KI=%.12g', cfg.pLoopKi), ...
        sprintf('-DGSI_PREF_RAMP_SLOPE=%.12g', cfg.pRefRampSlope), ...
        sprintf('-DGSI_V_LOOP_KP=%.12g', cfg.voltageLoopKp), ...
        sprintf('-DGSI_V_LOOP_KI=%.12g', cfg.voltageLoopKi), ...
        'main.c', 'svpwm.c', 'motorcontrol.c', 'grid_forming_control.c')
end

load_system(mdl);
cleanupModel = onCleanup(@() bdclose(mdl)); %#ok<NASGU>

% 仅在内存里替换旧 InitFcn，避免旧编译器路径和 mex -setup 交互。
% 关闭模型时不保存，因此磁盘上的原始 mdl 文件不会被覆盖。
set_param(mdl, 'InitFcn', 'dtime=4e-6; Ts_step=1e-6;');
set_param(mdl, 'StopTime', num2str(cfg.simStop));
set_param(mdl, 'LimitDataPoints', 'off');
set_param(mdl, 'Decimation', num2str(cfg.decimation));

ctrl = [mdl '/MOTOR_CONTROL1'];
ports = get_param(ctrl, 'PortHandles');

% MOTOR_CONTROL1 对外端口映射：
% 1 Pref, 2 Pmeas, 3 wref, 4 theta, 5/6 Ud/Uq ref, 7 Qpcc,
% 8 voltage_ref, 10/14 PCC dq voltage, 11/12 Id ref/Id,
% 15 Iq ref, 16 grid phase angle, 17 Pre_syn, 18 Iq.
signals = { ...
    'pref',       ports.Outport(1); ...
    'pmeas',      ports.Outport(2); ...
    'wref',       ports.Outport(3); ...
    'theta',      ports.Outport(4); ...
    'ud1ref',     ports.Outport(5); ...
    'uq1ref',     ports.Outport(6); ...
    'qpcc',       ports.Outport(7); ...
    'voltageRef', ports.Outport(8); ...
    'pccUd',      ports.Outport(10); ...
    'idref',      ports.Outport(11); ...
    'id',         ports.Outport(12); ...
    'pccUq',      ports.Outport(14); ...
    'iqref',      ports.Outport(15); ...
    'gridPhase',  ports.Outport(16); ...
    'presyn',     ports.Outport(17); ...
    'iq',         ports.Outport(18)};

% Udc 是 MOTOR_CONTROL1 的输入端口，需要沿输入线找到源端口后再分支。
udcLine = get_param(ports.Inport(4), 'Line');
udcSourcePort = get_param(udcLine, 'SrcPortHandle');
signals(end+1, :) = {'udc', udcSourcePort}; %#ok<AGROW>

for k = 1:size(signals, 1)
    variableName = ['diag_' signals{k, 1}];
    add_toworkspace_branch(mdl, signals{k, 2}, variableName, cfg.decimation, k);
end

sim(mdl);
data = collect_data(signals(:, 1));
metrics = compute_metrics(data);
samples = key_samples(data, metrics.SyncRiseTime);

outDir = fullfile(root, 'Validation_Results', 'PMSG1_No_Aero_Diagnosis');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

writetable(struct2table(metrics), fullfile(outDir, 'pmsg1_no_aero_metrics.csv'));
writetable(samples, fullfile(outDir, 'pmsg1_no_aero_key_samples.csv'));
save(fullfile(outDir, 'pmsg1_no_aero_results.mat'), 'cfg', 'data', 'metrics', 'samples');
plot_results(data, metrics, outDir);

results = struct('cfg', cfg, 'data', data, 'metrics', metrics, 'samples', samples);

fprintf('\n无气动基准模型诊断结果：\n');
disp(struct2table(metrics));
fprintf('关键采样点：\n');
disp(samples);
fprintf('结果已保存到：\n%s\n', outDir);
end

function cfg = local_defaults(cfg)
cfg = set_default(cfg, 'simStop', 8.0);
cfg = set_default(cfg, 'decimation', 1000);
cfg = set_default(cfg, 'compileMex', true);
cfg = set_default(cfg, 'presynSwitchTime', 2.0);
cfg = set_default(cfg, 'pLoopKp', 1e-6);
cfg = set_default(cfg, 'pLoopKi', 2e-5);
cfg = set_default(cfg, 'pRefRampSlope', 5e6);
cfg = set_default(cfg, 'voltageLoopKp', 1.1309733);
cfg = set_default(cfg, 'voltageLoopKi', 0.0282743);
end

function cfg = set_default(cfg, name, value)
if ~isfield(cfg, name) || isempty(cfg.(name))
    cfg.(name) = value;
end
end

function add_toworkspace_branch(mdl, sourcePort, variableName, decimation, index)
block = [mdl '/' variableName];
add_block('simulink/Sinks/To Workspace', block, ...
    'VariableName', variableName, ...
    'SaveFormat', 'Timeseries', ...
    'Decimation', num2str(decimation), ...
    'Position', [900, 30+35*index, 1030, 50+35*index]);
destination = get_param(block, 'PortHandles');
add_line(mdl, sourcePort, destination.Inport(1), 'autorouting', 'on');
end

function data = collect_data(signalNames)
for k = 1:numel(signalNames)
    name = signalNames{k};
    variableName = ['diag_' name];
    if evalin('caller', ['exist(''' variableName ''',''var'')'])
        data.(name) = evalin('caller', variableName);
    else
        data.(name) = evalin('base', variableName);
    end
end
end

function metrics = compute_metrics(data)
syncRise = find(data.presyn.Data(1:end-1) < 0.5 & data.presyn.Data(2:end) >= 0.5, 1, 'first');
if isempty(syncRise)
    syncTime = NaN;
else
    syncTime = data.presyn.Time(syncRise+1);
end

tail = 1.0;
metrics = struct();
metrics.SimStop = data.pmeas.Time(end);
metrics.SyncRiseTime = syncTime;
metrics.PrefEnd = data.pref.Data(end);
metrics.PmeasTailMean = window_mean(data.pmeas, data.pmeas.Time(end)-tail, data.pmeas.Time(end));
metrics.PmeasTailMin = window_min(data.pmeas, data.pmeas.Time(end)-tail, data.pmeas.Time(end));
metrics.PmeasTailMax = window_max(data.pmeas, data.pmeas.Time(end)-tail, data.pmeas.Time(end));
metrics.PmeasTailSlope = window_slope(data.pmeas, data.pmeas.Time(end)-tail, data.pmeas.Time(end));
metrics.UdcPreSyncMean = window_mean(data.udc, max(0, syncTime-0.5), syncTime);
metrics.UdcTailMean = window_mean(data.udc, data.udc.Time(end)-tail, data.udc.Time(end));
metrics.UdcTailMin = window_min(data.udc, data.udc.Time(end)-tail, data.udc.Time(end));
metrics.UdcTailMax = window_max(data.udc, data.udc.Time(end)-tail, data.udc.Time(end));
metrics.UdcTailSlope = window_slope(data.udc, data.udc.Time(end)-tail, data.udc.Time(end));
metrics.WrefTailMean = window_mean(data.wref, data.wref.Time(end)-tail, data.wref.Time(end));
metrics.PresynTailMean = window_mean(data.presyn, data.presyn.Time(end)-tail, data.presyn.Time(end));
end

function samples = key_samples(data, syncTime)
requested = unique([0, 0.1, 0.5, 1.0, syncTime-0.1, syncTime, syncTime+0.01, syncTime+0.1, data.pmeas.Time(end)]);
requested = requested(~isnan(requested) & requested >= 0 & requested <= data.pmeas.Time(end));
n = numel(requested);
rows = zeros(n, 6);
for k = 1:n
    rows(k, 1) = requested(k);
    rows(k, 2) = nearest_value(data.udc, requested(k));
    rows(k, 3) = nearest_value(data.pmeas, requested(k));
    rows(k, 4) = nearest_value(data.pref, requested(k));
    rows(k, 5) = nearest_value(data.wref, requested(k));
    rows(k, 6) = nearest_value(data.presyn, requested(k));
end
samples = array2table(rows, 'VariableNames', {'Time_s', 'Udc_V', 'Pmeas_W', 'Pref_W', 'Wref_rad_s', 'PreSyn'});
end

function value = nearest_value(ts, time)
[~, index] = min(abs(ts.Time-time));
value = ts.Data(index);
end

function value = window_mean(ts, t0, t1)
idx = ts.Time >= t0 & ts.Time <= t1;
value = mean(ts.Data(idx));
end

function value = window_min(ts, t0, t1)
idx = ts.Time >= t0 & ts.Time <= t1;
value = min(ts.Data(idx));
end

function value = window_max(ts, t0, t1)
idx = ts.Time >= t0 & ts.Time <= t1;
value = max(ts.Data(idx));
end

function value = window_slope(ts, t0, t1)
idx = ts.Time >= t0 & ts.Time <= t1;
coefficient = polyfit(ts.Time(idx), ts.Data(idx), 1);
value = coefficient(1);
end

function plot_results(data, metrics, outDir)
fig = figure('Color', 'w', 'Position', [90 90 1180 860]);
tiledlayout(4, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(data.pref.Time, data.pref.Data, 'LineWidth', 1.1, 'DisplayName', 'P_{ref}');
hold on;
plot(data.pmeas.Time, data.pmeas.Data, 'LineWidth', 1.1, 'DisplayName', 'P_{meas}');
xline(metrics.SyncRiseTime, 'k--', 'DisplayName', 'PreSyn 0->1');
grid on;
ylabel('Power / W');
legend('Location', 'best');
title('Grid\_Forming\_PMSG1 无气动无扰动有功功率');

nexttile;
plot(data.udc.Time, data.udc.Data, 'LineWidth', 1.1);
hold on;
xline(metrics.SyncRiseTime, 'k--');
grid on;
ylabel('V_{dc} / V');
title('直流母线电压');

nexttile;
plot(data.wref.Time, data.wref.Data, 'LineWidth', 1.1);
hold on;
xline(metrics.SyncRiseTime, 'k--');
grid on;
ylabel('\omega_{ref} / rad s^{-1}');
title('GFM 频率参考');

nexttile;
plot(data.presyn.Time, data.presyn.Data, 'LineWidth', 1.1);
grid on;
ylabel('PreSyn');
xlabel('Time / s');
title('预同步/并网时序标志');

exportgraphics(fig, fullfile(outDir, 'pmsg1_no_aero_overview.png'), 'Resolution', 300);
savefig(fig, fullfile(outDir, 'pmsg1_no_aero_overview.fig'));
end
