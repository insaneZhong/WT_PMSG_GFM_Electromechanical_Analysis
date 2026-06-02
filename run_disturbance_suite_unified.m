function suite = run_disturbance_suite_unified(cfg)
% 三类扰动统一仿真：相角扰动、风速阶跃、电压跌落
% 说明：
% 1) 风速阶跃：严格时域阶跃（Wind_Step）
% 2) 电压跌落：通过 Three-Phase Breaker 的开断时序实现
% 3) 相角扰动：当前模型源为 AC Voltage Source，暂用“相角偏置等效工况”
%    （非时域瞬时跳变）。若后续换成 Programmable Voltage Source，可升级为真阶跃。

if nargin < 1
    cfg = struct();
end
cfg = set_default(cfg, 'simStop', 10.0);
cfg = set_default(cfg, 'pref', 1e6);
cfg = set_default(cfg, 'qref', 0);
cfg = set_default(cfg, 'vacRef', 563);
cfg = set_default(cfg, 'vdcRef', 5000);
cfg = set_default(cfg, 'windStepMps', 0.5);
cfg = set_default(cfg, 'phaseStepDeg', 10);
cfg = set_default(cfg, 'sagStart', 3.0);
cfg = set_default(cfg, 'sagEnd', 3.15);
cfg = set_default(cfg, 'useSchemeA', false);

root = fileparts(mfilename('fullpath'));
outDir = fullfile(root, 'Validation_Results', 'Disturbance_Suite');
if ~exist(outDir, 'dir'), mkdir(outDir); end

compile_controller(cfg.useSchemeA);
run('GFM_MWT_Nonlinear_Params.m');

suite = struct();
suite.meta = cfg;
suite.baseline = run_one_case(cfg, struct('name','baseline'));
suite.wind_step = run_one_case(cfg, struct('name','wind_step', 'windStepMps', cfg.windStepMps));
suite.voltage_sag = run_one_case(cfg, struct('name','voltage_sag', 'sagStart', cfg.sagStart, 'sagEnd', cfg.sagEnd));
suite.phase_step = run_one_case(cfg, struct('name','phase_step', 'phaseStepDeg', cfg.phaseStepDeg));

save(fullfile(outDir, 'disturbance_suite_results.mat'), 'suite');
T = local_metrics_table(suite);
writetable(T, fullfile(outDir, 'disturbance_suite_metrics.csv'));
plot_suite(suite, outDir);
fprintf('Saved: %s\n', fullfile(outDir, 'disturbance_suite_metrics.csv'));
end

function r = run_one_case(cfg, caseCfg)
mdl = 'Grid_Forming_PMSG';
load_system(mdl);

% Common control setup
ctrlBlk = [mdl '/MOTOR_CONTROL1'];
ctrlSfun = [ctrlBlk '/S-Function1'];
set_param(ctrlBlk, 'Pref', num2str(cfg.pref), 'Qref', num2str(cfg.qref), ...
    'ReferenceACVoltage', num2str(cfg.vacRef), 'ReferenceDCVoltage', num2str(cfg.vdcRef));
set_param(ctrlSfun, 'Parameters', sprintf('%.12g, %.12g, %.12g, %.12g', cfg.pref, cfg.qref, cfg.vacRef, cfg.vdcRef));

% Reset grid source phases/amplitudes and breaker control
set_param([mdl '/A'], 'Phase', '0', 'Amplitude', '563');
set_param([mdl '/B'], 'Phase', '-120', 'Amplitude', '563');
set_param([mdl '/C1'], 'Phase', '120', 'Amplitude', '563');
set_param([mdl '/Three-Phase Breaker'], 'External', 'on');

assignin('base', 'sim_stop_time_override', cfg.simStop);
assignin('base', 'wind_step_mps_override', 0.0);

% Disturbance-specific setup
if isfield(caseCfg, 'windStepMps')
    assignin('base', 'wind_step_mps_override', caseCfg.windStepMps);
end
if isfield(caseCfg, 'sagStart') && isfield(caseCfg, 'sagEnd')
    set_param([mdl '/Three-Phase Breaker'], 'External', 'off', ...
        'SwitchTimes', sprintf('[%g %g]', caseCfg.sagStart, caseCfg.sagEnd), ...
        'InitialState', 'closed');
end
if isfield(caseCfg, 'phaseStepDeg')
    % 当前模型无可编程三相源，暂用“工况偏置”等效
    d = caseCfg.phaseStepDeg;
    set_param([mdl '/A'], 'Phase', num2str(d));
    set_param([mdl '/B'], 'Phase', num2str(d-120));
    set_param([mdl '/C1'], 'Phase', num2str(d+120));
end

set_param(mdl, 'StopTime', num2str(cfg.simStop));
set_param(mdl, 'SimulationCommand', 'update');
simOut = sim(mdl, 'ReturnWorkspaceOutputs', 'on');

r = struct();
r.name = caseCfg.name;
r.signals = local_get_signals(simOut);
r.metrics = local_metrics(r.signals, 1.0);
end

function S = local_get_signals(simOut)
names = {'omega_g','theta_tw','T_sh','pmeas_out','udc_meas'};
S = struct();
for k = 1:numel(names)
    nm = names{k};
    if isa(simOut,'Simulink.SimulationOutput') && any(strcmp(who(simOut), nm))
        v = simOut.get(nm);
        if isa(v, 'timeseries')
            S.(nm).t = v.Time(:);
            S.(nm).y = v.Data(:);
        elseif isstruct(v) && isfield(v,'time') && isfield(v,'signals') && isfield(v.signals,'values')
            S.(nm).t = v.time(:);
            S.(nm).y = v.signals.values(:);
        else
            S.(nm).t = zeros(0,1);
            S.(nm).y = zeros(0,1);
        end
    elseif evalin('base', sprintf('exist(''%s'',''var'')', nm))
        v = evalin('base', nm);
        if isa(v, 'timeseries')
            S.(nm).t = v.Time(:);
            S.(nm).y = v.Data(:);
        elseif isstruct(v) && isfield(v,'time') && isfield(v,'signals') && isfield(v.signals,'values')
            S.(nm).t = v.time(:);
            S.(nm).y = v.signals.values(:);
        else
            S.(nm).t = zeros(0,1);
            S.(nm).y = zeros(0,1);
        end
    else
        S.(nm).t = zeros(0,1);
        S.(nm).y = zeros(0,1);
    end
end
end

function m = local_metrics(S, tailSec)
m = struct();
m.udc_slope = tail_slope(S.udc_meas, tailSec);
m.omega_g_slope = tail_slope(S.omega_g, tailSec);
m.theta_tw_slope = tail_slope(S.theta_tw, tailSec);
m.T_sh_slope = tail_slope(S.T_sh, tailSec);
m.pmeas_mean = tail_mean(S.pmeas_out, tailSec);
end

function T = local_metrics_table(suite)
cases = {'baseline','wind_step','voltage_sag','phase_step'};
rows = struct('caseName',{},'udc_slope',{},'omega_g_slope',{},'theta_tw_slope',{},'T_sh_slope',{},'pmeas_mean',{});
for i = 1:numel(cases)
    c = suite.(cases{i});
    rows(end+1) = struct('caseName', string(c.name), ...
        'udc_slope', c.metrics.udc_slope, 'omega_g_slope', c.metrics.omega_g_slope, ...
        'theta_tw_slope', c.metrics.theta_tw_slope, 'T_sh_slope', c.metrics.T_sh_slope, ...
        'pmeas_mean', c.metrics.pmeas_mean); %#ok<AGROW>
end
T = struct2table(rows);
end

function plot_suite(suite, outDir)
fig = figure('Color','w','Position',[100 80 1250 760]);
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
nexttile; hold on; plot_case_group(suite, 'omega_g'); title('\omega_g');
nexttile; hold on; plot_case_group(suite, 'theta_tw'); title('\theta_{tw}');
nexttile; hold on; plot_case_group(suite, 'T_sh'); title('T_{sh}');
nexttile; hold on; plot_case_group(suite, 'udc_meas'); title('U_{dc}');
saveas(fig, fullfile(outDir, 'disturbance_suite_compare.png'));
end

function plot_case_group(suite, sigName)
cases = {'baseline','wind_step','voltage_sag','phase_step'};
for i = 1:numel(cases)
    d = suite.(cases{i}).signals.(sigName);
    if ~isempty(d.t)
        plot(d.t, d.y, 'LineWidth', 1.1, 'DisplayName', suite.(cases{i}).name);
    end
end
xlabel('Time (s)'); grid on; legend('Location','best');
end

function m = tail_mean(s, win)
if ~isfield(s,'t') || ~isfield(s,'y') || isempty(s.t) || isempty(s.y)
    m = NaN;
    return;
end
idx = s.t >= (s.t(end)-win);
m = mean(s.y(idx), 'omitnan');
end

function k = tail_slope(s, win)
if ~isfield(s,'t') || ~isfield(s,'y') || isempty(s.t) || isempty(s.y)
    k = NaN;
    return;
end
idx = s.t >= (s.t(end)-win);
if nnz(idx) < 3, k = NaN; return; end
p = polyfit(s.t(idx), s.y(idx), 1); k = p(1);
end

function compile_controller(useSchemeA)
setenv('MW_MINGW64_LOC', 'C:\mingw64');
evalin('base', 'clear mex');
evalin('base', 'bdclose(''all'')');
if useSchemeA
    evalin('base', 'mex -DENABLE_SCHEMEA_OVERRIDES main.c svpwm.c motorcontrol.c grid_forming_control.c;');
else
    evalin('base', 'mex main.c svpwm.c motorcontrol.c grid_forming_control.c;');
end
end

function cfg = set_default(cfg, name, value)
if ~isfield(cfg, name) || isempty(cfg.(name)), cfg.(name) = value; end
end
