function results = run_small_perturbation_validation(cfg)
%RUN_SMALL_PERTURBATION_VALIDATION 非线性小扰动验证模块。
%
% 使用前提：
%   1. 无扰动仿真已经基本达到稳态；
%   2. 小信号模型和非线性模型参数已经通过“同一对象参数表”统一；
%   3. 若 cfg.useSchemeA = true，则 C 控制器会使用 Scheme A 参数覆盖。
%
% 当前扰动：
%   通过 GFM_MWT_Nonlinear_Params.m 中的 wind_step_mps_override
%   施加一个很小的风速阶跃。建议先用 0.05~0.10 m/s，避免大扰动把
%   小信号/非线性对应关系打破。

if nargin < 1
    cfg = struct();
end
cfg = local_defaults(cfg);

root = fileparts(mfilename('fullpath'));
oldDir = pwd;
cleanupObj = onCleanup(@() cd(oldDir)); %#ok<NASGU>
cd(root);

outDir = fullfile(root, 'Validation_Results', 'Small_Perturbation');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

compile_controller(cfg.useSchemeA);

assignin('base', 'P_wt_rated_override', cfg.pref);
assignin('base', 'wind_step_mps_override', cfg.windStepMps);
assignin('base', 'sim_stop_time_override', cfg.simStop);
run('GFM_MWT_Nonlinear_Params.m');

mdl = 'Grid_Forming_PMSG';
load_system(mdl);
ctrlBlk = [mdl '/MOTOR_CONTROL1'];
ctrlSfun = [ctrlBlk '/S-Function1'];
set_param(ctrlBlk, ...
    'Pref', num2str(cfg.pref), ...
    'Qref', num2str(cfg.qref), ...
    'ReferenceACVoltage', num2str(cfg.vacRef), ...
    'ReferenceDCVoltage', num2str(cfg.vdcRef));
set_param(ctrlSfun, 'Parameters', ...
    sprintf('%.12g, %.12g, %.12g, %.12g', cfg.pref, cfg.qref, cfg.vacRef, cfg.vdcRef));
set_param(mdl, 'StopTime', num2str(cfg.simStop));
set_param(mdl, 'SimulationCommand', 'update');
sim(mdl);

results = collect_results(cfg);
results.cfg = cfg;
save(fullfile(outDir, 'small_perturbation_results.mat'), 'results');
writetable(struct2table(results.metrics), fullfile(outDir, 'small_perturbation_metrics.csv'));
plot_small_perturbation(results, outDir);

fprintf('\n小扰动验证完成，结果保存到：\n%s\n', outDir);
end

function cfg = local_defaults(cfg)
cfg = set_default(cfg, 'pref', 1e6);
cfg = set_default(cfg, 'qref', 0);
cfg = set_default(cfg, 'vacRef', 563);
cfg = set_default(cfg, 'vdcRef', 5000);
cfg = set_default(cfg, 'simStop', 5.0);
cfg = set_default(cfg, 'tailWindow', 1.0);
cfg = set_default(cfg, 'windStepMps', 0.10);
cfg = set_default(cfg, 'useSchemeA', true);
end

function cfg = set_default(cfg, name, value)
if ~isfield(cfg, name) || isempty(cfg.(name))
    cfg.(name) = value;
end
end

function compile_controller(useSchemeA)
setenv('MW_MINGW64_LOC', 'C:\mingw64');
evalin('base', 'clear mex');
evalin('base', 'bdclose(''all'')');
if useSchemeA
    mexCmd = 'mex -DENABLE_SCHEMEA_OVERRIDES main.c svpwm.c motorcontrol.c grid_forming_control.c;';
else
    mexCmd = 'mex main.c svpwm.c motorcontrol.c grid_forming_control.c;';
end
evalin('base', mexCmd);
end

function results = collect_results(cfg)
names = {'pref_out','pmeas_out','wref_out','presyn_out','udc_meas', ...
    'omega_t','omega_g','theta_tw','T_sh'};
for k = 1:numel(names)
    results.signals.(names{k}) = get_series(names{k});
end

tail = cfg.tailWindow;
results.metrics = struct();
results.metrics.Pref_end = tail_value(results.signals.pref_out);
results.metrics.Pmeas_mean = tail_mean(results.signals.pmeas_out, tail);
results.metrics.Pmeas_overshoot = max_abs_deviation(results.signals.pmeas_out, results.metrics.Pmeas_mean);
results.metrics.Udc_mean = tail_mean(results.signals.udc_meas, tail);
results.metrics.Udc_slope = tail_slope(results.signals.udc_meas, tail);
results.metrics.OmegaG_slope = tail_slope(results.signals.omega_g, tail);
results.metrics.ThetaTw_slope = tail_slope(results.signals.theta_tw, tail);
results.metrics.Torsion_peak_to_peak = peak_to_peak_tail(results.signals.theta_tw, tail);
end

function s = get_series(name)
if ~evalin('base', sprintf('exist(''%s'',''var'')', name))
    s = struct('t', zeros(0,1), 'y', zeros(0,1));
    return;
end
v = evalin('base', name);
if isa(v, 'timeseries')
    s.t = v.Time(:);
    s.y = squeeze(v.Data);
    s.y = s.y(:);
elseif isstruct(v) && isfield(v, 'time') && isfield(v, 'signals')
    s.t = v.time(:);
    s.y = squeeze(v.signals.values);
    s.y = s.y(:);
else
    s.t = (0:numel(v)-1).';
    s.y = v(:);
end
end

function v = tail_value(s)
if isempty(s.y)
    v = NaN;
else
    v = s.y(end);
end
end

function m = tail_mean(s, win)
idx = tail_index(s, win);
if isempty(idx)
    m = NaN;
else
    m = mean(s.y(idx), 'omitnan');
end
end

function k = tail_slope(s, win)
idx = tail_index(s, win);
if numel(idx) < 3
    k = NaN;
else
    p = polyfit(s.t(idx), s.y(idx), 1);
    k = p(1);
end
end

function idx = tail_index(s, win)
if isempty(s.t) || isempty(s.y)
    idx = [];
else
    idx = find(s.t >= s.t(end) - win);
end
end

function v = max_abs_deviation(s, ref)
if isempty(s.y) || isnan(ref)
    v = NaN;
else
    v = max(abs(s.y - ref));
end
end

function v = peak_to_peak_tail(s, win)
idx = tail_index(s, win);
if isempty(idx)
    v = NaN;
else
    v = max(s.y(idx)) - min(s.y(idx));
end
end

function plot_small_perturbation(results, outDir)
fig = figure('Color', 'w', 'Position', [80 80 1180 720]);
tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot_pair(results.signals.pref_out, results.signals.pmeas_out, 'P_{ref}', 'P_{meas}');
ylabel('Power / W');
title('小扰动后有功功率响应');

nexttile;
plot_one(results.signals.udc_meas, 'V_{dc}');
ylabel('DC voltage / V');
title('直流母线电压响应');

nexttile;
hold on
plot_one(results.signals.omega_t, '\omega_t');
plot_one(results.signals.omega_g, '\omega_g');
plot_one(results.signals.theta_tw, '\theta_{tw}');
grid on
xlabel('Time / s');
title('两质量块轴系响应');
legend('Location', 'best');

exportgraphics(fig, fullfile(outDir, 'small_perturbation_response.png'), 'Resolution', 300);
savefig(fig, fullfile(outDir, 'small_perturbation_response.fig'));
end

function plot_pair(a, b, nameA, nameB)
hold on
if ~isempty(a.t), plot(a.t, a.y, 'LineWidth', 1.2, 'DisplayName', nameA); end
if ~isempty(b.t), plot(b.t, b.y, 'LineWidth', 1.2, 'DisplayName', nameB); end
grid on
legend('Location', 'best');
end

function plot_one(s, name)
if ~isempty(s.t)
    plot(s.t, s.y, 'LineWidth', 1.2, 'DisplayName', name);
end
end
