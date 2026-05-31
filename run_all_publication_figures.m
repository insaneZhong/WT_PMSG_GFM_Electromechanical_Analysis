function results = run_all_publication_figures(cfg)
%RUN_ALL_PUBLICATION_FIGURES One-click no-disturbance simulation and plotting.
%
% Default workflow:
%   1) Compile and simulate baseline controller.
%   2) Compile and simulate Scheme-A controller.
%   3) Save raw data, summary metrics, PNG and FIG files.
%   4) Plot available diagnostic CSV results if they already exist.
%
% Optional:
%   cfg.runSlowScans = true;  % also run slow diagnostic scans before plotting

if nargin < 1
    cfg = struct();
end
cfg = apply_defaults(cfg);

root = fileparts(mfilename('fullpath'));
old = pwd;
cleanup = onCleanup(@() cd(old)); %#ok<NASGU>
cd(root);

outDir = fullfile(root, 'Validation_Results');
figDir = fullfile(outDir, 'Figures');
dataDir = fullfile(outDir, 'Figure_Data');
ensure_dir(outDir);
ensure_dir(figDir);
ensure_dir(dataDir);

if cfg.refreshTables
    build_same_object_parameter_table();
    export_schemeA_c_tuning_from_small_signal();
end

if cfg.runSlowScans
    diagnose_1mw_steady_root_cause('quick');
    scan_delivery_limits_no_disturbance();
end

scenarios = make_scenarios(cfg);
results = struct();
summaryRows = [];

for k = 1:numel(scenarios)
    s = scenarios(k);
    fprintf('\n[%d/%d] Running scenario: %s\n', k, numel(scenarios), s.name);
    runScenario = run_one_scenario(s, cfg);
    results.(s.name) = runScenario;
    summaryRows = [summaryRows; make_summary_row(s, runScenario)]; %#ok<AGROW>
    save(fullfile(dataDir, sprintf('%s_timeseries.mat', s.name)), '-struct', 'runScenario');
end

summary = struct2table(summaryRows);
writetable(summary, fullfile(outDir, 'publication_figure_summary.csv'));
save(fullfile(dataDir, 'publication_figure_results.mat'), 'results', 'summary', 'cfg');

plot_overview(results, summary, figDir);
plot_mechanical(results, figDir);
plot_control(results, figDir);
plot_power_dc(results, figDir);
plot_existing_diagnostics(outDir, figDir);

fprintf('\nFigures saved in:\n  %s\n', figDir);
fprintf('Summary saved:\n  %s\n', fullfile(outDir, 'publication_figure_summary.csv'));
end

function cfg = apply_defaults(cfg)
cfg = set_default(cfg, 'pref', 1e6);
cfg = set_default(cfg, 'qref', 0);
cfg = set_default(cfg, 'vacRef', 563);
cfg = set_default(cfg, 'vdcRef', 5000);
cfg = set_default(cfg, 'simStop', 3.0);
cfg = set_default(cfg, 'tailWindow', 1.0);
cfg = set_default(cfg, 'runBaseline', true);
cfg = set_default(cfg, 'runSchemeA', true);
cfg = set_default(cfg, 'runSlowScans', false);
cfg = set_default(cfg, 'refreshTables', true);
cfg = set_default(cfg, 'figureFormat', 'png');
end

function cfg = set_default(cfg, name, value)
if ~isfield(cfg, name) || isempty(cfg.(name))
    cfg.(name) = value;
end
end

function scenarios = make_scenarios(cfg)
scenarios = struct('name', {}, 'schemeA', {});
if cfg.runBaseline
    scenarios(end+1) = struct('name', 'baseline', 'schemeA', false); %#ok<AGROW>
end
if cfg.runSchemeA
    scenarios(end+1) = struct('name', 'schemeA', 'schemeA', true); %#ok<AGROW>
end
end

function r = run_one_scenario(s, cfg)
setenv('MW_MINGW64_LOC','C:\mingw64');
evalin('base','clear mex');
evalin('base','bdclose(''all'')');

if s.schemeA
    mexCmd = 'mex -DENABLE_SCHEMEA_OVERRIDES main.c svpwm.c motorcontrol.c grid_forming_control.c;';
else
    mexCmd = 'mex main.c svpwm.c motorcontrol.c grid_forming_control.c;';
end
evalin('base', mexCmd);

assignin('base','P_wt_rated_override',cfg.pref);
assignin('base','wind_step_mps_override',0.0);
assignin('base','sim_stop_time_override',cfg.simStop);
run('GFM_MWT_Nonlinear_Params.m');

mdl = 'Grid_Forming_PMSG';
load_system(mdl);
set_param(mdl, 'StopTime', num2str(cfg.simStop));
set_param([mdl '/MOTOR_CONTROL1'], ...
    'Pref', num2str(cfg.pref), ...
    'Qref', num2str(cfg.qref), ...
    'ReferenceACVoltage', num2str(cfg.vacRef), ...
    'ReferenceDCVoltage', num2str(cfg.vdcRef));
set_param([mdl '/MOTOR_CONTROL1/S-Function1'], 'Parameters', ...
    sprintf('%.12g, %.12g, %.12g, %.12g', cfg.pref, cfg.qref, cfg.vacRef, cfg.vdcRef));

set_param(mdl, 'SimulationCommand', 'update');
sim(mdl);

r = collect_run_data(cfg.tailWindow);
r.meta = s;
r.cfg = cfg;
end

function r = collect_run_data(tailWindow)
names = { ...
    'pref_out','pmeas_out','wref_out','presyn_out','udc_meas', ...
    'omega_t','omega_g','theta_tw','T_sh', ...
    'idref_out','id_out','ud1ref_out', ...
    'pcc_ia','pcc_ib','pcc_ic','pcc_uab','pcc_ubc','pcc_uca'};

for i = 1:numel(names)
    r.(names{i}) = get_series(names{i});
end

r.metrics = struct();
r.metrics.Pref_end = tail_value(r.pref_out);
r.metrics.Pmeas_mean = tail_mean(r.pmeas_out, tailWindow);
r.metrics.Udc_mean = tail_mean(r.udc_meas, tailWindow);
r.metrics.Udc_slope = tail_slope(r.udc_meas, tailWindow);
r.metrics.Wref_mean = tail_mean(r.wref_out, tailWindow);
r.metrics.PreSyn_mean = tail_mean(r.presyn_out, tailWindow);
r.metrics.OmegaG_slope = tail_slope(r.omega_g, tailWindow);
r.metrics.ThetaTw_slope = tail_slope(r.theta_tw, tailWindow);
end

function row = make_summary_row(s, r)
m = r.metrics;
row = struct( ...
    'scenario', string(s.name), ...
    'schemeA', double(s.schemeA), ...
    'Pref_end', m.Pref_end, ...
    'Pmeas_mean', m.Pmeas_mean, ...
    'Udc_mean', m.Udc_mean, ...
    'Udc_slope', m.Udc_slope, ...
    'Wref_mean', m.Wref_mean, ...
    'PreSyn_mean', m.PreSyn_mean, ...
    'OmegaG_slope', m.OmegaG_slope, ...
    'ThetaTw_slope', m.ThetaTw_slope);
end

function s = get_series(name)
if ~evalin('base', sprintf('exist(''%s'',''var'')', name))
    s = empty_series();
    return;
end
v = evalin('base', name);

if isa(v, 'timeseries')
    s.t = v.Time(:);
    s.y = squeeze(v.Data);
    s.y = s.y(:);
elseif isstruct(v) && isfield(v, 'time') && isfield(v, 'signals')
    s.t = v.time(:);
    y = v.signals.values;
    s.y = squeeze(y);
    s.y = s.y(:);
elseif isnumeric(v)
    if evalin('base', 'exist(''tout'',''var'')')
        t = evalin('base','tout');
        s.t = t(:);
        s.y = v(:);
        if numel(s.y) ~= numel(s.t)
            s.t = (0:numel(s.y)-1).';
        end
    else
        s.t = (0:numel(v)-1).';
        s.y = v(:);
    end
else
    s = empty_series();
end
end

function s = empty_series()
s = struct('t', zeros(0,1), 'y', zeros(0,1));
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

function plot_overview(results, summary, figDir)
fig = new_fig('overview_power_dc');
tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile; hold on;
plot_each(results, 'pmeas_out', 1e-6);
yline(1, 'k--', '1 MW');
xlabel('Time (s)'); ylabel('P_{meas} (MW)'); grid on; title('Active Power');

nexttile; hold on;
plot_each(results, 'udc_meas', 1);
xlabel('Time (s)'); ylabel('U_{dc} (V)'); grid on; title('DC-link Voltage');

nexttile; hold on;
plot_each(results, 'wref_out', 1);
xlabel('Time (s)'); ylabel('\omega_{ref} (rad/s)'); grid on; title('GFM Frequency Reference');

nexttile; hold on;
bar(categorical(summary.scenario), summary.Pmeas_mean * 1e-6);
yline(1, 'k--', '1 MW');
ylabel('Tail mean P (MW)'); grid on; title('Steady Power Comparison');

save_figure(fig, figDir, 'fig01_overview_power_dc');
end

function plot_mechanical(results, figDir)
fig = new_fig('mechanical_two_mass');
tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile; hold on;
plot_each(results, 'omega_t', 1);
plot_each(results, 'omega_g', 1, '--');
xlabel('Time (s)'); ylabel('\omega (rad/s)'); grid on; title('Two-mass Speeds');

nexttile; hold on;
plot_each(results, 'theta_tw', 1);
xlabel('Time (s)'); ylabel('\theta_{tw} (rad)'); grid on; title('Shaft Twist');

nexttile; hold on;
plot_each(results, 'T_sh', 1e-3);
xlabel('Time (s)'); ylabel('T_{sh} (kN m)'); grid on; title('Shaft Torque');

save_figure(fig, figDir, 'fig02_mechanical_two_mass');
end

function plot_control(results, figDir)
fig = new_fig('control_internal');
tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile; hold on;
plot_each(results, 'idref_out', 1);
plot_each(results, 'id_out', 1, '--');
xlabel('Time (s)'); ylabel('d-axis current (A)'); grid on; title('Id Reference and Feedback');

nexttile; hold on;
plot_each(results, 'ud1ref_out', 1);
xlabel('Time (s)'); ylabel('U_{d1,ref} (V)'); grid on; title('Grid-side Voltage Command');

nexttile; hold on;
plot_each(results, 'presyn_out', 1);
xlabel('Time (s)'); ylabel('PreSyn flag'); grid on; title('Synchronization State');

save_figure(fig, figDir, 'fig03_control_internal');
end

function plot_power_dc(results, figDir)
fig = new_fig('steady_metrics');
names = fieldnames(results);
P = zeros(numel(names),1);
dUdc = zeros(numel(names),1);
dOmega = zeros(numel(names),1);
dTheta = zeros(numel(names),1);
for i = 1:numel(names)
    m = results.(names{i}).metrics;
    P(i) = m.Pmeas_mean * 1e-6;
    dUdc(i) = m.Udc_slope;
    dOmega(i) = m.OmegaG_slope;
    dTheta(i) = m.ThetaTw_slope;
end

tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
c = categorical(names);
nexttile; bar(c, P); yline(1,'k--'); ylabel('MW'); grid on; title('Power');
nexttile; bar(c, dUdc); ylabel('V/s'); grid on; title('DC-link Slope');
nexttile; bar(c, dOmega); ylabel('rad/s^2'); grid on; title('Generator Speed Slope');
nexttile; bar(c, dTheta); ylabel('rad/s'); grid on; title('Shaft Twist Slope');

save_figure(fig, figDir, 'fig04_steady_metrics');
end

function plot_existing_diagnostics(outDir, figDir)
diagCsv = fullfile(outDir, 'diagnose_1mw_steady_root_cause_quick.csv');
if exist(diagCsv, 'file') == 2
    T = readtable(diagCsv);
    fig = new_fig('diagnosis_quick');
    tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    nexttile;
    bar(T.Pmeas_mean * 1e-6); yline(1,'k--');
    xlabel('Case index'); ylabel('P_{meas} (MW)'); grid on; title('Quick Diagnosis Power');
    nexttile;
    bar(T.Udc_slope);
    xlabel('Case index'); ylabel('U_{dc} slope (V/s)'); grid on; title('Quick Diagnosis DC-link Slope');
    save_figure(fig, figDir, 'fig05_quick_diagnosis');
end

limitCsv = fullfile(outDir, 'scan_delivery_limits_no_disturbance.csv');
if exist(limitCsv, 'file') == 2
    T = readtable(limitCsv);
    fig = new_fig('delivery_limits');
    scatter(T.ploop_out_max, T.Pmeas_mean * 1e-6, 60, T.wref_max, 'filled');
    colorbar; grid on;
    xlabel('Power-loop output limit'); ylabel('P_{meas} (MW)');
    title('Delivery Limit Scan');
    save_figure(fig, figDir, 'fig06_delivery_limit_scan');
end
end

function plot_each(results, field, scale, style)
if nargin < 4
    style = '-';
end
names = fieldnames(results);
for i = 1:numel(names)
    s = results.(names{i}).(field);
    if ~isempty(s.t)
        plot(s.t, s.y * scale, style, 'LineWidth', 1.2, 'DisplayName', names{i});
    end
end
legend('Location','best');
end

function fig = new_fig(name)
fig = figure('Name', name, 'Visible', 'off', 'Color', 'w', 'Position', [80 80 1100 760]);
end

function save_figure(fig, figDir, baseName)
savefig(fig, fullfile(figDir, [baseName '.fig']));
try
    exportgraphics(fig, fullfile(figDir, [baseName '.png']), 'Resolution', 220);
catch
    print(fig, fullfile(figDir, [baseName '.png']), '-dpng', '-r220');
end
close(fig);
end

function ensure_dir(pathName)
if ~exist(pathName, 'dir')
    mkdir(pathName);
end
end

