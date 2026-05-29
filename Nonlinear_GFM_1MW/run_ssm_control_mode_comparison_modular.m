function results = run_ssm_control_mode_comparison_modular(eigenDir, cfg)
%RUN_SSM_CONTROL_MODE_COMPARISON_MODULAR 小信号四拓扑对照与系统参数扫描模块。
%
% 输入：
%   eigenDir : EigenAnalysis 文件夹路径，里面需要有 Parameters.m 和 Unified_*.mat。
%   cfg      : 主程序传入的配置结构体，常用字段如下：
%              cfg.sweeps      - 扫描参数结构体数组，例如 SCR、XR、C_dc。
%              cfg.modelKeys   - 需要运行的模型键值，例如 ["GFM_MWT","GFM_MWT_AD"]。
%              cfg.targetFreqHz - 需要跟踪的机电模态频率，默认 2 Hz。
%              cfg.resultDir   - 结果保存目录。
%
% 输出：
%   results.baseline       - 基准点各模型的 2 Hz 模态指标。
%   results.participation  - 基准点 2 Hz 模态参与因子前 10。
%   results.allSweeps      - 所有扫描点的模态指标。
%   results.causalSweeps   - 相邻拓扑之间的阻尼变化，用于因果阐述。

if nargin < 2
    cfg = struct();
end
cfg = local_defaults(cfg);

oldDir = pwd;
cleanupObj = onCleanup(@() cd(oldDir)); %#ok<NASGU>
cd(eigenDir);

if ~exist(cfg.resultDir, 'dir')
    mkdir(cfg.resultDir);
end
if exist(cfg.matpowerDir, 'dir') == 7
    addpath(genpath(cfg.matpowerDir));
end

fprintf('读取 Parameters.m 和统一小信号模型...\n');
% Parameters.m 末尾含有 clear all。若在函数工作区直接 run，会清空本函数变量；
% 因此通过 base 工作区执行它，再用绝对路径读回 Parameters.mat。
paramMat = fullfile(eigenDir, 'Parameters.mat');
if exist(paramMat, 'file') ~= 2
    evalin('base', sprintf('cd(''%s''); run(''Parameters.m'');', escape_path(eigenDir)));
end
baseParams = load(paramMat);
mpopt = mpoption('verbose', 0, 'out.all', 0);

models = load_model_set(cfg.modelKeys);

%% 1. 写出四类拓扑的控制结构说明
architectures = table( ...
    string({models.label}).', ...
    string({models.msc}).', ...
    string({models.gsc}).', ...
    string({models.role}).', ...
    'VariableNames', {'Model', 'MSC', 'GSC', 'CausalRole'});
writetable(architectures, fullfile(cfg.resultDir, 'model_architectures.csv'));

%% 2. 基准点模态、阻尼、参与因子
baseline = table();
participation = table();
for m = 1:numel(models)
    A = make_A(models(m).data, mpopt, baseParams);
    [metric, modeIndex] = torsion_metric(A, models(m).data.X_stac, cfg.targetFreqHz);
    metric.Model = string(models(m).label);
    metric = movevars(metric, 'Model', 'Before', 1);
    baseline = [baseline; metric]; %#ok<AGROW>

    pfTable = mode_participation(A, models(m).data.X_stac, modeIndex, models(m).label);
    participation = [participation; pfTable(1:min(10, height(pfTable)), :)]; %#ok<AGROW>
end
writetable(baseline, fullfile(cfg.resultDir, 'baseline_torsional_modes.csv'));
writetable(participation, fullfile(cfg.resultDir, 'baseline_torsional_participation_top10.csv'));

%% 3. 相邻拓扑的基准点差异，用于说明因果链
[stageFrom, stageTo, stageFactor] = causal_stage_definition(models);
causalBaseline = table();
if ~isempty(stageFrom)
    causalBaseline = table(stageFactor, baseline.Model(stageFrom), baseline.Model(stageTo), ...
        baseline.DampingRatio(stageTo) - baseline.DampingRatio(stageFrom), ...
        baseline.Sigma(stageTo) - baseline.Sigma(stageFrom), ...
        baseline.MaxReal(stageTo) - baseline.MaxReal(stageFrom), ...
        'VariableNames', {'Factor', 'FromModel', 'ToModel', 'DeltaDampingRatio', 'DeltaSigma', 'DeltaMaxReal'});
    writetable(causalBaseline, fullfile(cfg.resultDir, 'causal_baseline_deltas.csv'));
end

fprintf('\n基准点 2 Hz 附近机电/轴系模态：\n');
disp(baseline(:, {'Model', 'MaxReal', 'DominantFrequencyHz', 'Sigma', 'FrequencyHz', 'DampingRatio', 'Stable'}));

%% 4. 系统参数扫描：SCR、XR、C_dc 或主程序里新增的其他变量
allSweeps = table();
for s = 1:numel(cfg.sweeps)
    sweep = cfg.sweeps(s);
    fprintf('\n扫描参数 %s，共 %d 个点...\n', sweep.name, numel(sweep.values));
    for m = 1:numel(models)
        for k = 1:numel(sweep.values)
            params = baseParams;
            params.(sweep.name) = sweep.values(k);
            if any(strcmp(sweep.name, {'SCR', 'XR'}))
                [params.rg, params.lg] = grid_impedance(params);
            end

            try
                A = make_A(models(m).data, mpopt, params);
                metric = torsion_metric(A, models(m).data.X_stac, cfg.targetFreqHz);
            catch ME
                warning('%s 在 %s=%.6g 时计算失败：%s', models(m).label, sweep.name, sweep.values(k), ME.message);
                metric = empty_metric();
            end

            metric.Sweep = string(sweep.name);
            metric.Value = sweep.values(k);
            metric.Model = string(models(m).label);
            metric = movevars(metric, {'Sweep', 'Value', 'Model'}, 'Before', 1);
            allSweeps = [allSweeps; metric]; %#ok<AGROW>
        end
    end
end
writetable(allSweeps, fullfile(cfg.resultDir, 'common_condition_sweeps.csv'));

%% 5. 扫参过程中相邻拓扑差异
causalSweeps = table();
if ~isempty(stageFrom)
    for s = 1:numel(cfg.sweeps)
        sweepName = string(cfg.sweeps(s).name);
        for c = 1:numel(stageFactor)
            fromRows = allSweeps(allSweeps.Sweep == sweepName & allSweeps.Model == models(stageFrom(c)).label, :);
            toRows = allSweeps(allSweeps.Sweep == sweepName & allSweeps.Model == models(stageTo(c)).label, :);
            if height(fromRows) ~= height(toRows)
                continue;
            end
            deltaRows = table(repmat(stageFactor(c), height(fromRows), 1), fromRows.Sweep, fromRows.Value, ...
                toRows.DampingRatio - fromRows.DampingRatio, ...
                toRows.Sigma - fromRows.Sigma, ...
                toRows.MaxReal - fromRows.MaxReal, ...
                'VariableNames', {'Factor', 'Sweep', 'Value', 'DeltaDampingRatio', 'DeltaSigma', 'DeltaMaxReal'});
            causalSweeps = [causalSweeps; deltaRows]; %#ok<AGROW>
        end
    end
    writetable(causalSweeps, fullfile(cfg.resultDir, 'causal_sweep_deltas.csv'));
end

%% 6. 保存图和数据
results = struct();
results.architectures = architectures;
results.baseline = baseline;
results.participation = participation;
results.causalBaseline = causalBaseline;
results.allSweeps = allSweeps;
results.causalSweeps = causalSweeps;
results.cfg = cfg;
save(fullfile(cfg.resultDir, 'modular_control_mode_comparison_results.mat'), 'results');

if cfg.saveFigures
    plot_sweep_figures(models, cfg.sweeps, baseline, allSweeps, cfg);
end

fprintf('\n小信号模块结果已保存到：\n%s\n', cfg.resultDir);
end

function cfg = local_defaults(cfg)
cfg = set_default(cfg, 'matpowerDir', '');
cfg = set_default(cfg, 'targetFreqHz', 2.0);
cfg = set_default(cfg, 'resultDir', fullfile(pwd, 'Modular_Control_Mode_Comparison_Results'));
cfg = set_default(cfg, 'saveFigures', true);
cfg = set_default(cfg, 'closeFigures', false);
cfg = set_default(cfg, 'modelKeys', ["GFL", "GFM_GWT", "GFM_MWT", "GFM_MWT_AD"]);
if ~isfield(cfg, 'sweeps') || isempty(cfg.sweeps)
    cfg.sweeps = struct('name', {"SCR", "XR"}, ...
        'label', {"SCR", "X/R"}, ...
        'values', {logspace(log10(1.25), log10(25), 40), linspace(1, 20, 40)});
end
end

function cfg = set_default(cfg, name, value)
if ~isfield(cfg, name) || isempty(cfg.(name))
    cfg.(name) = value;
end
end

function p = escape_path(p)
p = strrep(p, '''', '''''');
end

function models = load_model_set(modelKeys)
allModels = struct( ...
    'key', {"GFL", "GFM_GWT", "GFM_MWT", "GFM_MWT_AD"}, ...
    'label', {"GFL-WT", "GFM-GWT", "GFM-MWT", "GFM-MWT+AD"}, ...
    'file', {"Unified_WT_PMSG_GFL.mat", "Unified_WT_PMSG_GFM_GWT.mat", "Unified_WT_PMSG_VSG.mat", "Unified_WT_PMSG_VSG_Damping.mat"}, ...
    'msc', {"MSC-MPPT", "MSC-MPPT", "MSC-DVC", "MSC-DVC"}, ...
    'gsc', {"GSC-DVC + PLL", "GSC-DVC + GFM", "GSC-GFM", "GSC-GFM + APCAD"}, ...
    'role', {"跟网型基准", "隔离 PLL 到 GFM 同步方式变化", "隔离直流电压控制分配变化", "隔离附加阻尼控制作用"}, ...
    'color', {[0.05 0.42 0.62], [0.91 0.60 0.10], [0.78 0.28 0.18], [0.12 0.55 0.34]} );

models = struct('key', {}, 'label', {}, 'data', {}, 'msc', {}, 'gsc', {}, 'role', {}, 'color', {});
for k = 1:numel(allModels)
    if any(modelKeys == allModels(k).key)
        tmp = load(allModels(k).file);
        models(end+1) = struct( ... %#ok<AGROW>
            'key', allModels(k).key, ...
            'label', allModels(k).label, ...
            'data', tmp.Unified_GFMI, ...
            'msc', allModels(k).msc, ...
            'gsc', allModels(k).gsc, ...
            'role', allModels(k).role, ...
            'color', allModels(k).color);
    end
end
assert(~isempty(models), '没有找到需要运行的模型。请检查 cfg.modelKeys。');
end

function [stageFrom, stageTo, stageFactor] = causal_stage_definition(models)
labels = string({models.label});
chain = ["GFL-WT", "GFM-GWT", "GFM-MWT", "GFM-MWT+AD"];
chainIndex = zeros(size(chain));
for k = 1:numel(chain)
    idx = find(labels == chain(k), 1);
    if isempty(idx)
        chainIndex(k) = NaN;
    else
        chainIndex(k) = idx;
    end
end

stageFrom = [];
stageTo = [];
stageFactor = strings(0, 1);
names = ["同步方式影响：PLL 到 GFM"; "直流电压控制分配影响：GSC-DVC 到 MSC-DVC"; "附加阻尼控制影响：APCAD"];
for k = 1:3
    if ~isnan(chainIndex(k)) && ~isnan(chainIndex(k+1))
        stageFrom(end+1, 1) = chainIndex(k); %#ok<AGROW>
        stageTo(end+1, 1) = chainIndex(k+1); %#ok<AGROW>
        stageFactor(end+1, 1) = names(k); %#ok<AGROW>
    end
end
end

function [rg, lg] = grid_impedance(params)
rgpu = 1 / (params.SCR * sqrt(1 + params.XR^2));
lgpu = params.XR * rgpu;
rg = rgpu * params.Zb;
lg = lgpu * params.Lb;
end

function A = make_A(model, mpopt, params)
mpc = SMIB_PowerFlow(params.rg, params.lg);
pf = runpf(mpc, mpopt);
assert(pf.success == 1, '潮流计算失败，请检查该扫描点的 SCR/XR/运行点。');

angleRad = deg2rad(pf.bus(1, 9));
voltageMag = pf.bus(1, 8);
vcPhasor = voltageMag * params.V_LL / sqrt(3) * exp(1j * angleRad);
vgPhasor = params.V_LL / sqrt(3);
zt = params.rf2 + params.rg + 1j * 2*pi*params.f_base*(params.lf2 + params.lg);
z2 = params.rf2 + 1j * 2*pi*params.f_base*params.lf2;
i2Phasor = (vcPhasor - vgPhasor) / zt;
vpccPhasor = (vcPhasor - i2Phasor*z2) * sqrt(2);

params.delta0 = angleRad;
params.Vpcc_D0 = real(vpccPhasor);
params.Vpcc_Q0 = imag(vpccPhasor);
vcDq0 = vcPhasor * exp(-1j*angleRad) * sqrt(2);
params.Vc_d0 = real(vcDq0);
params.Vc_q0 = imag(vcDq0);
i2Dq0 = i2Phasor * exp(-1j*angleRad) * sqrt(2);
params.i2_d0 = real(i2Dq0);
params.i2_q0 = imag(i2Dq0);

names = fieldnames(params);
values = cell(size(names));
for k = 1:numel(names)
    values{k} = params.(names{k});
end
A = double(subs(model.sym_A, names, values));
end

function [metric, modeIndex] = torsion_metric(A, stateVector, targetFreqHz)
[V, D, W] = eig(A);
poles = diag(D);
[modeIndex, lam] = select_torsion_mode(V, W, poles, stateVector, targetFreqHz);
metric = table(max(real(poles)), dominant_frequency(poles), max(real(poles)), ...
    real(lam), imag(lam), abs(imag(lam))/(2*pi), -real(lam)/abs(lam), max(real(poles)) < 0, ...
    'VariableNames', {'MaxReal', 'DominantFrequencyHz', 'DominantSigma', 'Sigma', 'Omega', 'FrequencyHz', 'DampingRatio', 'Stable'});
end

function f = dominant_frequency(poles)
[~, idx] = max(real(poles));
f = abs(imag(poles(idx))) / (2*pi);
end

function [modeIndex, lam] = select_torsion_mode(V, W, poles, stateVector, targetFreqHz)
positiveModes = find(imag(poles) > 1e-6);
frequencies = abs(imag(poles(positiveModes))) / (2*pi);
states = string(arrayfun(@char, stateVector, 'UniformOutput', false));
mechanicalStates = ismember(states(:), ["omega_t", "omega_g", "theta_tw"]);

candidateModes = positiveModes(frequencies > 0.2 & frequencies < 8);
if isempty(candidateModes)
    [~, nearest] = min(abs(frequencies - targetFreqHz));
    candidateModes = positiveModes(nearest);
end

score = zeros(size(candidateModes));
for k = 1:numel(candidateModes)
    pf = abs(V(:, candidateModes(k)) .* conj(W(:, candidateModes(k))));
    score(k) = sum(pf(mechanicalStates)) / sum(pf);
end
[~, best] = max(score);
modeIndex = candidateModes(best);
lam = poles(modeIndex);
end

function rows = mode_participation(A, stateVector, modeIndex, modelLabel)
[V, D, W] = eig(A);
poles = diag(D);
pf = abs(V(:, modeIndex) .* conj(W(:, modeIndex)));
pf = pf / sum(pf);
states = string(arrayfun(@char, stateVector, 'UniformOutput', false));
states = states(:);
[sortedPf, order] = sort(pf, 'descend');
n = min(10, numel(order));
rows = table(repmat(string(modelLabel), n, 1), ...
    repmat(real(poles(modeIndex)), n, 1), ...
    repmat(abs(imag(poles(modeIndex)))/(2*pi), n, 1), ...
    (1:n).', states(order(1:n)), sortedPf(1:n), ...
    'VariableNames', {'Model', 'Sigma', 'FrequencyHz', 'Rank', 'State', 'Participation'});
end

function metric = empty_metric()
metric = table(nan, nan, nan, nan, nan, nan, nan, false, ...
    'VariableNames', {'MaxReal', 'DominantFrequencyHz', 'DominantSigma', 'Sigma', 'Omega', 'FrequencyHz', 'DampingRatio', 'Stable'});
end

function plot_sweep_figures(models, sweeps, baseline, allSweeps, cfg)
fig1 = figure('Color', 'w', 'Position', [100 100 680 420]);
bar(categorical(baseline.Model, baseline.Model), baseline.DampingRatio, 'FaceColor', 'flat');
ax = gca;
ax.Toolbar.Visible = 'off';
ax.Children.CData = vertcat(models.color);
grid on
ylabel('\zeta_{tor}');
title('基准点 2 Hz 机电/轴系模态阻尼比');
save_figure(fig1, fullfile(cfg.resultDir, 'baseline_torsional_damping'), cfg);

plot_metric_grid(models, sweeps, allSweeps, 'DampingRatio', '\zeta_{tor}', '2 Hz 模态阻尼比', cfg, 'sweep_torsional_damping');
plot_metric_grid(models, sweeps, allSweeps, 'Sigma', '\sigma_{tor} / s^{-1}', '2 Hz 模态实部', cfg, 'sweep_torsional_sigma');
plot_metric_grid(models, sweeps, allSweeps, 'MaxReal', 'max Re(\lambda) / s^{-1}', '全系统最大实部', cfg, 'sweep_overall_stability');
end

function plot_metric_grid(models, sweeps, allSweeps, metricName, yLabelText, titleText, cfg, fileName)
fig = figure('Color', 'w', 'Position', [80 80 1260 390]);
tiledlayout(1, numel(sweeps), 'Padding', 'compact', 'TileSpacing', 'compact');
for s = 1:numel(sweeps)
    ax = nexttile;
    ax.Toolbar.Visible = 'off';
    hold(ax, 'on');
    for m = 1:numel(models)
        rows = allSweeps(allSweeps.Sweep == string(sweeps(s).name) & allSweeps.Model == models(m).label, :);
        plot(ax, rows.Value, rows.(metricName), 'LineWidth', 1.8, ...
            'Color', models(m).color, 'DisplayName', models(m).label);
    end
    yline(ax, 0, 'k--', 'LineWidth', 0.8, 'HandleVisibility', 'off');
    grid(ax, 'on');
    xlabel(ax, sweeps(s).label);
    ylabel(ax, yLabelText);
    title(ax, sprintf('%s - %s', sweeps(s).name, titleText));
    if strcmp(sweeps(s).name, 'SCR')
        set(ax, 'XScale', 'log');
    end
    if s == 1
        legend(ax, 'Location', 'best');
    end
end
save_figure(fig, fullfile(cfg.resultDir, fileName), cfg);
end

function save_figure(fig, basePath, cfg)
exportgraphics(fig, [basePath '.png'], 'Resolution', 300);
savefig(fig, [basePath '.fig']);
if cfg.closeFigures
    close(fig);
end
end
