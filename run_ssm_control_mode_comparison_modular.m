function results = run_ssm_control_mode_comparison_modular(eigenDir, cfg)
% Modular small-signal comparison without fixed-frequency prior.
% Mode selection defaults to mechanical-participation-based auto detection.

if nargin < 2, cfg = struct(); end
cfg = local_defaults(cfg);

oldDir = pwd;
cleanupObj = onCleanup(@() cd(oldDir)); %#ok<NASGU>
cd(eigenDir);
if ~exist(cfg.resultDir, 'dir'), mkdir(cfg.resultDir); end
if exist(cfg.matpowerDir, 'dir') == 7, addpath(genpath(cfg.matpowerDir)); end

paramMat = fullfile(eigenDir, 'Parameters.mat');
if exist(paramMat, 'file') ~= 2
    evalin('base', sprintf('cd(''%s''); run(''Parameters.m'');', escape_path(eigenDir)));
end
baseParams = load(paramMat);
mpopt = mpoption('verbose', 0, 'out.all', 0);
models = load_model_set(cfg.modelKeys);

architectures = table(string({models.label}).', string({models.msc}).', string({models.gsc}).', string({models.role}).', ...
    'VariableNames', {'Model', 'MSC', 'GSC', 'CausalRole'});
writetable(architectures, fullfile(cfg.resultDir, 'model_architectures.csv'));

baseline = table(); participation = table();
for m = 1:numel(models)
    A = make_A(models(m).data, mpopt, baseParams);
    [metric, modeIndex] = torsion_metric(A, models(m).data.X_stac, cfg);
    metric.Model = string(models(m).label);
    metric = movevars(metric, 'Model', 'Before', 1);
    baseline = [baseline; metric]; %#ok<AGROW>
    pfTable = mode_participation(A, models(m).data.X_stac, modeIndex, models(m).label);
    participation = [participation; pfTable(1:min(10, height(pfTable)), :)]; %#ok<AGROW>
end
writetable(baseline, fullfile(cfg.resultDir, 'baseline_torsional_modes.csv'));
writetable(participation, fullfile(cfg.resultDir, 'baseline_torsional_participation_top10.csv'));

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

allSweeps = table();
for s = 1:numel(cfg.sweeps)
    sweep = cfg.sweeps(s);
    for m = 1:numel(models)
        for k = 1:numel(sweep.values)
            params = baseParams; params.(sweep.name) = sweep.values(k);
            if any(strcmp(sweep.name, {'SCR', 'XR'})), [params.rg, params.lg] = grid_impedance(params); end
            try
                A = make_A(models(m).data, mpopt, params);
                metric = torsion_metric(A, models(m).data.X_stac, cfg);
            catch
                metric = empty_metric();
            end
            metric.Sweep = string(sweep.name); metric.Value = sweep.values(k); metric.Model = string(models(m).label);
            metric = movevars(metric, {'Sweep', 'Value', 'Model'}, 'Before', 1);
            allSweeps = [allSweeps; metric]; %#ok<AGROW>
        end
    end
end
writetable(allSweeps, fullfile(cfg.resultDir, 'common_condition_sweeps.csv'));

causalSweeps = table();
if ~isempty(stageFrom)
    for s = 1:numel(cfg.sweeps)
        sweepName = string(cfg.sweeps(s).name);
        for c = 1:numel(stageFactor)
            fromRows = allSweeps(allSweeps.Sweep == sweepName & allSweeps.Model == models(stageFrom(c)).label, :);
            toRows = allSweeps(allSweeps.Sweep == sweepName & allSweeps.Model == models(stageTo(c)).label, :);
            if height(fromRows) ~= height(toRows), continue; end
            deltaRows = table(repmat(stageFactor(c), height(fromRows), 1), fromRows.Sweep, fromRows.Value, ...
                toRows.DampingRatio - fromRows.DampingRatio, toRows.Sigma - fromRows.Sigma, toRows.MaxReal - fromRows.MaxReal, ...
                'VariableNames', {'Factor', 'Sweep', 'Value', 'DeltaDampingRatio', 'DeltaSigma', 'DeltaMaxReal'});
            causalSweeps = [causalSweeps; deltaRows]; %#ok<AGROW>
        end
    end
    writetable(causalSweeps, fullfile(cfg.resultDir, 'causal_sweep_deltas.csv'));
end

results = struct('architectures',architectures,'baseline',baseline,'participation',participation, ...
    'causalBaseline',causalBaseline,'allSweeps',allSweeps,'causalSweeps',causalSweeps,'cfg',cfg);
save(fullfile(cfg.resultDir, 'modular_control_mode_comparison_results.mat'), 'results');
if cfg.saveFigures, plot_sweep_figures(models, cfg.sweeps, baseline, allSweeps, cfg); end
end

function cfg = local_defaults(cfg)
cfg = set_default(cfg, 'matpowerDir', '');
cfg = set_default(cfg, 'modeSelect', 'auto_mech');
cfg = set_default(cfg, 'targetFreqHz', NaN);
cfg = set_default(cfg, 'resultDir', fullfile(pwd, 'Modular_Control_Mode_Comparison_Results'));
cfg = set_default(cfg, 'saveFigures', true);
cfg = set_default(cfg, 'closeFigures', false);
cfg = set_default(cfg, 'modelKeys', ["GFL", "GFM_GWT", "GFM_MWT", "GFM_MWT_AD"]);
if ~isfield(cfg, 'sweeps') || isempty(cfg.sweeps)
    cfg.sweeps = struct('name', {"SCR", "XR"}, 'label', {"SCR", "X/R"}, ...
        'values', {logspace(log10(1.25), log10(25), 40), linspace(1, 20, 40)});
end
end
function cfg = set_default(cfg, name, value), if ~isfield(cfg,name) || isempty(cfg.(name)), cfg.(name)=value; end, end
function p = escape_path(p), p = strrep(p, '''', ''''''); end

function [metric, modeIndex] = torsion_metric(A, stateVector, cfg)
[V,D,W] = eig(A); poles = diag(D);
[modeIndex, lam] = select_torsion_mode(V, W, poles, stateVector, cfg);
metric = table(max(real(poles)), dominant_frequency(poles), max(real(poles)), real(lam), imag(lam), ...
    abs(imag(lam))/(2*pi), -real(lam)/max(abs(lam),eps), max(real(poles))<0, ...
    'VariableNames', {'MaxReal','DominantFrequencyHz','DominantSigma','Sigma','Omega','FrequencyHz','DampingRatio','Stable'});
end

function [modeIndex, lam] = select_torsion_mode(V, W, poles, stateVector, cfg)
states = string(arrayfun(@char, stateVector, 'UniformOutput', false));
mechanicalStates = ismember(states(:), ["omega_t","omega_g","theta_tw"]);
positiveModes = find(imag(poles) > 1e-6);
if isempty(positiveModes), [~,modeIndex]=max(real(poles)); lam=poles(modeIndex); return; end
f = abs(imag(poles(positiveModes))) / (2*pi);
candidateModes = positiveModes(f > 0.2 & f < 8);
if isempty(candidateModes), candidateModes = positiveModes; end
score = zeros(size(candidateModes));
for k = 1:numel(candidateModes)
    pf = abs(V(:,candidateModes(k)) .* conj(W(:,candidateModes(k))));
    mechShare = sum(pf(mechanicalStates)) / max(sum(pf), eps);
    score(k) = mechShare;
    if strcmpi(string(cfg.modeSelect), "nearest_freq") && isfinite(cfg.targetFreqHz)
        score(k) = score(k) - 0.02 * abs((abs(imag(poles(candidateModes(k))))/(2*pi)) - cfg.targetFreqHz);
    end
end
[~,iBest] = max(score);
modeIndex = candidateModes(iBest); lam = poles(modeIndex);
end

function f = dominant_frequency(poles), [~,idx] = max(real(poles)); f = abs(imag(poles(idx))) / (2*pi); end

function rows = mode_participation(A, stateVector, modeIndex, modelLabel)
[V,D,W] = eig(A); poles = diag(D);
pf = abs(V(:,modeIndex).*conj(W(:,modeIndex))); pf = pf / max(sum(pf),eps);
states = string(arrayfun(@char, stateVector, 'UniformOutput', false)); states = states(:);
[sortedPf, order] = sort(pf, 'descend'); n = min(10, numel(order));
rows = table(repmat(string(modelLabel), n, 1), repmat(real(poles(modeIndex)), n, 1), ...
    repmat(abs(imag(poles(modeIndex)))/(2*pi), n, 1), (1:n).', states(order(1:n)), sortedPf(1:n), ...
    'VariableNames', {'Model','Sigma','FrequencyHz','Rank','State','Participation'});
end
function metric = empty_metric(), metric = table(nan,nan,nan,nan,nan,nan,nan,false,'VariableNames',{'MaxReal','DominantFrequencyHz','DominantSigma','Sigma','Omega','FrequencyHz','DampingRatio','Stable'}); end

function models = load_model_set(modelKeys)
allModels = struct('key',{"GFL","GFM_GWT","GFM_MWT","GFM_MWT_AD"}, ...
    'label',{"GFL-WT","GFM-GWT","GFM-MWT","GFM-MWT+AD"}, ...
    'file',{"Unified_WT_PMSG_GFL.mat","Unified_WT_PMSG_GFM_GWT.mat","Unified_WT_PMSG_VSG.mat","Unified_WT_PMSG_VSG_Damping.mat"}, ...
    'msc',{"MSC-MPPT","MSC-MPPT","MSC-DVC","MSC-DVC"}, ...
    'gsc',{"GSC-DVC + PLL","GSC-DVC + GFM","GSC-GFM","GSC-GFM + APCAD"}, ...
    'role',{"baseline","pll_to_gfm","dc_control_shift","add_damping"}, ...
    'color',{[0.05 0.42 0.62],[0.91 0.60 0.10],[0.78 0.28 0.18],[0.12 0.55 0.34]});
models = struct('key',{},'label',{},'data',{},'msc',{},'gsc',{},'role',{},'color',{});
for k = 1:numel(allModels)
    if any(modelKeys == allModels(k).key)
        tmp = load(allModels(k).file);
        models(end+1) = struct('key',allModels(k).key,'label',allModels(k).label,'data',tmp.Unified_GFMI, ... %#ok<AGROW>
            'msc',allModels(k).msc,'gsc',allModels(k).gsc,'role',allModels(k).role,'color',allModels(k).color);
    end
end
assert(~isempty(models), 'No model selected.');
end

function [stageFrom, stageTo, stageFactor] = causal_stage_definition(models)
labels = string({models.label}); chain = ["GFL-WT","GFM-GWT","GFM-MWT","GFM-MWT+AD"];
chainIndex = nan(size(chain));
for k=1:numel(chain), idx=find(labels==chain(k),1); if ~isempty(idx), chainIndex(k)=idx; end, end
stageFrom=[]; stageTo=[]; stageFactor=strings(0,1);
names = ["PLL->GFM"; "GSC-DVC->MSC-DVC"; "APCAD on"];
for k=1:3
    if ~isnan(chainIndex(k)) && ~isnan(chainIndex(k+1))
        stageFrom(end+1,1)=chainIndex(k); stageTo(end+1,1)=chainIndex(k+1); stageFactor(end+1,1)=names(k); %#ok<AGROW>
    end
end
end

function [rg, lg] = grid_impedance(params)
rgpu = 1 / (params.SCR * sqrt(1 + params.XR^2)); lgpu = params.XR * rgpu;
rg = rgpu * params.Zb; lg = lgpu * params.Lb;
end

function A = make_A(model, mpopt, params)
mpc = SMIB_PowerFlow(params.rg, params.lg); pf = runpf(mpc, mpopt); assert(pf.success==1, 'Power flow failed.');
angleRad = deg2rad(pf.bus(1,9)); voltageMag = pf.bus(1,8);
vcPhasor = voltageMag * params.V_LL / sqrt(3) * exp(1j * angleRad); vgPhasor = params.V_LL / sqrt(3);
zt = params.rf2 + params.rg + 1j*2*pi*params.f_base*(params.lf2 + params.lg);
z2 = params.rf2 + 1j*2*pi*params.f_base*params.lf2;
i2Phasor = (vcPhasor - vgPhasor) / zt; vpccPhasor = (vcPhasor - i2Phasor*z2) * sqrt(2);
params.delta0 = angleRad; params.Vpcc_D0 = real(vpccPhasor); params.Vpcc_Q0 = imag(vpccPhasor);
vcDq0 = vcPhasor * exp(-1j*angleRad) * sqrt(2); params.Vc_d0 = real(vcDq0); params.Vc_q0 = imag(vcDq0);
i2Dq0 = i2Phasor * exp(-1j*angleRad) * sqrt(2); params.i2_d0 = real(i2Dq0); params.i2_q0 = imag(i2Dq0);
names = fieldnames(params); values = cell(size(names)); for k=1:numel(names), values{k}=params.(names{k}); end
A = double(subs(model.sym_A, names, values));
end

function plot_sweep_figures(models, sweeps, baseline, allSweeps, cfg)
fig1 = figure('Color','w','Position',[100 100 680 420]);
bar(categorical(baseline.Model, baseline.Model), baseline.DampingRatio, 'FaceColor', 'flat');
ax = gca; ax.Toolbar.Visible='off'; ax.Children.CData = vertcat(models.color); grid on
ylabel('\zeta_{tor}'); title('Baseline auto-identified electromechanical damping');
save_figure(fig1, fullfile(cfg.resultDir,'baseline_torsional_damping'), cfg);
plot_metric_grid(models, sweeps, allSweeps, 'DampingRatio', '\zeta_{tor}', 'Auto-identified mode damping', cfg, 'sweep_torsional_damping');
plot_metric_grid(models, sweeps, allSweeps, 'Sigma', '\sigma_{tor} / s^{-1}', 'Auto-identified mode sigma', cfg, 'sweep_torsional_sigma');
plot_metric_grid(models, sweeps, allSweeps, 'MaxReal', 'max Re(\lambda) / s^{-1}', 'Overall stability', cfg, 'sweep_overall_stability');
end

function plot_metric_grid(models, sweeps, allSweeps, metricName, yLabelText, titleText, cfg, fileName)
fig = figure('Color','w','Position',[80 80 1260 390]); tiledlayout(1, numel(sweeps), 'Padding', 'compact', 'TileSpacing', 'compact');
for s = 1:numel(sweeps)
    ax = nexttile; ax.Toolbar.Visible='off'; hold(ax, 'on');
    for m = 1:numel(models)
        rows = allSweeps(allSweeps.Sweep == string(sweeps(s).name) & allSweeps.Model == models(m).label, :);
        plot(ax, rows.Value, rows.(metricName), 'LineWidth', 1.8, 'Color', models(m).color, 'DisplayName', models(m).label);
    end
    yline(ax,0,'k--','LineWidth',0.8,'HandleVisibility','off'); grid(ax,'on');
    xlabel(ax, sweeps(s).label); ylabel(ax, yLabelText); title(ax, sprintf('%s - %s', sweeps(s).name, titleText));
    if strcmp(sweeps(s).name,'SCR'), set(ax,'XScale','log'); end
    if s == 1, legend(ax,'Location','best'); end
end
save_figure(fig, fullfile(cfg.resultDir, fileName), cfg);
end

function save_figure(fig, basePath, cfg)
exportgraphics(fig, [basePath '.png'], 'Resolution', 300); savefig(fig, [basePath '.fig']);
if cfg.closeFigures, close(fig); end
end
