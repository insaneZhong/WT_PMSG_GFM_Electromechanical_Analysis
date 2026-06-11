%% Four-Topology Causal Comparison for WT-PMSG Electromechanical Torsion
% Compares GFL-WT, GFM-GWT, GFM-MWT and GFM-MWT+AD so synchronization,
% DC-link control allocation and added damping can be separated.
% All cases share the turbine, drivetrain, PMSG, DC link, filter, grid
% operating point and network sweep variables.

close all
clear
clc

%% Paths and unified models
this_dir = fileparts(mfilename('fullpath'));
if isempty(this_dir)
    this_dir = pwd;
end
cd(this_dir);
addpath(genpath('D:\apps\matlab\R2024b\bin\matpower8.0'));

run("Parameters.m");
this_dir = pwd; % Parameters.m clears workspace variables after saving Parameters.mat.
base_params = load("Parameters.mat");
mpopt = mpoption('verbose', 0, 'out.all', 0);

gfl = load("Unified_WT_PMSG_GFL.mat");
gfm_gwt = load("Unified_WT_PMSG_GFM_GWT.mat");
gfm = load("Unified_WT_PMSG_VSG.mat");
gfm_damp = load("Unified_WT_PMSG_VSG_Damping.mat");

models = struct( ...
    'key', {"GFL", "GFM_GWT", "GFM_MWT", "GFM_MWT_AD"}, ...
    'label', {"GFL-WT", "GFM-GWT", "GFM-MWT", "GFM-MWT+AD"}, ...
    'data', {gfl.Unified_GFMI, gfm_gwt.Unified_GFMI, gfm.Unified_GFMI, gfm_damp.Unified_GFMI}, ...
    'color', {[0.05 0.42 0.62], [0.91 0.60 0.10], [0.78 0.28 0.18], [0.12 0.55 0.34]});

result_dir = fullfile(this_dir, 'Results', 'Control_Mode_Comparison_Results');
if ~exist(result_dir, 'dir')
    mkdir(result_dir);
end

architectures = table( ...
    string({models.label}).', ...
    ["MSC-MPPT"; "MSC-MPPT"; "MSC-DVC"; "MSC-DVC"], ...
    ["GSC-DVC + PLL"; "GSC-DVC + GFM"; "GSC-GFM"; "GSC-GFM + APCAD"], ...
    ["Reference baseline"; "Isolate PLL-to-GFM synchronization"; "Isolate DC-link control allocation"; "Isolate added damping control"], ...
    'VariableNames', {'Model', 'MSC', 'GSC', 'CausalRole'});
writetable(architectures, fullfile(result_dir, 'model_architectures.csv'));

%% Baseline torsional modal comparison and participation factors
baseline = table();
participation = table();
for m = 1:numel(models)
    A = make_A(models(m).data, mpopt, base_params);
    [metric, mode_index] = torsion_metric(A, models(m).data.X_stac, 2);
    metric.Model = string(models(m).label);
    metric = movevars(metric, 'Model', 'Before', 1);
    baseline = [baseline; metric]; %#ok<AGROW>

    pf_table = mode_participation(A, models(m).data.X_stac, mode_index, models(m).label);
    participation = [participation; pf_table(1:min(10, height(pf_table)), :)]; %#ok<AGROW>
end

writetable(baseline, fullfile(result_dir, 'baseline_torsional_modes.csv'));
writetable(participation, fullfile(result_dir, 'baseline_torsional_participation_top10.csv'));

stage_from = [1; 2; 3];
stage_to = [2; 3; 4];
stage_factor = ["Synchronization effect (PLL to GFM)"; ...
                "DC-link allocation effect (GSC-DVC to MSC-DVC)"; ...
                "APCAD damping effect"];
causal_baseline = table(stage_factor, baseline.Model(stage_from), baseline.Model(stage_to), ...
    baseline.DampingRatio(stage_to) - baseline.DampingRatio(stage_from), ...
    baseline.Sigma(stage_to) - baseline.Sigma(stage_from), ...
    baseline.MaxReal(stage_to) - baseline.MaxReal(stage_from), ...
    'VariableNames', {'Factor', 'FromModel', 'ToModel', 'DeltaDampingRatio', 'DeltaSigma', 'DeltaMaxReal'});
writetable(causal_baseline, fullfile(result_dir, 'causal_baseline_deltas.csv'));

fprintf('\nBaseline torsional mode comparison near 2 Hz\n');
    disp(baseline(:, {'Model', 'MaxReal', 'DominantFrequencyHz', 'Sigma', 'FrequencyHz', 'DampingRatio', 'Stable'}));
fprintf('\nCausal increments in the staged topology comparison\n');
disp(causal_baseline);
fprintf('\nTop modal participation states for the 2 Hz mode\n');
disp(participation);

%% Common system-condition sweeps
sweep_defs = struct( ...
    'name', {"SCR", "XR", "C_dc"}, ...
    'values', {logspace(log10(1.25), log10(25), 40), linspace(1, 20, 40), linspace(0.25*base_params.C_dc, 2*base_params.C_dc, 40)}, ...
    'label', {"SCR", "X/R", "C_{dc} (F)"});

all_sweeps = table();
for s = 1:numel(sweep_defs)
    values = sweep_defs(s).values;
    for m = 1:numel(models)
        for k = 1:numel(values)
            params = base_params;
            params.(sweep_defs(s).name) = values(k);
            if any(strcmp(sweep_defs(s).name, {'SCR', 'XR'}))
                [params.rg, params.lg] = grid_impedance(params);
            end
            A = make_A(models(m).data, mpopt, params);
            metric = torsion_metric(A, models(m).data.X_stac, 2);
            metric.Sweep = string(sweep_defs(s).name);
            metric.Value = values(k);
            metric.Model = string(models(m).label);
            metric = movevars(metric, {'Sweep', 'Value', 'Model'}, 'Before', 1);
            all_sweeps = [all_sweeps; metric]; %#ok<AGROW>
        end
    end
end

writetable(all_sweeps, fullfile(result_dir, 'common_condition_sweeps.csv'));

causal_sweeps = table();
for s = 1:numel(sweep_defs)
    for c = 1:numel(stage_factor)
        from_rows = all_sweeps(all_sweeps.Sweep == sweep_defs(s).name & all_sweeps.Model == models(stage_from(c)).label, :);
        to_rows = all_sweeps(all_sweeps.Sweep == sweep_defs(s).name & all_sweeps.Model == models(stage_to(c)).label, :);
        delta_rows = table(repmat(stage_factor(c), height(from_rows), 1), from_rows.Sweep, from_rows.Value, ...
            to_rows.DampingRatio - from_rows.DampingRatio, to_rows.Sigma - from_rows.Sigma, ...
            to_rows.MaxReal - from_rows.MaxReal, ...
            'VariableNames', {'Factor', 'Sweep', 'Value', 'DeltaDampingRatio', 'DeltaSigma', 'DeltaMaxReal'});
        causal_sweeps = [causal_sweeps; delta_rows]; %#ok<AGROW>
    end
end
writetable(causal_sweeps, fullfile(result_dir, 'causal_sweep_deltas.csv'));
save(fullfile(result_dir, 'control_mode_comparison_results.mat'), 'architectures', 'baseline', 'participation', 'causal_baseline', 'all_sweeps', 'causal_sweeps', 'sweep_defs');

fprintf('\nSweep summary: torsional damping ratio and overall stability\n');
for s = 1:numel(sweep_defs)
    for m = 1:numel(models)
        rows = all_sweeps(all_sweeps.Sweep == sweep_defs(s).name & all_sweeps.Model == models(m).label, :);
        fprintf('%s, %s: zeta=[%.6g, %.6g], sigma=[%.6g, %.6g], max_real_max=%.6g\n', ...
            sweep_defs(s).name, models(m).label, min(rows.DampingRatio), max(rows.DampingRatio), ...
            min(rows.Sigma), max(rows.Sigma), max(rows.MaxReal));
        [peak_real, peak_index] = max(rows.MaxReal);
        if peak_real >= 0
            fprintf('  unstable peak at %s=%.6g: dominant f=%.6g Hz, sigma=%.6g\n', ...
                sweep_defs(s).name, rows.Value(peak_index), rows.DominantFrequencyHz(peak_index), peak_real);
        end
    end
end

%% Figures for paper-ready comparison
fig_damping = figure('Color', 'w', 'Position', [80 80 1240 370]);
tiledlayout(1, numel(sweep_defs), 'Padding', 'compact', 'TileSpacing', 'compact');
for s = 1:numel(sweep_defs)
    ax = nexttile;
    ax.Toolbar.Visible = 'off';
    hold(ax, 'on');
    for m = 1:numel(models)
        rows = all_sweeps(all_sweeps.Sweep == sweep_defs(s).name & all_sweeps.Model == models(m).label, :);
        plot(ax, rows.Value, rows.DampingRatio, 'LineWidth', 1.8, 'Color', models(m).color, 'DisplayName', models(m).label);
    end
    yline(ax, 0, 'k--', 'LineWidth', 0.8, 'HandleVisibility', 'off');
    grid(ax, 'on');
    xlabel(ax, sweep_defs(s).label);
    ylabel(ax, '\zeta_{tor}');
    title(ax, sprintf('%s sweep', sweep_defs(s).name));
    if s == 1
        set(ax, 'XScale', 'log');
        legend(ax, 'Location', 'best');
    end
end
exportgraphics(fig_damping, fullfile(result_dir, 'torsional_damping_comparison.png'), 'Resolution', 300);

fig_sigma = figure('Color', 'w', 'Position', [80 500 1240 370]);
tiledlayout(1, numel(sweep_defs), 'Padding', 'compact', 'TileSpacing', 'compact');
for s = 1:numel(sweep_defs)
    ax = nexttile;
    ax.Toolbar.Visible = 'off';
    hold(ax, 'on');
    for m = 1:numel(models)
        rows = all_sweeps(all_sweeps.Sweep == sweep_defs(s).name & all_sweeps.Model == models(m).label, :);
        plot(ax, rows.Value, rows.Sigma, 'LineWidth', 1.8, 'Color', models(m).color, 'DisplayName', models(m).label);
    end
    yline(ax, 0, 'k--', 'LineWidth', 0.8, 'HandleVisibility', 'off');
    grid(ax, 'on');
    xlabel(ax, sweep_defs(s).label);
    ylabel(ax, '\sigma_{tor} (s^{-1})');
    title(ax, sprintf('%s sweep', sweep_defs(s).name));
    if s == 1
        set(ax, 'XScale', 'log');
        legend(ax, 'Location', 'best');
    end
end
exportgraphics(fig_sigma, fullfile(result_dir, 'torsional_real_part_comparison.png'), 'Resolution', 300);

fig_stability = figure('Color', 'w', 'Position', [80 500 1240 370]);
tiledlayout(1, numel(sweep_defs), 'Padding', 'compact', 'TileSpacing', 'compact');
for s = 1:numel(sweep_defs)
    ax = nexttile;
    ax.Toolbar.Visible = 'off';
    hold(ax, 'on');
    for m = 1:numel(models)
        rows = all_sweeps(all_sweeps.Sweep == sweep_defs(s).name & all_sweeps.Model == models(m).label, :);
        plot(ax, rows.Value, rows.MaxReal, 'LineWidth', 1.8, 'Color', models(m).color, 'DisplayName', models(m).label);
    end
    yline(ax, 0, 'k--', 'LineWidth', 0.8, 'HandleVisibility', 'off');
    grid(ax, 'on');
    xlabel(ax, sweep_defs(s).label);
    ylabel(ax, 'max Re(\lambda) (s^{-1})');
    title(ax, sprintf('%s overall stability', sweep_defs(s).name));
    if s == 1
        set(ax, 'XScale', 'log');
        legend(ax, 'Location', 'best');
    end
end
exportgraphics(fig_stability, fullfile(result_dir, 'overall_stability_comparison.png'), 'Resolution', 300);

fig_base = figure('Color', 'w', 'Position', [140 140 620 390]);
bar(categorical(baseline.Model, baseline.Model), baseline.DampingRatio, 'FaceColor', 'flat');
ax = gca;
ax.Toolbar.Visible = 'off';
ax.Children.CData = vertcat(models.color);
grid on
ylabel('\zeta_{tor}');
title('Baseline torsional damping near 2 Hz');
exportgraphics(fig_base, fullfile(result_dir, 'baseline_damping_bar.png'), 'Resolution', 300);

fprintf('\nResults written to:\n%s\n', result_dir);

%% Supporting functions
function [rg, lg] = grid_impedance(params)
rgpu = 1 / (params.SCR * sqrt(1 + params.XR^2));
lgpu = params.XR * rgpu;
rg = rgpu * params.Zb;
lg = lgpu * params.Lb;
end

function A = make_A(model, mpopt, params)
mpc = SMIB_PowerFlow(params.rg, params.lg);
pf = runpf(mpc, mpopt);
assert(pf.success == 1, 'Power flow failed for the requested comparison case.');

angle_rad = deg2rad(pf.bus(1, 9));
voltage_mag = pf.bus(1, 8);
vc_phasor = voltage_mag * params.V_LL / sqrt(3) * exp(1j * angle_rad);
vg_phasor = params.V_LL / sqrt(3);
zt = params.rf2 + params.rg + 1j * 2*pi*params.f_base*(params.lf2 + params.lg);
z2 = params.rf2 + 1j * 2*pi*params.f_base*params.lf2;
i2_phasor = (vc_phasor - vg_phasor) / zt;
vpcc_phasor = (vc_phasor - i2_phasor*z2) * sqrt(2);

params.delta0 = angle_rad;
params.Vpcc_D0 = real(vpcc_phasor);
params.Vpcc_Q0 = imag(vpcc_phasor);
vc_dq0 = vc_phasor * exp(-1j*angle_rad) * sqrt(2);
params.Vc_d0 = real(vc_dq0);
params.Vc_q0 = imag(vc_dq0);
i2_dq0 = i2_phasor * exp(-1j*angle_rad) * sqrt(2);
params.i2_d0 = real(i2_dq0);
params.i2_q0 = imag(i2_dq0);

names = fieldnames(params);
values = cell(size(names));
for k = 1:numel(names)
    values{k} = params.(names{k});
end
A = double(subs(model.sym_A, names, values));
end

function [metric, mode_index] = torsion_metric(A, state_vector, target_hz)
[V, D, W] = eig(A);
poles = diag(D);
[max_real, dominant_index] = max(real(poles));
dominant = poles(dominant_index);
positive_modes = find(imag(poles) > 1e-6);
assert(~isempty(positive_modes), 'No oscillatory mode was found.');
frequencies = abs(imag(poles(positive_modes))) / (2*pi);
states = string(arrayfun(@char, state_vector, 'UniformOutput', false));
states = states(:);
mechanical_states = ismember(states, ["omega_t", "omega_g", "theta_tw"]);
candidate_mask = frequencies > 0.2 & frequencies < 8;
candidate_modes = positive_modes(candidate_mask);
if isempty(candidate_modes)
    [~, nearest] = min(abs(frequencies - target_hz));
    mode_index = positive_modes(nearest);
else
    mechanical_score = zeros(size(candidate_modes));
    for k = 1:numel(candidate_modes)
        pf = abs(V(:, candidate_modes(k)) .* conj(W(:, candidate_modes(k))));
        pf = pf / sum(pf);
        mechanical_score(k) = sum(pf(mechanical_states));
    end
    [~, best] = max(mechanical_score);
    mode_index = candidate_modes(best);
end
lam = poles(mode_index);
metric = table(max_real, abs(imag(dominant))/(2*pi), real(dominant), real(lam), imag(lam), ...
    abs(imag(lam))/(2*pi), -real(lam)/abs(lam), max_real < 0, ...
    'VariableNames', {'MaxReal', 'DominantFrequencyHz', 'DominantSigma', 'Sigma', 'Omega', 'FrequencyHz', 'DampingRatio', 'Stable'});
end

function result = mode_participation(A, state_vector, mode_index, model_label)
[V, D, W] = eig(A);
poles = diag(D);
target = poles(mode_index);
[~, actual_index] = min(abs(poles - target));
pf = abs(V(:, actual_index) .* conj(W(:, actual_index)));
pf = pf / sum(pf);
states = string(arrayfun(@char, state_vector, 'UniformOutput', false));
states = states(:);
[sorted_pf, order] = sort(pf, 'descend');
result = table(repmat(string(model_label), numel(order), 1), (1:numel(order)).', states(order), sorted_pf, ...
    'VariableNames', {'Model', 'Rank', 'State', 'Participation'});
end
