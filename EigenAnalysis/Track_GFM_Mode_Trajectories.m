%% GFM-WT tracked torsional-mode eigenvalue trajectories
% Tracks the same torsional eigenpair by eigenvector correlation while one
% control parameter is varied. This complements participation factors:
% participation identifies a mode; trajectories show how tuning moves it.

close all
clear
clc

this_dir = fileparts(mfilename('fullpath'));
if isempty(this_dir)
    this_dir = pwd;
end
cd(this_dir);
addpath(genpath('D:\apps\matlab\R2024b\bin\matpower8.0'));

run("Parameters.m");
this_dir = pwd;
params = load("Parameters.mat");
params = add_operating_point(params);

gfm_data = load("Unified_WT_PMSG_VSG.mat");
ad_data = load("Unified_WT_PMSG_VSG_Damping.mat");
gfm = gfm_data.Unified_GFMI;
ad = ad_data.Unified_GFMI;

gfm_names = {'h', 'mp', 'k_pdc', 'k_idc'};
ad_names = {'h', 'mp', 'k_pdc', 'k_idc', 'K_damp'};
gfm_default = get_values(params, gfm_names);
ad_default = get_values(params, ad_names);
gfm_A = compile_A(gfm.sym_A, params, gfm_names);
ad_A = compile_A(ad.sym_A, params, ad_names);

factor = logspace(-2, 2, 81);
definitions = struct( ...
    'name', {"h", "mp", "k_pdc", "k_idc", "K_damp"}, ...
    'label', {"Virtual inertia h", "Active droop mp", "MSC DVC k_{pdc}", "MSC DVC k_{idc}", "APCAD K_{damp}"}, ...
    'gfm_index', {1, 2, 3, 4, NaN}, ...
    'ad_index', {1, 2, 3, 4, 5}, ...
    'values', {gfm_default(1)*factor, gfm_default(2)*factor, ...
               gfm_default(3)*factor, gfm_default(4)*factor, linspace(-1.5e7, 5e6, 121)});

result_dir = fullfile(this_dir, 'Results', 'Mode_Trajectory_Results');
if ~exist(result_dir, 'dir')
    mkdir(result_dir);
end

all_results = table();
for n = 1:numel(definitions)
    def = definitions(n);
    rows = table();
    if ~isnan(def.gfm_index)
        rows = track_sweep("GFM-MWT", def.name, def.values, gfm_A, gfm_default, ...
            def.gfm_index, gfm.X_stac, params.(def.name));
    end
    ad_rows = track_sweep("GFM-MWT+AD", def.name, def.values, ad_A, ad_default, ...
        def.ad_index, ad.X_stac, params.(def.name));
    rows = [rows; ad_rows]; %#ok<AGROW>
    all_results = [all_results; rows]; %#ok<AGROW>
    plot_trajectory(def, rows, result_dir);
end

writetable(all_results, fullfile(result_dir, 'tracked_torsional_mode_trajectories.csv'));
summary = groupsummary(all_results, {'Model', 'Parameter'}, {'min', 'max'}, {'Sigma', 'DampingRatio', 'MaxReal'});
writetable(summary, fullfile(result_dir, 'tracked_torsional_mode_summary.csv'));

fprintf('\nTracked torsional-mode trajectory summary\n');
disp(summary);
fprintf('\nResults written to:\n%s\n', result_dir);

%% Supporting functions
function params = add_operating_point(params)
mpopt = mpoption('verbose', 0, 'out.all', 0);
pf = runpf(SMIB_PowerFlow(params.rg, params.lg), mpopt);
assert(pf.success == 1, 'Power flow failed at the baseline condition.');

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
end

function values = get_values(params, names)
values = zeros(1, numel(names));
for k = 1:numel(names)
    values(k) = params.(names{k});
end
end

function A_fun = compile_A(sym_A, params, variable_names)
fixed_names = setdiff(fieldnames(params), variable_names, 'stable');
fixed_values = cell(size(fixed_names));
for k = 1:numel(fixed_names)
    fixed_values{k} = params.(fixed_names{k});
end
reduced_A = subs(sym_A, fixed_names, fixed_values);
remaining = string(arrayfun(@char, symvar(reduced_A), 'UniformOutput', false));
unexpected = setdiff(remaining, string(variable_names));
assert(isempty(unexpected), 'A matrix contains unsubstituted variables: %s', join(unexpected, ', '));
variables = cell(size(variable_names));
for k = 1:numel(variable_names)
    variables{k} = sym(variable_names{k});
end
A_fun = matlabFunction(reduced_A, 'Vars', variables);
end

function A = evaluate_A(A_fun, values)
args = num2cell(values);
A = double(A_fun(args{:}));
end

function rows = track_sweep(model, parameter, sweep_values, A_fun, defaults, index, state_vector, baseline_value)
n = numel(sweep_values);
[~, baseline_index] = min(abs(sweep_values - baseline_value));
all_A = cell(n, 1);
for k = 1:n
    values = defaults;
    values(index) = sweep_values(k);
    all_A{k} = evaluate_A(A_fun, values);
end

[base_mode, base_vector] = initial_torsion_mode(all_A{baseline_index}, state_vector);
tracked = zeros(n, 1);
tracked(baseline_index) = base_mode;
previous = base_vector;
for k = baseline_index+1:n
    [tracked(k), previous] = correlated_mode(all_A{k}, previous);
end
previous = base_vector;
for k = baseline_index-1:-1:1
    [tracked(k), previous] = correlated_mode(all_A{k}, previous);
end

sigma = zeros(n, 1);
frequency = zeros(n, 1);
damping = zeros(n, 1);
max_real = zeros(n, 1);
for k = 1:n
    poles = eig(all_A{k});
    pole = poles(tracked(k));
    sigma(k) = real(pole);
    frequency(k) = abs(imag(pole))/(2*pi);
    damping(k) = -real(pole)/abs(pole);
    max_real(k) = max(real(poles));
end
rows = table(repmat(string(model), n, 1), repmat(string(parameter), n, 1), sweep_values(:), ...
    sigma, frequency, damping, max_real, max_real < 0, ...
    'VariableNames', {'Model', 'Parameter', 'Value', 'Sigma', 'FrequencyHz', 'DampingRatio', 'MaxReal', 'Stable'});
end

function [mode_index, vector] = initial_torsion_mode(A, state_vector)
[V, D, W] = eig(A);
poles = diag(D);
modes = find(imag(poles) > 1e-6);
frequencies = abs(imag(poles(modes))) / (2*pi);
modes = modes(frequencies > 0.2 & frequencies < 8);
states = string(arrayfun(@char, state_vector, 'UniformOutput', false));
mechanical = ismember(states(:), ["omega_t", "omega_g", "theta_tw"]);
scores = zeros(size(modes));
for k = 1:numel(modes)
    pf = abs(V(:, modes(k)) .* conj(W(:, modes(k))));
    scores(k) = sum(pf(mechanical)) / sum(pf);
end
[~, selected] = max(scores);
mode_index = modes(selected);
vector = normalized(V(:, mode_index));
end

function [mode_index, vector] = correlated_mode(A, previous)
[V, D] = eig(A);
poles = diag(D);
candidates = find(imag(poles) > 1e-6);
scores = zeros(size(candidates));
for k = 1:numel(candidates)
    scores(k) = abs(previous' * normalized(V(:, candidates(k))));
end
[~, selected] = max(scores);
mode_index = candidates(selected);
vector = normalized(V(:, mode_index));
end

function v = normalized(v)
v = v / norm(v);
end

function plot_trajectory(def, rows, result_dir)
fig = figure('Color', 'w', 'Position', [90 90 1040 430]);
tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
models = unique(rows.Model, 'stable');
colors = [0.78 0.28 0.18; 0.12 0.55 0.34];

ax = nexttile;
ax.Toolbar.Visible = 'off';
hold(ax, 'on');
for k = 1:numel(models)
    data = rows(rows.Model == models(k), :);
    plot(ax, data.Sigma, data.FrequencyHz, '-o', 'MarkerSize', 2.8, 'LineWidth', 1.2, ...
        'Color', colors(k, :), 'DisplayName', models(k));
end
xline(ax, 0, 'k--', 'HandleVisibility', 'off');
grid(ax, 'on');
xlabel(ax, 'Real part \sigma (s^{-1})');
ylabel(ax, 'Frequency (Hz)');
title(ax, sprintf('%s: tracked pole trajectory', def.label));
legend(ax, 'Location', 'best');

ax = nexttile;
ax.Toolbar.Visible = 'off';
hold(ax, 'on');
for k = 1:numel(models)
    data = rows(rows.Model == models(k), :);
    plot(ax, data.Value, data.DampingRatio, 'LineWidth', 1.6, ...
        'Color', colors(k, :), 'DisplayName', models(k));
end
yline(ax, 0, 'k--', 'HandleVisibility', 'off');
if ~strcmp(def.name, 'K_damp')
    set(ax, 'XScale', 'log');
end
grid(ax, 'on');
xlabel(ax, def.label);
ylabel(ax, 'Torsional damping ratio \zeta');
title(ax, sprintf('%s: damping trend', def.label));
legend(ax, 'Location', 'best');
exportgraphics(fig, fullfile(result_dir, sprintf('%s_tracked_trajectory.png', def.name)), 'Resolution', 300);
end
