%% One-dimensional high-frequency stability diagnostics
% Fast sensitivity checks for the present dominant high-frequency mode.

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

ad_data = load("Unified_WT_PMSG_VSG_Damping.mat");
ad = ad_data.Unified_GFMI;
result_dir = fullfile(this_dir, 'Stability_Tuning_Results');
if ~exist(result_dir, 'dir')
    mkdir(result_dir);
end

variable_names = {'k_pi', 'k_ii', 'k_pv', 'k_iv', 'beta_v', 'beta_i', ...
    'td', 'rd', 'K_damp', 'zeta_damp', 'alpha_lead_damp'};
defaults = get_values(params, variable_names);
A_fun = compile_A(ad.sym_A, params, variable_names);

scans = {};
scans{end+1} = make_scan('current_scale', logspace(-8, 2, 161), @(v, x) set_pair(v, [1 2], [params.k_pi params.k_ii] * x));
scans{end+1} = make_scan('voltage_scale', logspace(-8, 2, 161), @(v, x) set_pair(v, [3 4], [params.k_pv params.k_iv] * x));
scans{end+1} = make_scan('beta_v', linspace(-100, 100, 201), @(v, x) set_one(v, 5, x));
scans{end+1} = make_scan('beta_i', linspace(-100, 100, 201), @(v, x) set_one(v, 6, x));
scans{end+1} = make_scan('td_scale', logspace(-3, 2, 161), @(v, x) set_one(v, 7, params.td * x));
scans{end+1} = make_scan('rd_scale', logspace(-2, 5, 181), @(v, x) set_one(v, 8, params.rd * x));

all_rows = table();
for s = 1:numel(scans)
    def = scans{s};
    rows = run_scan(def, A_fun, defaults, ad.X_stac);
    all_rows = [all_rows; rows]; %#ok<AGROW>
    best = sortrows(rows, 'MaxReal', 'ascend');
    fprintf('\n%s best points:\n', def.Name);
    disp(best(1:min(10, height(best)), :));
end

writetable(all_rows, fullfile(result_dir, 'high_frequency_1d_diagnostics.csv'));
fprintf('\nResults written to:\n%s\n', result_dir);

%% Supporting functions
function def = make_scan(name, values, setter)
def = struct('Name', string(name), 'Values', values, 'Setter', setter);
end

function rows = run_scan(def, A_fun, defaults, state_vector)
n = numel(def.Values);
max_real = zeros(n, 1);
dominant_frequency = zeros(n, 1);
torsion_sigma = zeros(n, 1);
torsion_damping = zeros(n, 1);
for k = 1:n
    values = def.Setter(defaults, def.Values(k));
    metric = mode_metrics(evaluate_A(A_fun, values), state_vector);
    max_real(k) = metric.MaxReal;
    dominant_frequency(k) = metric.DominantFrequencyHz;
    torsion_sigma(k) = metric.TorsionSigma;
    torsion_damping(k) = metric.TorsionDampingRatio;
end
rows = table(repmat(def.Name, n, 1), def.Values(:), max_real, dominant_frequency, ...
    torsion_sigma, torsion_damping, max_real < 0, ...
    'VariableNames', {'Scan', 'Value', 'MaxReal', 'DominantFrequencyHz', ...
    'TorsionSigma', 'TorsionDampingRatio', 'Stable'});
end

function values = set_one(values, index, value)
values(index) = value;
end

function values = set_pair(values, indexes, pair_values)
values(indexes) = pair_values;
end

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

function metric = mode_metrics(A, state_vector)
[V, D, W] = eig(A);
poles = diag(D);
[max_real, dominant_index] = max(real(poles));
dominant = poles(dominant_index);
torsion_index = select_torsion_mode(V, W, poles, state_vector);
torsion = poles(torsion_index);
metric = struct('MaxReal', max_real, ...
    'DominantFrequencyHz', abs(imag(dominant))/(2*pi), ...
    'TorsionSigma', real(torsion), ...
    'TorsionDampingRatio', -real(torsion)/abs(torsion));
end

function mode_index = select_torsion_mode(V, W, poles, state_vector)
positive_modes = find(imag(poles) > 1e-6);
frequencies = abs(imag(poles(positive_modes))) / (2*pi);
candidate_modes = positive_modes(frequencies > 0.2 & frequencies < 8);
if isempty(candidate_modes)
    [~, nearest] = min(abs(frequencies - 2));
    mode_index = positive_modes(nearest);
    return;
end
states = string(arrayfun(@char, state_vector, 'UniformOutput', false));
mechanical = ismember(states(:), ["omega_t", "omega_g", "theta_tw"]);
scores = zeros(size(candidate_modes));
for k = 1:numel(candidate_modes)
    pf = abs(V(:, candidate_modes(k)) .* conj(W(:, candidate_modes(k))));
    scores(k) = sum(pf(mechanical)) / sum(pf);
end
[~, selected] = max(scores);
mode_index = candidate_modes(selected);
end
