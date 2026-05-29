%% GFM-MWT+AD stability-oriented tuning
% Coarse-to-fine search for a parameter set that stabilizes both:
%   1) the high-frequency LCL/delay dominant mode, and
%   2) the low-frequency two-mass torsional mode around 2 Hz.
%
% The script does not edit Parameters.m. It writes candidate tables so the
% chosen values can be reviewed before being committed to the baseline.

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
    'K_damp', 'zeta_damp', 'alpha_lead_damp', 'T_lead_damp'};
defaults = get_values(params, variable_names);

fprintf('Compiling GFM-MWT+AD A-matrix for tuning search...\n');
A_fun = compile_A(ad.sym_A, params, variable_names);

base_A = evaluate_A(A_fun, defaults);
base_metric = mode_metrics(base_A, ad.X_stac);
fprintf('Baseline: MaxReal=%.6g, torsion sigma=%.6g, zeta=%.6g\n', ...
    base_metric.MaxReal, base_metric.TorsionSigma, base_metric.TorsionDampingRatio);

%% Stage 1: stabilize the high-frequency LCL/delay mode
ic_scales = logspace(-3, 0, 25);
vc_scales = logspace(-3, 0, 25);
beta_v_values = [0, 0.2, 0.5, 0.8, 1.0];
beta_i_values = [0, 0.2, 0.5, 0.8, 1.0];

stage1 = table();
count = 0;
for ic = 1:numel(ic_scales)
    for vc = 1:numel(vc_scales)
        for bv = 1:numel(beta_v_values)
            for bi = 1:numel(beta_i_values)
                values = defaults;
                values(1) = params.k_pi * ic_scales(ic);
                values(2) = params.k_ii * ic_scales(ic);
                values(3) = params.k_pv * vc_scales(vc);
                values(4) = params.k_iv * vc_scales(vc);
                values(5) = beta_v_values(bv);
                values(6) = beta_i_values(bi);
                metric = mode_metrics(evaluate_A(A_fun, values), ad.X_stac);
                count = count + 1;
                stage1 = [stage1; metric_row("stage1", count, values, metric, params)]; %#ok<AGROW>
            end
        end
    end
end
stage1 = sortrows(stage1, {'MaxReal', 'TorsionSigma'}, {'ascend', 'ascend'});
writetable(stage1, fullfile(result_dir, 'stage1_inner_loop_scan.csv'));

fprintf('\nBest inner-loop candidates:\n');
disp(stage1(1:min(15, height(stage1)), {'Index', 'CurrentScale', 'VoltageScale', 'BetaV', 'BetaI', ...
    'MaxReal', 'DominantFrequencyHz', 'TorsionSigma', 'TorsionDampingRatio', 'Stable'}));

%% Stage 2: tune APCAD around the best inner-loop candidates
top_n = min(20, height(stage1));
kdamp_values = linspace(-2.5e7, 2.5e6, 111);
zeta_values = [0.12, 0.20, 0.35, 0.50, 0.70];
alpha_values = [0.15, 0.25, 0.35, 0.50, 0.70];

stage2 = table();
count = 0;
for r = 1:top_n
    base_values = defaults;
    base_values(1) = stage1.Kpi(r);
    base_values(2) = stage1.Kii(r);
    base_values(3) = stage1.Kpv(r);
    base_values(4) = stage1.Kiv(r);
    base_values(5) = stage1.BetaV(r);
    base_values(6) = stage1.BetaI(r);
    for kd = 1:numel(kdamp_values)
        for z = 1:numel(zeta_values)
            for a = 1:numel(alpha_values)
                values = base_values;
                values(7) = kdamp_values(kd);
                values(8) = zeta_values(z);
                values(9) = alpha_values(a);
                values(10) = 1 / params.w_damp;
                metric = mode_metrics(evaluate_A(A_fun, values), ad.X_stac);
                count = count + 1;
                stage2 = [stage2; metric_row("stage2", count, values, metric, params)]; %#ok<AGROW>
            end
        end
    end
end
stage2 = sortrows(stage2, {'Stable', 'MaxReal', 'TorsionSigma'}, {'descend', 'ascend', 'ascend'});
writetable(stage2, fullfile(result_dir, 'stage2_apcad_scan.csv'));

fprintf('\nBest combined candidates:\n');
disp(stage2(1:min(20, height(stage2)), {'Index', 'CurrentScale', 'VoltageScale', 'BetaV', 'BetaI', ...
    'Kdamp', 'ZetaDamp', 'AlphaLead', 'MaxReal', 'DominantFrequencyHz', ...
    'TorsionSigma', 'TorsionDampingRatio', 'Stable'}));

best = stage2(1, :);
save(fullfile(result_dir, 'stability_tuning_results.mat'), 'base_metric', 'stage1', 'stage2', 'best');
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

function metric = mode_metrics(A, state_vector)
[V, D, W] = eig(A);
poles = diag(D);
[max_real, dominant_index] = max(real(poles));
dominant = poles(dominant_index);
torsion_index = select_torsion_mode(V, W, poles, state_vector);
torsion = poles(torsion_index);
metric = struct( ...
    'MaxReal', max_real, ...
    'DominantSigma', real(dominant), ...
    'DominantFrequencyHz', abs(imag(dominant))/(2*pi), ...
    'TorsionSigma', real(torsion), ...
    'TorsionFrequencyHz', abs(imag(torsion))/(2*pi), ...
    'TorsionDampingRatio', -real(torsion)/abs(torsion), ...
    'Stable', max_real < 0);
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

function row = metric_row(stage, index, values, metric, params)
row = table(string(stage), index, values(1) / params.k_pi, values(3) / params.k_pv, ...
    values(1), values(2), values(3), values(4), values(5), values(6), ...
    values(7), values(8), values(9), values(10), ...
    metric.MaxReal, metric.DominantSigma, metric.DominantFrequencyHz, ...
    metric.TorsionSigma, metric.TorsionFrequencyHz, metric.TorsionDampingRatio, metric.Stable, ...
    'VariableNames', {'Stage', 'Index', 'CurrentScale', 'VoltageScale', ...
    'Kpi', 'Kii', 'Kpv', 'Kiv', 'BetaV', 'BetaI', ...
    'Kdamp', 'ZetaDamp', 'AlphaLead', 'TLeadDamp', ...
    'MaxReal', 'DominantSigma', 'DominantFrequencyHz', ...
    'TorsionSigma', 'TorsionFrequencyHz', 'TorsionDampingRatio', 'Stable'});
end
