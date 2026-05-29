%% Final candidate search for stable GFM-MWT+AD settings
% Uses the high-frequency diagnostic result: voltage-loop gain scale near
% 27-42 suppresses the LCL/delay dominant mode. Then APCAD is tuned to move
% the remaining 2 Hz torsional mode left.

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

variable_names = {'k_pv', 'k_iv', 'K_damp', 'zeta_damp', 'alpha_lead_damp', 'T_lead_damp'};
defaults = get_values(params, variable_names);
A_fun = compile_A(ad.sym_A, params, variable_names);

voltage_scales = [23.714, 27.384, 31.623, 36.517, 42.170];
kdamp_values = linspace(-3.0e7, -2.0e6, 141);
zeta_values = [0.20, 0.35, 0.50, 0.70, 0.90];
alpha_values = [0.15, 0.25, 0.35, 0.50, 0.70];

rows = table();
idx = 0;
for vs = 1:numel(voltage_scales)
    for kd = 1:numel(kdamp_values)
        for z = 1:numel(zeta_values)
            for a = 1:numel(alpha_values)
                values = defaults;
                values(1) = params.k_pv * voltage_scales(vs);
                values(2) = params.k_iv * voltage_scales(vs);
                values(3) = kdamp_values(kd);
                values(4) = zeta_values(z);
                values(5) = alpha_values(a);
                values(6) = 1 / params.w_damp;
                metric = mode_metrics(evaluate_A(A_fun, values), ad.X_stac);
                idx = idx + 1;
                rows = [rows; table(idx, voltage_scales(vs), values(1), values(2), ...
                    values(3), values(4), values(5), metric.MaxReal, ...
                    metric.DominantFrequencyHz, metric.TorsionSigma, ...
                    metric.TorsionFrequencyHz, metric.TorsionDampingRatio, metric.Stable, ...
                    'VariableNames', {'Index', 'VoltageScale', 'Kpv', 'Kiv', ...
                    'Kdamp', 'ZetaDamp', 'AlphaLead', 'MaxReal', ...
                    'DominantFrequencyHz', 'TorsionSigma', 'TorsionFrequencyHz', ...
                    'TorsionDampingRatio', 'Stable'})]; %#ok<AGROW>
            end
        end
    end
end

rows = sortrows(rows, {'Stable', 'MaxReal', 'TorsionSigma'}, {'descend', 'ascend', 'ascend'});
writetable(rows, fullfile(result_dir, 'final_stable_candidate_scan.csv'));
best = rows(1:min(30, height(rows)), :);
save(fullfile(result_dir, 'final_stable_candidate_scan.mat'), 'rows', 'best');

fprintf('\nBest final candidates:\n');
disp(best);
fprintf('\nStable candidates found: %d / %d\n', sum(rows.Stable), height(rows));
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
metric = struct('MaxReal', max_real, ...
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
