%% Compare DC-link parameter scenarios without overwriting the frozen baseline
% The frozen baseline remains Parameters.m default. This script only changes
% the parameter struct in memory and writes a separate comparison table.

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
this_dir = pwd; % Parameters.m clears workspace variables after saving Parameters.mat.
base_params = load("Parameters.mat");
mpopt = mpoption('verbose', 0, 'out.all', 0);

models = localLoadFourTopologyModels();

scenarios = table( ...
    ["FrozenSmallSignal"; "NonlinearPhysicalInit"; "NonlinearValidationRef"], ...
    [base_params.Vdc; 1200; 1000], ...
    [base_params.C_dc; 0.03; 0.03], ...
    ["Current frozen small-signal baseline"; ...
     "Match nonlinear physical Cd initial voltage"; ...
     "Match current nonlinear validation VdcRef and Cd"], ...
    'VariableNames', {'Scenario', 'Vdc', 'C_dc', 'Meaning'});

result_dir = fullfile(this_dir, 'Results', 'DC_Link_Parameter_Scenarios');
if exist(result_dir, 'dir') ~= 7
    mkdir(result_dir);
end

allMetrics = table();
for s = 1:height(scenarios)
    params = localApplyDcScenario(base_params, scenarios.Vdc(s), scenarios.C_dc(s));
    for m = 1:numel(models)
        try
            A = make_A(models(m).data, mpopt, params);
            metric = torsion_metric(A, models(m).data.X_stac, 2);
        catch ME
            warning('Scenario %s model %s failed: %s', scenarios.Scenario(s), models(m).label, ME.message);
            metric = table(nan, nan, nan, nan, nan, nan, nan, false, ...
                'VariableNames', {'MaxReal', 'DominantFrequencyHz', 'DominantSigma', ...
                'Sigma', 'Omega', 'FrequencyHz', 'DampingRatio', 'Stable'});
        end
        metric.Scenario = scenarios.Scenario(s);
        metric.Vdc = scenarios.Vdc(s);
        metric.C_dc = scenarios.C_dc(s);
        metric.Model = string(models(m).label);
        metric = movevars(metric, {'Scenario', 'Vdc', 'C_dc', 'Model'}, 'Before', 1);
        allMetrics = [allMetrics; metric]; %#ok<AGROW>
    end
end

writetable(scenarios, fullfile(result_dir, 'dc_link_scenarios.csv'));
writetable(allMetrics, fullfile(result_dir, 'dc_link_scenario_torsional_modes.csv'));
localWriteMarkdown(fullfile(result_dir, 'DC_Link_Parameter_Scenario_Comparison_20260612.md'), scenarios, allMetrics);

fprintf('Saved DC-link scenario comparison in:\n  %s\n', result_dir);
disp(allMetrics(:, {'Scenario', 'Vdc', 'C_dc', 'Model', 'FrequencyHz', 'DampingRatio', 'Sigma', 'MaxReal', 'Stable'}));

function models = localLoadFourTopologyModels()
gfl = load("Unified_WT_PMSG_GFL.mat");
gfm_gwt = load("Unified_WT_PMSG_GFM_GWT.mat");
gfm = load("Unified_WT_PMSG_VSG.mat");
gfm_damp = load("Unified_WT_PMSG_VSG_Damping.mat");

models = struct( ...
    'key', {"GFL", "GFM_GWT", "GFM_MWT", "GFM_MWT_AD"}, ...
    'label', {"GFL-WT", "GFM-GWT", "GFM-MWT", "GFM-MWT+AD"}, ...
    'data', {gfl.Unified_GFMI, gfm_gwt.Unified_GFMI, gfm.Unified_GFMI, gfm_damp.Unified_GFMI});
end

function params = localApplyDcScenario(base_params, Vdc, C_dc)
params = base_params;
params.Vdc = Vdc;
params.V_dc0 = Vdc;
params.C_dc = C_dc;

% Preserve the intended GFM-GWT DC-loop natural frequency and damping ratio
% when the physical DC-link energy storage changes.
if isfield(params, 'w_dc_gwt') && isfield(params, 'zeta_dc_gwt')
    params.k_pdc_gwt = 2 * params.zeta_dc_gwt * params.w_dc_gwt * params.C_dc * params.V_dc0;
    params.k_idc_gwt = params.w_dc_gwt^2 * params.C_dc * params.V_dc0;
end
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
    'VariableNames', {'MaxReal', 'DominantFrequencyHz', 'DominantSigma', ...
    'Sigma', 'Omega', 'FrequencyHz', 'DampingRatio', 'Stable'});
end

function localWriteMarkdown(md_path, scenarios, allMetrics)
fid = fopen(md_path, 'w');
cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>

fprintf(fid, '# DC-Link Parameter Scenario Comparison\n\n');
fprintf(fid, 'This report does not overwrite the frozen small-signal baseline. It evaluates the four topology models under alternate `Vdc` and `C_dc` values in memory.\n\n');

fprintf(fid, '## Scenarios\n\n');
fprintf(fid, '| Scenario | Vdc/V | Cdc/F | Meaning |\n');
fprintf(fid, '|---|---:|---:|---|\n');
for k = 1:height(scenarios)
    fprintf(fid, '| `%s` | %.10g | %.10g | %s |\n', ...
        scenarios.Scenario(k), scenarios.Vdc(k), scenarios.C_dc(k), scenarios.Meaning(k));
end

fprintf(fid, '\n## Torsional Mode Metrics\n\n');
fprintf(fid, '| Scenario | Model | Frequency/Hz | DampingRatio | Sigma | MaxReal | Stable |\n');
fprintf(fid, '|---|---|---:|---:|---:|---:|---|\n');
for k = 1:height(allMetrics)
    fprintf(fid, '| `%s` | `%s` | %.6g | %.6g | %.6g | %.6g | %d |\n', ...
        allMetrics.Scenario(k), allMetrics.Model(k), allMetrics.FrequencyHz(k), ...
        allMetrics.DampingRatio(k), allMetrics.Sigma(k), allMetrics.MaxReal(k), allMetrics.Stable(k));
end

fprintf(fid, '\n## Use\n\n');
fprintf(fid, '- If the relative topology ordering changes after matching nonlinear DC parameters, regenerate the frozen small-signal baseline before using it in the paper.\n');
fprintf(fid, '- If the ordering is unchanged, the old baseline can still be used as a reproducible reference, but the DC-link mismatch must be disclosed as a parameter limitation.\n');
end
