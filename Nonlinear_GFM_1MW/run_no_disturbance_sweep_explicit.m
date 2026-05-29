function T = run_no_disturbance_sweep_explicit(pref_list, simStop, vacRef, vdcRef, qref)
% Explicit steady-state sweep for nonlinear GFM model.
% Bypasses mask/symbol ambiguity by directly setting S-Function parameters.

if nargin < 1 || isempty(pref_list), pref_list = [8.0e5 8.5e5 9.0e5 9.5e5 1.0e6]; end
if nargin < 2 || isempty(simStop),   simStop = 5.0; end
if nargin < 3 || isempty(vacRef),    vacRef = 563; end
if nargin < 4 || isempty(vdcRef),    vdcRef = 5000; end
if nargin < 5 || isempty(qref),      qref = 0; end

root = fileparts(mfilename('fullpath'));
old = pwd; cleanup = onCleanup(@() cd(old)); cd(root);

run('GFM_MWT_Nonlinear_Params.m');
assignin('base','wind_step_mps_override',0.0);
assignin('base','sim_stop_time_override',simStop);

n = numel(pref_list);
res = zeros(n, 9);

for k = 1:n
    pref = pref_list(k);
    % Force each operating point to run in a fresh model session to avoid
    % first-point parameter latching from model init callbacks.
    bdclose('all');
    clear mex;
    out = run_single_explicit_case(pref, simStop, vacRef, vdcRef, qref);

    res(k,1) = pref;
    res(k,2) = out.pref_out_mean;
    res(k,3) = out.pmeas_mean;
    res(k,4) = out.ppcc_mean;
    res(k,5) = out.udc_mean;
    res(k,6) = out.udc_slope;
    res(k,7) = out.wref_mean;
    res(k,8) = out.omega_g_slope;
    res(k,9) = out.theta_tw_slope;
end

T = array2table(res, 'VariableNames', { ...
    'Pref_cmd', 'Pref_out_mean', 'Pmeas_mean', 'Ppcc_mean', ...
    'Udc_mean', 'Udc_slope', 'Wref_mean', 'OmegaG_slope', 'ThetaTw_slope'});

outDir = fullfile(root, 'Validation_Results');
if ~exist(outDir,'dir'), mkdir(outDir); end
writetable(T, fullfile(outDir, 'no_disturbance_sweep_explicit.csv'));

md = fullfile(outDir, 'no_disturbance_sweep_explicit.md');
fid = fopen(md,'w');
fprintf(fid, '# 无扰动稳态扫点（显式参数）\\n\\n');
fprintf(fid, '| Pref_cmd | Pref_out | Pmeas | Ppcc | Udc | dUdc | Wref | dOmega_g | dTheta_tw |\\n');
fprintf(fid, '|---:|---:|---:|---:|---:|---:|---:|---:|---:|\\n');
for k = 1:height(T)
    fprintf(fid, '| %.0f | %.0f | %.0f | %.0f | %.2f | %.4f | %.4f | %.6f | %.6f |\\n', ...
        T.Pref_cmd(k), T.Pref_out_mean(k), T.Pmeas_mean(k), T.Ppcc_mean(k), ...
        T.Udc_mean(k), T.Udc_slope(k), T.Wref_mean(k), T.OmegaG_slope(k), T.ThetaTw_slope(k));
end
fclose(fid);

disp(T);
fprintf('Saved: %s\\n', fullfile(outDir,'no_disturbance_sweep_explicit.csv'));
fprintf('Saved: %s\\n', md);
end
