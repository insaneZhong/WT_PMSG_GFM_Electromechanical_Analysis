function row = run_pref_ramp_tuning_case(prefRamp_Wps, simStop, vdcRef)
% Test one non-structural GFM active-power reference ramp candidate.
% The selected ramp remains active in grid_forming_control.h after the run.

if nargin < 2
    simStop = 12.0;
end
if nargin < 3
    vdcRef = 1200;
end

root = fileparts(mfilename('fullpath'));
old = pwd;
cleanup = onCleanup(@() cd(old)); %#ok<NASGU>
cd(root);

patch_grid_header(fullfile(root, 'grid_forming_control.h'), prefRamp_Wps);
compile_mex(root);

diag = run_no_disturbance_diagnosis(false, simStop, 0.0, struct( ...
    'VdcRef_V', vdcRef, ...
    'VacRef_V', 563, ...
    'Pref_W', 1e6, ...
    'Qref_var', 0, ...
    'SkipMexCompile', true, ...
    'UseNumericSfunParams', true, ...
    'SaveSeries', true));

row = table(datetime('now'), simStop, vdcRef, prefRamp_Wps, ...
    diag.Ppcc_end_mean, diag.Ppcc_target_error_pu, ...
    diag.udc_end_mean, diag.udc_end_slope, ...
    local_tail_peak_to_peak(diag.series.udc_meas, 1.0), ...
    local_tail_peak_to_peak(diag.series.T_sh, 1.0), ...
    diag.omega_g_end_slope, diag.omega_t_end_slope, diag.theta_tw_end_slope, ...
    double(diag.power_reached_1MW_flag), double(diag.dc_settled_flag), ...
    double(diag.mech_settled_flag), double(diag.baseline_operational_flag), ...
    'VariableNames', { ...
    'timestamp','simStop','VdcRef_V','PrefRamp_Wps','Ppcc_mean','Ppcc_err_pu', ...
    'Udc_mean','Udc_slope','Udc_pp','T_sh_pp','omega_g_slope', ...
    'omega_t_slope','theta_tw_slope','flag_1MW','flag_dc_settled', ...
    'flag_mech_settled','flag_baseline_operational'});

outDir = fullfile(root, 'Validation_Results');
if ~exist(outDir, 'dir'), mkdir(outDir); end
outPath = fullfile(outDir, sprintf('pref_ramp_tuning_cases_%dV.csv', round(vdcRef)));
if exist(outPath, 'file')
    rows = [readtable(outPath); row]; %#ok<AGROW>
else
    rows = row;
end
writetable(rows, outPath);

disp(row);
fprintf('Saved: %s\n', outPath);
end

function patch_grid_header(hdrPath, prefRamp_Wps)
txt = fileread(hdrPath);
txt = regexprep(txt, '#define\s+GSI_PREF_RAMP_SLOPE\s+[0-9\.eE\+\-]+f?', ...
    sprintf('#define   GSI_PREF_RAMP_SLOPE                 %.8ff', prefRamp_Wps), 'once');
fid = fopen(hdrPath, 'w');
fwrite(fid, txt);
fclose(fid);
end

function compile_mex(root)
setenv('MW_MINGW64_LOC', 'C:\mingw64');
evalin('base', 'clear mex');
evalin('base', 'bdclose(''all'')');
evalin('base', sprintf( ...
    'cd(''%s''); mex main.c svpwm.c motorcontrol.c grid_forming_control.c;', ...
    strrep(root, '\', '\\')));
end

function pp = local_tail_peak_to_peak(s, tailSec)
t = s.t(:);
y = s.y(:);
idx = t >= t(end) - tailSec;
if nnz(idx) < 2
    pp = NaN;
else
    pp = max(y(idx)) - min(y(idx));
end
end
