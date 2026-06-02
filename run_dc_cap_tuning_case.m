function row = run_dc_cap_tuning_case(dcCapF, simStop)
% Run one aligned GFM case after synchronizing the physical and control Cdc.
% The selected Cdc remains active in Grid_Forming_PMSG.mdl and both headers.

if nargin < 2
    simStop = 4.0;
end

root = fileparts(mfilename('fullpath'));
old = pwd;
cleanup = onCleanup(@() cd(old)); %#ok<NASGU>
cd(root);

patch_macro(fullfile(root, 'motorcontrol.h'), dcCapF);
patch_macro(fullfile(root, 'grid_forming_control.h'), dcCapF);

mdl = 'Grid_Forming_PMSG';
load_system(mdl);
set_param([mdl '/Cd'], 'Capacitance', sprintf('%.12g', dcCapF));
set_param([mdl '/Cd'], 'InitialVoltage', '1200');
save_system(mdl);

compile_mex(root);
diag = run_no_disturbance_diagnosis(false, simStop, 0.0, struct( ...
    'VdcRef_V', 1200, ...
    'VacRef_V', 563, ...
    'Pref_W', 1e6, ...
    'Qref_var', 0, ...
    'SkipMexCompile', true, ...
    'UseNumericSfunParams', true, ...
    'SaveSeries', true));

udcPp = local_tail_peak_to_peak(diag.series.udc_meas, 1.0);
tshPp = local_tail_peak_to_peak(diag.series.T_sh, 1.0);

row = table(datetime('now'), simStop, dcCapF, ...
    diag.Ppcc_end_mean, diag.Ppcc_target_error_pu, ...
    diag.udc_end_mean, diag.udc_end_slope, udcPp, ...
    diag.omega_g_end_slope, diag.omega_t_end_slope, ...
    diag.theta_tw_end_slope, diag.T_sh_end_slope, tshPp, ...
    double(diag.power_reached_1MW_flag), ...
    double(diag.dc_settled_flag), ...
    double(diag.mech_settled_flag), ...
    'VariableNames', { ...
    'timestamp','simStop','Cdc_F','Ppcc_mean','Ppcc_err_pu', ...
    'Udc_mean','Udc_slope','Udc_pp','omega_g_slope','omega_t_slope', ...
    'theta_tw_slope','T_sh_slope','T_sh_pp', ...
    'flag_1MW','flag_dc_settled','flag_mech_settled'});

row.score = abs(row.Ppcc_err_pu)/0.05 + ...
    abs(row.Udc_slope)/5 + row.Udc_pp/120 + ...
    abs(row.omega_g_slope)/1e-2 + ...
    abs(row.omega_t_slope)/1e-2 + ...
    abs(row.theta_tw_slope)/5e-4 + ...
    abs(row.T_sh_slope)/1e4 + row.T_sh_pp/1e5;

outDir = fullfile(root, 'Validation_Results');
if ~exist(outDir, 'dir'), mkdir(outDir); end
outPath = fullfile(outDir, 'dc_cap_tuning_cases.csv');
if exist(outPath, 'file')
    oldRows = readtable(outPath);
    rows = [oldRows; row]; %#ok<AGROW>
else
    rows = row;
end
writetable(rows, outPath);

disp(row);
fprintf('Saved: %s\n', outPath);
end

function patch_macro(path, dcCapF)
txt = fileread(path);
txt = regexprep(txt, '#define\s+GRID_UDC__C\s+[0-9\.eE\+\-]+', ...
    sprintf('#define   GRID_UDC__C                           %.12g', dcCapF), 'once');
fid = fopen(path, 'w');
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
pp = max(y(idx)) - min(y(idx));
end
