function T = scan_msc_dc_loop_tuning(simStop)
% Scan machine-side DC-link loop tuning for no-disturbance 1MW delivery.
% Tuned symbols in motorcontrol.h:
%   - V_LOOP_BANDWITH
%   - MOTOR_PWM_SPEED_KP / MOTOR_PWM_SPEED_KI
% The operating point is explicitly fixed to the aligned 1MW / 1200V model.

if nargin < 1
    simStop = 4.0;
end

root = fileparts(mfilename('fullpath'));
old = pwd;
cleanup = onCleanup(@() cd(old));
cd(root);

hdrPath = fullfile(root, 'motorcontrol.h');
origHdr = fileread(hdrPath);
restoreObj = onCleanup(@() restore_file(hdrPath, origHdr)); %#ok<NASGU>

% [V_LOOP_BANDWITH, SPEED_KP, SPEED_KI]
cands = [ ...
    30, 0.24, 0.0024; ... % baseline
    20, 0.18, 0.0018; ...
    12, 0.12, 0.0012; ...
     8, 0.08, 0.0008; ...
     5, 0.05, 0.0005; ...
     3, 0.03, 0.0003];

rows = [];
for k = 1:size(cands, 1)
    vBw = cands(k, 1);
    skp = cands(k, 2);
    ski = cands(k, 3);

    patch_header(hdrPath, vBw, skp, ski);
    compile_mex(root);

    diag = run_no_disturbance_diagnosis(false, simStop, 0.0, struct( ...
        'VdcRef_V', 1200, ...
        'VacRef_V', 563, ...
        'Pref_W', 1e6, ...
        'Qref_var', 0, ...
        'SkipMexCompile', true, ...
        'UseNumericSfunParams', true, ...
        'SaveSeries', false));

    score = abs(diag.udc_end_slope)/1000 + ...
            abs(diag.Ppcc_target_error_pu)*2 + ...
            abs(diag.omega_g_end_slope)*2 + ...
            abs(diag.theta_tw_end_slope)*100 + ...
            abs(diag.T_sh_end_slope)/1e6;

    rows = [rows; ...
        vBw, skp, ski, ...
        diag.Ppcc_end_mean, diag.Ppcc_target_error_pu, ...
        diag.udc_end_mean, diag.udc_end_slope, ...
        diag.omega_g_end_slope, diag.omega_t_end_slope, ...
        diag.theta_tw_end_slope, diag.T_sh_end_slope, diag.torque_balance_Taero_Te_err_pu, ...
        double(diag.power_reached_1MW_flag), double(diag.dc_settled_flag), ...
        double(diag.mech_settled_flag), score]; %#ok<AGROW>
end

T = array2table(rows, 'VariableNames', { ...
    'V_LOOP_BW','SPD_KP','SPD_KI', ...
    'Ppcc_end_mean','Ppcc_err_pu', ...
    'Udc_end_mean','Udc_end_slope', ...
    'omega_g_slope','omega_t_slope','theta_tw_slope','T_sh_slope', ...
    'TaeroTe_err_pu', ...
    'flag_1MW','flag_dc_settled','flag_mech_settled','score'});

T = sortrows(T, 'score');

outDir = fullfile(root, 'Validation_Results');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
writetable(T, fullfile(outDir, 'scan_msc_dc_loop_tuning.csv'));
save(fullfile(outDir, 'scan_msc_dc_loop_tuning.mat'), 'T');
disp(T);
fprintf('Saved: %s\n', fullfile(outDir, 'scan_msc_dc_loop_tuning.csv'));
end

function patch_header(hdrPath, vBw, skp, ski)
txt = fileread(hdrPath);
txt = regexprep(txt, '#define\s+V_LOOP_BANDWITH\s+[0-9\.eE\+\-]+', ...
    sprintf('#define   V_LOOP_BANDWITH             %.8g   //Hz', vBw), 'once');
txt = regexprep(txt, '#define\s+MOTOR_PWM_SPEED_KP\s+[0-9\.eE\+\-]+', ...
    sprintf('#define   MOTOR_PWM_SPEED_KP                   %.8g', skp), 'once');
txt = regexprep(txt, '#define\s+MOTOR_PWM_SPEED_KI\s+[0-9\.eE\+\-]+', ...
    sprintf('#define   MOTOR_PWM_SPEED_KI                   %.8g', ski), 'once');

fid = fopen(hdrPath, 'w');
fwrite(fid, txt);
fclose(fid);
end

function compile_mex(root)
setenv('MW_MINGW64_LOC', 'C:\mingw64');
evalin('base', 'clear mex');
evalin('base', sprintf('cd(''%s''); mex main.c svpwm.c motorcontrol.c grid_forming_control.c;', strrep(root, '\', '\\')));
end

function restore_file(path, txt)
fid = fopen(path, 'w');
fwrite(fid, txt);
fclose(fid);
end
