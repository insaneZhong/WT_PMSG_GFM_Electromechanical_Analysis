function T = scan_dc_balance_10s_targeted()
% 10s 无扰动目标整定：
% 目标阈值（可按需改）：
%   |udc_end_slope|      < 5 V/s
%   |omega_g_end_slope|  < 1e-2
%   |theta_tw_end_slope| < 5e-4
%   |T_sh_end_slope|     < 1e4 N*m/s

root = fileparts(mfilename('fullpath'));
old = pwd;
cleanup = onCleanup(@() cd(old)); %#ok<NASGU>
cd(root);

motorHdr = fullfile(root, 'motorcontrol.h');
gridHdr  = fullfile(root, 'grid_forming_control.h');
origMotor = fileread(motorHdr);
origGrid  = fileread(gridHdr);
restoreObj = onCleanup(@() restore_files(motorHdr, origMotor, gridHdr, origGrid)); %#ok<NASGU>

% [V_LOOP_BW, SPEED_KP, SPEED_KI, GSI_V_KP, GSI_V_KI, GSI_P_KP, GSI_P_KI]
cands = [ ...
    30, 0.24, 0.0024, 1.1309733, 0.0282743, 1e-6, 2e-5;  % baseline
    20, 0.18, 0.0018, 1.1309733, 0.0282743, 1e-6, 2e-5;
    12, 0.12, 0.0012, 1.1309733, 0.0282743, 1e-6, 2e-5;
    30, 0.24, 0.0024, 0.90,      0.0200,    8e-7, 1.5e-5];

rows = [];
for k = 1:size(cands,1)
    p = cands(k,:);
    patch_motor_header(motorHdr, p(1), p(2), p(3));
    patch_grid_header(gridHdr, p(4), p(5), p(6), p(7));
    compile_mex(root);

    diag = run_no_disturbance_diagnosis(false, 10.0, 0.0, struct('SkipMexCompile', true, 'SaveSeries', false));
    ok = abs(diag.udc_end_slope) < 5 && ...
         abs(diag.omega_g_end_slope) < 1e-2 && ...
         abs(diag.theta_tw_end_slope) < 5e-4 && ...
         abs(diag.T_sh_end_slope) < 1e4;

    score = abs(diag.udc_end_slope)/5 + ...
            abs(diag.omega_g_end_slope)/1e-2 + ...
            abs(diag.theta_tw_end_slope)/5e-4 + ...
            abs(diag.T_sh_end_slope)/1e4 + ...
            abs(diag.Ppcc_target_error_pu)/0.05;

    rows = [rows; p, ...
        diag.udc_end_slope, diag.omega_g_end_slope, diag.theta_tw_end_slope, diag.T_sh_end_slope, ...
        diag.Ppcc_target_error_pu, double(diag.power_reached_1MW_flag), double(diag.dc_settled_flag), ...
        double(diag.mech_settled_flag), double(ok), score]; %#ok<AGROW>
end

T = array2table(rows, 'VariableNames', { ...
    'V_LOOP_BW','SPD_KP','SPD_KI','GSI_V_KP','GSI_V_KI','GSI_P_KP','GSI_P_KI', ...
    'udc_slope','omega_g_slope','theta_tw_slope','T_sh_slope','Ppcc_err_pu', ...
    'flag_1MW','flag_dc_settled','flag_mech_settled','flag_all_target','score'});
T = sortrows(T, 'score');

outDir = fullfile(root, 'Validation_Results');
if ~exist(outDir,'dir'), mkdir(outDir); end
writetable(T, fullfile(outDir, 'scan_dc_balance_10s_targeted.csv'));
save(fullfile(outDir, 'scan_dc_balance_10s_targeted.mat'), 'T');
disp(T(1:min(10,height(T)),:));
fprintf('Saved: %s\n', fullfile(outDir, 'scan_dc_balance_10s_targeted.csv'));
end

function patch_motor_header(hdrPath, vBw, skp, ski)
txt = fileread(hdrPath);
txt = regexprep(txt, '#define\s+V_LOOP_BANDWITH\s+[0-9\.eE\+\-]+', ...
    sprintf('#define   V_LOOP_BANDWITH             %.8g   //Hz', vBw), 'once');
txt = regexprep(txt, '#define\s+MOTOR_PWM_SPEED_KP\s+[0-9\.eE\+\-]+', ...
    sprintf('#define   MOTOR_PWM_SPEED_KP                   %.8g', skp), 'once');
txt = regexprep(txt, '#define\s+MOTOR_PWM_SPEED_KI\s+[0-9\.eE\+\-]+', ...
    sprintf('#define   MOTOR_PWM_SPEED_KI                   %.8g', ski), 'once');
fid = fopen(hdrPath, 'w'); fwrite(fid, txt); fclose(fid);
end

function patch_grid_header(hdrPath, vkp, vki, pkp, pki)
txt = fileread(hdrPath);
txt = regexprep(txt, '#define\s+GSI_V_LOOP_KP\s+[0-9\.eE\+\-]+', ...
    sprintf('#define   GSI_V_LOOP_KP               %.8g', vkp), 'once');
txt = regexprep(txt, '#define\s+GSI_V_LOOP_KI\s+[0-9\.eE\+\-]+', ...
    sprintf('#define   GSI_V_LOOP_KI               %.8g', vki), 'once');
txt = regexprep(txt, '#define\s+GSI_PLOOP_KP\s+[0-9\.eE\+\-]+', ...
    sprintf('#define   GSI_PLOOP_KP                        %.8g', pkp), 'once');
txt = regexprep(txt, '#define\s+GSI_PLOOP_KI\s+[0-9\.eE\+\-]+', ...
    sprintf('#define   GSI_PLOOP_KI                        %.8g', pki), 'once');
fid = fopen(hdrPath, 'w'); fwrite(fid, txt); fclose(fid);
end

function compile_mex(root)
setenv('MW_MINGW64_LOC', 'C:\mingw64');
evalin('base', 'clear mex');
evalin('base', sprintf('cd(''%s''); mex main.c svpwm.c motorcontrol.c grid_forming_control.c;', strrep(root, '\', '\\')));
end

function restore_files(motorPath, motorTxt, gridPath, gridTxt)
fid = fopen(motorPath, 'w'); fwrite(fid, motorTxt); fclose(fid);
fid = fopen(gridPath, 'w'); fwrite(fid, gridTxt); fclose(fid);
end
