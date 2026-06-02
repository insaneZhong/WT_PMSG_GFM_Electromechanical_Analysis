function results = scan_gfm_nonstructural_tuning_resumable(caseIndex)
% GFM-MWT 非结构参数扫描：单候选、可中断、可续跑。
% 用法：
%   scan_gfm_nonstructural_tuning_resumable()    % 依次运行尚未完成的候选
%   scan_gfm_nonstructural_tuning_resumable(3)   % 仅运行第 3 个候选
%
% 限制：仅修改参考值和 C 头文件中的控制参数，不改变控制结构。

root = fileparts(mfilename('fullpath'));
old = pwd;
cleanup = onCleanup(@() cd(old)); %#ok<NASGU>
cd(root);

cases = table( ...
    (1:6)', ...
    [1200; 1180; 1160; 1200; 1200; 1200], ...
    [0.05; 0.05; 0.05; 0.05; 0.05; 0.05], ...
    [0.0005; 0.0005; 0.0005; 0.0005; 0.0005; 0.0005], ...
    [5.0e6; 5.0e6; 5.0e6; 3.0e6; 2.0e6; 1.0e6], ...
    'VariableNames', {'CaseId','VdcRef_V','MSC_KP','MSC_KI','PrefRamp_Wps'});

outDir = fullfile(root, 'Validation_Results', 'Nonstructural_Tuning');
if ~exist(outDir, 'dir'), mkdir(outDir); end
outCsv = fullfile(outDir, 'nonstructural_tuning_cases.csv');

if nargin < 1 || isempty(caseIndex)
    caseList = cases.CaseId';
else
    caseList = caseIndex;
end

for id = caseList
    if exist(outCsv, 'file')
        done = readtable(outCsv);
        if any(done.CaseId == id)
            fprintf('Skip completed case %d.\n', id);
            continue;
        end
    end

    cfg = cases(cases.CaseId == id, :);
    patch_header(fullfile(root, 'motorcontrol.h'), 'MOTOR_PWM_SPEED_KP', cfg.MSC_KP);
    patch_header(fullfile(root, 'motorcontrol.h'), 'MOTOR_PWM_SPEED_KI', cfg.MSC_KI);
    patch_header(fullfile(root, 'grid_forming_control.h'), 'GSI_PREF_RAMP_SLOPE', cfg.PrefRamp_Wps);
    compile_mex(root);

    diag = run_no_disturbance_diagnosis(false, 8.0, 0.0, struct( ...
        'VdcRef_V', cfg.VdcRef_V, ...
        'VacRef_V', 563, ...
        'Pref_W', 1e6, ...
        'Qref_var', 0, ...
        'SkipMexCompile', true, ...
        'UseNumericSfunParams', true, ...
        'SaveSeries', false));

    row = table(id, cfg.VdcRef_V, cfg.MSC_KP, cfg.MSC_KI, cfg.PrefRamp_Wps, ...
        diag.Ppcc_end_mean, diag.Ppcc_target_error_pu, ...
        diag.udc_end_mean, diag.udc_end_slope, ...
        diag.omega_g_end_slope, diag.omega_t_end_slope, ...
        diag.theta_tw_end_slope, diag.T_sh_end_slope, ...
        diag.msc_iqref_end_maxabs, ...
        double(diag.power_reached_1MW_flag), ...
        double(diag.dc_bounded_flag), ...
        double(diag.baseline_operational_flag), ...
        'VariableNames', { ...
        'CaseId','VdcRef_V','MSC_KP','MSC_KI','PrefRamp_Wps', ...
        'Ppcc_mean_W','Ppcc_err_pu','Udc_mean_V','Udc_slope_Vps', ...
        'omega_g_slope','omega_t_slope','theta_tw_slope','T_sh_slope', ...
        'MSC_IqRef_maxabs_A','flag_1MW','flag_dc_bounded','flag_operational'});

    if exist(outCsv, 'file')
        results = [readtable(outCsv); row]; %#ok<AGROW>
    else
        results = row;
    end
    writetable(results, outCsv);
    save(fullfile(outDir, sprintf('case_%02d_diag.mat', id)), 'diag', 'cfg');
    fprintf('Saved case %d: %s\n', id, outCsv);
end

if exist(outCsv, 'file')
    results = readtable(outCsv);
else
    results = table();
end
disp(results);
end

function patch_header(hdrPath, symbol, value)
txt = fileread(hdrPath);
pattern = ['#define\s+' symbol '\s+[0-9\.eE\+\-]+f?'];
replacement = sprintf('#define   %-35s %.12g', symbol, value);
txt = regexprep(txt, pattern, replacement, 'once');
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
