function results = scan_gfm_warmstart_dvc_tuning(caseIndex)
% Compare MSC-DVC candidates from the same 30 s operating-point seed.

root = fileparts(mfilename('fullpath'));
old = pwd;
cleanup = onCleanup(@() cd(old)); %#ok<NASGU>
cd(root);

cases = table( ...
    (1:14)', ...
    [0.05; 0.05; 0.05; 0.05; 0.05; 0.05; 0.05; 0.05; 0.05; 0.05; 0.05; 0.10; 0.15; 0.20], ...
    [0.0007; 0.0005; 0.0003; 0.0007; 0.0007; 0.0005; 0.0007; 0.0007; 0.0007; 0.0007; 0.0007; 0.0007; 0.0007; 0.0007], ...
    [0.00015; 0.00015; 0.00015; 0.00010; 0.00020; 0.00020; 0.00030; 0.00040; 0.00018; 0.00022; 0.00025; 0.00020; 0.00020; 0.00020], ...
    'VariableNames', {'CaseId', 'MSC_KP', 'MSC_KI', 'IqPowerFF_A_per_W'});

if nargin < 1 || isempty(caseIndex)
    caseList = cases.CaseId';
else
    caseList = caseIndex;
end

seedFile = fullfile(root, 'Validation_Results', 'Initial_State', ...
    'Grid_Forming_PMSG_TuningSeed_30s.mat');
if exist(seedFile, 'file') ~= 2
    error('Tuning seed not found: %s', seedFile);
end

outDir = fullfile(root, 'Validation_Results', 'WarmStart_DVC_Tuning');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
outCsv = fullfile(outDir, 'warmstart_dvc_tuning.csv');

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
    patch_header(fullfile(root, 'motorcontrol.h'), 'MOTOR_IQ_POWER_FF_A_PER_W', cfg.IqPowerFF_A_per_W);
    compile_mex(root);

    diag = run_no_disturbance_diagnosis(false, 8.0, 0.0, struct( ...
        'VdcRef_V', 940, ...
        'VacRef_V', 563, ...
        'Pref_W', 1e6, ...
        'Qref_var', 0, ...
        'SkipMexCompile', true, ...
        'UseNumericSfunParams', true, ...
        'InitialStateFile', seedFile, ...
        'InitialStateVar', 'xInitial', ...
        'SaveSeries', false));

    row = table(id, cfg.MSC_KP, cfg.MSC_KI, cfg.IqPowerFF_A_per_W, ...
        diag.Ppcc_end_mean, diag.Ppcc_target_error_pu, ...
        diag.udc_end_mean, diag.udc_end_slope, ...
        diag.msc_iqref_end_mean, diag.msc_iqref_end_maxabs, ...
        diag.omega_g_end_slope, diag.omega_t_end_slope, ...
        diag.theta_tw_end_slope, diag.T_sh_end_slope, ...
        double(diag.power_reached_1MW_flag), ...
        double(diag.dc_settled_flag), ...
        double(diag.mech_settled_flag), ...
        double(diag.baseline_operational_flag), ...
        'VariableNames', { ...
        'CaseId', 'MSC_KP', 'MSC_KI', 'IqPowerFF_A_per_W', ...
        'Ppcc_mean_W', 'Ppcc_err_pu', 'Udc_mean_V', 'Udc_slope_Vps', ...
        'MSC_IqRef_mean_A', 'MSC_IqRef_maxabs_A', ...
        'omega_g_slope', 'omega_t_slope', 'theta_tw_slope', 'T_sh_slope', ...
        'flag_1MW', 'flag_dc_settled', 'flag_mech_settled', 'flag_operational'});

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
