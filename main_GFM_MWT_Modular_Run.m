%% GFM-MWT 主流程（稳健版）
clear; clc; close all;

% ---------------- switches ----------------
runCfg = struct();
runCfg.stage1_sameObjectParams     = true;
runCfg.stage1_smallSignalCompare   = true;
runCfg.stage1_originalTrajectories = false;
runCfg.stage1_originalControlScan  = false;
runCfg.stage2_noDisturbanceSteady  = false;
runCfg.stage3_rootCauseDiagnosis   = false;
runCfg.stage4_nonlinearFigures     = false;
runCfg.stage5_smallPerturbation    = false;

% 冻结开关，避免被下游 clear 影响
setappdata(0, 'GFM_MWT_RUNCFG_BACKUP', runCfg);

% ---------------- path/config ----------------
paths = local_paths();
oldDir = pwd;
cleanupObj = onCleanup(@() cd(oldDir)); %#ok<NASGU>

ssmCfg = struct();
ssmCfg.matpowerDir = paths.matpowerDir;
ssmCfg.resultDir = fullfile(paths.masterResultDir, 'Small_Signal_Comparison');
ssmCfg.modeSelect = 'auto_mech';
ssmCfg.targetFreqHz = NaN;
ssmCfg.saveFigures = true;
ssmCfg.closeFigures = false;
ssmCfg.modelKeys = ["GFL","GFM_GWT","GFM_MWT","GFM_MWT_AD"];
ssmCfg.sweeps = struct( ...
    'name',  {"SCR","XR","C_dc"}, ...
    'label', {"SCR","X/R","C_{dc} / F"}, ...
    'values', {logspace(log10(1.25), log10(25), 40), linspace(1, 20, 40), []});

nlCfg = struct('pref',1e6,'qref',0,'vacRef',563,'vdcRef',5000,'simStop',3.0, ...
    'tailWindow',1.0,'runBaseline',true,'runSchemeA',true,'runSlowScans',false,'refreshTables',true);
steadyCfg = struct('prefList',[0.85e6,1.0e6,1.2e6],'simStop',3.0,'vacRef',nlCfg.vacRef, ...
    'vdcRef',nlCfg.vdcRef,'qref',nlCfg.qref,'diagnosisMode','quick');
pertCfg = struct('pref',1e6,'qref',0,'vacRef',nlCfg.vacRef,'vdcRef',nlCfg.vdcRef, ...
    'simStop',5.0,'windStepMps',0.10,'useSchemeA',true);
setappdata(0, 'GFM_MWT_SSMCFG_BACKUP', ssmCfg);
setappdata(0, 'GFM_MWT_NLCFG_BACKUP', nlCfg);
setappdata(0, 'GFM_MWT_STEADYCFG_BACKUP', steadyCfg);
setappdata(0, 'GFM_MWT_PERTCFG_BACKUP', pertCfg);

fprintf('\n========== 主流程启动 ==========\n');
fprintf('Nonlinear dir: %s\n', paths.nonlinearDir);

% 1) same-object + scheme-A map
if get_switch('stage1_sameObjectParams')
    paths = local_paths();
    if exist(paths.smallSignalRoot, 'dir') == 7
        cd(paths.smallSignalRoot);
        if exist('sync_unified_parameters_to_all_models.m','file') == 2
            run('sync_unified_parameters_to_all_models.m');
        end
    end
    paths = local_paths();
    cd(paths.nonlinearDir);
    if exist('build_same_object_parameter_table.m','file') == 2, build_same_object_parameter_table(); end
    if exist('export_schemeA_c_tuning_from_small_signal.m','file') == 2, export_schemeA_c_tuning_from_small_signal(); end
end

% 2) small-signal compare
if get_switch('stage1_smallSignalCompare')
    ssmCfg = getappdata(0, 'GFM_MWT_SSMCFG_BACKUP');
    paths = local_paths();
    cd(paths.ssmEigenDir);
    paramMat = fullfile(paths.ssmEigenDir, 'Parameters.mat');
    if exist(paramMat, 'file') ~= 2
        evalin('base', sprintf('cd(''%s''); run(''Parameters.m'');', escape_path(paths.ssmEigenDir)));
    end
    baseParams = load(paramMat);
    for k = 1:numel(ssmCfg.sweeps)
        if strcmp(ssmCfg.sweeps(k).name, 'C_dc') && isempty(ssmCfg.sweeps(k).values)
            ssmCfg.sweeps(k).values = linspace(0.25 * baseParams.C_dc, 2.0 * baseParams.C_dc, 40);
        end
    end
    ssmResults = run_ssm_control_mode_comparison_modular(paths.ssmEigenDir, ssmCfg); %#ok<NASGU>
    paths = local_paths();
    save(fullfile(paths.masterResultDir, 'small_signal_comparison_results.mat'), 'ssmResults', 'ssmCfg');
end

if get_switch('stage1_originalTrajectories')
    paths = local_paths(); cd(paths.ssmEigenDir); run('Track_GFM_Mode_Trajectories.m');
end
if get_switch('stage1_originalControlScan')
    paths = local_paths(); cd(paths.ssmEigenDir); run('Scan_GFM_Control_Parameters_Run.m');
end

if get_switch('stage2_noDisturbanceSteady')
    steadyCfg = getappdata(0, 'GFM_MWT_STEADYCFG_BACKUP');
    paths = local_paths(); cd(paths.nonlinearDir);
    steadyResults = run_no_disturbance_sweep_reliable(steadyCfg.prefList, steadyCfg.simStop, steadyCfg.vacRef, steadyCfg.vdcRef, steadyCfg.qref); %#ok<NASGU>
    save(fullfile(paths.masterResultDir, 'stage2_no_disturbance_steady_results.mat'), 'steadyResults', 'steadyCfg');
end

if get_switch('stage3_rootCauseDiagnosis')
    steadyCfg = getappdata(0, 'GFM_MWT_STEADYCFG_BACKUP');
    paths = local_paths(); cd(paths.nonlinearDir);
    diagResults = diagnose_1mw_steady_root_cause(steadyCfg.diagnosisMode); %#ok<NASGU>
    deliveryResults = scan_delivery_limits_no_disturbance(); %#ok<NASGU>
    save(fullfile(paths.masterResultDir, 'stage3_root_cause_diagnosis_results.mat'), 'diagResults', 'deliveryResults', 'steadyCfg');
end

if get_switch('stage4_nonlinearFigures')
    nlCfg = getappdata(0, 'GFM_MWT_NLCFG_BACKUP');
    paths = local_paths(); cd(paths.nonlinearDir);
    nlResults = run_all_publication_figures(nlCfg); %#ok<NASGU>
    save(fullfile(paths.masterResultDir, 'nonlinear_publication_figure_results.mat'), 'nlResults', 'nlCfg');
end

if get_switch('stage5_smallPerturbation')
    pertCfg = getappdata(0, 'GFM_MWT_PERTCFG_BACKUP');
    paths = local_paths(); cd(paths.nonlinearDir);
    pertResults = run_small_perturbation_validation(pertCfg); %#ok<NASGU>
    save(fullfile(paths.masterResultDir, 'stage5_small_perturbation_results.mat'), 'pertResults', 'pertCfg');
end

paths = local_paths();
fprintf('\n完成。结果目录:\n%s\n', paths.masterResultDir);

function tf = get_switch(name)
cfgSw = getappdata(0, 'GFM_MWT_RUNCFG_BACKUP');
tf = isstruct(cfgSw) && isfield(cfgSw, name) && logical(cfgSw.(name));
end

function p = escape_path(p)
p = strrep(p, '''', '''''');
end

function paths = local_paths()
thisFile = mfilename('fullpath');
if isempty(thisFile)
    nonlinearDir = pwd;
else
    nonlinearDir = fileparts(thisFile);
end

paths = struct();
paths.nonlinearDir = nonlinearDir;
lvl1 = fileparts(nonlinearDir);
lvl2 = fileparts(lvl1);
paths.projectRoot = fileparts(lvl2);
paths.smallSignalRoot = fullfile(paths.projectRoot, '（1）小信号模型');
paths.smallSignalValidation = fullfile(paths.smallSignalRoot, 'WT_PMSG_GFM_Electromechanical_Validation');
paths.ssmEigenDir = fullfile(paths.smallSignalValidation, 'EigenAnalysis');
paths.matpowerDir = 'D:\apps\matlab\R2024b\bin\matpower8.0';
paths.masterResultDir = fullfile(paths.nonlinearDir, 'Validation_Results', 'Master_Run');
if ~exist(paths.masterResultDir, 'dir')
    mkdir(paths.masterResultDir);
end
end
