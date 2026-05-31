function main_project_pipeline()
% 主程序：构网型风电机组机电耦合（模块化一键入口）
% 说明：
% 1) 本主程序不改你原有算法，只做分块调度；
% 2) 你可以通过“开关区”选择要运行的部分；
% 3) 所有常用参数集中在顶部，便于人工改值和复现实验。

%% ========================= 开关区（按需开/关） =========================
sw = struct();
sw.refreshSmallSignalParams = true;   % 是否先刷新小信号 Parameters.mat
sw.buildSameObjectTables    = true;   % 是否重建同一对象参数表
sw.exportSchemeAHints       = true;   % 是否导出 Scheme A 参数映射提示
sw.runNoDisturbanceDiag     = true;   % 是否执行无扰动诊断
sw.runTimingPowerScan       = false;  % 是否扫描并网时序+功率环（较耗时）
sw.runVoltageLoopScan       = false;  % 是否扫描电压环（较耗时）
sw.runRootCauseRanking      = true;   % 是否执行1MW根因排序诊断
sw.runSmallPerturbation     = false;  % 是否执行小扰动验证（建议稳态后再开）
sw.runFigureExport          = false;  % 是否导出论文图（已有脚本时再开）

%% ========================= 参数区（常用可改） =========================
cfg = struct();
cfg.pref      = 1e6;   % 有功参考（W）
cfg.qref      = 0;     % 无功参考（var）
cfg.vacRef    = 563;   % 交流电压参考（V）
cfg.vdcRef    = 1200;  % 直流电压参考（V）
cfg.simStop   = 5.0;   % 无扰动仿真时长（s）
cfg.windStep  = 0.0;   % 风速阶跃（m/s），无扰动设为0
cfg.useSchemeA = false; % 是否启用 Scheme A 控制参数覆盖

% 小扰动验证专用参数（仅 sw.runSmallPerturbation=true 时使用）
pert = struct();
pert.pref        = cfg.pref;
pert.qref        = cfg.qref;
pert.vacRef      = cfg.vacRef;
pert.vdcRef      = cfg.vdcRef;
pert.simStop     = 5.0;
pert.tailWindow  = 1.0;
pert.windStepMps = 0.10;
pert.useSchemeA  = true;

%% ========================= 路径准备 =========================
root = fileparts(mfilename('fullpath'));
old = pwd;
cleanupObj = onCleanup(@() cd(old)); %#ok<NASGU>
cd(root);

fprintf('\n========== GFM-MWT 模块化主程序启动 ==========\n');
fprintf('工作目录: %s\n', root);

%% ========================= 模块 1：小信号参数刷新 =========================
if sw.refreshSmallSignalParams
    fprintf('\n[模块1] 刷新小信号 Parameters.mat ...\n');
    ssmMat = locate_ssm_parameters_mat();
    ssmMatMsg = ssmMat; % 防止 Parameters.m 内部 clear 操作后变量失效
    ssmEigenDir = fileparts(ssmMat);
    evalin('base', sprintf('run(''%s'');', strrep(fullfile(ssmEigenDir, 'Parameters.m'), '\', '\\')));
    fprintf('[模块1] 完成: %s\n', ssmMatMsg);
end

%% ========================= 模块 2：同一对象参数表 =========================
if sw.buildSameObjectTables
    fprintf('\n[模块2] 重建同一对象参数表 ...\n');
    build_same_object_parameter_table();
    build_wt_parameters_from_simulink();
    fprintf('[模块2] 完成: Validation_Results 下已更新 CSV/MD。\n');
end

%% ========================= 模块 3：导出 Scheme A 映射提示 =========================
if sw.exportSchemeAHints
    fprintf('\n[模块3] 导出 Scheme A 参数映射提示 ...\n');
    export_schemeA_c_tuning_from_small_signal();
    fprintf('[模块3] 完成。\n');
end

%% ========================= 模块 4：无扰动稳态诊断 =========================
if sw.runNoDisturbanceDiag
    fprintf('\n[模块4] 执行无扰动稳态诊断 ...\n');
    diag = run_no_disturbance_diagnosis( ...
        cfg.useSchemeA, cfg.simStop, cfg.windStep, ...
        struct('UseNumericSfunParams', true, ...
               'Pref_W', cfg.pref, ...
               'Qref_var', cfg.qref, ...
               'VacRef_V', cfg.vacRef, ...
               'VdcRef_V', cfg.vdcRef));
    disp(diag);
    fprintf('[模块4] 完成: 重点看 presyn_end_mean / Ppcc_end_mean / udc_end_slope。\n');
end

%% ========================= 模块 5：并网时序+功率环扫描 =========================
if sw.runTimingPowerScan
    fprintf('\n[模块5] 扫描并网时序与功率环参数（耗时）...\n');
    assignin('base','use_schemeA_overrides',cfg.useSchemeA);
    T1 = scan_steady_nondisturbance_timing_powerloop(); %#ok<NASGU>
    fprintf('[模块5] 完成。\n');
end

%% ========================= 模块 6：电压环扫描 =========================
if sw.runVoltageLoopScan
    fprintf('\n[模块6] 扫描电压环参数（耗时）...\n');
    assignin('base','use_schemeA_overrides',cfg.useSchemeA);
    T2 = scan_steady_nondisturbance_voltage_loop(); %#ok<NASGU>
    fprintf('[模块6] 完成。\n');
end

%% ========================= 模块 7：1MW根因排序诊断 =========================
if sw.runRootCauseRanking
    fprintf('\n[模块7] 执行1MW送功根因排序诊断 ...\n');
    T3 = diagnose_1mw_steady_root_cause('single'); %#ok<NASGU>
    fprintf('[模块7] 完成。\n');
end

%% ========================= 模块 8：小扰动验证 =========================
if sw.runSmallPerturbation
    fprintf('\n[模块8] 执行小扰动验证 ...\n');
    resultsPert = run_small_perturbation_validation(pert); %#ok<NASGU>
    fprintf('[模块8] 完成。\n');
end

%% ========================= 模块 9：论文图导出 =========================
if sw.runFigureExport
    fprintf('\n[模块9] 导出论文图 ...\n');
    run_all_publication_figures();
    fprintf('[模块9] 完成。\n');
end

fprintf('\n========== 主程序执行结束 ==========\n');
fprintf('结果路径: %s\n', fullfile(root, 'Validation_Results'));
end
