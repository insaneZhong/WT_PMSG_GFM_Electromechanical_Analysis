%% 构网型风电机组机电耦合分析总控主程序
% 作用：
%   1. 把原来分散的小信号分析、参数扫描、非线性仿真出图统一到一个入口。
%   2. 所有常改参数尽量放在本文件开头，例如 SCR、XR、Cdc、仿真时间、功率指令等。
%   3. 每个分析模块都用 if 开关控制；不想运行的部分可以把开关设为 false，或直接注释对应代码块。
%
% 建议使用方式：
%   第一次运行：先只打开 runSmallSignalCompare，确认小信号扫参能正常出图。
%   非线性较慢：确认参数后再打开 runNonlinearFigures。

clear
clc
close all

%% 0. 运行开关：需要哪部分就打开哪部分
runCfg = struct();
runCfg.stage1_sameObjectParams = true;   % 第一步：建立同一对象参数表，并导出 Scheme A 控制参数头文件。
runCfg.stage1_smallSignalCompare = true; % 第一步补充：小信号四拓扑对照 + SCR/XR/Cdc 等扫参。
runCfg.stage1_originalTrajectories = false; % 可选：运行旧版模态轨迹脚本 Track_GFM_Mode_Trajectories.m。
runCfg.stage1_originalControlScan  = false; % 可选：运行旧版 h/mp、DVC、RPC、K_damp 二维扫描脚本。
runCfg.stage2_noDisturbanceSteady = false;  % 第二步：无扰动稳态检查，先不加风速阶跃/故障/阻尼验证。
runCfg.stage3_rootCauseDiagnosis = false;   % 第三步：排查 1 MW 送出、DC-link、功率环、电压环、并网时序。
runCfg.stage4_nonlinearFigures = false;     % 第四步：基于 Scheme A/Baseline 统一出图。
runCfg.stage5_smallPerturbation = false;    % 第五步：无扰动稳住后再做小扰动验证。

%% 1. 路径设置：如果你以后移动文件夹，优先改这里
paths.nonlinearDir = fileparts(mfilename('fullpath'));
paths.projectRoot = 'D:\博士工作\论文工作';
paths.smallSignalRoot = fullfile(paths.projectRoot, '（1）小信号模型');
paths.smallSignalValidation = fullfile(paths.smallSignalRoot, 'WT_PMSG_GFM_Electromechanical_Validation');
paths.ssmEigenDir = fullfile(paths.smallSignalValidation, 'EigenAnalysis');
paths.matpowerDir = 'D:\apps\matlab\R2024b\bin\matpower8.0';

% 本次总控程序自己的结果目录，避免覆盖旧脚本结果。
paths.masterResultDir = fullfile(paths.nonlinearDir, 'Validation_Results', 'Master_Run');
if ~exist(paths.masterResultDir, 'dir')
    mkdir(paths.masterResultDir);
end

oldDir = pwd;
cleanupObj = onCleanup(@() cd(oldDir)); %#ok<NASGU>

%% 2. 小信号分析参数：这里改 SCR、XR、Cdc 或新增其他系统参数
ssmCfg = struct();
ssmCfg.matpowerDir = paths.matpowerDir;
ssmCfg.resultDir = fullfile(paths.masterResultDir, 'Small_Signal_Comparison');
ssmCfg.targetFreqHz = 2.0;           % 目标机电/轴系模态频率，默认跟踪 2 Hz 附近模态。
ssmCfg.saveFigures = true;           % 是否保存 png/fig。
ssmCfg.closeFigures = false;         % 批量出图时可设为 true，避免图窗太多。

% 需要比较的四类模型。若只想看 GFM-MWT 和 GFM-MWT+AD，可以删掉前两个字符串。
ssmCfg.modelKeys = ["GFL", "GFM_GWT", "GFM_MWT", "GFM_MWT_AD"];

% 系统条件扫描。name 必须是 Parameters.mat 里存在的变量。
% SCR/XR 比较特殊：程序会自动由 SCR、XR 重算 rg、lg。
% C_dc 是直流电容；也可以新增例如 h、mp，但控制参数更建议用后续专门控制扫描脚本。
ssmCfg.sweeps = struct( ...
    'name',  {"SCR", "XR", "C_dc"}, ...
    'label', {"SCR", "X/R", "C_{dc} / F"}, ...
    'values', { ...
        logspace(log10(1.25), log10(25), 40), ...  % SCR：弱网到强网。
        linspace(1, 20, 40), ...                   % X/R：电网阻抗电抗/电阻比。
        [] ...                                     % C_dc：下面读取基准值后自动生成。
    });

% 如果你想扫控制参数，也可以把上面的 ssmCfg.sweeps 换成下面这种形式。
% 注意：name 要和 Parameters.m/Parameters.mat 里的变量名完全一致。
% ssmCfg.sweeps = struct( ...
%     'name',  {"h", "mp", "k_pdc", "k_idc", "K_damp"}, ...
%     'label', {"h", "m_p", "k_{pdc}", "k_{idc}", "K_{damp}"}, ...
%     'values', { ...
%         linspace(0.2, 8, 40), ...
%         logspace(-3, 0, 40), ...
%         logspace(-2, 2, 40), ...
%         logspace(-2, 2, 40), ...
%         linspace(-1.5e7, 5e6, 60) ...
%     });

%% 3. 非线性仿真参数：这里改功率指令、仿真时间、是否运行 Scheme A
nlCfg = struct();
nlCfg.pref = 1e6;                  % 有功功率指令，单位 W。
nlCfg.qref = 0;                    % 无功功率指令，单位 var。
nlCfg.vacRef = 563;                % 交流电压参考，沿用当前模型 S-Function 参数。
nlCfg.vdcRef = 5000;               % 直流母线电压参考，单位 V。
nlCfg.simStop = 3.0;               % 仿真结束时间，单位 s。正式分析建议至少 3 s。
nlCfg.tailWindow = 1.0;            % 统计稳态指标时取末尾多少秒。
nlCfg.runBaseline = true;          % 是否运行未启用 Scheme A 覆盖的 C 控制器。
nlCfg.runSchemeA = true;           % 是否运行 Scheme A 小信号参数映射后的 C 控制器。
nlCfg.runSlowScans = false;        % 是否附带运行较慢诊断扫描。
nlCfg.refreshTables = true;        % 是否刷新同对象参数表和 Scheme A 头文件。

%% 3.1 无扰动稳态与排查参数：第二、三步使用
steadyCfg = struct();
steadyCfg.prefList = [0.85e6, 1.0e6, 1.2e6]; % 无扰动功率指令扫描。先看 1 MW 附近能否稳定送出。
steadyCfg.simStop = 3.0;                      % 无扰动稳态仿真时间。
steadyCfg.vacRef = nlCfg.vacRef;
steadyCfg.vdcRef = nlCfg.vdcRef;
steadyCfg.qref = nlCfg.qref;
steadyCfg.diagnosisMode = 'quick';            % 可选：single / single_schemea / ultraquick / quick / full。

%% 3.2 小扰动验证参数：第五步使用，必须在无扰动基本稳住后再打开
pertCfg = struct();
pertCfg.pref = 1e6;                % 小扰动验证时的基准有功指令。
pertCfg.qref = 0;
pertCfg.vacRef = nlCfg.vacRef;
pertCfg.vdcRef = nlCfg.vdcRef;
pertCfg.simStop = 5.0;
pertCfg.windStepMps = 0.10;        % 小风速阶跃，建议先从 0.05~0.10 m/s 开始。
pertCfg.useSchemeA = true;         % true：使用小信号映射后的 Scheme A 控制器。

%% 4. 第一步：同一对象参数表 + Scheme A 控制参数映射
if runCfg.stage1_sameObjectParams
    fprintf('\n========== 1/7 第一步：同一对象参数表与 Scheme A 映射 ==========\n');
    cd(paths.smallSignalRoot);
    if exist('sync_unified_parameters_to_all_models.m', 'file') == 2
        run('sync_unified_parameters_to_all_models.m');
    else
        warning('未找到 sync_unified_parameters_to_all_models.m，跳过小信号参数同步。');
    end

    cd(paths.nonlinearDir);
    if exist('build_same_object_parameter_table.m', 'file') == 2
        build_same_object_parameter_table();
    end
    if exist('export_schemeA_c_tuning_from_small_signal.m', 'file') == 2
        export_schemeA_c_tuning_from_small_signal();
    end
end

%% 5. 第一步补充：小信号四拓扑对照与系统参数扫描
if runCfg.stage1_smallSignalCompare
    fprintf('\n========== 2/7 第一步补充：小信号四拓扑对照与系统条件扫参 ==========\n');
    cd(paths.ssmEigenDir);

    % C_dc 扫描范围依赖 Parameters.mat 中的基准 C_dc，所以在这里补齐。
    % 注意：Parameters.m 末尾含有 clear all，不能在本主程序里直接 run，
    % 否则会清掉本主程序变量。这里优先读取已经生成的 Parameters.mat。
    paramMat = fullfile(paths.ssmEigenDir, 'Parameters.mat');
    if exist(paramMat, 'file') ~= 2
        evalin('base', sprintf('cd(''%s''); run(''Parameters.m'');', escape_path(paths.ssmEigenDir)));
    end
    baseParams = load(paramMat);
    for k = 1:numel(ssmCfg.sweeps)
        if strcmp(ssmCfg.sweeps(k).name, 'C_dc') && isempty(ssmCfg.sweeps(k).values)
            ssmCfg.sweeps(k).values = linspace(0.25*baseParams.C_dc, 2.0*baseParams.C_dc, 40);
        end
    end

    ssmResults = run_ssm_control_mode_comparison_modular(paths.ssmEigenDir, ssmCfg);
    save(fullfile(paths.masterResultDir, 'small_signal_comparison_results.mat'), 'ssmResults', 'ssmCfg');
end

%% 6. 可选：小信号旧版模态轨迹脚本，用于 h、mp、k_pdc/k_idc、K_damp 轨迹
if runCfg.stage1_originalTrajectories
    fprintf('\n========== 3/7 可选：小信号旧版模态轨迹脚本 ==========\n');
    cd(paths.ssmEigenDir);
    run('Track_GFM_Mode_Trajectories.m');
end

%% 7. 可选：小信号旧版控制参数稳定域扫描
if runCfg.stage1_originalControlScan
    fprintf('\n========== 4/7 可选：小信号旧版控制参数稳定域扫描 ==========\n');
    cd(paths.ssmEigenDir);
    run('Scan_GFM_Control_Parameters_Run.m');
end

%% 8. 第二步：非线性模型无扰动稳态检查
if runCfg.stage2_noDisturbanceSteady
    fprintf('\n========== 5/7 第二步：无扰动稳态检查 ==========\n');
    cd(paths.nonlinearDir);
    steadyResults = run_no_disturbance_sweep_reliable( ...
        steadyCfg.prefList, steadyCfg.simStop, steadyCfg.vacRef, steadyCfg.vdcRef, steadyCfg.qref);
    save(fullfile(paths.masterResultDir, 'stage2_no_disturbance_steady_results.mat'), 'steadyResults', 'steadyCfg');
end

%% 9. 第三步：非线性稳态不稳与 1 MW 送出能力排查
if runCfg.stage3_rootCauseDiagnosis
    fprintf('\n========== 6/7 第三步：1 MW 送出能力与控制环节排查 ==========\n');
    cd(paths.nonlinearDir);
    diagResults = diagnose_1mw_steady_root_cause(steadyCfg.diagnosisMode);
    deliveryResults = scan_delivery_limits_no_disturbance();
    save(fullfile(paths.masterResultDir, 'stage3_root_cause_diagnosis_results.mat'), ...
        'diagResults', 'deliveryResults', 'steadyCfg');
end

%% 10. 第四步：非线性 Simulink 验证与统一出图
if runCfg.stage4_nonlinearFigures
    fprintf('\n========== 7/7 第四步：非线性 Simulink 验证与统一出图 ==========\n');
    cd(paths.nonlinearDir);
    nlResults = run_all_publication_figures(nlCfg);
    save(fullfile(paths.masterResultDir, 'nonlinear_publication_figure_results.mat'), 'nlResults', 'nlCfg');
end

%% 11. 第五步：小扰动验证，必须在无扰动稳态基础上使用
if runCfg.stage5_smallPerturbation
    fprintf('\n========== 额外：第五步 小扰动验证 ==========\n');
    cd(paths.nonlinearDir);
    pertResults = run_small_perturbation_validation(pertCfg);
    save(fullfile(paths.masterResultDir, 'stage5_small_perturbation_results.mat'), 'pertResults', 'pertCfg');
end

fprintf('\n全部已选择模块运行结束。\n结果主目录：\n%s\n', paths.masterResultDir);

function p = escape_path(p)
%ESCAPE_PATH 把路径中的单引号转义，供 evalin 字符串使用。
p = strrep(p, '''', '''''');
end
