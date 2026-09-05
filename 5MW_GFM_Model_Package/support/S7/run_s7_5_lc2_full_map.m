function resultPath = run_s7_5_lc2_full_map()
%RUN_S7_5_LC2_FULL_MAP
% S7-5B/LC2：完整 Legacy 控制器一次/多步闭环 Replica 认证。
%
% 生产 C 源和串联 Replica 使用同一组输入、同一控制更新顺序。测试覆盖：
% nominal、Udc 小扰动、Udc+PCC 电流组合扰动；每个工况运行 100 个
% 连续控制事件，只保存逐项最大误差摘要和中文报告。
%
% 本 LC2 仍是控制器层认证：不包含 Simulink plant、PWM 门极和长时序。

here = fileparts(mfilename('fullpath'));
tempDir = fullfile(here, 'temp', 'S7_5_LegacyCertification');
if ~isfolder(tempDir)
    mkdir(tempDir);
end
addpath(here, '-begin');

% 生产源码目录和本轮临时 C probe。
srcDir = fullfile(fileparts(here), 'CurrentModel_Idealized');
mexName = 'main_s7_5_lc2_probe';
mexPath = fullfile(tempDir, [mexName, '.', mexext]);
if ~isfile(mexPath)
    oldDir = pwd;
    cleanupDir = onCleanup(@() cd(oldDir)); %#ok<NASGU>
    cd(srcDir);
    setenv('MW_MINGW64_LOC', 'C:\mingw64');
    clear mex;
    defs = {'-DLEGACY_AD_S_FUNCTION_NAME=main_s7_5_lc2_probe', ...
        '-DIDEAL_CONTINUOUS_CONTROLLER=1', ...
        '-DIDEAL_AVG_OUTPUTS=0', ...
        '-DLEGACY_SFUNCTION_SAMPLE_TIME_S=4e-6', ...
        '-DLEGACY_RESET_CONTROLLER_ON_INIT=1', ...
        '-DENABLE_VSG_EQUIV_WREF=1', ...
        '-DGSI_GFL_MODE=0', ...
        '-DPRESYN_SWITCH_TIME=0', ...
        '-DGSI_GFM_ENABLE_TIME_S=0'};
    mex(defs{:}, '-output', fullfile(tempDir, mexName), ...
        'main_legacy_ad_base.c', 'svpwm.c', ...
        'motorcontrol_legacy_ad_base.c', 'grid_forming_control.c');
end
addpath(tempDir, '-begin');
clear main_s7_5_lc2_probe;

TsMain = 4e-6;
N = 100;
PRef = 1e6;
QRef = 0;
VdcRef = 1000;
Vg = 563;
angleGrid = 0.02;
[ua, ub, uc] = phase_voltage(Vg, angleGrid);
uab = ua - ub;
ubc = ub - uc;
uca = uc - ua;

base = zeros(20, 1);
base(1:3) = [0.1; -0.05; -0.05];
base(4) = 900;
base(5) = 100;
base(6) = 0.01;
base(7:9) = [0.1; -0.05; -0.05];
base(10:12) = [uab; ubc; uca];
base(13:15) = [10; -5; -5];
base(16) = 3.0;
base(17) = 0;
base(18) = 1;
base(19) = 0;
base(20) = VdcRef;

caseNames = {'nominal'; 'udc_small'; 'combined'};
caseInputs = cell(numel(caseNames), 1);
caseInputs{1} = base;
caseInputs{2} = base;
caseInputs{2}(4) = base(4) + 0.5;
caseInputs{3} = caseInputs{2};
caseInputs{3}(13:15) = [10.2; -5.1; -5.1];

rows = cell(0, 1);
allCasePass = true;
for ic = 1:numel(caseNames)
    u0 = caseInputs{ic};
    yC = local_run_c_probe(mexName, u0, TsMain, N, PRef, QRef, Vg, VdcRef, ic);
    n = min(N, size(yC, 1));
    st = s7_legacy_replica_full_step('initial_state');
    p = s7_legacy_replica_full_step('defaults');
    rep = zeros(n, 25);
    cvals = zeros(n, 25);
    err = zeros(n, 25);
    for k = 1:n
        uk = struct('Udc', u0(4), 'VdcRef', VdcRef, ...
            'system_Time', u0(16), 'Ia1', u0(7), ...
            'Ib1', u0(8), 'Ic1', u0(9), 'We', u0(5), ...
            'RotorPos', u0(6), 'P_ref', PRef, 'Q_ref', QRef, ...
            'Pre_syn', true, 'GFM_enabled', true, ...
            'pcc_uab', u0(10), 'pcc_ubc', u0(11), 'pcc_uca', u0(12), ...
            'pcc_Ia', u0(13), 'pcc_Ib', u0(14), 'pcc_Ic', u0(15));
        [st, o, tr] = s7_legacy_replica_full_step(st, uk, p); %#ok<ASGLU>
        rep(k, :) = replica_vector(o);
        cvals(k, :) = c_vector(yC(k, :));
        err(k, :) = cvals(k, :) - rep(k, :);
    end
    maxAbs = max(abs(err), [], 1);
    maxRel = max(abs(err) ./ max(1, abs(cvals)), [], 1);
    % C and Replica both use single-precision arithmetic.  The first
    % voltage-reference residual is below 5e-5 V, so use a conservative
    % 1e-3 V numerical tolerance rather than interpreting float rounding as
    % a control-equation mismatch.
    tol = [0.05, 0.05, 0.002, 2e-5, 1e-3, 0.005, ...
        0.005, 0.005, 0.005, 0.005, 0.005, 0.005, ...
        0.005, 0.005, 0.005, 0.005, 0.005, 0.005, ...
        0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005];
    pass = maxAbs <= tol;
    allCasePass = allCasePass && all(pass);
    names = signal_names();
    for j = 1:numel(names)
        rows{end+1, 1} = table(string(caseNames{ic}), string(names{j}), ...
            maxAbs(j), maxRel(j), tol(j), pass(j), ...
            'VariableNames', {'Case', 'Signal', 'MaxAbsError', ...
            'MaxRelativeError', 'Tolerance', 'PASS'}); %#ok<AGROW>
    end
end

T = vertcat(rows{:});
csvPath = fullfile(tempDir, 'S7_Legacy_LC2_FullMap_Validation.csv');
writetable(T, csvPath);
resultPath = csvPath;

reportPath = fullfile(tempDir, 'S7_Legacy_LC2_FullMap_Report_CN.md');
fid = fopen(reportPath, 'w');
if fid < 0
    error('run_s7_5_lc2_full_map:ReportOpen', '无法写入 LC2 报告。');
end
cleanupFile = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# S7-5B/LC2 Legacy 完整控制器闭环认证报告\n\n');
fprintf(fid, '- C probe：IDEAL_CONTINUOUS_CONTROLLER=1，主步 Ts=4 us；GFM 严格 VSG 开启；PWM 门极未用于比较。\n');
fprintf(fid, '- Legacy 网侧控制器内部 grid_side.Ts=0.1 ms，Replica 保留该独立网侧更新周期。\n');
fprintf(fid, '- 每个工况 100 个连续控制事件；只保存逐项最大误差，不保存长时序。\n\n');
fprintf(fid, '## 工况\n\n');
fprintf(fid, '|工况|说明|\n|---|---|\n');
fprintf(fid, '|nominal|Udc=900 V，PCC 电流基准|\n');
fprintf(fid, '|udc_small|Udc=900.5 V，其他输入不变|\n');
fprintf(fid, '|combined|Udc=900.5 V，PCC 电流增加 2%%|\n\n');
fprintf(fid, '## 验收结果\n\n');
fprintf(fid, '|工况|信号|最大绝对误差|最大相对误差|容差|PASS|\n|---|---|---:|---:|---:|:---:|\n');
for i = 1:height(T)
    fprintf(fid, '|%s|%s|%.12g|%.6g|%.12g|%s|\n', ...
        T.Case(i), T.Signal(i), T.MaxAbsError(i), ...
        T.MaxRelativeError(i), T.Tolerance(i), string(T.PASS(i)));
end
fprintf(fid, '\n- **LC2 总结：%s**\n\n', string(allCasePass));
fprintf(fid, '## 复制范围\n\n');
fprintf(fid, 'MSC-DVC、MSC 电流 PI、机侧解耦；measurement-only PLL；P/Q 双线性滤波；Pref 斜率；VSG 摆动方程与角度更新；GSC Q-V、电压 PI、电流 PI、PCC 电流前馈、Ls 解耦和 dq→alpha-beta。\n');
fprintf(fid, '\n## 边界与后续\n\n');
fprintf(fid, '本结果仅认证控制器事件层，不等同于 Legacy 固定点、离散 SSM 或非线性 plant 验证。LC2 通过后才进入 S7-5C；若失败，应按报告中首个超差环节检查更新顺序、隐藏状态、角度帧和参数基准。\n');
end

function yC = local_run_c_probe(mexName, u0, Ts, N, PRef, QRef, Vg, VdcRef, caseIndex)
mdl = sprintf('S7_5_LC2_FullMap_%d', caseIndex);
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
assignin('base', 'u0', u0);
new_system(mdl);
load_system('simulink');
add_block('simulink/Sources/Constant', [mdl '/Input20'], ...
    'Value', 'u0', 'Position', [30 50 100 90]);
add_block('simulink/User-Defined Functions/S-Function', [mdl '/LegacyC'], ...
    'FunctionName', mexName, ...
    'Parameters', sprintf('%.12g,%.12g,%.12g,%.12g', PRef, QRef, Vg, VdcRef), ...
    'Position', [160 40 300 100]);
add_block('simulink/Sinks/To Workspace', [mdl '/Y'], ...
    'VariableName', 'yC', 'SaveFormat', 'Array', ...
    'Position', [370 48 485 92]);
add_line(mdl, 'Input20/1', 'LegacyC/1');
add_line(mdl, 'LegacyC/1', 'Y/1');
set_param(mdl, 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', num2str(Ts, '%.12g'), ...
    'StopTime', num2str(N * Ts, '%.12g'), ...
    'ReturnWorkspaceOutputs', 'on');
set_param(mdl, 'SimulationCommand', 'update');
simOut = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
try
    yC = simOut.get('yC');
catch
    yC = evalin('base', 'yC');
end
if isempty(yC) || size(yC, 2) < 37
    close_system(mdl, 0);
    error('run_s7_5_lc2_full_map:NoOutput', 'C probe 输出不足 37 通道。');
end
close_system(mdl, 0);
end

function v = c_vector(y)
% MATLAB 列号：13:37 对应输出 [12:36] 的控制诊断。
v = [y(13), y(14), y(15), y(16), y(17), y(18), y(19), y(20), ...
    y(21), y(22), y(23), y(24), y(25), y(26), y(27), y(28), ...
    y(29), y(30), y(31), y(32), y(33), y(34), y(35), y(36), y(37)];
end

function v = replica_vector(o)
v = [o.P_ref_raw, o.P, o.w_ref, o.theta_ref, o.Ud1_ref, o.Uq1_ref, ...
    o.Q, o.voltage_ref, o.U_od_ref, o.pcc_ud, o.Id_ref, o.Id, ...
    o.U_oq_ref, o.pcc_uq, o.Iq_ref, o.grid_phase_angle, o.Pre_syn, ...
    o.Iq, o.motor_Iq_ref, o.DVC_Out, o.motor_Iq, o.motor_Ud1_ref, ...
    o.motor_Uq1_ref, o.motor_voltage_mag, o.motor_modulation_index];
end

function names = signal_names()
names = {'P_ref_raw'; 'P_pcc'; 'w_ref'; 'theta_ref'; 'gsc_Ud1_ref'; ...
    'gsc_Uq1_ref'; 'Q_pcc'; 'voltage_ref'; 'U_od_ref'; 'pcc_u_d'; ...
    'Id_ref'; 'Id'; 'U_oq_ref'; 'pcc_u_q'; 'Iq_ref'; 'grid_phase_angle'; ...
    'Pre_syn'; 'gsc_Iq'; 'motor_Iq_ref'; 'DVC_Out'; 'motor_Iq'; ...
    'motor_Ud1_ref'; 'motor_Uq1_ref'; 'motor_voltage_mag'; ...
    'motor_modulation_index'};
end

function [ua, ub, uc] = phase_voltage(v, theta)
ua = v * cos(theta);
ub = v * cos(theta - 2*pi/3);
uc = v * cos(theta + 2*pi/3);
end
