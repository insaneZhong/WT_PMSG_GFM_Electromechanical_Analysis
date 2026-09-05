function resultPath = run_s7_5_b1_msc_lc1()
%RUN_S7_5_B1_MSC_LC1  S7-5B/B1 的可审计 Legacy MSC 验证入口。
% 1) 用 100 个确定性向量检查 MATLAB Replica 的多步状态更新。
% 2) 用生产版 main_legacy_ad.mexw64 取得第一次控制事件。
% 3) 对比 C-S-Function 与 Replica 的 DVC、电流参考、电流反馈和电压指令。
% 原模型、C 源和生产版 MEX 不修改；输出仅写入 temp/S7_5_LegacyCertification。

here = fileparts(mfilename('fullpath'));
tempDir = fullfile(here, 'temp', 'S7_5_LegacyCertification');
if ~isfolder(tempDir), mkdir(tempDir); end
mexDir = fullfile(here, 'temp', 'S7_5_LegacyProduction');
mexPath = fullfile(mexDir, 'main_legacy_ad.mexw64');
if ~isfile(mexPath)
    error('run_s7_5_b1_msc_lc1:MissingMex', '找不到生产版 MEX：%s', mexPath);
end

% ---------- A. Replica 100-step deterministic sequence ----------
p = s7_legacy_replica_b1_msc_step('defaults');
state = s7_legacy_replica_b1_msc_step('initial_state');
repFinite = true;
repLimitsInactive = true;
repRows = 100;
repLast = struct();
for k = 1:repRows
    uk = struct();
    uk.Udc = 1000 + 4*sin(2*pi*k/37) + 0.5*cos(2*pi*k/19);
    uk.VdcRef = 1000;
    uk.system_Time = 3 + (k-1)*1e-4;
    uk.Ia = 10 + 0.2*sin(2*pi*k/23);
    uk.Ib = -5 + 0.15*cos(2*pi*k/29);
    uk.Ic = -uk.Ia - uk.Ib;
    uk.We = 100 + 0.5*sin(2*pi*k/31);
    uk.RotorPos = 0.01 + 2e-4*k;
    uk.omega_rel_ad = 0;
    uk.ad_scale = 1;
    uk.iq_ff = 0;
    uk.lvrt_active = false;
    [state, repLast, tr] = s7_legacy_replica_b1_msc_step(state, uk, p);
    nums = [repLast.Id_ref, repLast.Iq_ref, repLast.DVC_Out, repLast.Id, ...
        repLast.Iq, repLast.Ud1_ref, repLast.Uq1_ref, repLast.Us_alfa, repLast.Us_beta];
    repFinite = repFinite && all(isfinite(double(nums))) && isfinite(double(state.dvc.Ui)) ...
        && isfinite(double(state.id.Ui)) && isfinite(double(state.iq.Ui));
    repLimitsInactive = repLimitsInactive && tr.all_limits_inactive;
end

% ---------- B. Production C-S-Function first controller event ----------
addpath(mexDir, '-begin');
clear main_legacy_ad;
u0 = zeros(20,1);
u0(1:3) = [10; -5; -5];       % Ia, Ib, Ic
u0(4) = 900;                  % Udc
u0(5) = 100;                  % We
u0(6) = 0.01;                 % mechanical rotor position [rad]
u0(16) = 3;                   % system_Time [s], DVC enabled
u0(17) = 0;                   % omega_rel_ad
u0(18) = 1;                   % legacy_msc_ad_scale
u0(19) = 0;                   % legacy_msc_iq_ff_a
u0(20) = 1000;                % Vdc reference
assignin('base', 'u0', u0);

mdl = 'S7_5_B1_MSC_LC1_Runtime';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
load_system('simulink');
add_block('simulink/Sources/Constant', [mdl '/Input20'], ...
    'Value', 'u0', 'Position', [30 50 100 90]);
add_block('simulink/User-Defined Functions/S-Function', [mdl '/LegacyC'], ...
    'FunctionName', 'main_legacy_ad', 'Parameters', '5e6,0,563,1000', ...
    'Position', [160 40 300 100]);
add_block('simulink/Sinks/To Workspace', [mdl '/Y'], ...
    'VariableName', 'yC', 'SaveFormat', 'Array', ...
    'Position', [370 48 485 92]);
add_line(mdl, 'Input20/1', 'LegacyC/1');
add_line(mdl, 'LegacyC/1', 'Y/1');
set_param(mdl, 'Solver', 'FixedStepDiscrete', 'FixedStep', '1e-6', ...
    'StopTime', '1.5e-4', 'ReturnWorkspaceOutputs', 'on');
set_param(mdl, 'SimulationCommand', 'update');
simOut = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
try
    yC = simOut.get('yC');
catch
    yC = evalin('base', 'yC');
end
if isempty(yC) || size(yC,2) < 35
    error('run_s7_5_b1_msc_lc1:NoCOutput', '生产版 C-S-Function 未返回预期的 37 通道输出。');
end

% C outputs 0-based [30:34] are MATLAB columns [31:35].
kEvent = find(abs(yC(:,32)) > 1e-7, 1, 'first');
if isempty(kEvent)
    error('run_s7_5_b1_msc_lc1:NoControllerEvent', ...
        '在 150 us 试验中没有检测到 DVC 第一次控制事件。');
end
yCevent = double(yC(kEvent, [31 32 33 34 35]));
uFirst = struct('Udc',900,'VdcRef',1000,'system_Time',3,'Ia',10,'Ib',-5,'Ic',-5, ...
    'We',100,'RotorPos',0.01,'omega_rel_ad',0,'ad_scale',1,'iq_ff',0,'lvrt_active',false);
s0 = s7_legacy_replica_b1_msc_step('initial_state');
[~, oR, trR] = s7_legacy_replica_b1_msc_step(s0, uFirst, p);
yR = double([oR.Iq_ref, oR.DVC_Out, oR.Iq, oR.Ud1_ref, oR.Uq1_ref]);
signalNames = {'Iq_ref'; 'DVC_Out'; 'Iq'; 'Ud1_ref'; 'Uq1_ref'};
tol = [1e-4; 1e-4; 1e-4; 1e-3; 1e-2];
err = yCevent(:) - yR(:);
pass = abs(err) <= tol;

T = table(signalNames, yCevent(:), yR(:), err, abs(err), tol, pass, ...
    'VariableNames', {'Signal','C_FirstEvent','Replica_FirstStep','SignedError', ...
    'AbsoluteError','Tolerance','PASS'});
csvPath = fullfile(tempDir, 'S7_Legacy_LC1_B1_Summary.csv');
writetable(T, csvPath);
resultPath = csvPath;

overallPass = all(pass) && repFinite && repLimitsInactive && trR.all_limits_inactive;
reportPath = fullfile(tempDir, 'S7_Legacy_LC1_B1_Report_CN.md');
fid = fopen(reportPath, 'w');
if fid < 0, error('run_s7_5_b1_msc_lc1:ReportOpen', '无法写入 %s', reportPath); end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# S7-5B/B1 Legacy MSC Replica—LC1 单步与多步报告\n\n');
fprintf(fid, '- 运行时间：%s\n', datestr(now, 31));
fprintf(fid, '- C 实现：生产版 `main_legacy_ad.mexw64`（未使用理想化宏）\n');
fprintf(fid, '- 测试输入：`Udc=900 V, VdcRef=1000 V, We=100 rad/s, RotorPos=0.01 rad, t=3 s`\n');
fprintf(fid, '- C 第一个控制事件索引：%d（固定步长 1 us）\n\n', kEvent);
fprintf(fid, '## 复现范围\n\n');
fprintf(fid, '本轮复制 `motorcontrol_legacy_ad_base.c` 的一次 `motor_control`：DVC PI、`Iq_ref` 构造、abc/dq、电流 PI、解耦前馈以及 dq/alpha-beta 输出。\n\n');
fprintf(fid, '## 结果\n\n');
fprintf(fid, '|信号|C 首事件|Replica 首步|绝对误差|容差|PASS|\n|---|---:|---:|---:|---:|:---:|\n');
for i = 1:height(T)
    fprintf(fid, '|%s|%.12g|%.12g|%.4g|%.4g|%s|\n', T.Signal{i}, ...
        T.C_FirstEvent(i), T.Replica_FirstStep(i), T.AbsoluteError(i), ...
        T.Tolerance(i), string(T.PASS(i)));
end
fprintf(fid, '\n- Replica 100 步确定性序列有限性：`%s`\n', string(repFinite));
fprintf(fid, '- Replica 100 步限幅未触发：`%s`\n', string(repLimitsInactive));
fprintf(fid, '- C/Replica 首步所有信号限幅未触发：`%s`\n', string(trR.all_limits_inactive));
fprintf(fid, '- **B1 LC1 总结：`%s`**\n\n', string(overallPass));
fprintf(fid, '## 边界\n\n');
fprintf(fid, '本报告证明的是 MSC 模块方程的单步 C↔Replica 一致性和 Replica 的 100 步确定性更新。生产 C 调度器的连续多事件状态闭环、GSC、P/Q 滤波、PLL/VSG 尚未在本轮完成；因此不能将本报告写成 LC2 或 Gate V3 通过。\n');
close_system(mdl, 0);
end
