function resultPath = run_s7_5c_legacy_fixed_point(varargin)
%RUN_S7_5C_LEGACY_FIXED_POINT
% S7-5C：在 Legacy C 控制器的显式输入序列下生成一致的控制器层
% 周期工作点。工作点采用旋转 abc 电压/电流，使 P/Q 在控制坐标中
% 保持恒定；不保存长时序，只保存最终状态和审计摘要。
%
% 重要边界：本脚本验证的是 Legacy 控制器及其 Replica 的固定点/周期
% 工作点，不替代包含 PMSG、两质量轴系、DC-link 和电网物理状态的
% 5 MW 全 plant 平衡点。物理 plant 对齐仍需后续在 S7-5D 前单独闭合。

here = fileparts(mfilename('fullpath'));
tempDir = fullfile(here, 'temp', 'S7_5_LegacyCertification');
if ~isfolder(tempDir)
    mkdir(tempDir);
end
addpath(here, '-begin');
mexDir = tempDir;
mexName = 'main_s7_5_lc2_probe';
mexPath = fullfile(mexDir, [mexName, '.', mexext]);
if ~isfile(mexPath)
    error('run_s7_5c_legacy_fixed_point:MissingMex', ...
        '缺少 LC2 C probe：%s。先运行 run_s7_5_lc2_full_map。', mexPath);
end
addpath(mexDir, '-begin');
clear main_s7_5_lc2_probe;

% 选择一个未触发控制限幅的 Legacy 控制器平衡探针。允许诊断时延长
% 收敛时间；默认值保持原测试规模，避免正常调用生成大数据。
opt.N = 5000;
opt.GridFreq = 314.0;
opt.InputMode = 'replica_locked';
if mod(numel(varargin), 2) ~= 0
    error('run_s7_5c_legacy_fixed_point:Options', '选项必须成对给出。');
end
for ii = 1:2:numel(varargin)
    key = lower(char(varargin{ii}));
    switch key
        case 'n'
            opt.N = varargin{ii+1};
        case 'gridfreq'
            opt.GridFreq = varargin{ii+1};
        case 'inputmode'
            opt.InputMode = lower(char(varargin{ii+1}));
        otherwise
            error('run_s7_5c_legacy_fixed_point:Options', '未知选项：%s', key);
    end
end
TsMain = 4e-6;
TsGrid = 1e-4;
N = opt.N;
Ninput = N + 1;
Vg = 563;
Iamp = 100;
angle0 = 0.05;
PRef = 1.5 * Vg * Iamp;
QRef = 0;
VdcRef = 1000;
We = 100;
RotorPos = 0.01;

t = (0:Ninput-1).' * TsMain;
uSeq = zeros(Ninput, 20);
% 采用单精度递推生成网侧角度，避免 MATLAB 双精度直接 k*dt 与
% Legacy C 的 float 角度累加产生一个随时间增长的假性相位漂移。
theta_grid = single(angle0);
theta_step = single(opt.GridFreq) * single(TsGrid);
locked_state = s7_legacy_replica_full_step('initial_state');
locked_p = s7_legacy_replica_full_step('defaults');
for k = 1:Ninput
    if strcmp(opt.InputMode, 'replica_locked')
        % 固定点探针的外部电压/电流相位跟随 Replica 的旧 VSG 角度。
        % 这不是物理电网闭环，而是控制器事件映射的周期性认证，
        % 用来消除“固定 314 rad/s 输入 vs 单精度 VSG 量化”的伪漂移。
        theta = single(locked_state.vsg.theta);
    elseif strcmp(opt.InputMode, 'fixed_frequency')
        theta = theta_grid;
    else
        error('run_s7_5c_legacy_fixed_point:InputMode', ...
            'InputMode 只能是 replica_locked 或 fixed_frequency。');
    end
    [ua, ub, uc] = phase_voltage(Vg, theta);
    [ia, ib, ic] = phase_current(Iamp, theta);
    uSeq(k, 1:3) = [0, 0, 0];
    uSeq(k, 4:6) = [VdcRef, We, RotorPos];
    uSeq(k, 7:9) = [ia, ib, ic];
    uSeq(k, 10:12) = [ua-ub, ub-uc, uc-ua];
    uSeq(k, 13:15) = [ia, ib, ic];
    uSeq(k, 16:20) = [3.0, 0, 1, 0, VdcRef];
    theta_grid = single(theta_grid + theta_step);
    if strcmp(opt.InputMode, 'replica_locked') && k <= N
        uk_locked = local_make_input(uSeq(k,:), PRef, QRef, VdcRef);
        [locked_state, ~] = s7_legacy_replica_full_step(locked_state, uk_locked, locked_p);
    end
end
u_ts = timeseries(uSeq, t);

% 运行独立 C probe；输入序列只驻留内存。
yC = local_run_probe(mexName, u_ts, TsMain, N, PRef, QRef, Vg, VdcRef);

% 同一输入序列重放显式 Replica，保存最终 hidden state。
st = s7_legacy_replica_full_step('initial_state');
p = s7_legacy_replica_full_step('defaults');
n = min(N, size(yC, 1));
rep = zeros(n, 25);
cvals = zeros(n, 25);
for k = 1:n
    uk = struct('Udc', uSeq(k, 4), 'VdcRef', VdcRef, ...
        'system_Time', uSeq(k, 16), 'Motor_Ia', uSeq(k, 1), ...
        'Motor_Ib', uSeq(k, 2), 'Motor_Ic', uSeq(k, 3), ...
        'Ia1', uSeq(k, 7), 'Ib1', uSeq(k, 8), 'Ic1', uSeq(k, 9), ...
        'We', uSeq(k, 5), ...
        'RotorPos', uSeq(k, 6), 'P_ref', PRef, 'Q_ref', QRef, ...
        'Pre_syn', true, 'GFM_enabled', true, ...
        'pcc_uab', uSeq(k, 10), 'pcc_ubc', uSeq(k, 11), ...
        'pcc_uca', uSeq(k, 12), 'pcc_Ia', uSeq(k, 13), ...
        'pcc_Ib', uSeq(k, 14), 'pcc_Ic', uSeq(k, 15));
    [st, o, tr] = s7_legacy_replica_full_step(st, uk, p); %#ok<ASGLU>
    rep(k, :) = replica_vector(o);
    cvals(k, :) = c_vector(yC(k, :));
end

maxRepC = max(abs(cvals - rep), [], 1);
selected = [2, 3, 5, 6, 7, 11, 12, 14, 15, 19, 20, 21, 22, 23];
tail = max(1, n-199):n;
% 角度是周期状态，不能把每个事件的正常角度推进误判成漂移；
% 对 theta_ref 另行检查其增量与 Ts*w_ref 的一致性。
selected_periodic = [2, 3, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15, ...
    17, 18, 19, 20, 21, 22, 23, 24, 25];
tailDiffC = max(abs(diff(cvals(tail, selected_periodic), 1, 1)), [], 1);
% 各输出单位不同，且 Q、pcc_uq 等接近零的量不能用 1 的统一
% 分母把单精度量化噪声放大。采用明确的物理绝对误差预算：
% [W,rad/s,V,var,V,A]，只用于周期性验收，不改变 LC2 门槛。
periodicTol = [0.1, 1e-4, 0.1, 0.1, 0.1, 0.1, 0.1, ...
    0.1, 0.1, 0.1, 0.1, 0.1, 1e-4, 0.1, 0.1, 0.1, ...
    0.1, 0.1, 0.1, 0.1, 1e-4];
maxNormalizedTailDrift = max(tailDiffC ./ periodicTol);
dtheta_tail = diff(cvals(tail, 4), 1, 1);
% 跨越 2*pi 时先取最小等价角差，再与内部角速度积分量比较。
dtheta_tail = dtheta_tail - round(dtheta_tail/(2*pi))*2*pi;
thetaIncrementError = max(abs(dtheta_tail - ...
    1.0e-4*cvals(tail(1:end-1), 3)));
finalC = cvals(n, :);
finalR = rep(n, :);

% 只在控制台保留首末少量样本，便于判断固定点失败来自初始暂态还是
% 旋转输入与 Legacy 内部角速度未闭合；不写入长时序文件。
fprintf('S7-5C probe first/last [P,w,theta,Q] =\n');
format long g;
disp([cvals(1,[2,3,4,7]); cvals(max(1,n-4):n,[2,3,4,7])]);

% 物理量/限幅代理验收（仍是控制器 probe，不是 plant 平衡）。
pErrorRel = abs(finalC(2) - PRef) / max(PRef, 1);
wError = abs(finalC(3) - 314);
dvcAbs = abs(finalC(20));
finitePass = all(isfinite([cvals(:); rep(:)]));
% C 输出的 GSC/MSC 电流参考及机侧调制度均应留有余量。
currentPeak = max(max(abs(cvals(:, [11, 15, 19, 21]))));
motorModulationPeak = max(abs(cvals(:, 25)));
% The probe is compiled with GSI_VOLTAGE_MODULATION_LIMIT=0, so the
% modulation-index diagnostic is informational and must not be treated as
% an active limit.  Only the enabled current PI output range is audited.
limitMarginPass = currentPeak < 700;
periodicPass = maxNormalizedTailDrift < 1e-4;
% Legacy 使用 float 保存 VSG 频率，313.9847717 rad/s 是最后一个可
% 表示的量化固定值。这里不把它与数学名义值 314.0 的 ULP 残差
% 误判为失衡；真正的验收是末段频率不再跳变。
wQuantizedStep = max(abs(diff(cvals(tail, 3))));
balancePass = pErrorRel < 1e-3 && wQuantizedStep < 1e-7 && ...
    thetaIncrementError < 1e-5 && dvcAbs < 1e-3;
% LC2 已经用严格逐项门槛认证了 100 步映射；长序列仅增加一个
% 明确记录的单精度累计误差预算，不用它反向否定已通过的 LC2。
longRunTol = max(lc2_tolerances(), 0.05*ones(size(maxRepC)));
longRunTol(4) = 2e-5; % theta_ref 仍按角度精度门槛
replicaPass = all(maxRepC <= longRunTol);
overall = finitePass && limitMarginPass && periodicPass && balancePass && replicaPass;

summary = struct();
% `overall` 只表示当前控制器探针是否同时满足严格的周期性门槛。
% 由于本脚本的 PCC 电压/电流是外部固定序列，不能把它解释成
% Legacy + PMSG + 两质量轴系 + DC-link + 电网的物理固定点。
% 因此物理闭环状态单独记录，并在 Gate V3 中保持阻塞。
summary.status = string(overall);
summary.controller_probe_status = "CONDITIONAL_CONTROLLER_PROBE";
summary.plant_fixed_point = false;
summary.physical_closure = false;
summary.gsc_voltage_pi_closure = false;
summary.gate_v3_status = "BLOCKED_NO_PHYSICAL_CLOSURE";
summary.controller_probe = "Legacy C + explicit Replica";
summary.P_ref_W = PRef;
summary.P_final_W = finalC(2);
summary.P_error_relative = pErrorRel;
summary.w_ref_final_radps = finalC(3);
summary.w_ref_error_radps = wError;
summary.DVC_final = dvcAbs;
summary.max_normalized_tail_drift = maxNormalizedTailDrift;
summary.theta_increment_error_rad = thetaIncrementError;
summary.w_quantized_step_radps = wQuantizedStep;
summary.w_quantized_offset_radps = finalC(3) - 314.0;
summary.current_peak_A = currentPeak;
summary.motor_modulation_peak = motorModulationPeak;
summary.replica_max_abs_error = max(maxRepC);
summary.finite = finitePass;
summary.limit_margin = limitMarginPass;
summary.periodic_residual = periodicPass;
summary.energy_balance_proxy = balancePass;
summary.replica_consistency = replicaPass;

% 固定 PCC 输入时，GSC 电压外环命令的末段变化只能作为“缺少
% 物理反馈”的诊断证据，不能当作整机失稳结论。
gsc_voltage_command_span = max(abs(cvals(tail(1), [5 6]) - ...
    cvals(tail(end), [5 6])));
summary.gsc_voltage_command_span_V = gsc_voltage_command_span;

% Keep the channel-level error visible in the console for diagnosis without
% writing another long time series file.
disp(table(signal_names(), maxRepC(:), ...
    'VariableNames', {'Signal', 'MaxAbsError'}));
allNames = signal_names();
disp(table(allNames(selected_periodic(:)), ...
    (tailDiffC ./ periodicTol).', 'VariableNames', {'Signal', 'TailDriftNormalized'}));

names = {'controller_probe'; 'P_ref_W'; 'P_final_W'; 'P_error_relative'; ...
    'w_ref_final_radps'; 'w_ref_error_radps'; 'DVC_final'; ...
    'max_normalized_tail_drift'; 'theta_increment_error_rad'; ...
    'w_quantized_step_radps'; 'w_quantized_offset_radps'; 'current_peak_A'; ...
    'motor_modulation_peak'; ...
    'replica_max_abs_error'; 'finite'; 'limit_margin'; 'periodic_residual'; ...
    'energy_balance_proxy'; 'replica_consistency'; 'controller_probe_status'; ...
    'plant_fixed_point'; 'physical_closure'; 'gsc_voltage_pi_closure'; ...
    'gsc_voltage_command_span_V'; 'gate_v3_status'; 'overall'};
values = {char(summary.controller_probe); PRef; finalC(2); pErrorRel; ...
    finalC(3); wError; dvcAbs; maxNormalizedTailDrift; ...
    thetaIncrementError; wQuantizedStep; finalC(3)-314.0; currentPeak; ...
    motorModulationPeak; max(maxRepC); finitePass; limitMarginPass; ...
    periodicPass; balancePass; replicaPass; ...
    char(summary.controller_probe_status); summary.plant_fixed_point; ...
    summary.physical_closure; summary.gsc_voltage_pi_closure; ...
    gsc_voltage_command_span; char(summary.gate_v3_status); overall};
T = table(names, values, 'VariableNames', {'Metric', 'Value'});
csvPath = fullfile(tempDir, 'S7_Legacy_FixedPoint.csv');
writetable(T, csvPath);
resultPath = csvPath;

% 显式保存必要固定点状态，不保存完整 workspace 或原始时序。
state_eq = st;
u_eq = uSeq(n, :);
fixed_point_summary = summary;
state_names = {'msc.dvc'; 'msc.id'; 'msc.iq'; 'vsg.theta'; 'vsg.w_vsg'; ...
    'vsg.w_sync'; 'gsc.dv'; 'gsc.qv'; 'gsc.di'; 'gsc.qi'; ...
    'gsc.P_filter'; 'gsc.Q_filter'; 'gsc.PrefRampOut'; ...
    'pll.phase'; 'pll.freq'; 'pll.pll'};
matPath = fullfile(tempDir, 'S7_Legacy_FixedPoint.mat');
save(matPath, 'state_eq', 'u_eq', 'fixed_point_summary', 'state_names');

reportPath = fullfile(tempDir, 'S7_Legacy_FixedPoint_Report_CN.md');
fid = fopen(reportPath, 'w');
if fid < 0
    error('run_s7_5c_legacy_fixed_point:ReportOpen', '无法写入报告。');
end
cleanupFile = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# S7-5C Legacy 控制器固定点审计报告\n\n');
fprintf(fid, '- 本轮使用旋转 abc 电压/电流序列保持控制器 P/Q 恒定；主步 4 us，Legacy 网侧内部周期 0.1 ms，共 %d 个控制事件。\n', N);
fprintf(fid, '- 输入相位模式：%s；replica_locked 仅用于控制器周期映射认证，不等同于物理电网闭环。\n', opt.InputMode);
fprintf(fid, '- 探针功率为 %.6g W（用于不触发限幅的控制器认证），不将其称为 5 MW plant 平衡点。\n', PRef);
fprintf(fid, '- 未保存原始时序；仅保存最终 Replica hidden state、最后输入和摘要。\n\n');
fprintf(fid, '- 末段周期性不把 theta_ref 的正常推进计入漂移；频率量化残差单独记录。\n');
fprintf(fid, '## 固定点/周期性验收\n\n');
fprintf(fid, '|指标|结果|\n|---|---:|\n');
for i = 1:height(T)
    fprintf(fid, '|%s|%s|\n', T.Metric{i}, string(T.Value{i}));
end
fprintf(fid, '\n## 判定\n\n');
fprintf(fid, '- Replica 与 C 的 LC2 最大逐项误差：%.12g。\n', max(maxRepC));
fprintf(fid, '- 末 200 个事件的最大归一化输出漂移：%.12g。\n', maxNormalizedTailDrift);
fprintf(fid, '- GSC 电压外环命令末段跨度：%.12g V。该量在固定 PCC 输入探针中只能用于诊断缺少物理反馈。\n', gsc_voltage_command_span);
fprintf(fid, '- **S7-5C 控制器探针结果：CONDITIONAL_CONTROLLER_PROBE**（严格周期门槛=%s）。\n', string(overall));
fprintf(fid, '- **S7-5C 物理 plant 固定点：BLOCKED**；当前探针没有 LCL/电网/DC-link/轴系闭环。\n\n');
fprintf(fid, '## 边界\n\n');
fprintf(fid, 'LC2 已证明 C 与显式 Replica 的逐事件映射一致；但固定 PCC 电压/电流序列没有把 GSC 电压 PI 命令反馈到 LCL 和电网，故不能满足 Phi_Legacy(X0)=X0 的整机固定点定义。必须先建立 Legacy 控制器与连续物理 plant 的闭环，再进入 S7-5D 离散 SSM；Gate V3 和 S7B EMT 保持阻塞。\n');
end

function yC = local_run_probe(mexName, u_ts, Ts, N, PRef, QRef, Vg, VdcRef)
mdl = 'S7_5C_LegacyFixedPoint';
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
assignin('base', 'u_ts', u_ts);
new_system(mdl);
load_system('simulink');
add_block('simulink/Sources/From Workspace', [mdl '/Input20'], ...
    'VariableName', 'u_ts', 'Position', [30 50 120 90]);
set_param([mdl '/Input20'], 'Interpolate', 'off');
try
    set_param([mdl '/Input20'], 'OutputAfterFinalValue', 'Holding final value');
catch
    % Older Simulink releases do not expose this optional parameter.
end
add_block('simulink/User-Defined Functions/S-Function', [mdl '/LegacyC'], ...
    'FunctionName', mexName, ...
    'Parameters', sprintf('%.12g,%.12g,%.12g,%.12g', PRef, QRef, Vg, VdcRef), ...
    'Position', [180 40 320 100]);
add_block('simulink/Sinks/To Workspace', [mdl '/Y'], ...
    'VariableName', 'yC', 'SaveFormat', 'Array', ...
    'Position', [390 48 505 92]);
add_line(mdl, 'Input20/1', 'LegacyC/1');
add_line(mdl, 'LegacyC/1', 'Y/1');
set_param(mdl, 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', num2str(Ts, '%.12g'), ...
    'StopTime', num2str((N-1) * Ts, '%.12g'), ...
    'ReturnWorkspaceOutputs', 'on');
set_param(mdl, 'SimulationCommand', 'update');
simOut = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
try
    yC = simOut.get('yC');
catch
    yC = evalin('base', 'yC');
end
close_system(mdl, 0);
if isempty(yC) || size(yC, 2) < 37
    error('run_s7_5c_legacy_fixed_point:NoOutput', '固定点 C probe 输出不足 37 通道。');
end
end

function v = c_vector(y)
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

function tol = lc2_tolerances()
tol = [0.05, 0.05, 0.002, 2e-5, 1e-3, 0.005, ...
    0.005, 0.005, 0.005, 0.005, 0.005, 0.005, ...
    0.005, 0.005, 0.005, 0.005, 0.005, 0.005, ...
    0.005, 0.005, 0.005, 0.005, 0.005, 0.005, ...
    0.005];
end

function u = local_make_input(row, PRef, QRef, VdcRef)
u = struct('Udc', row(4), 'VdcRef', VdcRef, ...
    'system_Time', row(16), 'Motor_Ia', row(1), 'Motor_Ib', row(2), ...
    'Motor_Ic', row(3), 'Ia1', row(7), 'Ib1', row(8), 'Ic1', row(9), ...
    'We', row(5), 'RotorPos', row(6), 'P_ref', PRef, 'Q_ref', QRef, ...
    'Pre_syn', true, 'GFM_enabled', true, 'pcc_uab', row(10), ...
    'pcc_ubc', row(11), 'pcc_uca', row(12), 'pcc_Ia', row(13), ...
    'pcc_Ib', row(14), 'pcc_Ic', row(15));
end

function [ua, ub, uc] = phase_voltage(v, theta)
ua = v * cos(theta);
ub = v * cos(theta - 2*pi/3);
uc = v * cos(theta + 2*pi/3);
end

function [ia, ib, ic] = phase_current(i, theta)
ia = i * cos(theta);
ib = i * cos(theta - 2*pi/3);
ic = i * cos(theta + 2*pi/3);
end
