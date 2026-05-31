function diag = run_no_disturbance_diagnosis(useSchemeA, simStop, windStep, ctrlOverrides)
% No-disturbance diagnosis for GFM-MWT nonlinear model.
% Goal:
% 1) keep wind constant (no step / no fault / no damping injection check)
% 2) verify mechanical-side settling first
% 3) verify whether GFM delivers ~1 MW to PCC
% 4) localize dominant non-steady cause among DC-link / power loop / breaker sequence

if nargin < 1
    useSchemeA = false;
end
if nargin < 2
    simStop = 5.0;
end
if nargin < 3
    windStep = 0.0;
end
if nargin < 4
    ctrlOverrides = struct();
end

root = fileparts(mfilename('fullpath'));
old = pwd;
cleanup = onCleanup(@() cd(old));
cd(root);

clearvars -except root old cleanup useSchemeA simStop windStep ctrlOverrides;
run('GFM_MWT_Nonlinear_Params.m');

% Force pure no-disturbance condition for this diagnosis.
wind_step_mps_override = windStep;
sim_stop_time_override = simStop;
assignin('base', 'wind_step_mps_override', wind_step_mps_override);
assignin('base', 'sim_stop_time_override', sim_stop_time_override);

mdl = 'Grid_Forming_PMSG';
load_system(mdl);

brkBlk = [mdl '/Three-Phase Breaker'];
origBrkExternal = get_param(brkBlk, 'External');
origBrkInit = get_param(brkBlk, 'InitialState');
forceBreakerClosed = false;
if isfield(ctrlOverrides, 'ForceBreakerClosed')
    forceBreakerClosed = logical(ctrlOverrides.ForceBreakerClosed);
end
if forceBreakerClosed
    % Optional: eliminate breaker timing uncertainty in diagnosis.
    set_param(brkBlk, 'External', 'off');
    set_param(brkBlk, 'InitialState', 'closed');
end

% Optional control-parameter overrides for S-Function mask.
ctrlBlk = [mdl '/MOTOR_CONTROL1'];
ctrlSfun = [mdl '/MOTOR_CONTROL1/S-Function1'];
origPref = get_param(ctrlBlk, 'Pref');
origQref = get_param(ctrlBlk, 'Qref');
origVacRef = get_param(ctrlBlk, 'ReferenceACVoltage');
origVdcRef = get_param(ctrlBlk, 'ReferenceDCVoltage');
origSfunParams = get_param(ctrlSfun, 'Parameters');

prefSet = str2double(origPref);
qrefSet = str2double(origQref);
vacSet = str2double(origVacRef);
vdcSet = str2double(origVdcRef);
if ~isempty(fieldnames(ctrlOverrides))
    if isfield(ctrlOverrides, 'Pref_W'), prefSet = ctrlOverrides.Pref_W; end
    if isfield(ctrlOverrides, 'Qref_var'), qrefSet = ctrlOverrides.Qref_var; end
    if isfield(ctrlOverrides, 'VacRef_V'), vacSet = ctrlOverrides.VacRef_V; end
    if isfield(ctrlOverrides, 'VdcRef_V'), vdcSet = ctrlOverrides.VdcRef_V; end

    set_param(ctrlBlk, 'Pref', num2str(prefSet));
    set_param(ctrlBlk, 'Qref', num2str(qrefSet));
    set_param(ctrlBlk, 'ReferenceACVoltage', num2str(vacSet));
    set_param(ctrlBlk, 'ReferenceDCVoltage', num2str(vdcSet));
end

% Some legacy models resolve S-function parameters from base/model workspace
% instead of mask scope. Enable deterministic numeric override when requested.
useNumericSfunParams = false;
if ~isempty(fieldnames(ctrlOverrides))
    if isfield(ctrlOverrides, 'UseNumericSfunParams')
        useNumericSfunParams = logical(ctrlOverrides.UseNumericSfunParams);
    elseif isfield(ctrlOverrides, 'Pref_W') || isfield(ctrlOverrides, 'Qref_var') || ...
            isfield(ctrlOverrides, 'VacRef_V') || isfield(ctrlOverrides, 'VdcRef_V')
        useNumericSfunParams = true;
    end
end
if useNumericSfunParams
    set_param(ctrlSfun, 'Parameters', sprintf('%.12g, %.12g, %.12g, %.12g', prefSet, qrefSet, vacSet, vdcSet));
end

% Optional: force Scheme-A tuned C parameters via compile-time header injection.
% This avoids modifying legacy GBK-encoded C sources and remains fully reversible.
initFcnBak = get_param(mdl, 'InitFcn');
if useSchemeA
    schemeInit = [ ...
        "mdlFile = get_param(bdroot,'FileName');", ...
        "mdlDir = fileparts(mdlFile);", ...
        "oldDir = pwd; cleanupObj = onCleanup(@() cd(oldDir));", ...
        "cd(mdlDir);", ...
        "setenv('MW_MINGW64_LOC','C:\mingw64');", ...
        "mex -setup C++;", ...
        "clc;", ...
        "mex main.c svpwm.c motorcontrol.c grid_forming_control.c schemeA_runtime_patch.c;", ...
        "display 'MEX compile done (SchemeA runtime patch).';", ...
        "dtime=4e-6; Ts_step=1e-6;", ...
        "run(fullfile(mdlDir,'GFM_MWT_Nonlinear_Params.m'));" ...
        ];
    set_param(mdl, 'InitFcn', strjoin(schemeInit, newline));
end

set_param(mdl, 'StopTime', num2str(sim_stop_time_override));
set_param(mdl, 'SimulationCommand', 'update');
if useNumericSfunParams
    % Re-apply after update in case symbolic expression is restored.
    set_param(ctrlSfun, 'Parameters', sprintf('%.12g, %.12g, %.12g, %.12g', prefSet, qrefSet, vacSet, vdcSet));
end
sim(mdl);
set_param(mdl, 'InitFcn', initFcnBak);
if forceBreakerClosed
    set_param(brkBlk, 'External', origBrkExternal);
    set_param(brkBlk, 'InitialState', origBrkInit);
end
if ~isempty(fieldnames(ctrlOverrides))
    set_param(ctrlBlk, 'Pref', origPref);
    set_param(ctrlBlk, 'Qref', origQref);
    set_param(ctrlBlk, 'ReferenceACVoltage', origVacRef);
    set_param(ctrlBlk, 'ReferenceDCVoltage', origVdcRef);
    if useNumericSfunParams
        set_param(ctrlSfun, 'Parameters', origSfunParams);
    end
end

% Pull timeseries from base workspace (written by ToWorkspace blocks).
omega_g = evalin('base','omega_g');
omega_t = evalin('base','omega_t');
theta_tw = evalin('base','theta_tw');
T_sh = evalin('base','T_sh');
pref_out = evalin('base','pref_out');
pmeas_out = evalin('base','pmeas_out');
wref_out = evalin('base','wref_out');
presyn_out = evalin('base','presyn_out');
udc_meas = evalin('base','udc_meas');
pcc_ia = evalin('base','pcc_ia');
pcc_ib = evalin('base','pcc_ib');
pcc_ic = evalin('base','pcc_ic');
pcc_uab = evalin('base','pcc_uab');
pcc_ubc = evalin('base','pcc_ubc');
pcc_uca = evalin('base','pcc_uca');

tail = 1.0; % last 1 s as steady-window check

% Mechanical-side trend
diag.omega_g_end_slope = localSlope(omega_g, tail);
diag.omega_t_end_slope = localSlope(omega_t, tail);
diag.T_sh_end_slope = localSlope(T_sh, tail);
diag.theta_tw_end_slope = localSlope(theta_tw, tail);

% Control-side trend
diag.pref_end_mean = localMean(pref_out, tail);
diag.pmeas_end_mean = localMean(pmeas_out, tail);
diag.wref_end_mean = localMean(wref_out, tail);
diag.presyn_end_mean = localMean(presyn_out, tail);
diag.udc_end_mean = localMean(udc_meas, tail);
diag.udc_end_slope = localSlope(udc_meas, tail);
diag.pmeas_end_slope = localSlope(pmeas_out, tail);

% PCC three-phase active power reconstruction from line-line voltages.
[tp, ppcc] = localPccPowerFromLineLine(pcc_uab, pcc_ubc, pcc_ia, pcc_ib, pcc_ic);
diag.Ppcc_end_mean = localMeanRaw(tp, ppcc, tail);
diag.Ppcc_end_slope = localSlopeRaw(tp, ppcc, tail);
diag.Ppcc_target_error = diag.Ppcc_end_mean - P_wt_rated;
diag.Ppcc_target_error_pu = diag.Ppcc_target_error / P_wt_rated;

% Phase-1 gate check: mechanical first
diag.mech_settled_flag = ...
    abs(diag.omega_g_end_slope) < 1e-2 && ...
    abs(diag.omega_t_end_slope) < 1e-2 && ...
    abs(diag.theta_tw_end_slope) < 5e-4;

% Phase-2 gate check: delivery and DC behavior
diag.power_reached_1MW_flag = abs(diag.Ppcc_target_error_pu) < 0.05;
diag.dc_settled_flag = abs(diag.udc_end_slope) < 5;

% Root-cause hints (rule-based, first-pass)
cause = "undetermined";
if diag.presyn_end_mean < 0.9
    cause = "breaker_sync_not_completed_or_sequence_issue";
elseif ~diag.dc_settled_flag
    cause = "dc_link_or_active_power_balance_not_settled";
elseif abs(diag.pmeas_end_slope) > 5e3
    cause = "power_loop_not_settled";
elseif ~diag.power_reached_1MW_flag
    cause = "grid_side_delivery_insufficient_check_voltage_current_loops";
elseif ~diag.mech_settled_flag
    cause = "mechanical_side_not_settled_first";
end
diag.primary_cause_hint = cause;

outDir = fullfile(root, 'Validation_Results');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
save(fullfile(outDir, 'no_disturbance_diagnosis.mat'), 'diag');

% Short text report
rep = fullfile(outDir, 'no_disturbance_diagnosis.md');
fid = fopen(rep, 'w');
fprintf(fid, '# 无扰动稳态诊断结果\n\n');
fprintf(fid, '- 仿真时长: %.2f s\n', sim_stop_time_override);
fprintf(fid, '- 风速扰动: %.2f m/s (固定)\n\n', wind_step_mps_override);
fprintf(fid, '## 机械侧\n');
fprintf(fid, '- omega_g 末端斜率: %.6g rad/s^2\n', diag.omega_g_end_slope);
fprintf(fid, '- omega_t 末端斜率: %.6g rad/s^2\n', diag.omega_t_end_slope);
fprintf(fid, '- theta_tw 末端斜率: %.6g rad/s\n', diag.theta_tw_end_slope);
fprintf(fid, '- T_sh 末端斜率: %.6g N*m/s\n', diag.T_sh_end_slope);
fprintf(fid, '- 机械先稳判定: %d\n\n', diag.mech_settled_flag);
fprintf(fid, '## 送功与控制\n');
fprintf(fid, '- P_ref 末端均值: %.6g W\n', diag.pref_end_mean);
fprintf(fid, '- P_meas 末端均值: %.6g W\n', diag.pmeas_end_mean);
fprintf(fid, '- P_pcc 末端均值: %.6g W\n', diag.Ppcc_end_mean);
fprintf(fid, '- P_pcc 相对1MW误差: %.6g pu\n', diag.Ppcc_target_error_pu);
fprintf(fid, '- Udc 末端均值: %.6g V\n', diag.udc_end_mean);
fprintf(fid, '- Udc 末端斜率: %.6g V/s\n', diag.udc_end_slope);
fprintf(fid, '- PreSyn 末端均值: %.6g\n', diag.presyn_end_mean);
fprintf(fid, '- 1MW送功判定: %d\n', diag.power_reached_1MW_flag);
fprintf(fid, '- DC稳态判定: %d\n\n', diag.dc_settled_flag);
fprintf(fid, '## 主要原因提示\n');
fprintf(fid, '- %s\n', diag.primary_cause_hint);
fclose(fid);

disp(diag);
fprintf('Saved: %s\n', rep);
end

function s = localSlope(ts, tailWin)
t = ts.Time(:); y = ts.Data(:);
idx = t >= (t(end) - tailWin);
p = polyfit(t(idx), y(idx), 1);
s = p(1);
end

function m = localMean(ts, tailWin)
t = ts.Time(:); y = ts.Data(:);
idx = t >= (t(end) - tailWin);
m = mean(y(idx));
end

function [t, p] = localPccPowerFromLineLine(uab, ubc, ia, ib, ic)
% Reconstruct phase voltages from line-line (assuming ua+ub+uc=0):
% ua=(2*uab+ubc)/3; ub=(-uab+ubc)/3; uc=(-uab-2*ubc)/3
t = uab.Time(:);
uabv = uab.Data(:);
ubcv = ubc.Data(:);
iav = ia.Data(:);
ibv = ib.Data(:);
icv = ic.Data(:);
ua = (2*uabv + ubcv) / 3;
ub = (-uabv + ubcv) / 3;
uc = (-uabv - 2*ubcv) / 3;
p = ua .* iav + ub .* ibv + uc .* icv;
end

function m = localMeanRaw(t, y, tailWin)
idx = t >= (t(end) - tailWin);
m = mean(y(idx));
end

function s = localSlopeRaw(t, y, tailWin)
idx = t >= (t(end) - tailWin);
p = polyfit(t(idx), y(idx), 1);
s = p(1);
end
