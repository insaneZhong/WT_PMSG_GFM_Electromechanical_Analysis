function [state, out, trace] = s7_legacy_replica_full_step(state, u, p)
%S7_LEGACY_REPLICA_FULL_STEP
% S7-5B/LC2：Legacy MSC、VSG 和 GSC 一次控制事件的串联复制器。
%
% 更新顺序与 main_legacy_ad_base.c/grid_forming_control.c 一致：
%   1) MSC motor_control；
%   2) GSC 事件中的 PCC 测量、P/Q 滤波、VSG 摆动和电压/电流环。
% PWM、门极重构和调度计数不在本函数内；调度器由 B6 独立复制器验证。
% theta_old 专门保留，用于复现 C 代码“先用旧角度测量，再更新 VSG 角度”的顺序。

if nargin == 1 && (ischar(state) || (isstring(state) && isscalar(state)))
    mode = lower(char(state));
    switch mode
        case 'defaults'
            state = local_defaults();
        case 'initial_state'
            state = local_initial_state();
        otherwise
            error('s7_legacy_replica_full_step:UnknownMode', '未知模式 %s。', mode);
    end
    out = [];
    trace = [];
    return;
end
if nargin < 2 || isempty(u)
    error('s7_legacy_replica_full_step:MissingInput', '必须提供 state、u 和 p。');
end
if nargin < 3 || isempty(p)
    p = local_defaults();
end
if isempty(state) || ~isstruct(state)
    state = local_initial_state();
end
state = local_complete_state(state);

% C main 在每个控制事件先执行机侧 motor.control。
um = struct('Udc', local_get(u, 'Udc', 0), ...
    'VdcRef', local_get(u, 'VdcRef', 0), ...
    'system_Time', local_get(u, 'system_Time', 0), ...
    'Ia', local_get(u, 'Motor_Ia', local_get(u, 'Ia1', 0)), ...
    'Ib', local_get(u, 'Motor_Ib', local_get(u, 'Ib1', 0)), ...
    'Ic', local_get(u, 'Motor_Ic', local_get(u, 'Ic1', 0)), ...
    'We', local_get(u, 'We', 0), ...
    'RotorPos', local_get(u, 'RotorPos', 0), ...
    'omega_rel_ad', local_get(u, 'omega_rel_ad', 0), ...
    'iq_ff', local_get(u, 'iq_ff', 0), ...
    'lvrt_active', local_get(u, 'lvrt_active', false));
[state.msc, om, tm] = s7_legacy_replica_b1_msc_step(state.msc, um, p.msc);

% C 在 grid_side_control 内先用旧 VSG 角度完成测量和 P/Q，
% 再根据功率误差更新 w_vsg 和 theta_ref，最后用新 theta 输出电压。
theta_old = state.vsg.theta;
% measurement-only PLL is updated before the P/Q calculation in the C code.
% It does not replace the VSG angle, but its diagnostic grid_phase_angle is
% part of the complete controller state and is therefore copied here.
upll = struct('Ts_grid', p.grid.Ts, ...
    'system_Time', local_get(u, 'system_Time', 0), ...
    'Pre_syn', local_get(u, 'Pre_syn', true), ...
    'GFM_enabled', local_get(u, 'GFM_enabled', true), ...
    'pcc_uab', local_get(u, 'pcc_uab', 0), ...
    'pcc_ubc', local_get(u, 'pcc_ubc', 0), ...
    'pcc_uca', local_get(u, 'pcc_uca', 0));
[state.pll, opl, tpl] = s7_legacy_replica_b4_pll_step(state.pll, upll, p.pll);
% In the GFM branch the C diagnostic p->val.grid_phase_angle is not
% overwritten by the measurement-only PLL; preserve its stale state.
if ~logical(local_get(u, 'Pre_syn', true)) || ...
        ~logical(local_get(u, 'GFM_enabled', true))
    state.grid_phase_diag = state.pll.phase;
end
uv = struct('Ts_grid', p.grid.Ts, ...
    'P_ref', local_get(u, 'P_ref', 0), ...
    'pcc_uab', local_get(u, 'pcc_uab', 0), ...
    'pcc_ubc', local_get(u, 'pcc_ubc', 0), ...
    'pcc_uca', local_get(u, 'pcc_uca', 0), ...
    'pcc_Ia', local_get(u, 'pcc_Ia', 0), ...
    'pcc_Ib', local_get(u, 'pcc_Ib', 0), ...
    'pcc_Ic', local_get(u, 'pcc_Ic', 0));
[state.vsg, ov, tv] = s7_legacy_replica_b5_vsg_step(state.vsg, uv, p.vsg);

ug = struct('Ts', p.grid.Ts, ...
    'theta_fixed', state.vsg.theta, ...
    'theta_voltage', theta_old, ...
    'theta_current', theta_old, ...
    'w_ref', state.vsg.w_vsg, ...
    'Udc', local_get(u, 'Udc', 0), ...
    'P_ref', local_get(u, 'P_ref', 0), ...
    'Q_ref', local_get(u, 'Q_ref', 0), ...
    'Pre_syn', local_get(u, 'Pre_syn', true), ...
    'GFM_enabled', local_get(u, 'GFM_enabled', true), ...
    'pcc_uab', local_get(u, 'pcc_uab', 0), ...
    'pcc_ubc', local_get(u, 'pcc_ubc', 0), ...
    'pcc_uca', local_get(u, 'pcc_uca', 0), ...
    'Ia1', local_get(u, 'Ia1', 0), ...
    'Ib1', local_get(u, 'Ib1', 0), ...
    'Ic1', local_get(u, 'Ic1', 0), ...
    'pcc_Ia', local_get(u, 'pcc_Ia', 0), ...
    'pcc_Ib', local_get(u, 'pcc_Ib', 0), ...
    'pcc_Ic', local_get(u, 'pcc_Ic', 0));
[state.gsc, og, tg] = s7_legacy_replica_b2_gsc_step(state.gsc, ug, p.gsc);

out = struct();
out.P_ref_ramped = ov.P_ref_ramped;
out.P_ref_raw = local_get(u, 'P_ref', 0);
out.P = ov.P;
out.Q = ov.Q;
out.w_ref = ov.w_ref;
out.theta_ref = ov.theta_ref;
out.Ud1_ref = og.Ud1_ref;
out.Uq1_ref = og.Uq1_ref;
out.voltage_ref = og.voltage_ref;
out.U_od_ref = og.U_od_ref;
out.pcc_ud = og.pcc_ud;
out.Id_ref = og.Id_ref;
out.Id = og.Id;
out.U_oq_ref = og.U_oq_ref;
out.pcc_uq = og.pcc_uq;
out.Iq_ref = og.Iq_ref;
out.grid_phase_angle = state.grid_phase_diag;
out.grid_freq = opl.freq;
out.Pre_syn = double(local_get(u, 'Pre_syn', true));
out.Iq = og.Iq;
out.motor_Iq_ref = om.Iq_ref;
out.DVC_Out = om.DVC_Out;
out.motor_Iq = om.Iq;
out.motor_Ud1_ref = om.Ud1_ref;
out.motor_Uq1_ref = om.Uq1_ref;
out.motor_voltage_mag = hypot(double(om.Ud1_ref), double(om.Uq1_ref));
out.motor_modulation_index = 1.5 * out.motor_voltage_mag / max(double(local_get(u, 'Udc', 0)), 1);
out.motor_Us_alfa = om.Us_alfa;
out.motor_Us_beta = om.Us_beta;
out.grid_Us_alfa = og.Us_alfa;
out.grid_Us_beta = og.Us_beta;

trace = struct();
trace.msc = tm;
trace.vsg = tv;
trace.gsc = tg;
trace.pll = tpl;
trace.theta_old = theta_old;
trace.theta_new = state.vsg.theta;
trace.all_limits_inactive = tm.all_limits_inactive && tg.all_limits_inactive;
trace.finite = all(isfinite(struct_values(out)));
end

function p = local_defaults()
p = struct();
p.msc = s7_legacy_replica_b1_msc_step('defaults');
p.vsg = s7_legacy_replica_b5_vsg_step('defaults');
p.gsc = s7_legacy_replica_b2_gsc_step('defaults');
p.pll = s7_legacy_replica_b4_pll_step('defaults');
p.grid = struct('Ts', single(1e-4));
end

function st = local_initial_state()
st = struct();
st.msc = s7_legacy_replica_b1_msc_step('initial_state');
st.vsg = s7_legacy_replica_b5_vsg_step('initial_state');
st.gsc = s7_legacy_replica_b2_gsc_step('initial_state');
st.pll = s7_legacy_replica_b4_pll_step('initial_state');
st.grid_phase_diag = single(0);
end

function st = local_complete_state(st)
z = local_initial_state();
if ~isfield(st, 'msc'), st.msc = z.msc; end
if ~isfield(st, 'vsg'), st.vsg = z.vsg; end
if ~isfield(st, 'gsc'), st.gsc = z.gsc; end
if ~isfield(st, 'pll'), st.pll = z.pll; end
if ~isfield(st, 'grid_phase_diag'), st.grid_phase_diag = z.grid_phase_diag; end
end

function v = local_get(s, name, default_value)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default_value;
end
end

function values = struct_values(s)
f = fieldnames(s);
values = zeros(numel(f), 1);
for k = 1:numel(f)
    values(k) = double(s.(f{k}));
end
end
