function [state, out, trace] = s7_legacy_replica_b1_msc_step(state, u, p)
%S7_LEGACY_REPLICA_B1_MSC_STEP
%  S7-5B/B1 的 Legacy MSC 单个控制事件复制器。
%
%  该函数按 motorcontrol_legacy_ad_base.c 的一次 motor_control 调用
%  重现：Udc-DVC PI -> iq_ref -> abc/dq -> id/iq PI -> 电压前馈 -> dq/ab。
%  它只表示一个控制器事件，不包含 PWM、采样调度或 Simulink plant。
%  过程验证脚本应把本函数与生产版 C-S-Function 的输出逐项比较。
%
%  重要约定
%  * C 代码中的 PI 更新顺序是 Ui 先更新，再计算 Out。
%  * C 代码使用 float；本实现对中间量显式使用 single，便于复现舍入。
%  * 与 C 源保持一致：Iq_ref=-iq_ff-DVC_Out+iq_ad。
%  * 默认 iq_ad=0、LVRT=false；限幅逻辑仍实现，但验证时需记录是否触发。

% 单参数字符串调用用于获取默认参数或零状态，避免额外配置文件。
if nargin == 1 && (ischar(state) || (isstring(state) && isscalar(state)))
    mode = lower(char(state));
    switch mode
        case 'defaults'
            state = local_defaults();
        case 'initial_state'
            state = local_initial_state();
        otherwise
            error('s7_legacy_replica_b1_msc_step:UnknownMode', '未知模式 %s。', mode);
    end
    out = [];
    trace = [];
    return;
end
if nargin < 3 || isempty(p)
    p = local_defaults();
end
if nargin < 2 || isempty(u)
    error('s7_legacy_replica_b1_msc_step:MissingInput', '必须提供状态 state、输入 u 和参数 p。');
end

% 采用 C float 等价的状态字段；兼容空状态调用。
if isempty(state) || ~isstruct(state)
    state = s7_legacy_replica_b1_msc_step('initial_state');
end
state = local_complete_state(state);

% C motor_control 的工作输入。
udc = single(local_get(u, 'Udc', 0));
vdc_ref = single(local_get(u, 'VdcRef', 0));
time_s = single(local_get(u, 'system_Time', 0));
ia = single(local_get(u, 'Ia', 0));
ib = single(local_get(u, 'Ib', 0));
ic = single(local_get(u, 'Ic', 0)); %#ok<NASGU> % C 代码只使用 Ia/Ib，保留用于审计
we = single(local_get(u, 'We', 0));
rotor_pos_mech = single(local_get(u, 'RotorPos', 0));
omega_rel_ad = single(local_get(u, 'omega_rel_ad', 0));
ad_scale = single(local_get(u, 'ad_scale', 1));
iq_ff = single(local_get(u, 'iq_ff', 0));
lvrt_active = logical(local_get(u, 'lvrt_active', false));

% 1) Udc DVC PI（C 中 pwm_speed_pi）。
state.dvc.Ref = vdc_ref;
state.dvc.Fdb = udc;
if time_s < single(p.DVC_EnableTime_s)
    state.dvc = local_reset_pi(state.dvc, p.DVC_OutMax, p.DVC_OutMin);
else
    state.dvc = local_pi2_step(state.dvc, p.DVC, p.DVC_OutMax, p.DVC_OutMin);
end

% 2) 参考值构造及可旁路的主动阻尼/LVRT支路。
id_ref = single(0);
iq_ad = ad_scale * single(p.AD_IqGain) * omega_rel_ad;
iq_ad_pre_limit = iq_ad;
iq_ad = min(max(iq_ad, single(-p.AD_IqLimit)), single(p.AD_IqLimit));
iq_ref_pre_lvrt = -iq_ff - state.dvc.Out + iq_ad;
iq_ref = iq_ref_pre_lvrt;
if lvrt_active && p.LVRT_IqLimit_A > 0
    iq_ref = min(max(iq_ref, single(-p.LVRT_IqLimit_A)), single(p.LVRT_IqLimit_A));
end

% 3) abc -> alpha beta -> dq，与 motorcontrol_legacy_ad_base.c 完全同向。
theta_e = rotor_pos_mech * single(p.Polar);
if theta_e > single(p.TwoPi)
    theta_e = theta_e - single(p.TwoPi);
end
if theta_e < 0
    theta_e = theta_e + single(p.TwoPi);
end
c = single(cos(theta_e));
s = single(sin(theta_e));
ialpha = ia;
ibeta = single(p.InvSqrt3) * (ia + single(2) * ib);
id = ialpha * c + ibeta * s;
iq = ibeta * c - ialpha * s;

% 4) 机侧 d/q 电流 PI。
state.id.Ref = id_ref;
state.id.Fdb = id;
if lvrt_active && p.LVRT_FreezeCurrentPI
    ui_hold = state.id.Ui;
    state.id = local_pi2_step(state.id, p.IdPI, p.Id_OutMax, p.Id_OutMin);
    state.id.Ui = ui_hold;
else
    state.id = local_pi2_step(state.id, p.IdPI, p.Id_OutMax, p.Id_OutMin);
end
state.iq.Ref = iq_ref;
state.iq.Fdb = iq;
if lvrt_active && p.LVRT_FreezeCurrentPI
    ui_hold = state.iq.Ui;
    state.iq = local_pi2_step(state.iq, p.IqPI, p.Iq_OutMax, p.Iq_OutMin);
    state.iq.Ui = ui_hold;
else
    state.iq = local_pi2_step(state.iq, p.IqPI, p.Iq_OutMax, p.Iq_OutMin);
end

% 5) dq 电压前馈；C 源没有把电机电压限幅放在默认小扰动路径。
ud_fwd = single(p.Rs) * state.id.Ref - single(p.Polar) * we * single(p.Lq) * state.iq.Ref;
uq_fwd = single(p.Rs) * state.iq.Ref + single(p.Polar) * we * ...
    (single(p.Ld) * state.id.Ref + single(p.Fm));
ud_unsat = state.id.Out + ud_fwd;
uq_unsat = state.iq.Out + uq_fwd;
ud = ud_unsat;
uq = uq_unsat;
voltage_limited = false;
if lvrt_active && p.LVRT_VoltageModulationLimit > 0 && udc > 1
    vm = single(sqrt(double(ud_unsat * ud_unsat + uq_unsat * uq_unsat)));
    vlim = single(p.LVRT_VoltageModulationLimit) * udc / single(1.5);
    if vm > vlim && vm > single(1e-6)
        voltage_limited = true;
        ud = ud_unsat * vlim / vm;
        uq = uq_unsat * vlim / vm;
        state.id.Ui = state.id.Ui + single(p.LVRT_VectorAWGain) * (ud - ud_unsat);
        state.iq.Ui = state.iq.Ui + single(p.LVRT_VectorAWGain) * (uq - uq_unsat);
    end
end

% 6) dq -> alpha beta。
usalfa = ud * c - uq * s;
usbeta = ud * s + uq * c;

out = struct();
out.Id_ref = id_ref;
out.Iq_ref = iq_ref;
out.Iq_ref_pre_lvrt = iq_ref_pre_lvrt;
out.Iq_ad = iq_ad;
out.DVC_Out = state.dvc.Out;
out.Id = id;
out.Iq = iq;
out.RotorPos_e = theta_e;
out.Ud_fwd = ud_fwd;
out.Uq_fwd = uq_fwd;
out.Ud1_ref = ud;
out.Uq1_ref = uq;
out.Us_alfa = usalfa;
out.Us_beta = usbeta;

trace = struct();
trace.dvc_sat = abs(double(state.dvc.SatErr)) > 1e-12;
trace.id_sat = abs(double(state.id.SatErr)) > 1e-12;
trace.iq_sat = abs(double(state.iq.SatErr)) > 1e-12;
trace.iq_ad_limited = abs(double(iq_ad - iq_ad_pre_limit)) > 1e-12;
trace.iq_ref_limited = abs(double(iq_ref - iq_ref_pre_lvrt)) > 1e-12;
trace.voltage_limited = voltage_limited;
trace.lvrt_active = lvrt_active;
trace.all_limits_inactive = ~(trace.dvc_sat || trace.id_sat || trace.iq_sat || ...
    trace.iq_ad_limited || trace.iq_ref_limited || trace.voltage_limited);
end

function p = local_defaults()
p = struct();
p.Rs = 0.0122; p.Ld = 0.00102; p.Lq = 0.00102; p.Fm = 8.64; p.Polar = 20;
p.TwoPi = 6.2831853; p.InvSqrt3 = 0.57735027;
p.DVC_EnableTime_s = 2.75; p.AD_IqGain = 0; p.AD_IqLimit = 50;
p.LVRT_IqLimit_A = 0; p.LVRT_VoltageModulationLimit = 0;
p.LVRT_VectorAWGain = 0; p.LVRT_FreezeCurrentPI = false;
p.DVC.Kp = 0.05; p.DVC.Ki = 0.0005; p.DVC.Kc = 0.00001; p.DVC.Kd = 0.000001;
p.DVC_OutMax = 1500; p.DVC_OutMin = -1500;
p.IdPI.Kp = 1.4; p.IdPI.Ki = 0.00290476; p.IdPI.Kc = 0.0001; p.IdPI.Kd = 0.000001;
p.IqPI = p.IdPI; p.Id_OutMax = 700; p.Id_OutMin = -700; p.Iq_OutMax = 700; p.Iq_OutMin = -700;
end

function state = local_initial_state()
state = struct('dvc', local_pi_state(), 'id', local_pi_state(), 'iq', local_pi_state());
end

function st = local_complete_state(st)
if ~isfield(st, 'dvc'), st.dvc = local_pi_state(); end
if ~isfield(st, 'id'), st.id = local_pi_state(); end
if ~isfield(st, 'iq'), st.iq = local_pi_state(); end
st.dvc = local_complete_pi(st.dvc); st.id = local_complete_pi(st.id); st.iq = local_complete_pi(st.iq);
end

function st = local_pi_state()
st = struct('Ts',single(0),'Ref',single(0),'Fdb',single(0),'Kp',single(0), ...
    'Ki',single(0),'Kc',single(0),'Kd',single(0),'Ui',single(0),'Up',single(0), ...
    'Up_old',single(0),'Ud',single(0),'SatErr',single(0),'Error',single(0), ...
    'OutMax',single(0),'OutMin',single(0),'OutPreSat',single(0),'Out',single(0));
end

function st = local_complete_pi(st)
z = local_pi_state(); f = fieldnames(z);
for k = 1:numel(f)
    if ~isfield(st, f{k}), st.(f{k}) = z.(f{k}); end
end
end

function st = local_reset_pi(st, out_max, out_min)
st = local_complete_pi(st);
st.Ui = single(0); st.Up = single(0); st.Up_old = single(0); st.Ud = single(0);
st.SatErr = single(0); st.Error = single(0); st.OutPreSat = single(0); st.Out = single(0);
st.OutMax = single(out_max); st.OutMin = single(out_min);
end

function st = local_pi2_step(st, par, out_max, out_min)
% 与 motor_PI2_calc 一致：Error -> Up -> Ui -> Ud -> OutPreSat -> clamp -> SatErr。
st = local_complete_pi(st);
e = single(st.Ref - st.Fdb);
up = single(par.Kp) * e;
ui = single(st.Ui + single(par.Ki) * up + single(par.Kc) * st.SatErr);
ud = single(par.Kd) * (up - st.Up_old);
out_pre = single(up + ui + ud);
out = min(max(out_pre, single(out_min)), single(out_max));
st.Error = e; st.Up = up; st.Ui = ui; st.Ud = ud; st.OutPreSat = out_pre;
st.Out = out; st.SatErr = single(out - out_pre); st.Up_old = up;
st.OutMax = single(out_max); st.OutMin = single(out_min);
end

function v = local_get(s, name, default_value)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default_value;
end
end
