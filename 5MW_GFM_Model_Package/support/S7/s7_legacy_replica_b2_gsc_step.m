function [state, out, trace] = s7_legacy_replica_b2_gsc_step(state, u, p)
%S7_LEGACY_REPLICA_B2_GSC_STEP
% S7-5B/B2：固定 VSG 角度输入下的 GSC 电压/电流控制器复制器。
%
% 复制范围对应 grid_forming_control.c 的 GFM、Pre_syn=1 分支：
% Clarke/Park、P/Q 计算、20-Hz 双线性低通、Q-V、电压 PI、电流 PI、
% Ls 交叉解耦以及 dq->alpha-beta。theta_fixed 和 w_ref 由调用者提供，
% 因而不会把 PLL/VSG 状态误混入 B2。所有限幅逻辑保留并返回触发标志。

if nargin == 1 && (ischar(state) || (isstring(state) && isscalar(state)))
    mode = lower(char(state));
    switch mode
        case 'defaults', state = local_defaults();
        case 'initial_state', state = local_initial_state();
        otherwise, error('s7_legacy_replica_b2_gsc_step:UnknownMode', '未知模式 %s。', mode);
    end
    out = []; trace = []; return;
end
if nargin < 3 || isempty(p), p = local_defaults(); end
if nargin < 2 || isempty(u), error('s7_legacy_replica_b2_gsc_step:MissingInput', '缺少输入。'); end
if isempty(state) || ~isstruct(state), state = local_initial_state(); end
state = local_complete_state(state);

Ts = single(local_get(u,'Ts',1e-4));
theta = single(local_get(u,'theta_fixed',0));
theta_u = single(local_get(u,'theta_voltage',theta));
% grid_forming_control.c 在更新 P-f/VSG 角度之前完成电压和电流反馈
% 的 Park 变换；因此允许把“测量/电流角”和最终输出角分开传入。
theta_current = single(local_get(u,'theta_current',theta_u));
wref = single(local_get(u,'w_ref',314));
udc = single(local_get(u,'Udc',1000)); %#ok<NASGU>
pref = single(local_get(u,'P_ref',0));
qref = single(local_get(u,'Q_ref',0));
pre_syn = logical(local_get(u,'Pre_syn',true));
gfm_enabled = logical(local_get(u,'GFM_enabled',true));

% 三相 PCC 电压由线电压恢复，再用指定的测量角度得到 ud/uq。
uab = single(local_get(u,'pcc_uab',0));
ubc = single(local_get(u,'pcc_ubc',0));
uca = single(local_get(u,'pcc_uca',0));
[ua, ub, uc] = local_line_to_phase(uab,ubc,uca);
[u_alpha,u_beta] = local_clarke(ua,ub,uc);
[pcc_ud,pcc_uq] = local_park(u_alpha,u_beta,theta_u);

% GSC 变流器侧和 PCC 侧三相电流均按当前固定控制角变换。
[ia,ib,ic] = local_get_abc(u,'Ia1','Ib1','Ic1');
[ialpha,ibeta] = local_clarke(ia,ib,ic);
[id,iq] = local_park(ialpha,ibeta,theta_current);
[pia,pib,pic] = local_get_abc(u,'pcc_Ia','pcc_Ib','pcc_Ic');
[pialpha,pibeta] = local_clarke(pia,pib,pic);
[pcc_id,pcc_iq] = local_park(pialpha,pibeta,theta_current);

P = single(1.5) * (pcc_id*pcc_ud + pcc_iq*pcc_uq);
Q = single(1.5) * (pcc_id*pcc_uq - pcc_iq*pcc_ud);
state.P_filter = local_lpf_step(state.P_filter,P,single(20));
state.Q_filter = local_lpf_step(state.Q_filter,Q,single(20));

% 与 C 的 vloop_slope 一致：首步从 0 以 1 MW/s 斜率向 P_ref 逼近。
if gfm_enabled
    dPmax = single(p.PrefRampSlope) * Ts;
    deltaP = pref - state.PrefRampOut;
    if deltaP > dPmax
        state.PrefRampOut = state.PrefRampOut + dPmax;
    elseif deltaP < -dPmax
        state.PrefRampOut = state.PrefRampOut - dPmax;
    else
        state.PrefRampOut = pref;
    end
else
    state.PrefRampOut = single(0);
end

% B2 固定角度 GFM：Q-V、电压外环、电流内环。
if gfm_enabled && ~pre_syn
    error('s7_legacy_replica_b2_gsc_step:UnsupportedBranch', 'B2 只验证 Pre_syn=1 的 GSC 闭合分支。');
end
if gfm_enabled
    voltage_ref = single(p.NominalVoltage) + single(p.QvDroop) * (qref - state.Q_filter.out);
else
    voltage_ref = single(local_get(u,'voltage_ref',p.NominalVoltage));
end
voltage_amp = min(max(voltage_ref,single(0)),single(p.EVoltageMax));
uod_ref = voltage_amp; uoq_ref = single(0);

state.dv.Ref = uod_ref; state.dv.Fdb = pcc_ud;
state.dv = local_pi2_step(state.dv,p.VoltagePI,p.VoltageOutMax,p.VoltageOutMin);
% CurrentModel_Idealized/grid_forming_control.c includes the PCC-current
% feed-forward term in the voltage-loop current reference.  It must be
% retained here; otherwise the Replica silently tests a different GSC.
id_ref_unsat = single(p.VoltageGridCurrentFFSign)*pcc_id + state.dv.Out ...
    - single(p.FilterC)*wref*pcc_uq;
state.qv.Ref = uoq_ref; state.qv.Fdb = pcc_uq;
state.qv = local_pi2_step(state.qv,p.VoltagePI,p.VoltageOutMax,p.VoltageOutMin);
iq_ref_unsat = single(p.VoltageGridCurrentFFSign)*pcc_iq + state.qv.Out ...
    + single(p.FilterC)*wref*pcc_ud;
id_ref = id_ref_unsat; iq_ref = iq_ref_unsat;

% 默认 GSI_CURRENT_VECTOR_LIMIT_A=0，因此不触发限流；仍保留审计标志。
current_ref_limited = false;
if p.CurrentVectorLimit > 0
    im = sqrt(double(id_ref*id_ref + iq_ref*iq_ref));
    if im > p.CurrentVectorLimit
        sc = single(p.CurrentVectorLimit/im);
        id_ref = id_ref*sc; iq_ref = iq_ref*sc; current_ref_limited = true;
    end
end

state.di.Ref = id_ref; state.di.Fdb = id;
state.di = local_pi2_step(state.di,p.CurrentPI,p.CurrentOutMax,p.CurrentOutMin);
ud_ref_unsat = state.di.Out - wref*single(p.FilterLs)*iq;
state.qi.Ref = iq_ref; state.qi.Fdb = iq;
state.qi = local_pi2_step(state.qi,p.CurrentPI,p.CurrentOutMax,p.CurrentOutMin);
uq_ref_unsat = state.qi.Out + wref*single(p.FilterLs)*id;
ud_ref = ud_ref_unsat; uq_ref = uq_ref_unsat;
voltage_limited = false;
if p.VoltageModulationLimit > 0
    vm = sqrt(double(ud_ref*ud_ref + uq_ref*uq_ref));
    vlim = single(p.VoltageModulationLimit)*udc/single(1.5);
    if vm > vlim && vm > 1e-6
        sc = single(vlim/vm); ud_ref=ud_ref*sc; uq_ref=uq_ref*sc; voltage_limited=true;
    end
end

c = single(cos(theta)); s = single(sin(theta));
usalfa = ud_ref*c - uq_ref*s; usbeta = ud_ref*s + uq_ref*c;
out = struct('pcc_ud',pcc_ud,'pcc_uq',pcc_uq,'pcc_id',pcc_id,'pcc_iq',pcc_iq, ...
    'P',P,'Q',Q,'P_filter',state.P_filter.out,'Q_filter',state.Q_filter.out, ...
    'P_ref_ramped',state.PrefRampOut,'voltage_ref',voltage_ref,'U_od_ref',uod_ref, ...
    'U_oq_ref',uoq_ref,'Id_ref',id_ref,'Iq_ref',iq_ref,'Id',id,'Iq',iq, ...
    'Ud1_ref',ud_ref,'Uq1_ref',uq_ref,'Us_alfa',usalfa,'Us_beta',usbeta, ...
    'theta_fixed',theta,'w_ref',wref,'Pre_syn',double(pre_syn));
trace = struct();
trace.current_ref_limited = current_ref_limited;
trace.voltage_limited = voltage_limited;
trace.pi_saturated = any([abs(double(state.dv.SatErr)),abs(double(state.qv.SatErr)), ...
    abs(double(state.di.SatErr)),abs(double(state.qi.SatErr))] > 1e-12);
trace.all_limits_inactive = ~(trace.current_ref_limited || trace.voltage_limited || trace.pi_saturated);
end

function p = local_defaults()
p = struct('FilterLs',0.00012,'FilterC',0.000055,'NominalVoltage',563, ...
    'EVoltageMax',800,'QvDroop',3.45e-5,'PrefRampSlope',1e6, ...
    'VoltageGridCurrentFFSign',1.0, ...
    'CurrentVectorLimit',0,'VoltageModulationLimit',0,'VoltageOutMax',1500, ...
    'VoltageOutMin',-1500,'CurrentOutMax',700,'CurrentOutMin',-700);
p.VoltagePI = struct('Kp',1.1309733,'Ki',0.0282743,'Kc',0.00001,'Kd',0.000001);
p.CurrentPI = struct('Kp',0.16,'Ki',0.0172917,'Kc',0,'Kd',0);
end

function st = local_initial_state()
st = struct('dv',local_pi_state(),'qv',local_pi_state(),'di',local_pi_state(), ...
    'qi',local_pi_state(),'P_filter',local_filter_state(),'Q_filter',local_filter_state(), ...
    'PrefRampOut',single(0));
end
function st = local_complete_state(st)
if ~isfield(st,'dv'),st.dv=local_pi_state();end; if ~isfield(st,'qv'),st.qv=local_pi_state();end
if ~isfield(st,'di'),st.di=local_pi_state();end; if ~isfield(st,'qi'),st.qi=local_pi_state();end
if ~isfield(st,'P_filter'),st.P_filter=local_filter_state();end; if ~isfield(st,'Q_filter'),st.Q_filter=local_filter_state();end
if ~isfield(st,'PrefRampOut'),st.PrefRampOut=single(0);end
st.dv=local_complete_pi(st.dv);st.qv=local_complete_pi(st.qv);st.di=local_complete_pi(st.di);st.qi=local_complete_pi(st.qi);
st.P_filter=local_complete_filter(st.P_filter);st.Q_filter=local_complete_filter(st.Q_filter);
end
function st=local_pi_state()
st=struct('Ts',single(0),'Ref',single(0),'Fdb',single(0),'Kp',single(0),'Ki',single(0), ...
    'Kc',single(0),'Kd',single(0),'Ui',single(0),'Up',single(0),'Up_old',single(0), ...
    'Ud',single(0),'SatErr',single(0),'Error',single(0),'OutMax',single(0),'OutMin',single(0), ...
    'OutPreSat',single(0),'Out',single(0));
end
function st=local_complete_pi(st)
z=local_pi_state(); f=fieldnames(z);
for k=1:numel(f), if ~isfield(st,f{k}),st.(f{k})=z.(f{k});end,end
end
function st=local_complete_filter(st)
z=local_filter_state(); f=fieldnames(z);
for k=1:numel(f), if ~isfield(st,f{k}),st.(f{k})=z.(f{k});end,end
end
function st=local_filter_state()
st=struct('out',single(0),'Ui_n_1',single(0),'fs_cutoff',single(20));
end
function st=local_pi2_step(st,par,omax,omin)
st=local_complete_pi(st);
e=single(st.Ref-st.Fdb); up=single(par.Kp)*e;
ui=single(st.Ui+single(par.Ki)*up+single(par.Kc)*st.SatErr);
ud=single(par.Kd)*(up-st.Up_old); pre=single(up+ui+ud);
out=min(max(pre,single(omin)),single(omax));
st.Error=e;st.Up=up;st.Ui=ui;st.Ud=ud;st.OutPreSat=pre;
st.Out=out;st.SatErr=single(out-pre);st.Up_old=up;
st.OutMax=single(omax);st.OutMin=single(omin);
end
function st=local_lpf_step(st,x,fc)
st=local_complete_filter(st); Ts=single(0.00025);
a0=single(1)+Ts*single(pi)*fc; a1=Ts*single(pi)*fc-single(1); b=single(pi)*fc*Ts;
st.out=single((b*x+b*st.Ui_n_1-a1*st.out)/a0); st.Ui_n_1=single(x);
end
function [a,b,c]=local_line_to_phase(uab,ubc,uca)
a=single(-1/3)*(uca-uab); b=single(-1/3)*(uab-ubc); c=single(-1/3)*(ubc-uca);
end
function [alpha,beta]=local_clarke(a,b,c)
alpha=single(2/3)*(a-single(0.5)*b-single(0.5)*c);
beta=single(2/3)*single(1.732)*(b-c)/single(2);
end
function [d,q]=local_park(alpha,beta,theta)
d=alpha*single(cos(theta))+beta*single(sin(theta));
q=beta*single(cos(theta))-alpha*single(sin(theta));
end
function [a,b,c]=local_get_abc(u,fa,fb,fc)
a=single(local_get(u,fa,0)); b=single(local_get(u,fb,0)); c=single(local_get(u,fc,0));
end
function v=local_get(s,n,d)
if isstruct(s)&&isfield(s,n)&&~isempty(s.(n)),v=s.(n);else,v=d;end
end
