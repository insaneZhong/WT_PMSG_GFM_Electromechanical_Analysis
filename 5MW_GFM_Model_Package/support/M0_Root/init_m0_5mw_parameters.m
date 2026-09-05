function P = init_m0_5mw_parameters(varargin)
%INIT_M0_5MW_PARAMETERS M0理想连续平均模型的唯一参数入口。
%
%  本参数集采用SI单位和相电压峰值dq定义：
%    P = 1.5*(vd*id + vq*iq)
%    Q = 1.5*(vq*id - vd*iq)
%  M0不读取旧C宏、不调用MEX，也不继承开关模型中的限幅、PLL、
%  预同步、斜率限制、anti-windup、PWM延迟或主动阻尼状态。

ip = inputParser;
ip.addParameter('H_s', 3, @(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('SCR', 4, @(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('DVCScale', 1, @(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('GSCCurrentBandwidth_Hz', 300, ...
    @(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('GSCVoltageBandwidth_Hz', 30, ...
    @(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('ControllerProfile', "AlignedStable", ...
    @(x)any(strcmpi(string(x),["AlignedStable","AuditedCode","StableHierarchy"])));
ip.addParameter('GSCFeedforwardMode', "AuditedNoPCC", ...
    @(x)any(strcmpi(string(x),["AuditedNoPCC","PhysicalFull"])));
ip.parse(varargin{:});
o = ip.Results;

P = struct();
P.model_version = 'M0-5MW-IdealContinuous-v2';
P.parameter_source = [ ...
    'Liu2024_5MW_Params.m ratings, audited continuous conversion, ' ...
    'and explicit M0 stable hierarchy with grid-current feedforward'];

%% 统一额定基值（SI，严禁1 MW宏覆盖）
P.Sbase_W = 5.0e6;
P.Vll_rms_V = 690;
% 当前权威 Idealized SPS 电网源的实际相量幅值为 563 V；
% 不能用铭牌 690 V 反算的 563.383 V 替代，否则网侧电感平衡残差约 5e3 A/s。
P.Vphase_peak_V = 563;
P.Iphase_peak_base_A = P.Sbase_W/(1.5*P.Vphase_peak_V);
P.Vdc_ref_V = 1500;
P.fgrid_Hz = 50;
P.omega0_radps = 2*pi*P.fgrid_Hz;
P.Zbase_ohm = P.Vll_rms_V^2/P.Sbase_W;
P.Lbase_H = P.Zbase_ohm/P.omega0_radps;

%% 网侧物理参数：Cf和Cdc明确分离
P.SCR = o.SCR;
P.Rf_ohm = 0.005*P.Zbase_ohm;
P.Lf_H = 0.15*P.Lbase_H;
P.Cf_F = 5*55e-6;      % 5 MW EMT实现的交流滤波电容
P.Rd_ohm = 0.1;        % 电容串联阻尼电阻
P.Rg_ohm = 0.02*P.Zbase_ohm*(4/P.SCR);
P.Lg_H = 0.25*P.Lbase_H*(4/P.SCR);
P.Cdc_F = 0.3;         % 直流母线电容；不得误作Cf

%% PMSG与两质量轴系
P.omega_m0_radps = 1.27;
P.Rs_ohm = 5.0e-4;
P.Ld_H = 1.05e-3;
P.Lq_H = 1.05e-3;
P.psi_f_Wb = 8.64;
P.pole_pairs = 20;
P.Kt_Nm_per_A = 1.5*P.pole_pairs*P.psi_f_Wb;

P.Ht_s = 1.93;
P.Hg_s = 0.8;
P.Ksh_pu = 280;
P.Dsh_pu = 1.0;
P.Jbase_per_H = 2*P.Sbase_W/P.omega_m0_radps^2;
P.Kbase_Nm_per_rad = P.Sbase_W/P.omega_m0_radps^2;
P.Jt_kgm2 = P.Ht_s*P.Jbase_per_H;
P.Jg_kgm2 = P.Hg_s*P.Jbase_per_H;
P.Ksh_Nm_per_rad = P.Ksh_pu*P.Kbase_Nm_per_rad;
P.Dsh_Nms_per_rad = P.Dsh_pu*P.Kbase_Nm_per_rad;
% 公共转速被动阻尼按两惯量同比例分配。Dt/Jt=Dg/Jg，因此该项
% 在相对加速度方程中严格抵消，只锚定刚体/COI转速，不充当主动轴系阻尼。
P.Dcoi_pu = 0.12;
Jsum = P.Jt_kgm2 + P.Jg_kgm2;
P.Dt_Nms_per_rad = P.Dcoi_pu*P.Kbase_Nm_per_rad*P.Jt_kgm2/Jsum;
P.Dg_Nms_per_rad = P.Dcoi_pu*P.Kbase_Nm_per_rad*P.Jg_kgm2/Jsum;
P.fshaft_openloop_Hz = sqrt(P.Ksh_Nm_per_rad* ...
    (1/P.Jt_kgm2+1/P.Jg_kgm2))/(2*pi);

%% MSC连续控制：Type-A DVC + 有限带宽电流环
% 旧数字PI：Ui[k+1]=Ui[k]+Ki_fraction*Kp*e[k]
% 连续等效：dxI/dt=(Ki_fraction*Kp/Ts)*e
P.controller_Ts_s = 1/10e3;
% 取最终5 MW控制器的离散PI并按100 us控制周期精确连续化。
P.Kp_dvc_A_per_V = o.DVCScale*7.8983182658;
P.Ki_dvc_A_per_Vs = o.DVCScale*8.6071416999;

P.Kp_msc_i_V_per_A = 1.4*(P.Ld_H/1.02e-3);
ki_msc_fraction = 0.00290476*((P.Rs_ohm/P.Ld_H)/(0.0122/1.02e-3));
P.Ki_msc_i_V_per_As = ki_msc_fraction* ...
    P.Kp_msc_i_V_per_A/P.controller_Ts_s;

%% GSC连续控制：有限带宽电压/电流双闭环，无限幅、无延迟
% M0将带宽层级显式设为“电流环快、电压环慢”。旧开关控制器的
% 离散等效增益同时保留在审计字段中，但不作为首个稳定M0的默认值。
P.gsc_current_bw_Hz = o.GSCCurrentBandwidth_Hz;
P.gsc_voltage_bw_Hz = o.GSCVoltageBandwidth_Hz;
wci = 2*pi*P.gsc_current_bw_Hz;
wcv = 2*pi*P.gsc_voltage_bw_Hz;
P.Kp_gsc_i_V_per_A = P.Lf_H*wci;
P.Ki_gsc_i_V_per_As = P.Rf_ohm*wci;
zeta_v = 0.9;
P.Kp_gsc_v_A_per_V = 2*zeta_v*wcv*P.Cf_F;
P.Ki_gsc_v_A_per_Vs = wcv^2*P.Cf_F;

legacy_Kp_i = 0.16*(P.Lf_H/120e-6);
legacy_ki_i_fraction = 0.0172917*(P.Rf_ohm/0.0002);
P.audit.legacy_Kp_gsc_i_V_per_A = legacy_Kp_i;
P.audit.legacy_Ki_gsc_i_V_per_As = legacy_ki_i_fraction* ...
    legacy_Kp_i/P.controller_Ts_s;
P.audit.legacy_Kp_gsc_v_A_per_V = 1.1309733;
P.audit.legacy_Ki_gsc_v_A_per_Vs = 0.0282743* ...
    1.1309733/P.controller_Ts_s;
P.controller_profile = char(string(o.ControllerProfile));
if strcmpi(P.controller_profile,'AuditedCode')
    P.Kp_gsc_i_V_per_A = P.audit.legacy_Kp_gsc_i_V_per_A;
    P.Ki_gsc_i_V_per_As = P.audit.legacy_Ki_gsc_i_V_per_As;
    P.Kp_gsc_v_A_per_V = P.audit.legacy_Kp_gsc_v_A_per_V;
    P.Ki_gsc_v_A_per_Vs = P.audit.legacy_Ki_gsc_v_A_per_Vs;
end
P.gsc_feedforward_mode = char(string(o.GSCFeedforwardMode));
P.gsc_grid_current_feedforward = double(strcmpi( ...
    P.gsc_feedforward_mode,'PhysicalFull'));
P.gsc_pcc_voltage_feedforward = P.gsc_grid_current_feedforward;
if strcmpi(P.controller_profile,'AlignedStable')
    % 电容方程 Cf*dv/dt=if-ig 要求电压环显式前馈 +ig；同时保持
    % PCC电压前馈关闭，以与本M0的电流环/植物定义避免重复前馈。
    P.gsc_feedforward_mode = 'GridCurrentOnly';
    P.gsc_grid_current_feedforward = 1;
    P.gsc_pcc_voltage_feedforward = 0;
end

%% 真VSG、连续P/Q滤波和Q-V下垂
P.H_s = o.H_s;
% 最终5 MW编译控制器采用2% P-f下垂。旧参数文件中的0.1%仅是
% 启动锚点，不能作为M0主参数。
P.pf_droop_fraction = 0.02;
P.mp_radps_per_W = 2*pi*(P.pf_droop_fraction*P.fgrid_Hz)/P.Sbase_W;
% 旧开关模型/MEX因实测增量P-angle方向为负而使用-1；M0重新定义
% delta=theta_vsg-theta_grid，并以dP/ddelta>0为硬门槛，故必须使用+1。
P.vsg_physical_power_error_sign = 1;
P.audit.legacy_code_power_error_sign = -1;
P.pq_filter_radps = 2*pi*20;
P.qv_droop_V_per_var = 9.2e-8;
P.Pref_W = 5.0e6;
P.Qref_var = 0;

%% M0结构开关（固定，不作为运行时状态机）
P.dvc_type = 'Type-A';
P.active_damping_present = false;
P.pll_present = false;
P.presync_present = false;
P.pwm_present = false;
P.digital_delay_present = false;
P.limiters_present = false;
P.mppt_dynamic_present = false;
P.pitch_dynamic_present = false;
end
