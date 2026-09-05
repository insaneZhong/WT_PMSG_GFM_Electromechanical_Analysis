function cfg = prepare_s7_legacy_hotstart(saveFile)
%PREPARE_S7_LEGACY_HOTSTART
% 从 M0 的唯一 5 MW 工作点反求隔离 Legacy C 控制器的稳态状态。
%
% 输出 vector 的 32 个元素（按 Legacy 热启动 MEX 的读取顺序）：
%  1-2   MSC-DVC PI: Ui, Out
%  3-6   MSC d/q 电流 PI: Ui/Out
%  7-10  GSC d/q 电压 PI: Ui/Out
% 11-14  GSC d/q 电流 PI: Ui/Out
% 15-18  P/Q 低通滤波器: out/上一拍输入
% 19-22  Legacy 相角/频率/PLL测量状态
% 23-26  VSG 状态及测量PLL状态
% 27-30  前馈、功率斜坡状态
% 31-32  控制定时器计数/首个控制事件标志
%
% 只保存显式必要变量；不保存工作区或仿真时序。

if nargin < 1 || isempty(saveFile)
    here = fileparts(mfilename('fullpath'));
    saveFile = fullfile(here,'temp','S7_5_LegacyPlant', ...
        'S7_Legacy_HotStart_Config.mat');
end
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
src = fullfile(root,'CurrentModel_Idealized');
addpath(here,root,src);
P = init_m0_5mw_parameters();
OP = solve_currentmodel_source_aligned_equilibrium();

% M0 retained source phase at t=0 and the global angle used by Legacy Park.
thetaGrid0 = -pi/2;
delta = OP.delta_vsg_rad;
thetaVsg0 = wrapToPiLocal(thetaGrid0 + delta);

% Build the exact 20-input Legacy vector from the M0 physical work point.
imabc = dq2abc(OP.pmsg_id0,OP.pmsg_iq0,0);
ifabc = dq2abc(OP.if_grid_dq_A(1),OP.if_grid_dq_A(2),thetaGrid0);
igabc = dq2abc(OP.ig_grid_dq_A(1),OP.ig_grid_dq_A(2),thetaGrid0);
% The retained GSC current sensor has the same grid-to-converter
% orientation as the PCC-current sensor.  The M0 phasor current is defined
% positive from the converter toward the grid, so Legacy sees -if.
ifabc_legacy = -ifabc;
% The retained PCC current sensors are oriented from the grid-side
% breaker toward the converter (opposite to the positive export current
% used by the M0 phasor equations).  Keep the plant untouched and express
% only the Legacy controller measurements in its native sign convention.
igabc_pcc = -igabc;
vpabc = dq2abc(OP.vnode_grid_dq_V(1),OP.vnode_grid_dq_V(2),thetaGrid0);
uab = vpabc(1)-vpabc(2); ubc = vpabc(2)-vpabc(3); uca = vpabc(3)-vpabc(1);
legacyPref = -abs(P.Pref_W);
input0 = [imabc(:); P.Vdc_ref_V; OP.omega0; 0; ifabc_legacy(:); ...
    uab; ubc; uca; igabc_pcc(:); 0; 0; legacyPref; 0; P.Vdc_ref_V];

% Reconstruct the exact alpha/beta and Legacy Park measurements.
vpabc_from_line = [-(uca-uab)/3; -(uab-ubc)/3; -(ubc-uca)/3];
[vpA,vpB] = clarke(vpabc_from_line);
[ifA,ifB] = clarke(ifabc_legacy);
[igA,igB] = clarke(igabc_pcc);
[vpd,vpq] = park(vpA,vpB,thetaVsg0);
[ifd,ifq] = park(ifA,ifB,thetaVsg0);
[igd,igq] = park(igA,igB,thetaVsg0);
Pmeas = 1.5*(vpd*igd + vpq*igq);
Qmeas = 1.5*(vpq*igd - vpd*igq);

% The Legacy GSC voltage reference must equal the M0 physical PCC
% magnitude; retaining the hard-coded 563 V would leave a nonzero voltage
% PI error and make a true fixed point impossible.
E0 = hypot(vpd,vpq);
kq = 3.45e-5; % GSI_QV_DROOP_V_PER_VAR in the isolated Legacy build
Vref = E0 + kq*(0-Qmeas);

% The converter voltage target in the Legacy local frame is the M0 uInv
% source-dq vector rotated by the VSG angle difference delta.
[uinvA,uinvB] = dq2ab(OP.uinv_grid_dq_V(1),OP.uinv_grid_dq_V(2),thetaGrid0);
[uinvD,uinvQ] = park(uinvA,uinvB,thetaVsg0);
% GSC outer voltage PI outputs needed to make the inner current references
% equal the measured converter-side LCL current.  Since pcc_Id/pcc_Iq are
% from the grid-to-converter PCC sensors while ifd/ifq are converter-to-grid
% LCL currents, use the explicitly mapped PCC values below; reusing igd/igq
% here would leave a two-current mismatch in the outer-loop equilibrium.
Cf = 0.000275; Lf = 0.00012; w0 = P.omega0_radps;
% In the Legacy frame both measured currents are negative of the M0 export
% convention.  The GSC current-loop references must therefore reproduce
% if_legacy, not the positive M0 current.
idRef = ifd; iqRef = ifq;
dvo = idRef - igd + Cf*w0*vpq;
qvo = iqRef - igq - Cf*w0*vpd;
Kpv = 1.1309733; Kiv = 0.0282743;
% motor_PI2_calc performs Ui <- Ui + Ki*Up before Out=Up+Ui.
upVd = Kpv*(Vref-vpd);
upVq = Kpv*(0-vpq);
xiVd = dvo - (1+Kiv)*upVd;
xiVq = qvo - (1+Kiv)*upVq;

% With zero current error, inner PI outputs are Ui and the decoupling terms
% are added afterwards.  Choose Ui to reproduce the M0 converter voltage.
xiId = uinvD + w0*Lf*ifq;
xiIq = uinvQ - w0*Lf*ifd;

% MSC DVC: Iq_ref = -legacy_msc_iq_ff_a - pwm_speed_pi.Out.  The M0
% generating current is positive, hence the Legacy output state is -iq.
xiDc = -OP.pmsg_iq0;

vector = zeros(32,1);
vector(1:2) = [xiDc; xiDc];
vector(3:6) = [0;0;0;0];
vector(7:10) = [xiVd;dvo;xiVq;qvo];
vector(11:14) = [xiId;uinvD+w0*Lf*ifq;xiIq;uinvQ-w0*Lf*ifd];
vector(15:18) = [Pmeas;Pmeas;Qmeas;Qmeas];
vector(19:22) = [thetaVsg0;w0;thetaGrid0;w0];
vector(23:26) = [w0;w0;thetaGrid0;w0];
vector(27:30) = [0;0;legacyPref;legacyPref];
vector(31:32) = [0;1];

state_names = { ...
    'motor.pwm_speed_pi.Ui','motor.pwm_speed_pi.Out', ...
    'motor.id_pi.Ui','motor.id_pi.Out','motor.iq_pi.Ui','motor.iq_pi.Out', ...
    'd_voltage_loop_pi.Ui','d_voltage_loop_pi.Out', ...
    'q_voltage_loop_pi.Ui','q_voltage_loop_pi.Out', ...
    'd_loop_pi.Ui','d_loop_pi.Out','q_loop_pi.Ui','q_loop_pi.Out', ...
    'lpf.out','lpf.Ui_n_1','lpf1.out','lpf1.Ui_n_1', ...
    'grid_side.pf.thet_ref','grid_side.pf.w_ref', ...
    'grid_side.val.grid_phase_angle','grid_side.val.freq', ...
    'w_vsg_state','w_vsg_sync_anchor','grid_pll_phase','grid_pll_freq', ...
    'legacy_msc_iq_ff_a','legacy_msc_pcc_ff_filter_w', ...
    'vloop_slope.Out','vloop_slope.In','ControlTimerCounter','ControlTimerFlag'};

cfg = struct();
cfg.vector = vector;
cfg.input0 = input0;
cfg.vref_V = Vref;
cfg.expected = struct('Ppcc_W',Pmeas,'Qpcc_var',Qmeas, ...
    'Ppcc_physical_export_W',OP.P_pcc_measurement_W, ...
    'Pmsc_W',OP.P_msc_W,'Pgsc_W',OP.P_gsc_W, ...
    'Pgsc_legacy_export_W',-OP.P_gsc_W,'legacy_pref_W',legacyPref, ...
    'Udc_V',P.Vdc_ref_V,'theta_vsg_global_rad',thetaVsg0, ...
    'vmsc_alpha_beta_V',[OP.vmsc_generator_dq_V(1);OP.vmsc_generator_dq_V(2)], ...
    'vgsc_alpha_beta_V',[OP.uinv_grid_dq_V(1);OP.uinv_grid_dq_V(2)]);
cfg.workpoint = struct('model','M0 Grid_Forming_PMSG5MW_TwoMass_Idealized', ...
    'source_aligned_equilibrium',OP,'theta_grid0_rad',thetaGrid0, ...
    'delta_vsg_rad',delta,'theta_vsg_global_rad',thetaVsg0);
cfg.state_names = state_names;
cfg.metadata = struct('created',datestr(now,30), ...
    'source','solve_currentmodel_source_aligned_equilibrium', ...
    'purpose','S7-C2-R Legacy controller hot-start only', ...
    'production_source_unchanged',true,'vector_length',numel(vector), ...
    'pcc_current_sensor_direction','grid_to_converter', ...
    'legacy_pcc_power_sign',-1,'physical_export_power_sign',1);

outDir = fileparts(saveFile);
if ~isfolder(outDir), mkdir(outDir); end
hotstart_vector = cfg.vector; %#ok<NASGU>
hotstart_input0 = cfg.input0; %#ok<NASGU>
hotstart_vref_V = cfg.vref_V; %#ok<NASGU>
hotstart_expected = cfg.expected; %#ok<NASGU>
hotstart_state_names = cfg.state_names; %#ok<NASGU>
hotstart_metadata = cfg.metadata; %#ok<NASGU>
save(saveFile,'hotstart_vector','hotstart_input0','hotstart_vref_V', ...
    'hotstart_expected','hotstart_state_names','hotstart_metadata');

summaryFile = fullfile(fileparts(saveFile),'S7_Legacy_HotStart_Summary.csv');
T = table(Pmeas,Qmeas,OP.P_msc_W,OP.P_gsc_W,P.Vdc_ref_V,Vref, ...
    'VariableNames',{'Ppcc_W','Qpcc_var','Pmsc_W','Pgsc_W','Udc_V','Vref_V'});
writetable(T,summaryFile);
fprintf('S7 Legacy hot-start config saved: %s\n',saveFile);
fprintf('Ppcc=%.9g W, Pmsc=%.9g W, Pgsc=%.9g W, Vref=%.9g V\n', ...
    Pmeas,OP.P_msc_W,OP.P_gsc_W,Vref);
end

function abc = dq2abc(d,q,theta)
alpha=d*cos(theta)-q*sin(theta);
beta=d*sin(theta)+q*cos(theta);
abc=[alpha;-0.5*alpha+sqrt(3)/2*beta; ...
    -0.5*alpha-sqrt(3)/2*beta];
end

function [alpha,beta] = dq2ab(d,q,theta)
%DQ2AB 将 dq 量按给定角度变换到全局 alpha-beta 坐标。
alpha=d*cos(theta)-q*sin(theta);
beta=d*sin(theta)+q*cos(theta);
end

function [a,b] = clarke(x)
a=(2/3)*(x(1)-0.5*x(2)-0.5*x(3));
b=(sqrt(3)/3)*(x(2)-x(3));
end

function [d,q] = park(a,b,theta)
d=a*cos(theta)+b*sin(theta);
q=b*cos(theta)-a*sin(theta);
end

function y = wrapToPiLocal(x)
y=mod(x+pi,2*pi)-pi;
end
