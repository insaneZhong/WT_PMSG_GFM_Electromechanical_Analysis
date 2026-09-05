%INITIALIZE_CURRENTMODEL_CONTINUOUS_CONTROLLER
% Loads only the source-aligned physical-topology equilibrium and matching
% continuous-controller states.  No full workspace or raw trajectory is saved.
here=fileparts(mfilename('fullpath'));
OP=solve_currentmodel_source_aligned_equilibrium();
IdealCtrlPVec=OP.pvec(:);
IdealCtrlX0=OP.controller_x0(:);
IdealPmsgId0=OP.pmsg_id0;
IdealPmsgIq0=OP.pmsg_iq0;
% 以同源 M0 严格平衡点统一“机械—电磁—控制”初值。原冷启动
% 参数只取 P_wt/omega，未计入发电机铜耗，因此其轴转矩低于维持
% 5 MW PCC 输出所需的电磁转矩，会在 t=0 人为注入轴系失配。
% 这里不改变原模型拓扑，只为理想连续副本提供同一平衡点的初值。
IdealTgen0=OP.Tgen_Nm;
IdealThetaTw0=OP.theta_tw0;
IdealOmega0=OP.omega0;
% The current physical grid source starts with v_alpha=0, v_beta=-Vpeak;
% therefore its d-axis angle is -pi/2 at t=0.  Map the M0 LCL equilibrium
% to phase initial conditions for the retained physical LCL branches.
thetaGrid0=-pi/2;
ifd=OP.if_grid_dq_A(1); ifq=OP.if_grid_dq_A(2);
igd=OP.ig_grid_dq_A(1); igq=OP.ig_grid_dq_A(2);
vcd=OP.vcap_grid_dq_V(1); vcq=OP.vcap_grid_dq_V(2);
IdealLfIabc0=dq2abc(ifd,ifq,thetaGrid0);
IdealLgIabc0=dq2abc(igd,igq,thetaGrid0);
IdealCfIabc0=dq2abc(ifd-igd,ifq-igq,thetaGrid0);
IdealCfVabc0=dq2abc(vcd,vcq,thetaGrid0);
% The retained SPS controlled sources require one explicit continuous
% regularizer per alpha/beta command to avoid an ideal-source DAE loop.
% Initialize every regularizer at the same source-aligned voltage command;
% otherwise a zero-state filter injects a nonphysical voltage step at t=0.
IdealMSCAlpha0=OP.vmsc_generator_dq_V(1);
IdealMSCBeta0=OP.vmsc_generator_dq_V(2);
uGridAlphaBeta=[0 1;-1 0]*OP.uinv_grid_dq_V; % theta_grid(0)=-pi/2
IdealGSCAlpha0=uGridAlphaBeta(1);
IdealGSCBeta0=uGridAlphaBeta(2);
assignin('base','IdealCtrlPVec',IdealCtrlPVec);
assignin('base','IdealCtrlX0',IdealCtrlX0);
assignin('base','IdealPmsgId0',IdealPmsgId0);
assignin('base','IdealPmsgIq0',IdealPmsgIq0);
assignin('base','IdealTgen0',IdealTgen0);
assignin('base','IdealThetaTw0',IdealThetaTw0);
assignin('base','IdealOmega0',IdealOmega0);
% 覆盖仅服务冷启动斜坡的同名机械初值，使保留的风轮与两质量轴系
% 与 PMSG 电流状态处于统一 5 MW 发电平衡点。
assignin('base','T_e0',IdealTgen0);
assignin('base','T_aero0',IdealTgen0);
assignin('base','theta_tw0',IdealThetaTw0);
assignin('base','theta_tw_stage4_0',IdealThetaTw0);
assignin('base','omega_m0',IdealOmega0);
assignin('base','i_m_q0',IdealPmsgIq0);
assignin('base','D_aero',IdealTgen0/IdealOmega0);
% v_w0 由模型 InitFcn 的机械参数脚本提供；独立调用本初始化脚本
% （例如 LCL 初值写入）时不重新加载机械参数，以免覆盖上述平衡点。
if exist('v_w0','var')
    assignin('base','K_v_aero',3*IdealTgen0/max(v_w0,eps));
end
assignin('base','IdealLfIabc0',IdealLfIabc0);
assignin('base','IdealLgIabc0',IdealLgIabc0);
assignin('base','IdealCfIabc0',IdealCfIabc0);
assignin('base','IdealCfVabc0',IdealCfVabc0);
assignin('base','IdealMSCAlpha0',IdealMSCAlpha0);
assignin('base','IdealMSCBeta0',IdealMSCBeta0);
assignin('base','IdealGSCAlpha0',IdealGSCAlpha0);
assignin('base','IdealGSCBeta0',IdealGSCBeta0);

function abc=dq2abc(d,q,theta)
alpha=d*cos(theta)-q*sin(theta);
beta=d*sin(theta)+q*cos(theta);
abc=[alpha; -0.5*alpha+sqrt(3)/2*beta; -0.5*alpha-sqrt(3)/2*beta];
end
