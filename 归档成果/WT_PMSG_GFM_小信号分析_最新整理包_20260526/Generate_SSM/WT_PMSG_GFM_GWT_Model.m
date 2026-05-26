%% WT-PMSG Grid-Forming Generator-Side MPPT Small-Signal Model
% GFM-GWT comparison case: MSC-MPPT and GSC-GFM with DC-link power control.
% The physical plant and two-mass shaft remain identical to GFM-MWT.

close all
clear all
clc

%% Subsystem modeling-- Algebraic functions + Components 
%% Algebraic:

syms delta delta0 i2_d i2_q i2_d0 i2_q0 Vpcc_D Vpcc_Q Vpcc_D0 Vpcc_Q0 Vc_d Vc_q Vc_d0 Vc_q0
R_delta0=[cos(delta0) -sin(delta0);
          sin(delta0) cos(delta0)];
U_delta0=[-sin(delta0) -cos(delta0);
          cos(delta0) -sin(delta0)];

%定义从虚拟dq坐标系到网络DQ坐标系的旋转变换矩阵及其对角度扰动的雅可比矩阵。用于处理非线性坐标变换的线性化
%虚拟转子角 δ 发生小扰动时，会对电流和电压在不同坐标系下的映射产生影响
i2_DQ_eq = R_delta0*[i2_d;i2_q] + U_delta0*[i2_d0;i2_q0]*delta  
Vpcc_dq_eq = R_delta0'*[Vpcc_D;Vpcc_Q] + U_delta0'*[Vpcc_D0;Vpcc_Q0]*delta
%瞬时功率公式的小信号展开形式，用于反馈至VSG和无功控制环节
p_m_eq = (3/2)*(Vc_d0*i2_d+Vc_q0*i2_q+i2_d0*Vc_d+i2_q0*Vc_q)
q_m_eq = (3/2)*(Vc_q0*i2_d-Vc_d0*i2_q-i2_q0*Vc_d+i2_d0*Vc_q)
%% Components:
%% 1) Grid Model

syms i2_D i2_Q Vg_D Vg_Q ig_D ig_Q Vpcc_D Vpcc_Q Vcs_D Vcs_Q %Variables
syms lg rg rs cs wn %Parameters 
%Vectors
x_Grid =[ig_D; ig_Q; Vcs_D; Vcs_Q]; e_Grid=[Vpcc_D,Vpcc_Q]; u_Grid = [i2_D; i2_Q; Vg_D; Vg_Q]; y_Grid = [Vpcc_D; Vpcc_Q];
%Nonlinear DAE
f_Grid=[...
   (1/lg)*(Vpcc_D- Vg_D -rg*ig_D + wn*ig_Q*lg);
   (1/lg)*(Vpcc_Q- Vg_Q -rg*ig_Q - wn*ig_D*lg);
   (1/cs)*(i2_D - ig_D) + wn*Vcs_Q;
   (1/cs)*(i2_Q - ig_Q) - wn*Vcs_D]; 

g_Grid=[Vcs_D+(i2_D-ig_D)*rs-Vpcc_D;
   Vcs_Q+(i2_Q-ig_Q)*rs-Vpcc_Q]; 

h_Grid=[Vpcc_D; 
        Vpcc_Q];

J_fx = jacobian(f_Grid, x_Grid);
J_fe = jacobian(f_Grid, e_Grid);
J_fu = jacobian(f_Grid, u_Grid);
J_gx = jacobian(g_Grid, x_Grid);
J_ge = jacobian(g_Grid, e_Grid);
J_gu = jacobian(g_Grid, u_Grid);
J_hx = jacobian(h_Grid, x_Grid);
J_he = jacobian(h_Grid, e_Grid);
J_hu = jacobian(h_Grid, u_Grid);
% 建立微分-代数方程（DAE），使用雅可比方法消去代数变量，得到标准状态空间形式
A_Grid = J_fx - J_fe * inv(J_ge) * J_gx
B_Grid = J_fu - J_fe * inv(J_ge) * J_gu
C_Grid = J_hx - J_he * inv(J_ge) * J_gx
D_Grid = J_hu - J_he * inv(J_ge) * J_gu

%% 这里可以考虑加一个“_*虚拟阻抗*_”模块（抑制高频谐振）
% （可在特征值分析中观察极点左移幅度。）
%% 2) LCL Model

syms Vi_d Vi_q i1_d i1_q Vcf_d Vcf_q Vc_d Vc_q i2_d i2_q ig_d ig_q Vpcc_d Vpcc_q %Variables
syms lf1 rf1 rd cf lf2 rf2 w %Parameters 
%Vectors
x_LCL = [i1_d; i1_q; Vcf_d; Vcf_q ; i2_d; i2_q]; 
e_LCL= [Vc_d; Vc_q]; 
u_LCL = [Vi_d; Vi_q; Vpcc_d; Vpcc_q]; 
y_LCL = [i1_d; i1_q; Vc_d; Vc_q; i2_d; i2_q];
%Nonlinear DAE
f_LCL=[...
  (1/lf1)*(Vi_d - Vc_d - rf1*i1_d + lf1*i1_q*w);
  (1/lf1)*(Vi_q - Vc_q - rf1*i1_q - lf1*i1_d*w);
  (1/cf)*(i1_d - i2_d) + w*Vcf_q;
  (1/cf)*(i1_q - i2_q) - w*Vcf_d;
  (1/lf2)*(Vc_d - Vpcc_d - rf2*i2_d + lf2*i2_q*w);
  (1/lf2)*(Vc_q - Vpcc_q - rf2*i2_q - lf2*i2_d*w)];

g_LCL=[Vcf_d + rd*(i1_d - i2_d)-Vc_d;
   Vcf_q + rd*(i1_q - i2_q)-Vc_q]; 

h_LCL=[i1_d;
  i1_q;
  Vc_d; 
  Vc_q; 
  i2_d;
  i2_q];

J_fx = jacobian(f_LCL, x_LCL);J_fe = jacobian(f_LCL, e_LCL);J_fu = jacobian(f_LCL, u_LCL);
J_gx = jacobian(g_LCL, x_LCL);J_ge = jacobian(g_LCL, e_LCL);J_gu = jacobian(g_LCL, u_LCL);
J_hx = jacobian(h_LCL, x_LCL);J_he = jacobian(h_LCL, e_LCL);J_hu = jacobian(h_LCL, u_LCL);

A_LCL = J_fx - J_fe * inv(J_ge) * J_gx
B_LCL = J_fu - J_fe * inv(J_ge) * J_gu
C_LCL = J_hx - J_he * inv(J_ge) * J_gx
D_LCL = J_hu - J_he * inv(J_ge) * J_gu
%% 3) VSI Delay（这是一个 *6阶Pade近似模型*，用来逼近纯延迟环节 _e_−_std_�，模拟采样、计算、PWM生成带来的总延迟（通常取 _td_�=0.5/_fsw_�）。）

syms Vm_d Vm_q Vi_d Vi_q x_del1 x_del2 x_del3 x_del4 x_del5 x_del6%Variables
syms td %Parameters 
%Vectors
x_Del = [x_del1; x_del2; x_del3; x_del4; x_del5; x_del6];
u_Del = [Vm_d; Vm_q];
y_Del = [Vi_d; Vi_q];
%Nonlinear DAE
f_Del=[...
 x_del2;
 x_del3;
 Vm_d-(12/td)*x_del3-(60/td^2)*x_del2-(120/td^3)*x_del1;
 x_del5;
 x_del6;
 Vm_q-(12/td)*x_del6-(60/td^2)*x_del5-(120/td^3)*x_del4;
];

h_Del=[...
    (24/td)*x_del3-Vm_d+(240/td^3)*x_del1;
    (24/td)*x_del6-Vm_q+(240/td^3)*x_del4];

J_fx = jacobian(f_Del, x_Del); J_fu = jacobian(f_Del, u_Del);
J_hx = jacobian(h_Del, x_Del); J_hu = jacobian(h_Del, u_Del);

A_Del = J_fx
B_Del = J_fu
C_Del = J_hx
D_Del = J_hu
%% 4) Current Control

syms i1_d_ref i1_q_ref i1_d i1_q Vc_d Vc_q Vm_d Vm_q U_id U_iq gamma_id gamma_iq%Variables
syms k_pi k_ii beta_v wn lf1%Parameters
% 实现 d/q 轴独立电流跟踪
% beta_v*Vc_d 和 -wn*lf1*i1_q 是解耦项，消除轴间耦合
% 控制器带宽应远高于外环（建议 >5倍）
u_CC=[i1_d_ref; i1_q_ref; i1_d; i1_q; Vc_d; Vc_q]; %Input
y_CC=[Vm_d; Vm_q]; %Output
x_CC=[gamma_id;gamma_iq]; %Internal states
e_CC=[U_id;U_iq];

f_CC=[i1_d_ref - i1_d;
   i1_q_ref - i1_q]; %State equation of voltage controller

g_CC=[k_pi*(i1_d_ref-i1_d)+k_ii*gamma_id-U_id;
   k_pi*(i1_q_ref-i1_q)+k_ii*gamma_iq-U_iq];

h_CC=[U_id + beta_v*Vc_d - wn*lf1*i1_q;
  U_iq + beta_v*Vc_q + wn*lf1*i1_d];

J_fx = jacobian(f_CC, x_CC); J_fe = jacobian(f_CC, e_CC); J_fu = jacobian(f_CC, u_CC);
J_gx = jacobian(g_CC, x_CC); J_ge = jacobian(g_CC, e_CC); J_gu = jacobian(g_CC, u_CC);
J_hx = jacobian(h_CC, x_CC); J_he = jacobian(h_CC, e_CC); J_hu = jacobian(h_CC, u_CC);

A_CC = J_fx - J_fe * inv(J_ge) * J_gx
B_CC = J_fu - J_fe * inv(J_ge) * J_gu
C_CC = J_hx - J_he * inv(J_ge) * J_gx
D_CC = J_hu - J_he * inv(J_ge) * J_gu

%% 5) Voltage Control

syms Vc_d_ref Vc_q_ref Vc_d Vc_q i2_d i2_q i1_d_ref i1_q_ref U_vd U_vq gamma_vd gamma_vq%Variables
syms k_pv k_iv beta_i wn cf%Parameters

u_VC=[Vc_d_ref; Vc_q_ref; Vc_d; Vc_q; i2_d; i2_q]; %Input
y_VC=[i1_d_ref; i1_q_ref]; %Output
x_VC=[gamma_vd;gamma_vq]; %Internal states
e_VC=[U_vd;U_vq];

f_VC=[Vc_d_ref - Vc_d;
   Vc_q_ref - Vc_q]; %State equation of voltage controller

g_VC=[k_pv*(Vc_d_ref-Vc_d)+k_iv*gamma_vd-U_vd;
   k_pv*(Vc_q_ref-Vc_q)+k_iv*gamma_vq-U_vq];

h_VC=[U_vd + beta_i*i2_d - wn*cf*Vc_q;
  U_vq + beta_i*i2_q + wn*cf*Vc_d];

J_fx = jacobian(f_VC, x_VC); J_fe = jacobian(f_VC, e_VC); J_fu = jacobian(f_VC, u_VC);
J_gx = jacobian(g_VC, x_VC); J_ge = jacobian(g_VC, e_VC); J_gu = jacobian(g_VC, u_VC);
J_hx = jacobian(h_VC, x_VC); J_he = jacobian(h_VC, e_VC); J_hu = jacobian(h_VC, u_VC);

A_VC = J_fx - J_fe * inv(J_ge) * J_gx
B_VC = J_fu - J_fe * inv(J_ge) * J_gu
C_VC = J_hx - J_he * inv(J_ge) * J_gx
D_VC = J_hu - J_he * inv(J_ge) * J_gu

%% 6) Reactive Power Control

syms q_ref q_m u_q Vc_d_ref Vc_q_ref gamma_q %Variables
syms k_pq k_iq V_nom %Parameters

u_RPC = [q_ref; q_m]; %Input
y_RPC = [Vc_d_ref; Vc_q_ref]; %Output
x_RPC = gamma_q; %Internal states
e_RPC = u_q;%Algebraic vector

f_RPC=q_ref-q_m; %State equation of Reactive Power controller

g_RPC=k_pq*(q_ref-q_m)+k_iq*gamma_q-u_q;

h_RPC=[u_q + V_nom;
  0]; %Output equation

J_fx = jacobian(f_RPC, x_RPC); J_fe = jacobian(f_RPC, e_RPC); J_fu = jacobian(f_RPC, u_RPC);
J_gx = jacobian(g_RPC, x_RPC); J_ge = jacobian(g_RPC, e_RPC); J_gu = jacobian(g_RPC, u_RPC);
J_hx = jacobian(h_RPC, x_RPC); J_he = jacobian(h_RPC, e_RPC); J_hu = jacobian(h_RPC, u_RPC);

A_RPC = J_fx - J_fe * inv(J_ge) * J_gx
B_RPC = J_fu - J_fe * inv(J_ge) * J_gu
C_RPC = J_hx - J_he * inv(J_ge) * J_gx
D_RPC = J_hu - J_he * inv(J_ge) * J_gu
%% 7) VSG

syms p_ref p_m delta %Variables
syms wr mp w wn h%Parameters

u_VSG=[p_ref; p_m]; %Input
y_VSG=delta; %Output
x_VSG=[w; delta]; %Internal states

f_VSG=[(1/(2*h*wn))*(p_ref - p_m - (w-wn)/mp); ...
   w-wn]; %State equation of Reactive Power controller
h_VSG=delta; %Output equation

J_fx = jacobian(f_VSG, x_VSG); J_fu = jacobian(f_VSG, u_VSG);
J_hx = jacobian(h_VSG, x_VSG); J_hu = jacobian(h_VSG, u_VSG);

A_VSG = J_fx
B_VSG = J_fu
C_VSG = J_hx
D_VSG = J_hu

%% Component Connection Method
%% 9) Two-mass drivetrain small-signal model for WT-PMSG

% 9) Two-mass drivetrain small-signal model for WT-PMSG
% omega_t: turbine-side speed; omega_g: generator-side speed;
% theta_tw: shaft twist angle. T_m is an additive mechanical disturbance;
% the operating-point aerodynamic torque feedback is included explicitly.
syms omega_t omega_g theta_tw T_m T_aero T_e T_sh v_w beta %Variables
syms J_t J_g K_sh D_sh D_t D_g D_aero K_v_aero K_beta_aero %Parameters

x_TMS = [omega_t; omega_g; theta_tw];
u_TMS = [T_m; v_w; beta; T_e];
y_TMS = [omega_t; omega_g; theta_tw; T_sh; T_aero];

T_sh_eq = K_sh*theta_tw + D_sh*(omega_t - omega_g);
T_aero_eq = -D_aero*omega_t + K_v_aero*v_w + K_beta_aero*beta;

f_TMS = [
    (1/J_t)*(T_m + T_aero_eq - T_sh_eq - D_t*omega_t);
    (1/J_g)*(T_sh_eq - T_e - D_g*omega_g);
    omega_t - omega_g];

h_TMS = [omega_t;
         omega_g;
         theta_tw;
         T_sh_eq;
         T_aero_eq];

J_fx = jacobian(f_TMS, x_TMS); J_fu = jacobian(f_TMS, u_TMS);
J_hx = jacobian(h_TMS, x_TMS); J_hu = jacobian(h_TMS, u_TMS);

A_TMS = J_fx
B_TMS = J_fu
C_TMS = J_hx
D_TMS = J_hu

% 10) PMSG electrical small-signal model
% The model is linearized around (i_m_d0, i_m_q0, omega_g0).
% Inputs: machine-side converter voltage and generator speed.
% Outputs: stator currents, electromagnetic torque, and MSC-side power.
syms i_m_d i_m_q v_m_d v_m_q p_msc %Variables
syms R_s L_d L_q psi_f n_p omega_g0 i_m_d0 i_m_q0 %Parameters

x_PMSG = [i_m_d; i_m_q];
u_PMSG = [v_m_d; v_m_q; omega_g];
y_PMSG = [i_m_d; i_m_q; T_e; p_msc];

T_e0 = (3/2)*n_p*(psi_f*i_m_q0 + (L_d - L_q)*i_m_d0*i_m_q0);
T_e_eq = (3/2)*n_p*(psi_f*i_m_q + (L_d - L_q)*(i_m_q0*i_m_d + i_m_d0*i_m_q));
p_msc_eq = omega_g0*T_e_eq + T_e0*omega_g;

f_PMSG = [
    (1/L_d)*(v_m_d - R_s*i_m_d + n_p*L_q*(omega_g0*i_m_q + i_m_q0*omega_g));
    (1/L_q)*(v_m_q - R_s*i_m_q - n_p*(omega_g0*L_d*i_m_d + (L_d*i_m_d0 + psi_f)*omega_g))];

h_PMSG = [i_m_d;
          i_m_q;
          T_e_eq;
          p_msc_eq];

J_fx = jacobian(f_PMSG, x_PMSG); J_fu = jacobian(f_PMSG, u_PMSG);
J_hx = jacobian(h_PMSG, x_PMSG); J_hu = jacobian(h_PMSG, u_PMSG);

A_PMSG = J_fx
B_PMSG = J_fu
C_PMSG = J_hx
D_PMSG = J_hu

% 11) Machine-side converter current controller
% Inner current controller for the PMSG-side converter. The feedforward
% terms compensate the PMSG cross coupling at the operating point.
syms i_m_d_ref i_m_q_ref gamma_md gamma_mq %Variables
syms k_pm k_im %Parameters

x_MSC = [gamma_md; gamma_mq];
u_MSC = [i_m_d_ref; i_m_q_ref; i_m_d; i_m_q; omega_g];
y_MSC = [v_m_d; v_m_q];

f_MSC = [i_m_d_ref - i_m_d;
         i_m_q_ref - i_m_q];

h_MSC = [
    k_pm*(i_m_d_ref - i_m_d) + k_im*gamma_md - n_p*(omega_g0*L_q*i_m_q + L_q*i_m_q0*omega_g);
    k_pm*(i_m_q_ref - i_m_q) + k_im*gamma_mq + n_p*(omega_g0*L_d*i_m_d + (L_d*i_m_d0 + psi_f)*omega_g)];

J_fx = jacobian(f_MSC, x_MSC); J_fu = jacobian(f_MSC, u_MSC);
J_hx = jacobian(h_MSC, x_MSC); J_hu = jacobian(h_MSC, u_MSC);

A_MSC = J_fx
B_MSC = J_fu
C_MSC = J_hx
D_MSC = J_hu

% 12) MSC MPPT torque reference
% Generator speed sets torque reference while the GSC regulates DC energy.
syms k_t_mppt

x_MPPT = sym(zeros(0, 1));
u_MPPT = [p_ref; omega_g];
y_MPPT = [i_m_d_ref; i_m_q_ref];
h_MPPT = [0;
          (p_ref/omega_g0 + k_t_mppt*omega_g)/(1.5*n_p*psi_f)];

A_MPPT = sym(zeros(0, 0));
B_MPPT = sym(zeros(0, 2));
C_MPPT = sym(zeros(2, 0));
D_MPPT = jacobian(h_MPPT, u_MPPT)

% 13) GSC DC-link voltage power controller
% The DC correction enters the VSG power command, retaining GFM operation.
syms vdc_ref v_dc gamma_dc_gwt p_dc
syms k_pdc_gwt k_idc_gwt

x_GDVC = gamma_dc_gwt;
u_GDVC = [vdc_ref; v_dc];
y_GDVC = p_dc;

f_GDVC = v_dc - vdc_ref;
h_GDVC = k_pdc_gwt*(v_dc - vdc_ref) + k_idc_gwt*gamma_dc_gwt;

J_fx = jacobian(f_GDVC, x_GDVC); J_fu = jacobian(f_GDVC, u_GDVC);
J_hx = jacobian(h_GDVC, x_GDVC); J_hu = jacobian(h_GDVC, u_GDVC);

A_GDVC = J_fx
B_GDVC = J_fu
C_GDVC = J_hx
D_GDVC = J_hu

% 14) DC-link voltage dynamics
% C_dc*V_dc0*d(v_dc)/dt = p_msc - p_gsc. The grid-side converter power
% p_gsc is represented by the measured active power p_m from the GSC/LCL.
syms C_dc V_dc0 %Parameters

x_DC = v_dc;
u_DC = [p_msc; p_m];
y_DC = v_dc;

f_DC = (1/(C_dc*V_dc0))*(p_msc - p_m);
h_DC = v_dc;

J_fx = jacobian(f_DC, x_DC); J_fu = jacobian(f_DC, u_DC);
J_hx = jacobian(h_DC, x_DC); J_hu = jacobian(h_DC, u_DC);

A_DC = J_fx
B_DC = J_fu
C_DC = J_hx
D_DC = J_hu
% Unified State Space Model 对角拼接成大矩阵
A_d0 = blkdiag(A_VSG, A_RPC, A_VC, A_CC, A_Del, A_LCL, A_Grid, A_TMS, A_PMSG, A_MSC, A_MPPT, A_GDVC, A_DC);
B_d0 = blkdiag(B_VSG, B_RPC, B_VC, B_CC, B_Del, B_LCL, B_Grid, B_TMS, B_PMSG, B_MSC, B_MPPT, B_GDVC, B_DC);
C_d0 = blkdiag(C_VSG, C_RPC, C_VC, C_CC, C_Del, C_LCL, C_Grid, C_TMS, C_PMSG, C_MSC, C_MPPT, C_GDVC, C_DC);
D_d0 = blkdiag(D_VSG, D_RPC, D_VC, D_CC, D_Del, D_LCL, D_Grid, D_TMS, D_PMSG, D_MSC, D_MPPT, D_GDVC, D_DC);

X_stac = [x_VSG; x_RPC; x_VC; x_CC; x_Del; x_LCL; x_Grid; x_TMS; x_PMSG; x_MSC; x_MPPT; x_GDVC; x_DC];
U_stac = [u_VSG; u_RPC; u_VC; u_CC; u_Del; u_LCL; u_Grid; u_TMS; u_PMSG; u_MSC; u_MPPT; u_GDVC; u_DC];
Y_stac = [y_VSG; y_RPC; y_VC; y_CC; y_Del; y_LCL; y_Grid; y_TMS; y_PMSG; y_MSC; y_MPPT; y_GDVC; y_DC];

ustac_eq = [% Input to Active Controller
            p_ref + p_dc;
            p_m_eq; % measured GSC active power
            % Input to Reactive Controller
            q_ref;
            q_m_eq; % measured GSC reactive power
            % Input to Voltage Controller 
            Vc_d_ref;
            Vc_q_ref;
            Vc_d; 
            Vc_q; 
            i2_d; 
            i2_q; 
            % Input to Current Controller
            i1_d_ref;
            i1_q_ref;
            i1_d;
            i1_q;
            Vc_d;
            Vc_q;
            % Input to Delay
            Vm_d;
            Vm_q;
            % Input to LCL
            Vi_d; 
            Vi_q; 
            Vpcc_dq_eq(1);
            Vpcc_dq_eq(2);
            % Input to Grid
            i2_DQ_eq(1);
            i2_DQ_eq(2);
            Vg_D;
            Vg_Q;
            % Input to two-mass drivetrain
            T_m;
            v_w;
            beta;
            T_e;
            % Input to PMSG electrical model
            v_m_d;
            v_m_q;
            omega_g;
            % Input to MSC current controller
            i_m_d_ref;
            i_m_q_ref;
            i_m_d;
            i_m_q;
            omega_g;
            % Input to MSC MPPT torque reference
            p_ref;
            omega_g;
            % Input to GSC DC-link voltage power controller
            vdc_ref;
            v_dc;
            % Input to DC-link dynamics
            p_msc;
            p_m_eq;
            ];

U_sys = [p_ref; q_ref; Vg_D; Vg_Q; T_m; v_w; beta; vdc_ref];
y_sys_eq = [i1_d; i1_q; Vc_d; Vc_q; p_m_eq; q_m_eq; omega_t; omega_g; theta_tw; T_sh; T_aero; T_e; p_msc; v_dc; i_m_d; i_m_q]; 
Y_sys = [i1_d; i1_q; Vc_d; Vc_q; p_m; q_m; omega_t; omega_g; theta_tw; T_sh; T_aero; T_e; p_msc; v_dc; i_m_d; i_m_q];

R1 = equationsToMatrix(ustac_eq, Y_stac)
R2 = equationsToMatrix(ustac_eq, U_sys)
R3 = equationsToMatrix(y_sys_eq, Y_stac)
R4 = equationsToMatrix(y_sys_eq, U_sys)

I=eye(size(D_d0*R1));

A_sys0=A_d0+B_d0*R1*inv(I-D_d0*R1)*C_d0;
B_sys0=B_d0*R1*inv(I-D_d0*R1)*D_d0*R2+B_d0*R2;
C_sys0=R3*inv(I-D_d0*R1)*C_d0;
D_sys0=R3*inv(I-D_d0*R1)*D_d0*R2+R4;
%% 
% Save the GFMI Structure

GFM_GWT.sym_A=A_sys0;
GFM_GWT.sym_B=B_sys0;
GFM_GWT.sym_C=C_sys0;
GFM_GWT.sym_D=D_sys0;

GFM_GWT.U_stac=U_stac;
GFM_GWT.Y_stac=Y_stac;
GFM_GWT.X_stac=X_stac;
GFM_GWT.U_unified=U_sys;
GFM_GWT.Y_unified=Y_sys;

Unified_GFMI = struct(GFM_GWT);
save('Unified_WT_PMSG_GFM_GWT','Unified_GFMI');
