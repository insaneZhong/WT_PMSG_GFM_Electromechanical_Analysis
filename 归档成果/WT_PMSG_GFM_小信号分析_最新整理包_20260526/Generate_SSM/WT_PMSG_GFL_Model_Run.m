%% WT-PMSG Grid-Following Small-Signal Model
% Standard Type-IV GFL comparison case for the GFM-MWT study:
%   - MSC: MPPT torque/current control.
%   - GSC: PLL synchronization, DC-link voltage and reactive-power control.
%   - The turbine, two-mass drivetrain, PMSG, DC link, filter and grid are
%     retained so control-mode comparisons use the same physical plant.

close all
clear all
clc

%% Algebraic coordinate transforms and power measurement
syms delta_pll delta0 i2_d i2_q i2_d0 i2_q0
syms Vpcc_D Vpcc_Q Vpcc_D0 Vpcc_Q0 Vc_d Vc_q Vc_d0 Vc_q0 p_m q_m

R_delta0 = [cos(delta0) -sin(delta0);
            sin(delta0)  cos(delta0)];
U_delta0 = [-sin(delta0) -cos(delta0);
             cos(delta0) -sin(delta0)];

i2_DQ_eq = R_delta0*[i2_d; i2_q] + U_delta0*[i2_d0; i2_q0]*delta_pll;
Vpcc_dq_eq = R_delta0'*[Vpcc_D; Vpcc_Q] + U_delta0'*[Vpcc_D0; Vpcc_Q0]*delta_pll;
p_m_eq = (3/2)*(Vc_d0*i2_d + Vc_q0*i2_q + i2_d0*Vc_d + i2_q0*Vc_q);
q_m_eq = (3/2)*(Vc_q0*i2_d - Vc_d0*i2_q - i2_q0*Vc_d + i2_d0*Vc_q);

%% 1) Grid model
syms i2_D i2_Q Vg_D Vg_Q ig_D ig_Q Vcs_D Vcs_Q
syms lg rg rs cs wn

x_Grid = [ig_D; ig_Q; Vcs_D; Vcs_Q];
e_Grid = [Vpcc_D; Vpcc_Q];
u_Grid = [i2_D; i2_Q; Vg_D; Vg_Q];
y_Grid = [Vpcc_D; Vpcc_Q];

f_Grid = [
    (Vpcc_D - Vg_D - rg*ig_D + wn*lg*ig_Q)/lg;
    (Vpcc_Q - Vg_Q - rg*ig_Q - wn*lg*ig_D)/lg;
    (i2_D - ig_D)/cs + wn*Vcs_Q;
    (i2_Q - ig_Q)/cs - wn*Vcs_D];
g_Grid = [
    Vcs_D + rs*(i2_D - ig_D) - Vpcc_D;
    Vcs_Q + rs*(i2_Q - ig_Q) - Vpcc_Q];
h_Grid = y_Grid;

[A_Grid, B_Grid, C_Grid, D_Grid] = local_dae_ss(f_Grid, g_Grid, h_Grid, x_Grid, e_Grid, u_Grid);

%% 2) LCL filter model in PLL-oriented dq coordinates
syms Vi_d Vi_q i1_d i1_q Vcf_d Vcf_q ig_d ig_q Vpcc_d Vpcc_q
syms lf1 rf1 rd cf lf2 rf2 w

x_LCL = [i1_d; i1_q; Vcf_d; Vcf_q; i2_d; i2_q];
e_LCL = [Vc_d; Vc_q];
u_LCL = [Vi_d; Vi_q; Vpcc_d; Vpcc_q];
y_LCL = [i1_d; i1_q; Vc_d; Vc_q; i2_d; i2_q];

f_LCL = [
    (Vi_d - Vc_d - rf1*i1_d + lf1*w*i1_q)/lf1;
    (Vi_q - Vc_q - rf1*i1_q - lf1*w*i1_d)/lf1;
    (i1_d - i2_d)/cf + w*Vcf_q;
    (i1_q - i2_q)/cf - w*Vcf_d;
    (Vc_d - Vpcc_d - rf2*i2_d + lf2*w*i2_q)/lf2;
    (Vc_q - Vpcc_q - rf2*i2_q - lf2*w*i2_d)/lf2];
g_LCL = [
    Vcf_d + rd*(i1_d - i2_d) - Vc_d;
    Vcf_q + rd*(i1_q - i2_q) - Vc_q];
h_LCL = y_LCL;

[A_LCL, B_LCL, C_LCL, D_LCL] = local_dae_ss(f_LCL, g_LCL, h_LCL, x_LCL, e_LCL, u_LCL);

%% 3) Grid-side converter PWM and calculation delay
syms Vm_d Vm_q x_del1 x_del2 x_del3 x_del4 x_del5 x_del6 td

x_Del = [x_del1; x_del2; x_del3; x_del4; x_del5; x_del6];
u_Del = [Vm_d; Vm_q];
y_Del = [Vi_d; Vi_q];
f_Del = [
    x_del2;
    x_del3;
    Vm_d - (12/td)*x_del3 - (60/td^2)*x_del2 - (120/td^3)*x_del1;
    x_del5;
    x_del6;
    Vm_q - (12/td)*x_del6 - (60/td^2)*x_del5 - (120/td^3)*x_del4];
h_Del = [
    (24/td)*x_del3 - Vm_d + (240/td^3)*x_del1;
    (24/td)*x_del6 - Vm_q + (240/td^3)*x_del4];
[A_Del, B_Del, C_Del, D_Del] = local_ss(f_Del, h_Del, x_Del, u_Del);

%% 4) GSC current controller
syms i1_d_ref i1_q_ref U_id U_iq gamma_id gamma_iq
syms k_pi k_ii beta_v

x_CC = [gamma_id; gamma_iq];
e_CC = [U_id; U_iq];
u_CC = [i1_d_ref; i1_q_ref; i1_d; i1_q; Vc_d; Vc_q];
y_CC = [Vm_d; Vm_q];
f_CC = [i1_d_ref - i1_d; i1_q_ref - i1_q];
g_CC = [
    k_pi*(i1_d_ref - i1_d) + k_ii*gamma_id - U_id;
    k_pi*(i1_q_ref - i1_q) + k_ii*gamma_iq - U_iq];
h_CC = [
    U_id + beta_v*Vc_d - wn*lf1*i1_q;
    U_iq + beta_v*Vc_q + wn*lf1*i1_d];
[A_CC, B_CC, C_CC, D_CC] = local_dae_ss(f_CC, g_CC, h_CC, x_CC, e_CC, u_CC);

%% 5) PLL synchronization of the grid-following converter
% The q-axis PCC voltage drives the PLL. Its angle perturbation replaces
% the VSG angle state used by the grid-forming comparison model.
syms v_pll_q gamma_pll omega_pll
syms k_p_pll k_i_pll

x_PLL = [gamma_pll; delta_pll];
u_PLL = v_pll_q;
y_PLL = [delta_pll; omega_pll];
f_PLL = [
    v_pll_q;
    k_p_pll*v_pll_q + k_i_pll*gamma_pll];
h_PLL = [
    delta_pll;
    k_p_pll*v_pll_q + k_i_pll*gamma_pll];
[A_PLL, B_PLL, C_PLL, D_PLL] = local_ss(f_PLL, h_PLL, x_PLL, u_PLL);

%% 6) GSC DC-link voltage outer loop
% In GFL operation an increase of v_dc must increase exported active
% current, hence the positive (v_dc - vdc_ref) sign differs from MSC-DVC.
syms vdc_ref v_dc gamma_vdc_gfl
syms k_pdc_gfl k_idc_gfl

x_GDVC = gamma_vdc_gfl;
u_GDVC = [vdc_ref; v_dc];
y_GDVC = i1_d_ref;
f_GDVC = v_dc - vdc_ref;
h_GDVC = k_pdc_gfl*(v_dc - vdc_ref) + k_idc_gfl*gamma_vdc_gfl;
[A_GDVC, B_GDVC, C_GDVC, D_GDVC] = local_ss(f_GDVC, h_GDVC, x_GDVC, u_GDVC);

%% 7) GSC reactive-power outer loop
% With q_m ~= -1.5*Vc_d*i_q around Vc_q=0, a negative q-current
% command corresponds to increasing positive reactive-power injection.
syms q_ref q_m gamma_q_gfl
syms k_pq_gfl k_iq_gfl

x_GQ = gamma_q_gfl;
u_GQ = [q_ref; q_m];
y_GQ = i1_q_ref;
f_GQ = q_ref - q_m;
h_GQ = -(k_pq_gfl*(q_ref - q_m) + k_iq_gfl*gamma_q_gfl);
[A_GQ, B_GQ, C_GQ, D_GQ] = local_ss(f_GQ, h_GQ, x_GQ, u_GQ);

%% 8) Two-mass drivetrain with aerodynamic torque linearization
% T_m is an additive torque disturbance; wind-speed and pitch perturbations
% enter through the linearized aerodynamic torque around the MPPT point.
syms omega_t omega_g theta_tw T_m T_aero T_e T_sh v_w beta
syms J_t J_g K_sh D_sh D_t D_g D_aero K_v_aero K_beta_aero

x_TMS = [omega_t; omega_g; theta_tw];
u_TMS = [T_m; v_w; beta; T_e];
y_TMS = [omega_t; omega_g; theta_tw; T_sh; T_aero];
T_sh_eq = K_sh*theta_tw + D_sh*(omega_t - omega_g);
T_aero_eq = -D_aero*omega_t + K_v_aero*v_w + K_beta_aero*beta;
f_TMS = [
    (T_m + T_aero_eq - T_sh_eq - D_t*omega_t)/J_t;
    (T_sh_eq - T_e - D_g*omega_g)/J_g;
    omega_t - omega_g];
h_TMS = [omega_t; omega_g; theta_tw; T_sh_eq; T_aero_eq];
[A_TMS, B_TMS, C_TMS, D_TMS] = local_ss(f_TMS, h_TMS, x_TMS, u_TMS);

%% 9) PMSG electrical dynamics
syms i_m_d i_m_q v_m_d v_m_q p_msc
syms R_s L_d L_q psi_f n_p omega_g0 i_m_d0 i_m_q0

x_PMSG = [i_m_d; i_m_q];
u_PMSG = [v_m_d; v_m_q; omega_g];
y_PMSG = [i_m_d; i_m_q; T_e; p_msc];
T_e0 = (3/2)*n_p*(psi_f*i_m_q0 + (L_d - L_q)*i_m_d0*i_m_q0);
T_e_eq = (3/2)*n_p*(psi_f*i_m_q + (L_d - L_q)*(i_m_q0*i_m_d + i_m_d0*i_m_q));
p_msc_eq = omega_g0*T_e_eq + T_e0*omega_g;
f_PMSG = [
    (v_m_d - R_s*i_m_d + n_p*L_q*(omega_g0*i_m_q + i_m_q0*omega_g))/L_d;
    (v_m_q - R_s*i_m_q - n_p*(omega_g0*L_d*i_m_d + (L_d*i_m_d0 + psi_f)*omega_g))/L_q];
h_PMSG = [i_m_d; i_m_q; T_e_eq; p_msc_eq];
[A_PMSG, B_PMSG, C_PMSG, D_PMSG] = local_ss(f_PMSG, h_PMSG, x_PMSG, u_PMSG);

%% 10) Machine-side current controller
syms i_m_d_ref i_m_q_ref gamma_md gamma_mq
syms k_pm k_im

x_MSC = [gamma_md; gamma_mq];
u_MSC = [i_m_d_ref; i_m_q_ref; i_m_d; i_m_q; omega_g];
y_MSC = [v_m_d; v_m_q];
f_MSC = [i_m_d_ref - i_m_d; i_m_q_ref - i_m_q];
h_MSC = [
    k_pm*(i_m_d_ref - i_m_d) + k_im*gamma_md - n_p*(omega_g0*L_q*i_m_q + L_q*i_m_q0*omega_g);
    k_pm*(i_m_q_ref - i_m_q) + k_im*gamma_mq + n_p*(omega_g0*L_d*i_m_d + (L_d*i_m_d0 + psi_f)*omega_g)];
[A_MSC, B_MSC, C_MSC, D_MSC] = local_ss(f_MSC, h_MSC, x_MSC, u_MSC);

%% 11) MSC MPPT torque reference
% Around an MPPT operating point T_ref = Kopt*omega_g^2. The incremental
% reference slope k_t_mppt = dT_ref/domega_g keeps the mechanical-side
% baseline consistent with conventional Type-IV GFL operation.
syms p_ref k_t_mppt

x_MPPT = sym(zeros(0, 1));
u_MPPT = [p_ref; omega_g];
y_MPPT = [i_m_d_ref; i_m_q_ref];
h_MPPT = [
    0;
    (p_ref/omega_g0 + k_t_mppt*omega_g)/(1.5*n_p*psi_f)];
A_MPPT = sym(zeros(0, 0));
B_MPPT = sym(zeros(0, 2));
C_MPPT = sym(zeros(2, 0));
D_MPPT = jacobian(h_MPPT, u_MPPT);

%% 12) DC-link energy balance
syms C_dc V_dc0

x_DC = v_dc;
u_DC = [p_msc; p_m];
y_DC = v_dc;
f_DC = (p_msc - p_m)/(C_dc*V_dc0);
h_DC = v_dc;
[A_DC, B_DC, C_DC, D_DC] = local_ss(f_DC, h_DC, x_DC, u_DC);

%% Component connection method
A_d0 = blkdiag(A_PLL, A_GDVC, A_GQ, A_CC, A_Del, A_LCL, A_Grid, A_TMS, A_PMSG, A_MSC, A_MPPT, A_DC);
B_d0 = blkdiag(B_PLL, B_GDVC, B_GQ, B_CC, B_Del, B_LCL, B_Grid, B_TMS, B_PMSG, B_MSC, B_MPPT, B_DC);
C_d0 = blkdiag(C_PLL, C_GDVC, C_GQ, C_CC, C_Del, C_LCL, C_Grid, C_TMS, C_PMSG, C_MSC, C_MPPT, C_DC);
D_d0 = blkdiag(D_PLL, D_GDVC, D_GQ, D_CC, D_Del, D_LCL, D_Grid, D_TMS, D_PMSG, D_MSC, D_MPPT, D_DC);

X_stac = [x_PLL; x_GDVC; x_GQ; x_CC; x_Del; x_LCL; x_Grid; x_TMS; x_PMSG; x_MSC; x_MPPT; x_DC];
U_stac = [u_PLL; u_GDVC; u_GQ; u_CC; u_Del; u_LCL; u_Grid; u_TMS; u_PMSG; u_MSC; u_MPPT; u_DC];
Y_stac = [y_PLL; y_GDVC; y_GQ; y_CC; y_Del; y_LCL; y_Grid; y_TMS; y_PMSG; y_MSC; y_MPPT; y_DC];

ustac_eq = [
    % PLL
    Vpcc_dq_eq(2);
    % GSC DC voltage controller
    vdc_ref;
    v_dc;
    % GSC reactive power controller
    q_ref;
    q_m_eq;
    % GSC current controller
    i1_d_ref;
    i1_q_ref;
    i1_d;
    i1_q;
    Vc_d;
    Vc_q;
    % Delay and LCL filter
    Vm_d;
    Vm_q;
    Vi_d;
    Vi_q;
    Vpcc_dq_eq(1);
    Vpcc_dq_eq(2);
    % Network
    i2_DQ_eq(1);
    i2_DQ_eq(2);
    Vg_D;
    Vg_Q;
    % Two-mass drivetrain and PMSG
    T_m;
    v_w;
    beta;
    T_e;
    v_m_d;
    v_m_q;
    omega_g;
    % MSC current controller
    i_m_d_ref;
    i_m_q_ref;
    i_m_d;
    i_m_q;
    omega_g;
    % MPPT torque reference
    p_ref;
    omega_g;
    % DC-link dynamics
    p_msc;
    p_m_eq];

U_sys = [p_ref; q_ref; Vg_D; Vg_Q; T_m; v_w; beta; vdc_ref];
y_sys_eq = [i1_d; i1_q; Vc_d; Vc_q; p_m_eq; q_m_eq; delta_pll; omega_pll; ...
            omega_t; omega_g; theta_tw; T_sh; T_aero; T_e; p_msc; v_dc; i_m_d; i_m_q];
Y_sys = [i1_d; i1_q; Vc_d; Vc_q; p_m; q_m; delta_pll; omega_pll; ...
         omega_t; omega_g; theta_tw; T_sh; T_aero; T_e; p_msc; v_dc; i_m_d; i_m_q];

R1 = equationsToMatrix(ustac_eq, Y_stac);
R2 = equationsToMatrix(ustac_eq, U_sys);
R3 = equationsToMatrix(y_sys_eq, Y_stac);
R4 = equationsToMatrix(y_sys_eq, U_sys);

I = eye(size(D_d0*R1));
A_sys0 = A_d0 + B_d0*R1/(I - D_d0*R1)*C_d0;
B_sys0 = B_d0*R1/(I - D_d0*R1)*D_d0*R2 + B_d0*R2;
C_sys0 = R3/(I - D_d0*R1)*C_d0;
D_sys0 = R3/(I - D_d0*R1)*D_d0*R2 + R4;

%% Save unified GFL model
GFL.sym_A = A_sys0;
GFL.sym_B = B_sys0;
GFL.sym_C = C_sys0;
GFL.sym_D = D_sys0;
GFL.U_stac = U_stac;
GFL.Y_stac = Y_stac;
GFL.X_stac = X_stac;
GFL.U_unified = U_sys;
GFL.Y_unified = Y_sys;

Unified_GFMI = struct(GFL);
save('Unified_WT_PMSG_GFL', 'Unified_GFMI');

%% Local linearization helpers
function [A, B, C, D] = local_ss(f, h, x, u)
A = jacobian(f, x);
B = jacobian(f, u);
C = jacobian(h, x);
D = jacobian(h, u);
end

function [A, B, C, D] = local_dae_ss(f, g, h, x, e, u)
J_fx = jacobian(f, x); J_fe = jacobian(f, e); J_fu = jacobian(f, u);
J_gx = jacobian(g, x); J_ge = jacobian(g, e); J_gu = jacobian(g, u);
J_hx = jacobian(h, x); J_he = jacobian(h, e); J_hu = jacobian(h, u);
A = J_fx - J_fe/J_ge*J_gx;
B = J_fu - J_fe/J_ge*J_gu;
C = J_hx - J_he/J_ge*J_gx;
D = J_hu - J_he/J_ge*J_gu;
end
