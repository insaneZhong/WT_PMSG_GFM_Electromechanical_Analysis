%% Parameter Classification Notes
% [USER]     Must be set according to the target wind turbine, converter, or grid case.
% [DERIVED]  Calculated from other parameters. Do not tune directly unless the formula changes.
% [DESIGN]   Controller design parameter. It should be tuned and verified by eigenvalue/time-domain tests.
% [TEMP]     Temporary 1 MW-class value used to run the WT-PMSG workflow. Replace before final study.

%% Power Injection Setpoints
P_inj = 1.0;                 % [USER] Active power injection operating point (pu of S_base). Use 1.0 for strict 1 MW same-object alignment.
Q_inj = 0.0;                 % [USER] Reactive power injection operating point (pu of S_base). Use 0 for baseline no-disturbance steady test.

%% General System Parameters
f_base = 50;                 % [USER] Grid nominal frequency (Hz), normally 50 Hz in China.
wn = 2 * pi * f_base;        % [DERIVED] Electrical base angular frequency (rad/s).
w = wn;                      % [DERIVED] Synchronous reference-frame angular speed used by dq models (rad/s).
f_sample = 20e3;             % [USER/DESIGN] Control sampling frequency (Hz). Match Simulink/RT-LAB setup.
f_step = 100e3;              % [USER/DESIGN] Simulation calculation/update frequency (Hz). Match EMT step size.

%% Inverter Ratings
Vdc = 1.5e3;                 % [USER] Rated DC-link voltage (V). Re-check against nonlinear VdcRef/initial voltage before final alignment.
if exist('Vdc_override', 'var')
    Vdc = Vdc_override;      % [USER/OVERRIDE] Optional reproducibility override for DC-link voltage sensitivity checks.
end
V_LL = 0.69e3;               % [USER] Rated AC line-to-line voltage (V), aligned with nonlinear model setup.
S_base = 1e6;                % [TEMP/USER] Rated apparent power (VA). Current value is for a 1 MW-class test case.
Zb = V_LL^2 / S_base;        % [DERIVED] Base impedance (Ohms), used for pu-to-SI conversion.
Lb = Zb / wn;                % [DERIVED] Base inductance (H), used for grid inductance conversion.
%% Grid Model Parameters
SCR = 5;                     % [USER/SWEEP] Short-circuit ratio at PCC. Sweep this to study weak/strong grid.
XR = 5;                      % [USER/SWEEP] Grid X/R ratio. Sweep this to study resistive/inductive grid effect.
rgpu = 1 / (SCR * sqrt(1 + XR^2));  % [DERIVED] Grid resistance in pu from SCR and X/R.
lgpu = XR * rgpu;            % [DERIVED] Grid reactance/inductance term in pu from X/R relation.
rg = rgpu * Zb;              % [DERIVED] Grid resistance in Ohms.
lg = lgpu * Lb;              % [DERIVED] Grid inductance in H.

%% Switching and Control Delays
fsw = 4e3;                   % [USER/DESIGN] PWM switching frequency (Hz), aligned with C controller Ts=250 us.
td = 1.5 / fsw;              % [DERIVED/DESIGN] Equivalent digital/PWM delay used by inner-loop model (s).

%% LCL Filter Parameters
lf1 = 120e-6;                % [USER] Converter-side LCL inductor (H), from nonlinear model constant GRID_FILTER__LS.
rf1 = 8.3e-3;                % [USER] Converter-side inductor resistance (Ohms), aligned to nonlinear LCL branch.
lf2 = 120e-6;                % [USER] Grid-side LCL inductor (H), equivalent branch used in nonlinear model.
rf2 = 8.3e-3;                % [USER] Grid-side inductor resistance (Ohms), same-order as converter-side branch.
cf = 55e-6;                  % [USER] LCL filter capacitor (F), from nonlinear model constant GRID_FILTER__C.
rd = 0.1;                    % [USER] Passive damping resistor (Ohms), aligned to nonlinear capacitor branch damping.
L_t = lf1 + lf2 + lg;        % [DERIVED] Total series inductance seen by converter and grid (H).
f_res = (1/(2*pi)) * sqrt((lf1 + lf2 + lg) / (lf1 *(lf2+lg)* cf)); % [DERIVED] LCL resonance frequency with grid inductance (Hz).
f_ares = (1/(2*pi)) * sqrt(1 / (lf1 * cf)); % [DERIVED] Anti-resonance frequency approximation (Hz).
%% Current Control Parameters
Tic = 2 * td;                % [DESIGN] Current-loop target time constant (s). Usually tied to delay/bandwidth.
k_pi = lf1 / Tic;            % [DERIVED/DESIGN] Current-loop proportional gain.
k_ii = rf1 / Tic;            % [DERIVED/DESIGN] Current-loop integral gain.
beta_v = 0.5;                % [DESIGN/SWEEP] Capacitor-voltage feedforward/feedback factor in current controller.

%% Voltage Control Parameters
wm = 2000;                   % [DESIGN] Voltage-loop target bandwidth parameter (rad/s). Tune with stability margin.
k_pv = cf * wm;              % [DERIVED/DESIGN] Voltage-loop proportional gain.
k_iv = Tic * cf * wm^3;      % [DERIVED/DESIGN] Voltage-loop integral gain.
beta_i = 0.8;                % [DESIGN/SWEEP] Grid-side current feedback/feedforward factor in voltage loop.

%% Frequency & Voltage Droop Requirements
mp = 2 * pi * (0.01 * f_base) / (2 * S_base); % [DESIGN] Active-power/frequency droop coefficient for 1% frequency droop.
T_RoCoF = 0.5;               % [DESIGN] RoCoF design limit or virtual inertia design requirement (Hz/s).
V_drop = 0.05 * sqrt(2) * V_LL; % [DESIGN] Voltage droop target, here 5% phase-voltage peak value (V).

%% Fictitious Shunt Parameters (for stability augmentation)
damping_cs = 1;              % [DESIGN] Damping ratio for fictitious shunt branch used in grid model regularization.
wn_cs = 2 * pi * 200e3;      % [DESIGN] Natural frequency of dummy shunt branch (rad/s), set high to avoid low-frequency impact.
cs = 1 / (lg * wn_cs^2);     % [DERIVED] Fictitious shunt capacitance (F).
rs = 2 * damping_cs * sqrt(lg / cs); % [DERIVED] Fictitious shunt damping resistance (Ohms).
%% APC - Virtual Synchronous Generator (VSG) Control
h = T_RoCoF * (1 / (2 * mp * wn)); % [DERIVED/DESIGN/SWEEP] Virtual inertia coefficient. Sweep to study VSG low-frequency mode.

%% Reactive Power Control (RPC) - PI with Feedforward
k_pq = V_drop / (2 * S_base); % [DERIVED/DESIGN] Reactive-power/voltage droop proportional gain.
k_iq = 1e-3;                 % [DESIGN/TEMP] Reactive-power PI integral gain. Tune for voltage/reactive-power dynamics.

%% CG-VSG Parameters (Control Gain Virtual Synchronous Generator)
Kg = (3 * (V_LL/sqrt(3))^2) / (2 * pi * f_base * lg); % [DERIVED] Grid-dependent control gain used by CG-VSG formulas.

Alpha_PC = T_RoCoF;          % [DERIVED/DESIGN] CG-VSG alpha parameter from RoCoF requirement.
Beta_PC = (T_RoCoF^2) * (((Kg^2 * mp^2) / T_RoCoF) - 1)^(1/3); % [DERIVED] CG-VSG beta parameter.
Gamma_PC = 1 / (((Kg^2 * mp^2) / T_RoCoF) - 1)^(1/3); % [DERIVED] CG-VSG gamma parameter.

a = Alpha_PC;                % [DERIVED] CG-VSG coefficient a.
b = (Beta_PC * Gamma_PC) / (Beta_PC + Gamma_PC - Alpha_PC); % [DERIVED] CG-VSG coefficient b.
c = (Beta_PC + Gamma_PC - Alpha_PC) / mp; % [DERIVED] CG-VSG coefficient c.

%% Droop Low-Pass Filter
w_cp = 0.5;                  % [DESIGN] Power measurement/control low-pass cutoff frequency (rad/s).

%% Virtual Impedance
lv = 5e-6;                   % [DESIGN/TEMP] Virtual inductance in grid-side converter control (H).
rv = 0;                      % [DESIGN/TEMP] Virtual resistance in grid-side converter control (Ohms).

%% WT-PMSG Electromechanical Parameters
% First-pass 1 MW class values for eigenvalue workflow validation.
% Replace these with the target turbine data before final paper/simulation conclusions.
n_p = 20;                    % [USER] PMSG pole pairs, aligned with nonlinear controller macro MOTOR_POLE_PAIR.
omega_g0 = 2*pi*30/60;       % [USER] Generator mechanical operating speed (rad/s), aligned with Grid_Forming_PMSG annotation and original speedin (=pi rad/s, 30 rpm).
R_s = 0.0122;                % [USER] PMSG stator phase resistance (Ohms), from nonlinear controller macro MOTOR_RS.
L_d = 1.05e-3;               % [USER] PMSG d-axis stator inductance (H), aligned with PMSM mask.
L_q = 1.05e-3;               % [USER] PMSG q-axis stator inductance (H), aligned with PMSM mask.
psi_f = 8.64;                % [USER] PM flux linkage (Wb), from nonlinear model PMSM1 Flux.
i_m_d0 = 0;                  % [USER/DESIGN] d-axis current operating point (A). Often set to zero for surface-PMSG/MTPA simplification.
T_e0 = P_inj * S_base / omega_g0; % [DERIVED] Electromagnetic torque operating point (N*m), aligned with 1 MW / omega_g0.
i_m_q0 = T_e0 / (1.5 * n_p * psi_f); % [DERIVED] q-axis current operating point from torque equation (A).

P_wt_rated = S_base;         % [USER] Wind turbine rated power used for steady operating point consistency.
omega_m0 = omega_g0;         % [USER] Same-object alignment: use the same generator-side operating speed as nonlinear model.
J_g = 1.8375e5;              % [USER] Generator inertia (kg*m^2), from nonlinear PMSM/mechanical setup.
J_t = 8.0 * J_g;             % [USER] Turbine inertia (kg*m^2), aligned with nonlinear two-mass assumption.
f_sh_init_guess = 2.0;       % [USER] Initial shaft mode guess (Hz), used to initialize K_sh/D_sh consistently.
zeta_sh_init_guess = 0.01;   % [USER] Initial shaft damping ratio guess.
J_eq = J_t * J_g / (J_t + J_g);               % [DERIVED] Equivalent two-mass inertia for torsional mode.
w_sh_init_guess = 2*pi*f_sh_init_guess;       % [DERIVED] Shaft modal angular frequency guess.
K_sh = J_eq * w_sh_init_guess^2;              % [DERIVED/USER] Shaft stiffness aligned with nonlinear initialization.
D_sh = 2*zeta_sh_init_guess*w_sh_init_guess*J_eq; % [DERIVED/USER] Shaft damping aligned with nonlinear initialization.

% Linear aerodynamic restoring path at the below-rated MPPT operating point.
% Around the optimum tip-speed ratio, dTm/domega_t = -T0/omega0 and
% dTm/dvw = 3*T0/vw0. The pitch derivative must be replaced for above-rated studies.
v_w0 = 12;                   % [USER] Baseline wind speed (m/s) for the current MPPT operating point.
T_aero0 = T_e0;              % [DERIVED/USER] Initial aerodynamic torque balance for no-acceleration equilibrium.
theta_tw0 = T_e0 / K_sh;     % [DERIVED/USER] Initial shaft twist at equilibrium.
D_aero = T_aero0 / omega_m0; % [DERIVED/USER] Aerodynamic damping coefficient around operating point.
K_v_aero = 3*T_e0 / v_w0;   % [DERIVED/TEMP] Wind-speed-to-aerodynamic-torque gain (N*m/(m/s)).
K_beta_aero = 0;             % [TEMP/USER] Pitch-to-torque gain (N*m/rad); zero in below-rated operation.
D_t = 0.005 * D_aero;        % [TEMP/USER] Turbine-side viscous/self damping (N*m*s/rad).
D_g = 0.005 * D_aero;        % [TEMP/USER] Generator-side viscous/self damping (N*m*s/rad).
k_p_mppt = 3*P_wt_rated / omega_g0; % [DERIVED] GFM-MWT active-power MPPT slope dPref/domega_g (W/(rad/s)) at 1 MW operating point.

Tmsc = 2 * td;               % [DESIGN] Machine-side converter current-loop target time constant (s).
k_pm = L_d / Tmsc;           % [DERIVED/DESIGN] MSC current-loop proportional gain.
k_im = R_s / Tmsc;           % [DERIVED/DESIGN] MSC current-loop integral gain.
k_pdc = 0.5;                 % [TEMP/DESIGN] DC-link voltage controller proportional gain. Needs retuning for target converter.
k_idc = 50;                  % [TEMP/DESIGN] DC-link voltage controller integral gain. Needs retuning for target converter.
k_ff_msc_typec = i_m_q0 / P_wt_rated; % [DERIVED/DESIGN/SWEEP] MSC-DVC Type-c active-power feedforward gain (A/W).
C_dc = 1.5e-3;               % [USER] DC-link capacitance (F). Current frozen small-signal baseline; nonlinear Cd is checked separately.
if exist('C_dc_override', 'var')
    C_dc = C_dc_override;    % [USER/OVERRIDE] Optional reproducibility override for nonlinear-alignment checks.
end
V_dc0 = Vdc;                 % [DERIVED] DC-link voltage operating point (V).

%% Shaft Torsional Damping Controller
% The damping path is added in WT_PMSG_VSG_Damping_Model.mlx.
% Current values are preliminary and tuned around the present 2 Hz torsional mode.
% Re-tune them after replacing the temporary WT-PMSG parameters or changing SCR/XR range.
f_damp = f_sh_init_guess;     % [DESIGN/TEMP] Target torsional mode frequency (Hz), initialized from shaft-mode estimate.
w_damp = 2 * pi * f_damp;     % [DERIVED] Band-pass filter center angular frequency (rad/s).
zeta_damp = 0.20;             % [DESIGN/TEMP] Band-pass damping ratio / bandwidth factor. Larger means wider pass band.
T_lead_damp = 1 / w_damp;     % [DESIGN/TEMP] Lead compensation time constant (s), initial value matched to target frequency.
alpha_lead_damp = 0.30;       % [DESIGN/TEMP] Lead ratio, 0 < alpha < 1. Determines phase lead magnitude.
K_damp = -5e6;                % [DESIGN/TEMP] Auxiliary active-power damping gain. Negative is damping direction for current interconnection.

%% WT-PMSG Grid-Following Comparison Controller
% Used by WT_PMSG_GFL_Model.mlx. These values define the standard GFL
% baseline: MSC follows MPPT torque, while GSC regulates Vdc/Q through a
% PLL-oriented current controller. Re-tune the outer-loop gains after the
% target turbine and converter ratings replace the temporary 1 MW data.
k_p_pll = 0.5;                % [DESIGN/TEMP] SRF-PLL proportional gain relating PCC q-voltage error to frequency.
k_i_pll = 50;                 % [DESIGN/TEMP] SRF-PLL integral gain. Sweep bandwidth when studying PLL/torsional coupling.
k_pdc_gfl = 0.5;              % [DESIGN/TEMP] GSC DC-link outer-loop proportional gain producing d-axis current reference.
k_idc_gfl = 50;               % [DESIGN/TEMP] GSC DC-link outer-loop integral gain producing d-axis current reference.
k_pq_gfl = 1e-4;              % [DESIGN/TEMP] GSC reactive-power outer-loop proportional gain producing q-axis current reference.
k_iq_gfl = 1e-3;              % [DESIGN/TEMP] GSC reactive-power outer-loop integral gain producing q-axis current reference.
k_t_mppt = 2*T_e0/omega_g0;  % [DERIVED] Linear MPPT torque slope from T_ref = Kopt*omega_g^2 at the operating point.

%% WT-PMSG Grid-Forming Generator-Side MPPT Controller
% Used by WT_PMSG_GFM_GWT_Model.mlx. The GSC remains a voltage-forming VSG
% and regulates the DC link through an active-power-reference correction.
w_dc_gwt = 2*pi*0.1;          % [DESIGN/TEMP] GSC DC-voltage-loop natural frequency (rad/s); chosen within the screened stable band.
zeta_dc_gwt = 0.7;            % [DESIGN/TEMP] Desired GSC DC-voltage-loop damping ratio.
k_pdc_gwt = 2*zeta_dc_gwt*w_dc_gwt*C_dc*V_dc0; % [DERIVED/DESIGN] DC-to-power proportional gain (W/V).
k_idc_gwt = w_dc_gwt^2*C_dc*V_dc0;             % [DERIVED/DESIGN] DC-to-power integral gain (W/(V*s)).

%% Save Parameters for Simulink
save('Parameters.mat', 'f_base', 'f_sample','f_step', 'wn', 'w','V_LL', 'S_base', 'Zb', 'Vdc', 'Lb', ...
     'fsw','f_res', 'f_ares', 'f_ares','td', 'lf1', 'rf1', 'lf2', 'rf2','cf', 'rd', 'SCR', 'XR', 'rgpu', 'lgpu', 'rg', 'lg', 'k_pi', 'k_ii', ...
     'beta_v', 'k_pv','k_iv','beta_i','Tic', 'mp', 'h', 'k_pq', 'cs','rs', 'k_iq', 'P_inj', 'Q_inj','a','b','c','w_cp','lv','rv', ...
     'n_p', 'omega_g0', 'R_s', 'L_d', 'L_q', 'psi_f', 'i_m_d0', 'i_m_q0', ...
     'P_wt_rated', 'omega_m0', 'J_t', 'J_g', 'K_sh', 'D_sh', 'T_aero0', 'theta_tw0', ...
     'v_w0', 'D_aero', 'K_v_aero', 'K_beta_aero', 'D_t', 'D_g', 'k_p_mppt', ...
     'k_pm', 'k_im', 'k_pdc', 'k_idc', 'k_ff_msc_typec', 'C_dc', 'V_dc0', ...
     'f_damp', 'w_damp', 'zeta_damp', 'T_lead_damp', 'alpha_lead_damp', 'K_damp', ...
     'k_p_pll', 'k_i_pll', 'k_pdc_gfl', 'k_idc_gfl', 'k_pq_gfl', 'k_iq_gfl', 'k_t_mppt', ...
     'w_dc_gwt', 'zeta_dc_gwt', 'k_pdc_gwt', 'k_idc_gwt');

clear all
