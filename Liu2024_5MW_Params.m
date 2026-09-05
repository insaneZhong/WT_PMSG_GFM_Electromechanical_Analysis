function p = Liu2024_5MW_Params()
%LIU2024_5MW_PARAMS Unified target for the 5 MW GFM-PMSG reproduction.
% Primary source: Liu et al., IEEE TEC 2024, DOI 10.1109/TEC.2024.3394753.
% Items marked placeholder are retained only where the paper table does not
% define the parameter and must not be presented as sourced paper data.

p = struct();
p.object_name = '5MW_two_mass_GFM_PMSG_EMT_blueprint_consistent';
p.S_base = 5.0e6;
p.P_wt_rated = 5.0e6;
p.Vdc = 1500;
p.V_LL = 690;
p.V_phase_peak = sqrt(2)*p.V_LL/sqrt(3); % physical phase-to-neutral peak
p.V_control_d = (2/3)*p.V_LL;            % legacy line-voltage dq scaling
p.f_base = 50;
p.omega_base = 2*pi*p.f_base;
% Liu Table I rated switching frequency.
p.fsw = 10.0e3;       % diagnostic: separate LCL resonance from 5 kHz delay
p.control_Ts = 4e-6;
p.sim_fixed_step = 1e-6;
p.transport_delay = 4e-6;

p.SCR = 4;
p.Rg_pu = 0.02;
p.Lg_pu = 0.25;
p.Lf_pu = 0.15;
p.Rf_pu = 0.005;
p.Cf = 5*55e-6;     % scaled from the 1 MW blueprint to preserve per-unit C
p.Rd = 0.1;         % PLACEHOLDER retained from the runnable model
p.Cdc = 0.3;        % Liu Table I, not a 1 MW capacity scaling

p.omega_m0 = 1.27;
p.omega_g0 = p.omega_m0;
p.v_w0 = 12.20;     % approximately 12 m/s; retains a small pitch-control margin
p.rated_wind_speed = 11.487;
p.rotor_radius = 63;
p.air_density = 1.225;
p.rotor_area = pi*p.rotor_radius^2;
p.lambda_opt = 7.55;
p.Copt = wind_cp_5mw(p.lambda_opt,0);
p.K_opt = 0.5*p.air_density*p.rotor_area*p.Copt* ...
    p.rotor_radius^3/p.lambda_opt^3;
p.K_rated_recovery = p.P_wt_rated/p.omega_m0^3;
p.K_speed_recovery_W_per_radps = 7.0e6;
p.mppt_rated_blend_low_pu = 0.75;
p.mppt_rated_blend_high_pu = 0.98;
p.pitch_beta_rated_ff_deg = 0.0;
p.pitch_beta_ff_gain_deg_per_mps = p.pitch_beta_rated_ff_deg/ ...
    (p.v_w0-p.rated_wind_speed);
p.pitch_kp_deg_per_radps = 8.0;
p.pitch_ki_deg_per_rad = 0.02;
p.pitch_rate_deg_per_s = 8.0;
p.pitch_beta_max_deg = 25.0;
p.cp_lambda_bp = 0.1:0.25:15.1;
p.cp_beta_bp = 0:1:25;
[cpLambdaGrid,cpBetaGrid] = ndgrid(p.cp_lambda_bp,p.cp_beta_bp);
p.cp_table = wind_cp_5mw(cpLambdaGrid,cpBetaGrid);

p.R_s = 5.0e-4;
% Liu Table I gives Ldq = 4 mH.  At 5 MW and Vdc = 1500 V that value
% requires about 2.29 kV phase-peak at id = 0, while a linear two-level
% converter can provide only Vdc/sqrt(3) = 0.866 kV.  The switching EMT
% realization therefore uses 1 mH (the validated blueprint order of
% magnitude), giving rated-current voltage margin without changing the
% Liu mechanical, dc-link, grid, or GFM ratings.  Keep this distinction
% explicit when comparing this runnable plant with the exact paper model.
p.L_d_paper = 4.0e-3;
p.L_q_paper = 4.0e-3;
p.L_d = 1.05e-3;
p.L_q = 1.05e-3;
p.electrical_realization = '5 MW EMT realization retaining the validated 1 MW voltage base and PMSG flux convention';
p.psi_f = 8.64;
p.n_p = 20;         % PLACEHOLDER: pole pairs absent from the Liu table

p.H_t = 1.93;
p.H_g = 0.8;
p.K_sh_pu = 280;
p.D_sh_pu = 1.0;

p.I_base = p.S_base/(sqrt(3)*p.V_LL);
% The legacy controller does not use the physical line-current RMS value as
% its dq current base.  Its power calculation is P = 1.5*ud*id with
% ud = V_control_d, hence rated active power requires the following id.
% Keep both quantities explicit to prevent a 1 MW/RMS scaling residue from
% clipping the 5 MW command near 3.5 MW.
p.I_grid_rms_rated = p.I_base;
p.I_control_dq_rated = p.S_base/(1.5*p.V_control_d);
p.Z_base = p.V_LL^2/p.S_base;
p.L_base = p.Z_base/p.omega_base;
p.T_base = p.S_base/p.omega_m0;
p.J_base_per_H = 2*p.S_base/p.omega_m0^2;
p.K_base = p.S_base/p.omega_m0^2;
p.D_base = p.K_base;

p.J_t = p.H_t*p.J_base_per_H;
p.J_g = p.H_g*p.J_base_per_H;
p.K_sh = p.K_sh_pu*p.K_base;
p.D_sh = p.D_sh_pu*p.D_base;
p.J_eq = p.J_t*p.J_g/(p.J_t+p.J_g);
p.f_sh = sqrt(p.K_sh*(1/p.J_t + 1/p.J_g))/(2*pi);
p.zeta_sh = p.D_sh*(1/p.J_t + 1/p.J_g)/(2*(2*pi*p.f_sh));

p.Lf = p.Lf_pu*p.L_base;
p.Rf = p.Rf_pu*p.Z_base;
p.Lg = p.Lg_pu*p.L_base;
p.Rg = p.Rg_pu*p.Z_base;

p.T_e0 = p.P_wt_rated/p.omega_m0;
p.i_m_q0 = p.T_e0/(1.5*p.n_p*p.psi_f);
p.theta_tw0 = p.T_e0/p.K_sh;
p.D_aero = p.T_e0/p.omega_m0;
p.K_v_aero = 3*p.T_e0/p.v_w0;
p.D_t = 0.005*p.D_aero;
p.D_g = 0.005*p.D_aero;

p.VSG_H_paper = 3;
% The runnable controller enables the VSG swing-equation branch through
% ENABLE_VSG_EQUIV_WREF=1.  PLL is used only before the timed GFM takeover.
p.VSG_H = p.VSG_H_paper;
p.VSG_Kp_literature = 0.0104;
% Normal power tracking confirms the implemented swing equation must use
% Pref-Ppcc.  Start the operating-point characterization from a 0.1 percent
% P-f droop stability anchor, then increase it only after the physical SI
% swing equation passes the no-disturbance regression.
p.vsg_startup_power_error_sign = 1;
p.vsg_power_error_sign = 1;
p.pf_droop_fraction = 0.001;
p.mp = 2*pi*(p.pf_droop_fraction*p.f_base)/p.S_base;
p.vsg_startup_mp = 0.2 * 2*pi*(0.01*p.f_base)/(2*p.S_base);
p.vsg_dynamics_transition_start_s = 15.0;
p.vsg_dynamics_transition_duration_s = 5.0;
p.k_pq = 0.001;
p.qv_droop_V_per_var = p.k_pq*p.V_control_d/p.S_base;
p.k_pdc_pu = 0.78;
% Retain the Liu Type-c integral gain; cross-section tests showed that
% reducing it weakened DC-link recovery without removing the mechanical
% Region-2.5/pitch oscillation.
p.k_idc_pu_per_s = 0.85;

% Current C controller uses A/V proportional output and a fractional
% integrator Ui += Ki*Up once per control step.
% Scale the validated blueprint DVC by rated machine current and dc voltage.
% Convert Liu's per-unit Type-c DVC gains to the legacy controller units.
% The legacy integrator is Ui += Ki_fraction*Up once per PWM control step.
p.dvc_kp_A_per_V = p.k_pdc_pu*p.i_m_q0/p.Vdc;
p.dvc_ki_fraction = (p.k_idc_pu_per_s/p.k_pdc_pu)/p.fsw;
p.msc_power_ff_loss_factor = 1.02;
p.msc_power_ff_A_per_W = p.msc_power_ff_loss_factor* ...
    p.i_m_q0/p.P_wt_rated;
p.motor_current_limit = 1.2*p.i_m_q0;
p.grid_current_limit = 1.20*p.I_control_dq_rated;
p.gsc_current_vector_limit = 1.20*p.I_control_dq_rated;
p.gsc_modulation_limit = 0.90;
p.gsc_current_ref_aw_gain = 0.05;
p.gsc_voltage_aw_gain = 0.05;
p.motor_voltage_limit = 0.9*p.Vdc/sqrt(3);
p.grid_voltage_limit = 0.95*p.Vdc/1.5;

% Preserve the validated 1 MW controller bandwidths after plant scaling.
p.motor_current_kp = 1.4*(p.L_d/1.02e-3);
p.motor_current_ki_fraction = 0.00290476*((p.R_s/p.L_d)/(0.0122/1.02e-3));
p.grid_current_kp = 0.16*(p.Lf/120e-6);
p.grid_current_ki_fraction = 0.0172917*(p.Rf/0.0002);
p.grid_voltage_kp = 1.1309733;
p.grid_voltage_ki_fraction = 0.0282743;
p.grid_power_kp = 1.0e-6/5;
p.grid_power_ki = 2.0e-5/5;

p.pref_ramp_slope = 0.50e6; % coordinated commissioning ramp before MPPT/pitch
p.presyn_switch_time = 1.75;
p.gfm_enable_time = 2.25;
p.dvc_enable_time = 1.25; % establish the DC link before GSC synchronization
p.aero_enable_time = p.gfm_enable_time; % no mechanical-power lead at takeover
p.aero_release_delay_s = 0.75; % align aerodynamic buildup with measured PCC export

% Keep the active-damping action at the same fraction of rated Iq as the
% validated 1 MW model, with gain scaled by Dsh / torque constant.
p.msc_ad_iq_limit = 0.04*p.i_m_q0;
p.msc_ad_iq_gain = 2*p.D_sh/(1.5*p.n_p*p.psi_f);
end
