%% Liu 2024 5 MW nonlinear drivetrain and aerodynamic parameters
p5 = Liu2024_5MW_Params();

P_wt_rated = p5.P_wt_rated;
omega_m0 = p5.omega_m0;
v_w0 = p5.v_w0;
J_t = p5.J_t;
J_g = p5.J_g;
K_sh = p5.K_sh;
D_sh = p5.D_sh;

shaft_damping_scale = 1.0;
if exist('shaft_damping_scale_override','var')
    shaft_damping_scale = shaft_damping_scale_override;
elseif evalin('base','exist(''shaft_damping_scale_override'',''var'')')
    shaft_damping_scale = evalin('base','shaft_damping_scale_override');
end
D_sh = shaft_damping_scale*D_sh;

T_e0 = P_wt_rated/omega_m0;
T_aero0 = T_e0;
theta_tw0 = T_e0/K_sh;
D_aero = T_aero0/omega_m0;
K_v_aero = 3*T_aero0/v_w0;
D_t = p5.D_t;
D_g = p5.D_g;

Vdc = p5.Vdc;
VdcRef_V = p5.Vdc;
VacRef_V = p5.V_control_d;
Pref_W = p5.P_wt_rated;
Qref_var = 0;
if exist('qref_var_override','var')
    Qref_var = qref_var_override;
elseif evalin('base','exist(''qref_var_override'',''var'')')
    Qref_var = evalin('base','qref_var_override');
end
C_dc = p5.Cdc;
Cdc = p5.Cdc;
V_dc0 = p5.Vdc;
S_base = p5.S_base;
V_LL = p5.V_LL;
f_base = p5.f_base;
R_s = p5.R_s;
L_d = p5.L_d;
L_q = p5.L_q;
psi_f = p5.psi_f;
n_p = p5.n_p;
i_m_d0 = 0;
i_m_q0 = p5.i_m_q0;

GridFilterL = p5.Lf;
GridFilterR = p5.Rf;
GridFilterC = p5.Cf;
SCR_runtime = p5.SCR;
if exist('scr_override','var')
    SCR_runtime = scr_override;
elseif evalin('base','exist(''scr_override'',''var'')')
    SCR_runtime = evalin('base','scr_override');
end
assert(isnumeric(SCR_runtime) && isscalar(SCR_runtime) && ...
    isfinite(SCR_runtime) && SCR_runtime > 0, ...
    'SCR override must be a positive finite scalar.');
grid_impedance_scale = p5.SCR/SCR_runtime;
GridLineL = p5.Lg*grid_impedance_scale;
GridRs = p5.Rg*grid_impedance_scale;
CurrentLimit = p5.grid_current_limit;
MotorCurrentLimit = p5.motor_current_limit;
MotorVoltLimit = p5.motor_voltage_limit;
VSG_H = p5.VSG_H;
VSG_MP = p5.mp;
k_pdc = p5.dvc_kp_A_per_V;
k_idc = p5.dvc_ki_fraction;
k_ff_msc_typec = p5.msc_power_ff_A_per_W;
DvcType = 3;
rated_wind_speed = p5.rated_wind_speed;
rotor_radius = p5.rotor_radius;
air_density = p5.air_density;
rotor_area = p5.rotor_area;
K_opt = p5.K_opt;
K_rated_recovery = p5.K_rated_recovery;
K_speed_recovery_W_per_radps = p5.K_speed_recovery_W_per_radps;
mppt_rated_blend_low_pu = p5.mppt_rated_blend_low_pu;
mppt_rated_blend_high_pu = p5.mppt_rated_blend_high_pu;
pitch_beta_ff_gain_deg_per_mps = p5.pitch_beta_ff_gain_deg_per_mps;
pitch_kp_deg_per_radps = p5.pitch_kp_deg_per_radps;
pitch_ki_deg_per_rad = p5.pitch_ki_deg_per_rad;
pitch_rate_deg_per_s = p5.pitch_rate_deg_per_s;
pitch_beta_max_deg = p5.pitch_beta_max_deg;
aero_release_delay_s = p5.aero_release_delay_s;
cp_lambda_bp = p5.cp_lambda_bp;
cp_beta_bp = p5.cp_beta_bp;
cp_table = p5.cp_table;

wind_step_time = 45;
wind_step_mps = 0;
if exist('wind_step_time_override','var')
    wind_step_time = wind_step_time_override;
elseif evalin('base','exist(''wind_step_time_override'',''var'')')
    wind_step_time = evalin('base','wind_step_time_override');
end
if exist('wind_step_mps_override','var')
    wind_step_mps = wind_step_mps_override;
elseif evalin('base','exist(''wind_step_mps_override'',''var'')')
    wind_step_mps = evalin('base','wind_step_mps_override');
end

mech_log_decimation = 100;
frequency_estimation_start = wind_step_time + 0.02;
sim_stop_time = 6;

param_sync_info_5mw = p5;
param_sync_info_5mw.D_sh_runtime = D_sh;
param_sync_info_5mw.shaft_damping_scale = shaft_damping_scale;
param_sync_info_5mw.SCR_runtime = SCR_runtime;
param_sync_info_5mw.grid_impedance_scale = grid_impedance_scale;

names5 = { ...
    'P_wt_rated','omega_m0','v_w0','J_t','J_g','K_sh','D_sh','D_t','D_g', ...
    'shaft_damping_scale','T_e0','T_aero0','theta_tw0','D_aero','K_v_aero', ...
    'Vdc','VdcRef_V','VacRef_V','Pref_W','Qref_var','C_dc','Cdc','V_dc0', ...
    'S_base','V_LL','f_base','R_s','L_d','L_q','psi_f','n_p','i_m_d0','i_m_q0', ...
    'GridFilterL','GridFilterR','GridFilterC','GridLineL','GridRs','CurrentLimit', ...
    'SCR_runtime','grid_impedance_scale', ...
    'MotorCurrentLimit','MotorVoltLimit','VSG_H','VSG_MP','k_pdc','k_idc', ...
    'k_ff_msc_typec','DvcType','rated_wind_speed','rotor_radius', ...
    'air_density','rotor_area','K_opt','K_rated_recovery', ...
    'K_speed_recovery_W_per_radps','mppt_rated_blend_low_pu', ...
    'mppt_rated_blend_high_pu', ...
    'pitch_beta_ff_gain_deg_per_mps', ...
    'pitch_kp_deg_per_radps','pitch_ki_deg_per_rad','pitch_rate_deg_per_s', ...
    'pitch_beta_max_deg','aero_release_delay_s', ...
    'cp_lambda_bp','cp_beta_bp','cp_table', ...
    'wind_step_time','wind_step_mps', ...
    'mech_log_decimation','frequency_estimation_start','sim_stop_time', ...
    'param_sync_info_5mw'};
for k5 = 1:numel(names5)
    assignin('base',names5{k5},eval(names5{k5}));
end
