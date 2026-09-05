%% Liu 2024 5 MW nonlinear drivetrain and aerodynamic parameters
p5 = Liu2024_5MW_Params();

P_wt_rated = p5.P_wt_rated;
P_aero_rated_emt = p5.P_aero_rated_emt;
omega_m0 = p5.omega_m0;
v_w0 = p5.v_w0;
J_t = p5.J_t;
J_g = p5.J_g;
K_sh = p5.K_sh;
D_sh = p5.D_sh;

shaft_stiffness_scale = 1.0;
if exist('shaft_stiffness_scale_override','var')
    shaft_stiffness_scale = shaft_stiffness_scale_override;
elseif evalin('base','exist(''shaft_stiffness_scale_override'',''var'')')
    shaft_stiffness_scale = evalin('base','shaft_stiffness_scale_override');
end
K_sh = shaft_stiffness_scale*K_sh;

shaft_damping_scale = 1.0;
if exist('shaft_damping_scale_override','var')
    shaft_damping_scale = shaft_damping_scale_override;
elseif evalin('base','exist(''shaft_damping_scale_override'',''var'')')
    shaft_damping_scale = evalin('base','shaft_damping_scale_override');
end
D_sh = shaft_damping_scale*D_sh;

shaft_damping_ratio_target = NaN;
if exist('shaft_damping_ratio_target_override','var')
    shaft_damping_ratio_target = shaft_damping_ratio_target_override;
elseif evalin('base','exist(''shaft_damping_ratio_target_override'',''var'')')
    shaft_damping_ratio_target = evalin('base','shaft_damping_ratio_target_override');
end
omega_sh_runtime = sqrt(K_sh*(1/J_t + 1/J_g));
if isfinite(shaft_damping_ratio_target)
    assert(shaft_damping_ratio_target > 0 && shaft_damping_ratio_target < 1, ...
        'Shaft damping-ratio target must be between zero and one.');
    D_sh = 2*shaft_damping_ratio_target*omega_sh_runtime/(1/J_t + 1/J_g);
end
shaft_damping_ratio_runtime = D_sh*(1/J_t + 1/J_g)/(2*omega_sh_runtime);

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
rated_power_transition_start_s = p5.rated_power_transition_start_s;
rated_power_transition_duration_s = p5.rated_power_transition_duration_s;
pitch_ff_start_wind_speed = p5.pitch_ff_start_wind_speed;
pitch_beta_rated_ff_deg = p5.pitch_beta_rated_ff_deg;
pitch_beta_ff_gain_deg_per_mps = p5.pitch_beta_ff_gain_deg_per_mps;
pitch_kp_deg_per_radps = p5.pitch_kp_deg_per_radps;
pitch_ki_deg_per_rad = p5.pitch_ki_deg_per_rad;
pitch_rate_deg_per_s = p5.pitch_rate_deg_per_s;
pitch_beta_max_deg = p5.pitch_beta_max_deg;
aero_release_delay_s = p5.aero_release_delay_s;
cp_lambda_bp = p5.cp_lambda_bp;
cp_beta_bp = p5.cp_beta_bp;
cp_table = p5.cp_table;

% Runtime experiment profiles. These variables drive explicit From Workspace
% blocks in the single master model; they are not metadata-only overrides.
wind_initial_mps = v_w0;
wind_final_mps = v_w0;
wind_ramp_start_s = 45;
wind_ramp_duration_s = 1;
pref_scale_initial = 1;
pref_scale_final = 1;
pref_scale_start_s = 45;
pref_scale_ramp_duration_s = 0.05;
aero_torque_scale_initial = 1;
aero_torque_scale_final = 1;
aero_torque_scale_start_s = 45;
aero_torque_scale_ramp_duration_s = 0.05;
% Baseline-mechanism model: the optional shaft active-damping channel is
% disabled unless an experiment explicitly enables it.
% The frozen 5 MW two-mass baseline uses the validated shaft-speed active
% damping channel.  Experiments can still bypass it explicitly with
% active_damping_scale_override = 0.
active_damping_scale = 1;
% Keep the rated 5 MW direct-run operating point available while making the
% pitch actuator independently bypassable for MPPT-only studies.
pitch_enable = 1;

profileOverrideNames = { ...
    'wind_initial_mps','wind_final_mps','wind_ramp_start_s','wind_ramp_duration_s', ...
    'pref_scale_initial','pref_scale_final','pref_scale_start_s','pref_scale_ramp_duration_s', ...
    'aero_torque_scale_initial','aero_torque_scale_final', ...
    'aero_torque_scale_start_s','aero_torque_scale_ramp_duration_s', ...
    'active_damping_scale','pitch_enable'};
for kp = 1:numel(profileOverrideNames)
    overrideName = [profileOverrideNames{kp} '_override'];
    if exist(overrideName,'var')
        eval([profileOverrideNames{kp} ' = ' overrideName ';']);
    elseif evalin('base',sprintf('exist(''%s'',''var'')',overrideName))
        eval([profileOverrideNames{kp} ' = evalin(''base'',overrideName);']);
    end
end

% Initialize the slow pitch integrator close to the rated-region
% aerodynamic equilibrium.  The bias fades linearly to zero at the pitch
% feedforward threshold, so below-rated MPPT cases retain a zero-pitch
% initial state.
pitchInitial13mps = p5.pitch_integrator_initial_13mps_deg;
currentInitModel = bdroot;
if contains(currentInitModel,'Grid_Following_PMSG5MW')
    pitchInitial13mps = p5.pitch_integrator_initial_13mps_deg_gfl;
end
pitch_integrator_initial_deg = ...
    pitchInitial13mps * max(0,min(1, ...
    (wind_initial_mps-pitch_ff_start_wind_speed) / ...
    max(13-pitch_ff_start_wind_speed,eps)));
if exist('pitch_integrator_initial_deg_override','var') && ...
        isfinite(pitch_integrator_initial_deg_override)
    pitch_integrator_initial_deg = pitch_integrator_initial_deg_override;
elseif evalin('base', ...
        'exist(''pitch_integrator_initial_deg_override'',''var'')')
    candidatePitchInitial = evalin('base', ...
        'pitch_integrator_initial_deg_override');
    if isfinite(candidatePitchInitial)
        pitch_integrator_initial_deg = candidatePitchInitial;
    end
end

assert(wind_initial_mps > 0 && wind_final_mps > 0, ...
    'Wind-speed profile values must be positive.');
assert(pref_scale_initial >= 0 && pref_scale_final >= 0, ...
    'Active-power scale profile values must be nonnegative.');
assert(aero_torque_scale_initial >= 0 && aero_torque_scale_final >= 0, ...
    'Aerodynamic-torque scale profile values must be nonnegative.');
assert(active_damping_scale >= 0, ...
    'Active-damping scale must be nonnegative.');

profileEpsilon_s = max([dtime,Ts_step,1e-6]);
windRampEnd = wind_ramp_start_s + max(wind_ramp_duration_s,profileEpsilon_s);
prefRampEnd = pref_scale_start_s + max(pref_scale_ramp_duration_s,profileEpsilon_s);
aeroRampEnd = aero_torque_scale_start_s + max(aero_torque_scale_ramp_duration_s,profileEpsilon_s);
profileEnd_s = 1e9;
wind_profile_ts = [0 wind_initial_mps; wind_ramp_start_s wind_initial_mps; ...
    windRampEnd wind_final_mps; profileEnd_s wind_final_mps];
if exist('wind_profile_ts_override','var') && ~isempty(wind_profile_ts_override)
    wind_profile_ts = wind_profile_ts_override;
elseif evalin('base','exist(''wind_profile_ts_override'',''var'')')
    candidateWindProfile = evalin('base','wind_profile_ts_override');
    if ~isempty(candidateWindProfile), wind_profile_ts = candidateWindProfile; end
end
assert(isnumeric(wind_profile_ts) && size(wind_profile_ts,2) == 2 && ...
    size(wind_profile_ts,1) >= 2 && all(diff(wind_profile_ts(:,1)) > 0) && ...
    all(wind_profile_ts(:,2) > 0), ...
    'Wind profile must be an Nx2 [time, speed] matrix with increasing time.');
pref_scale_profile_ts = [0 pref_scale_initial; pref_scale_start_s pref_scale_initial; ...
    prefRampEnd pref_scale_final; profileEnd_s pref_scale_final];
aero_torque_scale_profile_ts = [0 aero_torque_scale_initial; ...
    aero_torque_scale_start_s aero_torque_scale_initial; ...
    aeroRampEnd aero_torque_scale_final; profileEnd_s aero_torque_scale_final];
if exist('aero_torque_scale_profile_ts_override','var') && ...
        ~isempty(aero_torque_scale_profile_ts_override)
    aero_torque_scale_profile_ts = aero_torque_scale_profile_ts_override;
elseif evalin('base', ...
        'exist(''aero_torque_scale_profile_ts_override'',''var'')')
    candidateAeroProfile = evalin('base', ...
        'aero_torque_scale_profile_ts_override');
    if ~isempty(candidateAeroProfile)
        aero_torque_scale_profile_ts = candidateAeroProfile;
    end
end
assert(isnumeric(aero_torque_scale_profile_ts) && ...
    size(aero_torque_scale_profile_ts,2) == 2 && ...
    size(aero_torque_scale_profile_ts,1) >= 2 && ...
    all(diff(aero_torque_scale_profile_ts(:,1)) > 0) && ...
    all(aero_torque_scale_profile_ts(:,2) >= 0), ...
    ['Aerodynamic-torque scale profile must be an Nx2 [time, scale] ' ...
     'matrix with increasing time and nonnegative scale.']);

% Compatibility aliases retained for older diagnostics.
wind_step_time = wind_ramp_start_s;
wind_step_mps = wind_final_mps - wind_initial_mps;

% Direct model runs retain about 250 Hz of slow/mechanical data. This is
% sufficient for the 0--10 Hz drivetrain band without accumulating PWM-step
% data over the default 60 s run.
mech_log_decimation = 4000;
if exist('mech_log_decimation_override','var')
    mech_log_decimation = mech_log_decimation_override;
elseif evalin('base','exist(''mech_log_decimation_override'',''var'')')
    mech_log_decimation = evalin('base','mech_log_decimation_override');
end
assert(mech_log_decimation >= 1 && mech_log_decimation == round(mech_log_decimation), ...
    'Mechanical log decimation must be a positive integer.');

% Grid-voltage phase reconstruction needs a substantially higher record
% rate than the slow drivetrain outputs.  These three PCC line voltages are
% logged only for frequency-input verification and do not affect control.
grid_frequency_log_decimation = 100;
frequency_estimation_start = wind_step_time + 0.02;
sim_stop_time = 6;

param_sync_info_5mw = p5;
param_sync_info_5mw.D_sh_runtime = D_sh;
param_sync_info_5mw.K_sh_runtime = K_sh;
param_sync_info_5mw.shaft_damping_scale = shaft_damping_scale;
param_sync_info_5mw.shaft_stiffness_scale = shaft_stiffness_scale;
param_sync_info_5mw.shaft_damping_ratio_target = shaft_damping_ratio_target;
param_sync_info_5mw.shaft_damping_ratio_runtime = shaft_damping_ratio_runtime;
param_sync_info_5mw.SCR_runtime = SCR_runtime;
param_sync_info_5mw.grid_impedance_scale = grid_impedance_scale;

names5 = { ...
    'P_wt_rated','P_aero_rated_emt','omega_m0','v_w0','J_t','J_g','K_sh','D_sh','D_t','D_g', ...
    'shaft_damping_scale','shaft_stiffness_scale','shaft_damping_ratio_target', ...
    'shaft_damping_ratio_runtime','omega_sh_runtime', ...
    'T_e0','T_aero0','theta_tw0','D_aero','K_v_aero', ...
    'Vdc','VdcRef_V','VacRef_V','Pref_W','Qref_var','C_dc','Cdc','V_dc0', ...
    'S_base','V_LL','f_base','R_s','L_d','L_q','psi_f','n_p','i_m_d0','i_m_q0', ...
    'GridFilterL','GridFilterR','GridFilterC','GridLineL','GridRs','CurrentLimit', ...
    'SCR_runtime','grid_impedance_scale', ...
    'MotorCurrentLimit','MotorVoltLimit','VSG_H','VSG_MP','k_pdc','k_idc', ...
    'k_ff_msc_typec','DvcType','rated_wind_speed','rotor_radius', ...
    'air_density','rotor_area','K_opt','K_rated_recovery', ...
    'K_speed_recovery_W_per_radps','mppt_rated_blend_low_pu', ...
    'mppt_rated_blend_high_pu','rated_power_transition_start_s', ...
    'rated_power_transition_duration_s', ...
    'pitch_ff_start_wind_speed','pitch_beta_rated_ff_deg', ...
    'pitch_beta_ff_gain_deg_per_mps', ...
    'pitch_kp_deg_per_radps','pitch_ki_deg_per_rad', ...
    'pitch_integrator_initial_deg','pitch_rate_deg_per_s', ...
    'pitch_beta_max_deg','aero_release_delay_s', ...
    'cp_lambda_bp','cp_beta_bp','cp_table', ...
    'wind_initial_mps','wind_final_mps','wind_ramp_start_s','wind_ramp_duration_s', ...
    'pref_scale_initial','pref_scale_final','pref_scale_start_s', ...
    'pref_scale_ramp_duration_s','aero_torque_scale_initial', ...
    'aero_torque_scale_final','aero_torque_scale_start_s', ...
    'aero_torque_scale_ramp_duration_s','active_damping_scale', ...
    'wind_profile_ts','pref_scale_profile_ts','aero_torque_scale_profile_ts', ...
    'wind_step_time','wind_step_mps', ...
    'mech_log_decimation','grid_frequency_log_decimation', ...
    'frequency_estimation_start','sim_stop_time', ...
    'param_sync_info_5mw'};
for k5 = 1:numel(names5)
    assignin('base',names5{k5},eval(names5{k5}));
end
