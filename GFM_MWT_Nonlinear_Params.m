%% GFM-MWT nonlinear drivetrain and aerodynamic parameters
% Initial runnable parameters for the 1 MW model.

%% Optional: inherit the "same-object" WT/PMSG parameters from small-signal model
% If this MAT exists, nonlinear and small-signal models will share the same
% plant-side parameters (J_t, J_g, K_sh, D_sh, v_w0, etc.) by default.
thisDir = fileparts(mfilename('fullpath')); %#ok<NASGU>
ssmParamsPath = locate_ssm_parameters_mat();
use_ssm_same_object_params = exist(ssmParamsPath, 'file') == 2;
if exist('use_ssm_same_object_params_override', 'var')
    use_ssm_same_object_params = logical(use_ssm_same_object_params_override);
elseif evalin('base','exist(''use_ssm_same_object_params_override'',''var'')')
    use_ssm_same_object_params = logical(evalin('base','use_ssm_same_object_params_override'));
end

ssm = struct();
if use_ssm_same_object_params
    ssm = load(ssmParamsPath);
end

P_wt_rated = 1.0e6;           % W
omega_m0 = 2*pi*30/60;        % rad/s
v_w0 = 12.0;                  % m/s
if exist('P_wt_rated_override','var')
    P_wt_rated = P_wt_rated_override;
elseif evalin('base','exist(''P_wt_rated_override'',''var'')')
    P_wt_rated = evalin('base','P_wt_rated_override');
end

J_g = 1.83750e5;              % kg*m^2
J_t = 8.0 * J_g;              % kg*m^2, provisional if not inherited
f_sh_init_guess = 1.0;        % Hz, provisional placeholder only (nonlinear ID is data-driven)
zeta_sh_init_guess = 0.010;   % pu, provisional shaft damping

J_eq = J_t * J_g / (J_t + J_g);
w_sh_init_guess = 2*pi*f_sh_init_guess;
K_sh = J_eq * w_sh_init_guess^2;                      % N*m/rad
D_sh = 2*zeta_sh_init_guess*w_sh_init_guess*J_eq;     % N*m*s/rad
if exist('D_sh_override', 'var')
    D_sh = D_sh_override;
elseif evalin('base','exist(''D_sh_override'',''var'')')
    D_sh = evalin('base','D_sh_override');
end

T_e0 = P_wt_rated / omega_m0;                         % N*m
T_aero0 = T_e0;                                       % N*m, initial balance
theta_tw0 = T_e0 / K_sh;                              % rad

D_aero = T_aero0 / omega_m0;                          % N*m*s/rad
K_v_aero = 3*T_aero0 / v_w0;                          % N*m/(m/s)
D_t = 0.005 * D_aero;                                 % N*m*s/rad
D_g = 0.005 * D_aero;                                 % N*m*s/rad

if use_ssm_same_object_params
    if isfield(ssm, 'S_base');   P_wt_rated = ssm.S_base; end
    if isfield(ssm, 'omega_g0'); omega_m0 = ssm.omega_g0; end
    if isfield(ssm, 'v_w0');     v_w0 = ssm.v_w0; end
    if isfield(ssm, 'J_g');      J_g = ssm.J_g; end
    if isfield(ssm, 'J_t');      J_t = ssm.J_t; end
    if isfield(ssm, 'K_sh');     K_sh = ssm.K_sh; end
    if isfield(ssm, 'D_sh');     D_sh = ssm.D_sh; end
    if isfield(ssm, 'D_t');      D_t = ssm.D_t; end
    if isfield(ssm, 'D_g');      D_g = ssm.D_g; end
    if isfield(ssm, 'T_e0');     T_e0 = ssm.T_e0; end
    if isfield(ssm, 'D_aero');   D_aero = ssm.D_aero; end
    if isfield(ssm, 'K_v_aero'); K_v_aero = ssm.K_v_aero; end
    T_aero0 = T_e0;
    theta_tw0 = T_e0 / K_sh;
end

% Keep explicit runtime override as highest priority active-power setpoint.
if exist('P_wt_rated_override','var')
    P_wt_rated = P_wt_rated_override;
elseif evalin('base','exist(''P_wt_rated_override'',''var'')')
    P_wt_rated = evalin('base','P_wt_rated_override');
end

% Rebuild the mechanical equilibrium after all power/speed overrides.
% This prevents a stale 30 rpm torque from surviving after omega_m0 is
% aligned to the 150 rpm small-signal operating point.
T_e0 = P_wt_rated / omega_m0;
T_aero0 = T_e0;
theta_tw0 = T_e0 / K_sh;
D_aero = T_aero0 / omega_m0;
K_v_aero = 3 * T_aero0 / v_w0;
D_t = 0.005 * D_aero;
D_g = 0.005 * D_aero;

wind_step_time = 1.50;                                % s
wind_step_mps = 0.00;                                 % m/s, no-disturbance stage
mech_log_decimation = 100;                            % at Ts=1 us
frequency_estimation_start = wind_step_time + 0.02;   % s
sim_stop_time = 3.0;                                  % s
if exist('wind_step_mps_override', 'var')
    wind_step_mps = wind_step_mps_override;
elseif evalin('base','exist(''wind_step_mps_override'',''var'')')
    wind_step_mps = evalin('base','wind_step_mps_override');
end
if exist('sim_stop_time_override', 'var')
    sim_stop_time = sim_stop_time_override;
elseif evalin('base','exist(''sim_stop_time_override'',''var'')')
    sim_stop_time = evalin('base','sim_stop_time_override');
end

% Quick sanity info
param_sync_info = struct( ...
    'use_ssm_same_object_params', use_ssm_same_object_params, ...
    'ssmParamsPath', ssmParamsPath, ...
    'P_wt_rated', P_wt_rated, ...
    'omega_m0', omega_m0, ...
    'v_w0', v_w0, ...
    'J_t', J_t, ...
    'J_g', J_g, ...
    'K_sh', K_sh, ...
    'D_sh', D_sh, ...
    'D_t', D_t, ...
    'D_g', D_g, ...
    'D_aero', D_aero, ...
    'K_v_aero', K_v_aero);

% Simulink block parameters are resolved from the base workspace even when
% this initialization script is called from a diagnostic function.
model_param_names = { ...
    'P_wt_rated', 'omega_m0', 'v_w0', 'J_t', 'J_g', 'K_sh', 'D_sh', ...
    'D_t', 'D_g', 'T_e0', 'T_aero0', 'theta_tw0', 'D_aero', 'K_v_aero', ...
    'wind_step_time', 'wind_step_mps', 'mech_log_decimation', ...
    'frequency_estimation_start', 'sim_stop_time', 'param_sync_info'};
for model_param_index = 1:numel(model_param_names)
    model_param_name = model_param_names{model_param_index};
    assignin('base', model_param_name, eval(model_param_name));
end
