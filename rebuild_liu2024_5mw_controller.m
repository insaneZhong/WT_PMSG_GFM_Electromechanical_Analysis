function mex_path = rebuild_liu2024_5mw_controller()
%REBUILD_LIU2024_5MW_CONTROLLER Build one no-LVRT 5 MW controller.
p = Liu2024_5MW_Params();
root = fileparts(mfilename('fullpath'));
old_folder = pwd;
folder_cleanup = onCleanup(@() cd(old_folder)); %#ok<NASGU>
cd(root);

defs = struct();
defs.MOTOR_RS = p.R_s;
defs.MOTOR_LD = p.L_d;
defs.MOTOR_LQ = p.L_q;
defs.MOTOR_FM_25_TEMPERATURE = p.psi_f;
defs.MOTOR_POLE_PAIR = p.n_p;
defs.MOTOR_JM = p.J_g;
defs.GRID_UDC__C = p.Cdc;
defs.MOTOR_ID_LIMIT_MAX = p.motor_current_limit;
defs.MOTOR_ID_REF_SLOPE_LIMIT_MAX = 5.0e5;
defs.MOTOR_IQ_REF_SLOPE_LIMIT_MAX = 5.0e5;
defs.MOTOR_ID_KP = p.motor_current_kp;
defs.MOTOR_IQ_KP = p.motor_current_kp;
defs.MOTOR_ID_KI = p.motor_current_ki_fraction;
defs.MOTOR_IQ_KI = p.motor_current_ki_fraction;
defs.GRID_FILTER__LS = p.Lf;
defs.GRID_FILTER__C = p.Cf;
defs.GRID_LINE_impedance__L = p.Lg;
defs.GRID__RS = p.Rg;
defs.VSG_EQUIV_W0 = p.omega_base;
defs.GSI_NOMINAL_OMEGA_RADPS = p.omega_base;
defs.GSI_NOMINAL_VOLTAGE_PHASE_PEAK_V = p.V_control_d;
defs.GSI_E_VOLTAGE_MAX_V = p.grid_voltage_limit;
defs.PWM_SWITCH_FREQUENCY_HZ = p.fsw;
defs.VSG_EQUIV_H = p.VSG_H;
defs.VSG_EQUIV_MP = p.mp;
defs.VSG_EQUIV_SBASE_W = p.S_base;
defs.VSG_STARTUP_MP = p.vsg_startup_mp;
defs.VSG_DYNAMICS_TRANSITION_START_S = p.vsg_dynamics_transition_start_s;
defs.VSG_DYNAMICS_TRANSITION_DURATION_S = p.vsg_dynamics_transition_duration_s;
defs.ENABLE_VSG_EQUIV_WREF = 1;
defs.VSG_POWER_ERROR_SIGN = p.vsg_power_error_sign;
defs.VSG_STARTUP_POWER_ERROR_SIGN = p.vsg_startup_power_error_sign;
defs.CURRENT_LIMIT_MAX = p.grid_current_limit;
defs.CURRENT_ID_KP = p.grid_current_kp;
defs.CURRENT_IQ_KP = p.grid_current_kp;
defs.CURRENT_ID_KI = p.grid_current_ki_fraction;
defs.CURRENT_IQ_KI = p.grid_current_ki_fraction;
defs.CURRENT_PI_ID_OUT_MAX = p.grid_voltage_limit;
defs.CURRENT_PI_ID_OUT_MIN = -p.grid_voltage_limit;
defs.CURRENT_PI_IQ_OUT_MAX = p.grid_voltage_limit;
defs.CURRENT_PI_IQ_OUT_MIN = -p.grid_voltage_limit;
defs.GSI_V_LOOP_KP = p.grid_voltage_kp;
defs.GSI_V_LOOP_KI = p.grid_voltage_ki_fraction;
defs.GSI_PLOOP_KP = p.grid_power_kp;
defs.GSI_PLOOP_KI = p.grid_power_ki;
defs.GSI_PF_LOOP_SIGN = 1;
defs.GSI_QV_DROOP_V_PER_VAR = p.qv_droop_V_per_var;
defs.GSI_NORMAL_LIMITS_ENABLE = 1;
defs.MSC_TYPEC_USE_PCC_POWER_FF = 1;
defs.MSC_TYPEC_SPEED_NORMALIZE = 1;
defs.MSC_TYPEC_RATED_ELEC_OMEGA_RADPS = p.omega_g0;
defs.MSC_TYPEC_MIN_ELEC_OMEGA_RADPS = 0.20*p.omega_g0;
defs.GSI_DYNAMIC_PREF_INPUT_ENABLE = 1;
defs.GSI_GFM_ENABLE_TIME_S = p.gfm_enable_time;

controller = 'main_legacy_liu2024_5mw_stable';
compile_main_legacy_ad(p.msc_ad_iq_gain,p.msc_ad_iq_limit, ...
    p.pref_ramp_slope,p.presyn_switch_time,controller,p.motor_current_limit, ...
    0,0,p.Vdc,0,6.0,p.dvc_kp_A_per_V,p.dvc_ki_fraction,1e-5, ...
    p.msc_power_ff_A_per_W,p.dvc_enable_time,true,true, ...
    p.motor_voltage_limit,p.motor_voltage_limit,0,1e9,1e9,0, ...
    1e9,1,0,p.gsc_current_vector_limit,p.gsc_modulation_limit,0, ...
    0,0,0,0,0,p.gsc_current_ref_aw_gain,p.gsc_voltage_aw_gain, ...
    false,false,false,false,defs);
mex_path = fullfile(root,[controller '.' mexext]);
buildMetadata = table( ...
    string({'VSG_EQUIV_H';'VSG_EQUIV_MP';'VSG_EQUIV_SBASE_W'; ...
    'VSG_STARTUP_MP';'VSG_POWER_ERROR_SIGN'; ...
    'VSG_STARTUP_POWER_ERROR_SIGN';'VSG_DYNAMICS_TRANSITION_START_S'; ...
    'VSG_DYNAMICS_TRANSITION_DURATION_S';'GSI_QV_DROOP_V_PER_VAR'; ...
    'GSI_GFM_ENABLE_TIME_S'}), ...
    [p.VSG_H;p.mp;p.S_base;p.vsg_startup_mp;p.vsg_power_error_sign; ...
    p.vsg_startup_power_error_sign;p.vsg_dynamics_transition_start_s; ...
    p.vsg_dynamics_transition_duration_s;p.qv_droop_V_per_var; ...
    p.gfm_enable_time], ...
    string({'s';'rad/s/W';'W';'rad/s/W';'1';'1';'s';'s';'V/var';'s'}), ...
    'VariableNames',{'macro','value','unit'});
writetable(buildMetadata,fullfile(root, ...
    'main_legacy_liu2024_5mw_stable_vsg_metadata.csv'));
fprintf('Built Liu2024 5 MW controller: %s\n',mex_path);
end
