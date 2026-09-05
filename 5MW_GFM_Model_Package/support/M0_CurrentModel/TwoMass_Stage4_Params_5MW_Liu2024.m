%% Coordinated cold start for the Liu 2024 5 MW model
run(fullfile(fileparts(mfilename('fullpath')), ...
    'GFM_MWT_Nonlinear_Params_5MW_Liu2024.m'));

% 理想连续副本从物理转矩平衡开始：T_sh=K_sh*theta_tw=T_e0，
% 避免原冷启动专用的零扭转角在 t=0 注入非物理轴系瞬态。
theta_tw_stage4_0 = theta_tw0;
% Keep the aerodynamic/pitch-release ramp aligned with the bumpless VSG
% takeover.  Starting this ramp at breaker close (1.75 s) made the turbine
% torque lead the electrical power ramp by 0.5 s and stored the mismatch as
% rotor kinetic energy during 5 MW startup.
stage4_start_s = 2.25;
% 热启动加载时，快照已经处于目标功率运行点，不能在仿真 t=0
% 再次重放冷启动功率/转矩斜坡。该覆盖默认关闭，只由独立 EMT
% 热启动验证器显式打开。
stage4_hotstart_final = false;
if exist('stage4_hotstart_final_override','var')
    stage4_hotstart_final = logical(stage4_hotstart_final_override);
elseif evalin('base','exist(''stage4_hotstart_final_override'',''var'')')
    stage4_hotstart_final = logical(evalin('base','stage4_hotstart_final_override'));
end
% Keep direct model runs consistent with the validated 5 MW commissioning
% case. The former 0.10 MW/s residual required a 50 s ramp and could make a
% direct SLX run fail even though the scripted 0.50 MW/s case was stable.
stage4_power_ramp_Wps = param_sync_info_5mw.pref_ramp_slope;
if exist('stage4_power_ramp_Wps_override','var')
    stage4_power_ramp_Wps = stage4_power_ramp_Wps_override;
elseif evalin('base','exist(''stage4_power_ramp_Wps_override'',''var'')')
    stage4_power_ramp_Wps = evalin('base','stage4_power_ramp_Wps_override');
end
stage4_ramp_duration_s = P_wt_rated/stage4_power_ramp_Wps;
stage4_torque_ramp_Nmps = T_aero0/stage4_ramp_duration_s;
if stage4_hotstart_final
    % 在 t=0 前已完成斜坡；保留有限的 50 ms 斜坡长度，避免除零。
    stage4_start_s = -0.05;
    stage4_ramp_duration_s = 0.05;
    stage4_power_ramp_Wps = P_wt_rated/stage4_ramp_duration_s;
    stage4_torque_ramp_Nmps = T_aero0/stage4_ramp_duration_s;
end
stage4_aero_speed_damping_scale = 1.20;
stage4_torsional_frequency_hz = sqrt(K_sh*(1/J_t + 1/J_g))/(2*pi);
stage4_shaper_zeta = max(0.001,min(0.20,param_sync_info_5mw.zeta_sh));
stage4_shaper_delay_s = 0.5/(stage4_torsional_frequency_hz*sqrt(1-stage4_shaper_zeta^2));
stage4_shaper_decay = exp(-stage4_shaper_zeta*pi/sqrt(1-stage4_shaper_zeta^2));
stage4_shaper_A1 = 1/(1+stage4_shaper_decay);
stage4_shaper_A2 = stage4_shaper_decay/(1+stage4_shaper_decay);
stage4_stop_time_s = 8;

stageNames5 = { ...
    'theta_tw_stage4_0','stage4_start_s','stage4_hotstart_final', ...
    'stage4_power_ramp_Wps', ...
    'stage4_ramp_duration_s','stage4_torque_ramp_Nmps', ...
    'stage4_aero_speed_damping_scale','stage4_torsional_frequency_hz', ...
    'stage4_shaper_delay_s','stage4_shaper_A1','stage4_shaper_A2','stage4_stop_time_s'};
for stageIndex5 = 1:numel(stageNames5)
    assignin('base',stageNames5{stageIndex5},eval(stageNames5{stageIndex5}));
end
