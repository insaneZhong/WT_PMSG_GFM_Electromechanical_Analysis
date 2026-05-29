function build_GFM_MWT_Drivetrain()
% Add aerodynamic input and a two-mass drivetrain to Grid_Forming_PMSG.
% The PMSM remains in externally imposed mechanical-speed mode; omega_g is
% calculated by the new drivetrain using the measured electromagnetic torque.
% PMSM1 reports generating torque as negative; the generator equation uses
% T_sh + T_e so electrical generation opposes mechanical acceleration.

root = fileparts(mfilename('fullpath'));
oldFolder = pwd;
cleanup = onCleanup(@() cd(oldFolder));
cd(root);

run('GFM_MWT_Nonlinear_Params.m');
mdl = 'Grid_Forming_PMSG';
load_system(mdl);

drv = [mdl '/Wind_TwoMass'];
if getSimulinkBlockHandle(drv) ~= -1
    error('Wind_TwoMass already exists in %s. Refusing duplicate insertion.', mdl);
end

appendParameterInit(mdl);

add_block('simulink/Ports & Subsystems/Subsystem', drv, ...
    'Position', [945 355 1095 510]);
delete_block([drv '/In1']);
delete_block([drv '/Out1']);
buildDrivetrainSubsystem(drv);

wind = [mdl '/Wind_Step'];
add_block('simulink/Sources/Step', wind, ...
    'Time', 'wind_step_time', 'Before', 'v_w0', ...
    'After', 'v_w0 + wind_step_mps', ...
    'Position', [780 445 850 475]);

addSignalSink(mdl, 'omega_t', [1160 330 1255 350]);
addSignalSink(mdl, 'omega_g', [1160 370 1255 390]);
addSignalSink(mdl, 'theta_tw', [1160 410 1255 430]);
addSignalSink(mdl, 'T_sh', [1160 450 1255 470]);

bus = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'BusSelector');
assert(~isempty(bus), 'Top-level PMSM Bus Selector was not found.');
bus = bus{1};
pmsm = [mdl '/PMSM1'];

busPorts = get_param(bus, 'PortHandles');
drvPorts = get_param(drv, 'PortHandles');
pmsmPorts = get_param(pmsm, 'PortHandles');

% Mechanical.Te is Bus Selector output 6 in this source model.
add_line(mdl, busPorts.Outport(6), drvPorts.Inport(1), 'autorouting', 'on');
add_line(mdl, get_param(wind, 'PortHandles').Outport, drvPorts.Inport(2), ...
    'autorouting', 'on');

% Keep PMSM speed-input mode (0speed1torque = 0), but replace the imposed
% Ramp/Saturation speed by the generator-side two-mass state omega_g.
oldSpeedLine = get_param(pmsmPorts.Inport(2), 'Line');
if oldSpeedLine ~= -1
    delete_line(oldSpeedLine);
end
add_line(mdl, drvPorts.Outport(2), pmsmPorts.Inport(2), 'autorouting', 'on');

connectOutput(mdl, drvPorts.Outport(1), 'omega_t');
connectOutput(mdl, drvPorts.Outport(2), 'omega_g');
connectOutput(mdl, drvPorts.Outport(3), 'theta_tw');
connectOutput(mdl, drvPorts.Outport(4), 'T_sh');

set_param([mdl '/0speed1torque'], 'Value', '0');
set_param(mdl, 'SimulationCommand', 'update');
save_system(mdl);
close_system(mdl, 0);
fprintf('Added Wind_TwoMass drivetrain and mechanical validation signals to %s.mdl\n', mdl);
end

function appendParameterInit(mdl)
initFcn = get_param(mdl, 'InitFcn');
marker = 'GFM_MWT_Nonlinear_Params.m';
if contains(initFcn, marker)
    return;
end
paramCall = sprintf('\nrun(fullfile(fileparts(get_param(bdroot,''FileName'')),''%s''));', marker);
set_param(mdl, 'InitFcn', [initFcn paramCall]);
end

function buildDrivetrainSubsystem(drv)
% Inputs and outputs.
add_block('simulink/Sources/In1', [drv '/T_e'], 'Port', '1', 'Position', [25 205 55 225]);
add_block('simulink/Sources/In1', [drv '/v_w'], 'Port', '2', 'Position', [25 35 55 55]);
add_block('simulink/Sinks/Out1', [drv '/omega_t'], 'Port', '1', 'Position', [735 40 765 60]);
add_block('simulink/Sinks/Out1', [drv '/omega_g'], 'Port', '2', 'Position', [735 110 765 130]);
add_block('simulink/Sinks/Out1', [drv '/theta_tw'], 'Port', '3', 'Position', [735 180 765 200]);
add_block('simulink/Sinks/Out1', [drv '/T_sh'], 'Port', '4', 'Position', [735 250 765 270]);

% Aerodynamic torque: Taero0 + Kv*(vw-vw0) - Daero*(omega_t-omega0).
add_block('simulink/Sources/Constant', [drv '/v_w0'], 'Value', 'v_w0', 'Position', [75 70 110 90]);
add_block('simulink/Math Operations/Sum', [drv '/Delta_v_w'], 'Inputs', '+-', 'Position', [135 35 160 85]);
add_block('simulink/Math Operations/Gain', [drv '/K_v_aero'], 'Gain', 'K_v_aero', 'Position', [185 35 260 65]);
add_block('simulink/Sources/Constant', [drv '/T_aero0'], 'Value', 'T_aero0', 'Position', [180 85 260 105]);
add_block('simulink/Sources/Constant', [drv '/omega_m0_t'], 'Value', 'omega_m0', 'Position', [75 125 150 145]);
add_block('simulink/Math Operations/Sum', [drv '/Delta_omega_t'], 'Inputs', '+-', 'Position', [285 100 310 150]);
add_block('simulink/Math Operations/Gain', [drv '/D_aero'], 'Gain', 'D_aero', 'Position', [335 105 400 135]);
add_block('simulink/Math Operations/Sum', [drv '/T_aero'], 'Inputs', '++-', 'Position', [430 35 455 105]);

% Shaft torque: Ksh*theta + Dsh*(omega_t-omega_g).
add_block('simulink/Math Operations/Sum', [drv '/Delta_omega_sh'], 'Inputs', '+-', 'Position', [280 225 305 275]);
add_block('simulink/Math Operations/Gain', [drv '/D_sh'], 'Gain', 'D_sh', 'Position', [335 235 400 265]);
add_block('simulink/Math Operations/Gain', [drv '/K_sh'], 'Gain', 'K_sh', 'Position', [335 285 400 315]);
add_block('simulink/Math Operations/Sum', [drv '/Shaft_Torque'], 'Inputs', '++', 'Position', [430 245 455 295]);

% Turbine-side acceleration.
add_block('simulink/Math Operations/Gain', [drv '/D_t'], 'Gain', 'D_t', 'Position', [335 145 400 175]);
add_block('simulink/Math Operations/Sum', [drv '/Turbine_Net_Torque'], 'Inputs', '+--', 'Position', [490 55 515 125]);
add_block('simulink/Math Operations/Gain', [drv '/Inv_J_t'], 'Gain', '1/J_t', 'Position', [545 70 605 100]);
add_block('simulink/Continuous/Integrator', [drv '/Omega_t_State'], ...
    'InitialCondition', 'omega_m0', 'Position', [630 35 665 65]);

% Generator-side acceleration.
add_block('simulink/Sources/Constant', [drv '/omega_m0_g'], 'Value', 'omega_m0', 'Position', [75 325 150 345]);
add_block('simulink/Math Operations/Sum', [drv '/Delta_omega_g'], 'Inputs', '+-', 'Position', [285 320 310 370]);
add_block('simulink/Math Operations/Gain', [drv '/D_g'], 'Gain', 'D_g', 'Position', [335 330 400 360]);
add_block('simulink/Math Operations/Sum', [drv '/Generator_Net_Torque'], 'Inputs', '++-', 'Position', [490 170 515 240]);
add_block('simulink/Math Operations/Gain', [drv '/Inv_J_g'], 'Gain', '1/J_g', 'Position', [545 185 605 215]);
add_block('simulink/Continuous/Integrator', [drv '/Omega_g_State'], ...
    'InitialCondition', 'omega_m0', 'Position', [630 100 665 130]);

% Shaft twist state.
add_block('simulink/Math Operations/Sum', [drv '/Relative_Speed'], 'Inputs', '+-', 'Position', [545 255 570 305]);
add_block('simulink/Continuous/Integrator', [drv '/Theta_tw_State'], ...
    'InitialCondition', 'theta_tw0', 'Position', [630 175 665 205]);

% Aerodynamics.
add_line(drv, 'v_w/1', 'Delta_v_w/1');
add_line(drv, 'v_w0/1', 'Delta_v_w/2');
add_line(drv, 'Delta_v_w/1', 'K_v_aero/1');
add_line(drv, 'K_v_aero/1', 'T_aero/2');
add_line(drv, 'T_aero0/1', 'T_aero/1');
add_line(drv, 'Omega_t_State/1', 'Delta_omega_t/1');
add_line(drv, 'omega_m0_t/1', 'Delta_omega_t/2');
add_line(drv, 'Delta_omega_t/1', 'D_aero/1');
add_line(drv, 'D_aero/1', 'T_aero/3');

% Shaft and turbine dynamics.
add_line(drv, 'Omega_t_State/1', 'Delta_omega_sh/1');
add_line(drv, 'Omega_g_State/1', 'Delta_omega_sh/2');
add_line(drv, 'Delta_omega_sh/1', 'D_sh/1');
add_line(drv, 'Theta_tw_State/1', 'K_sh/1');
add_line(drv, 'D_sh/1', 'Shaft_Torque/1');
add_line(drv, 'K_sh/1', 'Shaft_Torque/2');
add_line(drv, 'T_aero/1', 'Turbine_Net_Torque/1');
add_line(drv, 'Shaft_Torque/1', 'Turbine_Net_Torque/2');
add_line(drv, 'Delta_omega_t/1', 'D_t/1');
add_line(drv, 'D_t/1', 'Turbine_Net_Torque/3');
add_line(drv, 'Turbine_Net_Torque/1', 'Inv_J_t/1');
add_line(drv, 'Inv_J_t/1', 'Omega_t_State/1');

% Generator dynamics.
add_line(drv, 'Omega_g_State/1', 'Delta_omega_g/1');
add_line(drv, 'omega_m0_g/1', 'Delta_omega_g/2');
add_line(drv, 'Delta_omega_g/1', 'D_g/1');
add_line(drv, 'Shaft_Torque/1', 'Generator_Net_Torque/1');
add_line(drv, 'T_e/1', 'Generator_Net_Torque/2');
add_line(drv, 'D_g/1', 'Generator_Net_Torque/3');
add_line(drv, 'Generator_Net_Torque/1', 'Inv_J_g/1');
add_line(drv, 'Inv_J_g/1', 'Omega_g_State/1');

% Shaft-twist integration and exposed truth outputs.
add_line(drv, 'Omega_t_State/1', 'Relative_Speed/1');
add_line(drv, 'Omega_g_State/1', 'Relative_Speed/2');
add_line(drv, 'Relative_Speed/1', 'Theta_tw_State/1');
add_line(drv, 'Omega_t_State/1', 'omega_t/1');
add_line(drv, 'Omega_g_State/1', 'omega_g/1');
add_line(drv, 'Theta_tw_State/1', 'theta_tw/1');
add_line(drv, 'Shaft_Torque/1', 'T_sh/1');
end

function addSignalSink(mdl, name, pos)
sink = [mdl '/' name '_out'];
add_block('simulink/Sinks/To Workspace', sink, ...
    'VariableName', name, 'SaveFormat', 'Timeseries', ...
    'Decimation', 'mech_log_decimation', ...
    'Position', pos);
end

function connectOutput(mdl, sourcePort, name)
sink = [mdl '/' name '_out'];
sinkPorts = get_param(sink, 'PortHandles');
line = add_line(mdl, sourcePort, sinkPorts.Inport, 'autorouting', 'on');
set_param(line, 'Name', name);
end
