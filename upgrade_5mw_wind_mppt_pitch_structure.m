function report = upgrade_5mw_wind_mppt_pitch_structure()
%UPGRADE_5MW_WIND_MPPT_PITCH_STRUCTURE Add the active 5 MW wind hierarchy.
% The function edits the single working model in place and never creates a
% model copy.  It is intentionally idempotent: an existing hierarchy is
% inspected and reused rather than duplicated.

root = fileparts(mfilename('fullpath'));
model = 'Grid_Forming_PMSG5MW_Liu2024_TwoMass';
oldFolder = pwd;
folderCleanup = onCleanup(@() cd(oldFolder)); %#ok<NASGU>
cd(root);
load_system(model);
modelCleanup = onCleanup(@() close_system(model,0)); %#ok<NASGU>
run(fullfile(root,'TwoMass_Stage4_Params_5MW_Liu2024.m'));

windPlant = [model '/Wind_Turbine_Aero_MPPT_Pitch'];
if getSimulinkBlockHandle(windPlant) == -1
    add_block('simulink/Ports & Subsystems/Subsystem',windPlant, ...
        'Position',[1040 1210 1300 1390], ...
        'BackgroundColor','lightBlue');
    buildWindPlant(windPlant);
end

pmpptOut = [windPlant '/P_mppt'];
if getSimulinkBlockHandle(pmpptOut) == -1
    add_block('simulink/Ports & Subsystems/Out1',pmpptOut, ...
        'Port','7','Position',[790 430 820 450]);
end
connectIfNeeded(windPlant,'MPPT_Power_Manager/1','P_mppt/1');
retrofitMpptRatedBlend([windPlant '/MPPT_Power_Manager']);
set_param([windPlant '/Startup_Coordinator/release_time'], ...
    'Value','stage4_start_s+aero_release_delay_s');

controller = [model '/MOTOR_CONTROL1'];
prefIn = [controller '/P_ref_MPPT'];
if getSimulinkBlockHandle(prefIn) == -1
    add_block('simulink/Ports & Subsystems/In1',prefIn, ...
        'Port','18','Position',[20 1045 50 1065]);
end
set_param([controller '/Mux3'],'Inputs','18');
connectIfNeeded(controller,'P_ref_MPPT/1','Mux3/18');

drvPorts = get_param([model '/Drivetrain_TwoMass'],'PortHandles');
clockPorts = get_param([model '/Clock'],'PortHandles');
windPorts = get_param(windPlant,'PortHandles');
controllerPorts = get_param(controller,'PortHandles');
addLineByHandleIfNeeded(model,drvPorts.Outport(1),windPorts.Inport(1));
addLineByHandleIfNeeded(model,clockPorts.Outport,windPorts.Inport(2));
addLineByHandleIfNeeded(model,windPorts.Outport(2),controllerPorts.Inport(18));

replaceInputSource(model,[model '/TaeroEffective'],1,windPorts.Outport(1));
replaceInputSource(model,[model '/stage4_Taero'],1,windPorts.Outport(1));

addLog(model,windPorts.Outport(2),'wind_Pref_mppt', ...
    [1390 1210 1510 1230]);
addLog(model,windPorts.Outport(3),'wind_beta_deg', ...
    [1390 1245 1510 1265]);
addLog(model,windPorts.Outport(4),'wind_lambda', ...
    [1390 1280 1510 1300]);
addLog(model,windPorts.Outport(5),'wind_Cp', ...
    [1390 1315 1510 1335]);
addLog(model,windPorts.Outport(6),'wind_Paero', ...
    [1390 1350 1510 1370]);
addLog(model,windPorts.Outport(7),'wind_Pmppt_raw', ...
    [1390 1385 1510 1405]);

legacyBlocks = {'T_aero_ramp','T_aero_limit','AeroSpeedError', ...
    'AeroSpeedDamping','TaeroWithSpeedRestoration','omega_m0_aero'};
for k = 1:numel(legacyBlocks)
    block = [model '/' legacyBlocks{k}];
    if getSimulinkBlockHandle(block) ~= -1
        set_param(block,'ForegroundColor','gray');
    end
end

% A newly inserted library Subsystem contains default In1/Out1 blocks.
% Explicitly remove those unused placeholders after the named interface has
% been built; otherwise they appear as confusing extra ports.
interfaceSubsystems = {windPlant,[windPlant '/Pitch_Controller'], ...
    [windPlant '/MPPT_Power_Manager'],[windPlant '/Startup_Coordinator'], ...
    [windPlant '/Wind_Aero']};
for k = 1:numel(interfaceSubsystems)
    defaultIn = [interfaceSubsystems{k} '/In1'];
    defaultOut = [interfaceSubsystems{k} '/Out1'];
    if getSimulinkBlockHandle(defaultIn) ~= -1
        delete_block(defaultIn);
    end
    if getSimulinkBlockHandle(defaultOut) ~= -1
        delete_block(defaultOut);
    end
    cleanupDanglingLines(interfaceSubsystems{k});
end

set_param(model,'SimulationCommand','update');
save_system(model);
report = struct('model',model,'wind_subsystem',windPlant, ...
    'dynamic_pref_port',18,'edited_in_place',true);
disp(report);
end

function buildWindPlant(parent)
add_block('simulink/Ports & Subsystems/In1',[parent '/omega_t'], ...
    'Port','1','Position',[25 45 55 65]);
add_block('simulink/Ports & Subsystems/In1',[parent '/clock'], ...
    'Port','2','Position',[25 305 55 325]);
add_block('simulink/Sources/Constant',[parent '/Wind_Speed_12mps'], ...
    'Value','v_w0','Position',[25 115 90 145]);

pitch = [parent '/Pitch_Controller'];
mppt = [parent '/MPPT_Power_Manager'];
aero = [parent '/Wind_Aero'];
startup = [parent '/Startup_Coordinator'];
add_block('simulink/Ports & Subsystems/Subsystem',pitch, ...
    'Position',[150 30 340 145],'BackgroundColor','yellow');
add_block('simulink/Ports & Subsystems/Subsystem',mppt, ...
    'Position',[150 175 340 270],'BackgroundColor','green');
add_block('simulink/Ports & Subsystems/Subsystem',startup, ...
    'Position',[150 295 340 355],'BackgroundColor','orange');
add_block('simulink/Ports & Subsystems/Subsystem',aero, ...
    'Position',[420 55 620 205],'BackgroundColor','lightBlue');
buildPitch(pitch);
buildMppt(mppt);
buildStartup(startup);
buildAero(aero);

add_block('simulink/Math Operations/Product',[parent '/Apply_Startup_To_Torque'], ...
    'Inputs','**','Position',[680 80 710 120]);
add_block('simulink/Math Operations/Product',[parent '/Apply_Startup_To_Power'], ...
    'Inputs','**','Position',[680 160 710 200]);

outNames = {'T_aero','P_ref','beta_deg','lambda','Cp','P_aero'};
outY = [90 235 275 315 355 395];
for k = 1:numel(outNames)
    add_block('simulink/Ports & Subsystems/Out1', ...
        [parent '/' outNames{k}],'Port',num2str(k), ...
        'Position',[790 outY(k) 820 outY(k)+20]);
end

add_line(parent,'omega_t/1','Pitch_Controller/1');
add_line(parent,'omega_t/1','MPPT_Power_Manager/1');
add_line(parent,'omega_t/1','Wind_Aero/2');
add_line(parent,'Wind_Speed_12mps/1','Pitch_Controller/2');
add_line(parent,'Wind_Speed_12mps/1','MPPT_Power_Manager/2');
add_line(parent,'Wind_Speed_12mps/1','Wind_Aero/1');
add_line(parent,'clock/1','Startup_Coordinator/1');
add_line(parent,'Pitch_Controller/1','Wind_Aero/3');
add_line(parent,'Wind_Aero/2','Apply_Startup_To_Torque/1');
add_line(parent,'Startup_Coordinator/1','Apply_Startup_To_Torque/2');
add_line(parent,'Wind_Aero/1','Apply_Startup_To_Power/1');
add_line(parent,'Startup_Coordinator/1','Apply_Startup_To_Power/2');
add_line(parent,'Apply_Startup_To_Torque/1','T_aero/1');
add_line(parent,'MPPT_Power_Manager/2','P_ref/1');
add_line(parent,'Pitch_Controller/1','beta_deg/1');
add_line(parent,'Wind_Aero/3','lambda/1');
add_line(parent,'Wind_Aero/4','Cp/1');
add_line(parent,'Apply_Startup_To_Power/1','P_aero/1');
end

function buildPitch(subsys)
add_block('simulink/Ports & Subsystems/In1',[subsys '/omega_t'], ...
    'Port','1','Position',[20 45 50 65]);
add_block('simulink/Ports & Subsystems/In1',[subsys '/wind_speed'], ...
    'Port','2','Position',[20 125 50 145]);
add_block('simulink/Sources/Constant',[subsys '/omega_rated'], ...
    'Value','omega_m0','Position',[75 75 135 95]);
add_block('simulink/Math Operations/Sum',[subsys '/speed_error'], ...
    'Inputs','+-','Position',[160 40 185 80]);
add_block('simulink/Math Operations/Gain',[subsys '/Kp_pitch'], ...
    'Gain','pitch_kp_deg_per_radps','Position',[215 35 285 65]);
add_block('simulink/Math Operations/Gain',[subsys '/Ki_pitch'], ...
    'Gain','pitch_ki_deg_per_rad','Position',[215 80 285 110]);
add_block('simulink/Continuous/Integrator',[subsys '/Pitch_Integrator'], ...
    'InitialCondition','0','Position',[315 80 345 110]);
add_block('simulink/Sources/Constant',[subsys '/wind_rated'], ...
    'Value','rated_wind_speed','Position',[75 155 135 175]);
add_block('simulink/Math Operations/Sum',[subsys '/wind_excess'], ...
    'Inputs','+-','Position',[160 125 185 165]);
add_block('simulink/Math Operations/Gain',[subsys '/Pitch_Feedforward'], ...
    'Gain','pitch_beta_ff_gain_deg_per_mps','Position',[215 130 310 160]);
add_block('simulink/Math Operations/Sum',[subsys '/beta_unsat'], ...
    'Inputs','+++','Position',[380 55 405 125]);
add_block('simulink/Discontinuities/Rate Limiter',[subsys '/Pitch_Rate_Limit'], ...
    'RisingSlewLimit','pitch_rate_deg_per_s', ...
    'FallingSlewLimit','-pitch_rate_deg_per_s', ...
    'Position',[440 70 520 110]);
add_block('simulink/Discontinuities/Saturation',[subsys '/Pitch_Limits'], ...
    'LowerLimit','0','UpperLimit','pitch_beta_max_deg', ...
    'Position',[555 70 625 110]);
add_block('simulink/Ports & Subsystems/Out1',[subsys '/beta_deg'], ...
    'Port','1','Position',[675 80 705 100]);
add_line(subsys,'omega_t/1','speed_error/1');
add_line(subsys,'omega_rated/1','speed_error/2');
add_line(subsys,'speed_error/1','Kp_pitch/1');
add_line(subsys,'speed_error/1','Ki_pitch/1');
add_line(subsys,'Ki_pitch/1','Pitch_Integrator/1');
add_line(subsys,'wind_speed/1','wind_excess/1');
add_line(subsys,'wind_rated/1','wind_excess/2');
add_line(subsys,'wind_excess/1','Pitch_Feedforward/1');
add_line(subsys,'Kp_pitch/1','beta_unsat/1');
add_line(subsys,'Pitch_Integrator/1','beta_unsat/2');
add_line(subsys,'Pitch_Feedforward/1','beta_unsat/3');
add_line(subsys,'beta_unsat/1','Pitch_Rate_Limit/1');
add_line(subsys,'Pitch_Rate_Limit/1','Pitch_Limits/1');
add_line(subsys,'Pitch_Limits/1','beta_deg/1');
end

function buildMppt(subsys)
add_block('simulink/Ports & Subsystems/In1',[subsys '/omega_t'], ...
    'Port','1','Position',[20 40 50 60]);
add_block('simulink/Ports & Subsystems/In1',[subsys '/wind_speed'], ...
    'Port','2','Position',[20 115 50 135]);
add_block('simulink/Math Operations/Product',[subsys '/omega_cubed'], ...
    'Inputs','***','Position',[100 25 130 75]);
add_block('simulink/Math Operations/Gain',[subsys '/Kopt'], ...
    'Gain','K_opt','Position',[165 35 230 65]);
add_block('simulink/Discontinuities/Saturation',[subsys '/MPPT_Limit'], ...
    'LowerLimit','0','UpperLimit','P_wt_rated', ...
    'Position',[265 35 335 65]);
add_block('simulink/Sources/Constant',[subsys '/Rated_Power'], ...
    'Value','P_wt_rated','Position',[320 95 390 115]);
add_block('simulink/Signal Routing/Switch',[subsys '/Below_Above_Rated'], ...
    'Criteria','u2 >= Threshold','Threshold','rated_wind_speed', ...
    'Position',[385 65 430 125]);
add_block('simulink/Ports & Subsystems/Out1',[subsys '/P_mppt'], ...
    'Port','1','Position',[485 35 515 55]);
add_block('simulink/Ports & Subsystems/Out1',[subsys '/P_ref'], ...
    'Port','2','Position',[485 95 515 115]);
add_line(subsys,'omega_t/1','omega_cubed/1');
add_line(subsys,'omega_t/1','omega_cubed/2');
add_line(subsys,'omega_t/1','omega_cubed/3');
add_line(subsys,'omega_cubed/1','Kopt/1');
add_line(subsys,'Kopt/1','MPPT_Limit/1');
add_line(subsys,'MPPT_Limit/1','P_mppt/1');
add_line(subsys,'Rated_Power/1','Below_Above_Rated/1');
add_line(subsys,'wind_speed/1','Below_Above_Rated/2');
add_line(subsys,'MPPT_Limit/1','Below_Above_Rated/3');
add_line(subsys,'Below_Above_Rated/1','P_ref/1');
end

function buildStartup(subsys)
add_block('simulink/Ports & Subsystems/In1',[subsys '/clock'], ...
    'Port','1','Position',[20 45 50 65]);
add_block('simulink/Sources/Constant',[subsys '/release_time'], ...
    'Value','stage4_start_s+aero_release_delay_s', ...
    'Position',[75 80 145 100]);
add_block('simulink/Math Operations/Sum',[subsys '/elapsed'], ...
    'Inputs','+-','Position',[175 40 200 80]);
add_block('simulink/Math Operations/Gain',[subsys '/to_fraction'], ...
    'Gain','1/stage4_ramp_duration_s','Position',[235 45 315 75]);
add_block('simulink/Discontinuities/Saturation',[subsys '/fraction_limits'], ...
    'LowerLimit','0','UpperLimit','1','Position',[350 45 415 75]);
add_block('simulink/Ports & Subsystems/Out1',[subsys '/power_fraction'], ...
    'Port','1','Position',[460 50 490 70]);
add_line(subsys,'clock/1','elapsed/1');
add_line(subsys,'release_time/1','elapsed/2');
add_line(subsys,'elapsed/1','to_fraction/1');
add_line(subsys,'to_fraction/1','fraction_limits/1');
add_line(subsys,'fraction_limits/1','power_fraction/1');
end

function buildAero(subsys)
add_block('simulink/Ports & Subsystems/In1',[subsys '/wind_speed'], ...
    'Port','1','Position',[20 45 50 65]);
add_block('simulink/Ports & Subsystems/In1',[subsys '/omega_t'], ...
    'Port','2','Position',[20 115 50 135]);
add_block('simulink/Ports & Subsystems/In1',[subsys '/beta_deg'], ...
    'Port','3','Position',[20 185 50 205]);
add_block('simulink/Discontinuities/Saturation',[subsys '/Wind_Minimum'], ...
    'LowerLimit','0.5','UpperLimit','60','Position',[85 35 155 75]);
add_block('simulink/Math Operations/Gain',[subsys '/Rotor_Radius'], ...
    'Gain','rotor_radius','Position',[85 105 155 135]);
add_block('simulink/Math Operations/Product',[subsys '/Tip_Speed_Ratio'], ...
    'Inputs','*/','Position',[195 65 225 115]);
add_block(sprintf('simulink/Lookup\nTables/2-D Lookup\nTable'), ...
    [subsys '/Cp_lambda_beta'], ...
    'BreakpointsForDimension1','cp_lambda_bp', ...
    'BreakpointsForDimension2','cp_beta_bp', ...
    'Table','cp_table','Position',[305 105 430 165]);
add_block('simulink/Math Operations/Product',[subsys '/wind_cubed'], ...
    'Inputs','***','Position',[195 10 225 55]);
add_block('simulink/Math Operations/Gain',[subsys '/Half_rho_A'], ...
    'Gain','0.5*air_density*rotor_area','Position',[270 20 370 50]);
add_block('simulink/Math Operations/Product',[subsys '/Aerodynamic_Power'], ...
    'Inputs','**','Position',[470 60 500 105]);
add_block('simulink/Discontinuities/Saturation',[subsys '/Omega_Minimum'], ...
    'LowerLimit','0.2','UpperLimit','3','Position',[305 190 375 220]);
add_block('simulink/Math Operations/Product',[subsys '/Aerodynamic_Torque'], ...
    'Inputs','*/','Position',[545 105 575 155]);
add_block('simulink/Ports & Subsystems/Out1',[subsys '/P_aero'], ...
    'Port','1','Position',[640 65 670 85]);
add_block('simulink/Ports & Subsystems/Out1',[subsys '/T_aero'], ...
    'Port','2','Position',[640 125 670 145]);
add_block('simulink/Ports & Subsystems/Out1',[subsys '/lambda'], ...
    'Port','3','Position',[640 175 670 195]);
add_block('simulink/Ports & Subsystems/Out1',[subsys '/Cp'], ...
    'Port','4','Position',[640 220 670 240]);
add_line(subsys,'wind_speed/1','Wind_Minimum/1');
add_line(subsys,'Wind_Minimum/1','Tip_Speed_Ratio/2');
add_line(subsys,'Wind_Minimum/1','wind_cubed/1');
add_line(subsys,'Wind_Minimum/1','wind_cubed/2');
add_line(subsys,'Wind_Minimum/1','wind_cubed/3');
add_line(subsys,'omega_t/1','Rotor_Radius/1');
add_line(subsys,'Rotor_Radius/1','Tip_Speed_Ratio/1');
add_line(subsys,'Tip_Speed_Ratio/1','Cp_lambda_beta/1');
add_line(subsys,'beta_deg/1','Cp_lambda_beta/2');
add_line(subsys,'wind_cubed/1','Half_rho_A/1');
add_line(subsys,'Half_rho_A/1','Aerodynamic_Power/1');
add_line(subsys,'Cp_lambda_beta/1','Aerodynamic_Power/2');
add_line(subsys,'Aerodynamic_Power/1','P_aero/1');
add_line(subsys,'Aerodynamic_Power/1','Aerodynamic_Torque/1');
add_line(subsys,'omega_t/1','Omega_Minimum/1');
add_line(subsys,'Omega_Minimum/1','Aerodynamic_Torque/2');
add_line(subsys,'Aerodynamic_Torque/1','T_aero/1');
add_line(subsys,'Tip_Speed_Ratio/1','lambda/1');
add_line(subsys,'Cp_lambda_beta/1','Cp/1');
end

function connectIfNeeded(parent,src,dst)
dstBlock = extractBefore(dst,'/');
dstPort = str2double(extractAfter(dst,'/'));
ph = get_param([parent '/' dstBlock],'PortHandles');
if get_param(ph.Inport(dstPort),'Line') == -1
    add_line(parent,src,dst,'autorouting','on');
end
end

function addLineByHandleIfNeeded(model,srcPort,dstPort)
if get_param(dstPort,'Line') == -1
    add_line(model,srcPort,dstPort,'autorouting','on');
end
end

function replaceInputSource(model,dstBlock,dstIndex,srcPort)
dstPorts = get_param(dstBlock,'PortHandles');
oldLine = get_param(dstPorts.Inport(dstIndex),'Line');
if oldLine ~= -1
    delete_line(oldLine);
end
add_line(model,srcPort,dstPorts.Inport(dstIndex),'autorouting','on');
end

function addLog(model,srcPort,varName,position)
block = [model '/' varName];
if getSimulinkBlockHandle(block) == -1
    add_block('simulink/Sinks/To Workspace',block, ...
        'VariableName',varName,'SaveFormat','Timeseries', ...
        'MaxDataPoints','200000','Decimation','1000', ...
        'Position',position);
end
set_param(block,'VariableName',varName,'SaveFormat','Timeseries', ...
    'MaxDataPoints','200000','Decimation','1000');
ports = get_param(block,'PortHandles');
addLineByHandleIfNeeded(model,srcPort,ports.Inport);
end


function retrofitMpptRatedBlend(mppt)
% Standard variable-speed wind-turbine power management:
% below rated use Kopt*omega^3; above rated command constant rated power
% and let the pitch loop regulate rotor speed. Region 2.5 blends the two
% references so a low-speed rotor is never forced to export rated power.
ratedPower = [mppt '/Rated_Power'];
if getSimulinkBlockHandle(ratedPower) == -1
    add_block('simulink/Sources/Constant',ratedPower, ...
        'Value','P_wt_rated','Position',[320 95 390 115]);
end
obsolete = {'Below_Above_Rated','omega_rated_recovery','recovery_speed_error', ...
    'K_speed_recovery','Rated_Power_Recovery','Recovery_Power', ...
    'Recovery_Limit','K_rated_recovery'};
for k = 1:numel(obsolete)
    block = [mppt '/' obsolete{k}];
    if getSimulinkBlockHandle(block) ~= -1
        delete_block(block);
    end
end

blocks = { ...
    'Blend_Low_Speed','simulink/Sources/Constant', ...
        {'Value','mppt_rated_blend_low_pu*omega_m0','Position',[260 105 350 125]}; ...
    'Speed_Above_Low','simulink/Math Operations/Sum', ...
        {'Inputs','+-','Position',[380 75 405 115]}; ...
    'Blend_Normalize','simulink/Math Operations/Gain', ...
        {'Gain','1/((mppt_rated_blend_high_pu-mppt_rated_blend_low_pu)*omega_m0)', ...
         'Position',[435 80 535 110]}; ...
    'Blend_Fraction','simulink/Discontinuities/Saturation', ...
        {'LowerLimit','0','UpperLimit','1','Position',[565 80 630 110]}; ...
    'One','simulink/Sources/Constant', ...
        {'Value','1','Position',[565 145 600 165]}; ...
    'One_Minus_Blend','simulink/Math Operations/Sum', ...
        {'Inputs','+-','Position',[640 125 665 165]}; ...
    'Rated_Contribution','simulink/Math Operations/Product', ...
        {'Inputs','**','Position',[690 70 720 105]}; ...
    'Mppt_Contribution','simulink/Math Operations/Product', ...
        {'Inputs','**','Position',[690 125 720 160]}; ...
    'Region_2p5_Power','simulink/Math Operations/Sum', ...
        {'Inputs','++','Position',[750 95 775 135]}};
for k = 1:size(blocks,1)
    block = [mppt '/' blocks{k,1}];
    if getSimulinkBlockHandle(block) == -1
        args = blocks{k,3};
        add_block(blocks{k,2},block,args{:});
    end
end
connectIfNeeded(mppt,'omega_t/1','Speed_Above_Low/1');
connectIfNeeded(mppt,'Blend_Low_Speed/1','Speed_Above_Low/2');
connectIfNeeded(mppt,'Speed_Above_Low/1','Blend_Normalize/1');
connectIfNeeded(mppt,'Blend_Normalize/1','Blend_Fraction/1');
connectIfNeeded(mppt,'Blend_Fraction/1','Rated_Contribution/2');
connectIfNeeded(mppt,'Rated_Power/1','Rated_Contribution/1');
connectIfNeeded(mppt,'One/1','One_Minus_Blend/1');
connectIfNeeded(mppt,'Blend_Fraction/1','One_Minus_Blend/2');
connectIfNeeded(mppt,'MPPT_Limit/1','Mppt_Contribution/1');
connectIfNeeded(mppt,'One_Minus_Blend/1','Mppt_Contribution/2');
connectIfNeeded(mppt,'Rated_Contribution/1','Region_2p5_Power/1');
connectIfNeeded(mppt,'Mppt_Contribution/1','Region_2p5_Power/2');
regionPorts = get_param([mppt '/Region_2p5_Power'],'PortHandles');
replaceInputSource(mppt,[mppt '/P_ref'],1,regionPorts.Outport);
end

function cleanupDanglingLines(subsystem)
lines = find_system(subsystem,'FindAll','on','Type','line');
for lineIndex = 1:numel(lines)
    try
        src = get_param(lines(lineIndex),'SrcPortHandle');
        dst = get_param(lines(lineIndex),'DstPortHandle');
    catch
        continue;
    end
    if src == -1 || isempty(dst) || all(dst == -1)
        delete_line(lines(lineIndex));
    end
end
end
