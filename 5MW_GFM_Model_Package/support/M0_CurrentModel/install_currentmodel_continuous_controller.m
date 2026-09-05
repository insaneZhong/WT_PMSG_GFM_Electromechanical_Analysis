function report = install_currentmodel_continuous_controller()
%INSTALL_CURRENTMODEL_CONTINUOUS_CONTROLLER
% Replaces only MOTOR_CONTROL1/S-Function1 inside the one maintained
% idealized copy.  The original two-mass drivetrain, PMSG, LCL, grid,
% measurements, ideal VSCs, and all external output wiring are retained.
% The replacement has 11 visible continuous Integrator states and an
% interpreted transparent RHS; no MEX controller state remains.

here=fileparts(mfilename('fullpath'));
mdl='Grid_Forming_PMSG5MW_TwoMass_Idealized';
load_system(fullfile(here,[mdl '.slx']));
ss=[mdl '/MOTOR_CONTROL1'];
old=[ss '/S-Function1'];
assert(~isempty(find_system(old,'SearchDepth',0)), ...
    'Expected legacy S-Function1 was not found.');

% Preserve the existing model initialization and append the unique, audited
% parameter/state initializer for the explicit continuous controller.
initOld=get_param(mdl,'InitFcn');
initCall=['run(fullfile(fileparts(get_param(bdroot,''FileName'')),' ...
    '''initialize_currentmodel_continuous_controller.m''));'];
if ~contains(initOld,'initialize_currentmodel_continuous_controller')
    set_param(mdl,'InitFcn',[initOld newline initCall]);
end

% Remove the only hidden dynamic controller implementation.  Its connected
% lines are removed by Simulink; Demux2 and all original Outport/Goto paths
% intentionally remain in place.
delete_block(old);
demuxLine=get_param([ss '/Demux2'],'LineHandles');
if isfield(demuxLine,'Inport') && demuxLine.Inport~=-1
    delete_line(demuxLine.Inport);
end

% Visible state network.
add_block('simulink/Signal Routing/Mux',[ss '/IdealCtrlStateMux'], ...
    'Inputs','11','Position',[350 90 355 330]);
add_block('simulink/Sources/Constant',[ss '/IdealCtrlParamVec'], ...
    'Value','IdealCtrlPVec','Position',[300 370 330 400]);
add_block('simulink/Signal Routing/Mux',[ss '/IdealCtrlInputMux'], ...
    'Inputs','3','Position',[405 210 410 265]);
add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', ...
    [ss '/IdealCtrlRHS'], ...
    'MATLABFcn','currentmodel_continuous_controller_io(u)', ...
    'OutputDimensions','52','OutputSignalType','real', ...
    'Output1D','on','SampleTime','0','Position',[455 215 555 260]);
add_block('simulink/Signal Routing/Demux',[ss '/IdealCtrlDerivativeSplit'], ...
    'Outputs','[11 41]','Position',[600 215 605 260]);
add_block('simulink/Signal Routing/Demux',[ss '/IdealCtrlDxSplit'], ...
    'Outputs','11','Position',[650 85 655 330]);

for k=1:11
    name=sprintf('%s/IdealCtrlInt%02d',ss,k);
    y=35+30*(k-1);
    add_block('simulink/Continuous/Integrator',name, ...
        'InitialCondition',sprintf('IdealCtrlX0(%d)',k), ...
        'Position',[705 y 735 y+20]);
    add_line(ss,sprintf('IdealCtrlDxSplit/%d',k), ...
        sprintf('IdealCtrlInt%02d/1',k),'autorouting','on');
    add_line(ss,sprintf('IdealCtrlInt%02d/1',k), ...
        sprintf('IdealCtrlStateMux/%d',k),'autorouting','on');
end

% Existing Mux3 is the original 20-signal S-Function input vector.
add_line(ss,'Mux3/1','IdealCtrlInputMux/1','autorouting','on');
add_line(ss,'IdealCtrlStateMux/1','IdealCtrlInputMux/2','autorouting','on');
add_line(ss,'IdealCtrlParamVec/1','IdealCtrlInputMux/3','autorouting','on');
add_line(ss,'IdealCtrlInputMux/1','IdealCtrlRHS/1','autorouting','on');
add_line(ss,'IdealCtrlRHS/1','IdealCtrlDerivativeSplit/1','autorouting','on');
add_line(ss,'IdealCtrlDerivativeSplit/1','IdealCtrlDxSplit/1','autorouting','on');
add_line(ss,'IdealCtrlDerivativeSplit/2','Demux2/1','autorouting','on');

set_param(mdl,'Solver','ode15s','SolverType','Variable-step', ...
    'RelTol','1e-6','MaxStep','1e-4','SimulationMode','normal');
set_param(mdl,'SimulationCommand','update');
save_system(mdl,fullfile(here,[mdl '.slx']));

report=struct('model',fullfile(here,[mdl '.slx']), ...
    'controllerImplementation','11 explicit continuous Integrators + transparent RHS', ...
    'hiddenMEXPresent',false,'stateCount',11, ...
    'sourceInputCount',20,'parameterCount',43,'legacyOutputCount',41);
fid=fopen(fullfile(here,'05_Continuous_Controller_Migration_CN.md'), ...
    'w','n','UTF-8');
fprintf(fid,'# 当前5 MW理想化副本：显式连续控制器迁移\n\n');
fprintf(fid,'- 唯一模型：`%s.slx`\n',mdl);
fprintf(fid,'- 已替换：`MOTOR_CONTROL1/S-Function1`\n');
fprintf(fid,'- 新控制器状态：11 个可见连续 Integrator\n');
fprintf(fid,'- 已移除：MEX 内部状态、控制采样、SVPWM、数字延迟、限幅、PLL/预同步、主动阻尼。\n');
fprintf(fid,'- 保留：原始两质量轴系、PMSG、LCL、电网、测量面和理想VSC端口。\n');
fprintf(fid,'- 后续门槛：先验证功率面/DC-link与平衡点，再比较六个输出的小扰动。\n');
fclose(fid);
close_system(mdl,0);
end
