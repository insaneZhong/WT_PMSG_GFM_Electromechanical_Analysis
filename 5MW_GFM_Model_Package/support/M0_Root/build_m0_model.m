function info = build_m0_model(varargin)
%BUILD_M0_MODEL 生成或原位更新唯一的5 MW M0理想连续平均模型。
%
%  模型文件：M0_PMSG_GFM_5MW.slx
%  连续状态：23个显式Integrator（顺序由m0_state_names唯一规定）
%  根输入：  u[6] = [dTm,dPref,dQref,dOmegaGrid,dVgrid_pu,dVdcRef]'
%  根输出：  x[23]、y[29]
%
%  本脚本不复制源开关模型、不创建其他SLX，也不保存仿真结果。
%  所有微分方程只由m0_nonlinear_dynamics.m维护，Simulink模型仅负责
%  显式状态积分、外部输入和诊断输出。

ip = inputParser;
ip.addParameter('Parameters',[],@(x)isempty(x)||isstruct(x));
ip.addParameter('Compile',true,@(x)islogical(x)&&isscalar(x));
ip.parse(varargin{:});

modelDir = fileparts(mfilename('fullpath'));
addpath(modelDir);
mdl = 'M0_PMSG_GFM_5MW';
modelFile = fullfile(modelDir,[mdl '.slx']);

% 编译缓存统一写入系统临时目录，避免在研究目录生成slprj等过程产物。
cacheRoot = fullfile(tempdir,'M0_5MW_IdealContinuous_Cache');
codegenRoot = fullfile(tempdir,'M0_5MW_IdealContinuous_Codegen');
if ~isfolder(cacheRoot), mkdir(cacheRoot); end
if ~isfolder(codegenRoot), mkdir(codegenRoot); end
Simulink.fileGenControl('set','CacheFolder',cacheRoot, ...
    'CodeGenFolder',codegenRoot,'createDir',true);

P = ip.Results.Parameters;
if isempty(P)
    P = init_m0_5mw_parameters();
end
[OP,P] = solve_m0_equilibrium(P);
[pvec,pnames] = m0_pack_parameters(P,OP);
stateNames = m0_state_names();
outputNames = m0_output_names();

assert(numel(OP.x0)==23,'M0平衡点必须严格包含23个状态。');
assert(numel(stateNames)==23,'m0_state_names必须严格包含23个状态名。');
assert(numel(pvec)==43,'M0参数向量必须严格包含43个参数。');
assert(numel(pnames)==43,'M0参数名必须严格包含43项。');
assert(numel(outputNames)==29,'m0_output_names必须严格包含29个输出名。');
assert(all(isfinite(OP.x0))&&all(isfinite(pvec)), ...
    'M0模型工作区初值或参数中存在NaN/Inf。');

% 只操作本模型，绝不关闭用户已经打开的其他模型。
if bdIsLoaded(mdl)
    close_system(mdl,0);
end
if isfile(modelFile)
    load_system(modelFile);
else
    new_system(mdl,'Model');
end

% 原位重建根层结构。删除的是模型内的块和连线，不涉及磁盘文件。
clearSubsystemContents(mdl);

%% 根层接口和单一连续RHS
add_block('simulink/Sources/In1',[mdl '/u'], ...
    'Position',[35 285 65 305], ...
    'Port','1','PortDimensions','6', ...
    'OutDataTypeStr','double','SampleTime','0');

add_block('simulink/Sources/Constant',[mdl '/M0_Parameters'], ...
    'Position',[390 390 535 425], ...
    'Value','M0_pvec','VectorParams1D','on', ...
    'OutDataTypeStr','double','SampleTime','inf');

stateSS = [mdl '/Continuous_States_23'];
add_block('simulink/Ports & Subsystems/Subsystem',stateSS, ...
    'Position',[145 145 365 245], ...
    'BackgroundColor','lightBlue');
clearSubsystemContents(stateSS);
buildStateIntegratorSubsystem(stateSS,stateNames);

rhsBlk = [mdl '/M0_RHS'];
add_block('simulink/User-Defined Functions/MATLAB Function',rhsBlk, ...
    'Position',[585 185 785 345]);
setMatlabFunctionScript(rhsBlk);

add_block('simulink/Sinks/Out1',[mdl '/x'], ...
    'Position',[885 145 915 165],'Port','1');
add_block('simulink/Sinks/Out1',[mdl '/y'], ...
    'Position',[885 315 915 335],'Port','2');

% RHS端口顺序由函数签名固定：x、u、p -> dx、y。
lh = add_line(mdl,'Continuous_States_23/1','M0_RHS/1', ...
    'autorouting','on');
set_param(lh,'Name','x');
add_line(mdl,'u/1','M0_RHS/2','autorouting','on');
add_line(mdl,'M0_Parameters/1','M0_RHS/3','autorouting','on');
add_line(mdl,'M0_RHS/1','Continuous_States_23/1','autorouting','on');
lh = add_line(mdl,'M0_RHS/2','y/1','autorouting','on');
set_param(lh,'Name','y');
add_line(mdl,'Continuous_States_23/1','x/1','autorouting','on');

%% 模型工作区：只嵌入可直接运行所需的初值和参数
mws = get_param(mdl,'ModelWorkspace');
assignin(mws,'M0_x0',OP.x0(:));
assignin(mws,'M0_pvec',pvec(:));

%% 唯一连续仿真配置
set_param(mdl, ...
    'SolverType','Variable-step', ...
    'Solver','ode15s', ...
    'StartTime','0', ...
    'StopTime','0.2', ...
    'RelTol','1e-7', ...
    'AbsTol','auto', ...
    'MaxStep','2e-5', ...
    'AlgebraicLoopMsg','error', ...
    'UnconnectedInputMsg','error', ...
    'UnconnectedOutputMsg','warning', ...
    'SignalLogging','off', ...
    'SaveState','off', ...
    'SaveOutput','on', ...
    'OutputSaveName','yout', ...
    'SaveFormat','Dataset', ...
    'SaveTime','on', ...
    'TimeSaveName','tout', ...
    'ReturnWorkspaceOutputs','on');

cs = getActiveConfigSet(mdl);
try
    cs.Name = 'M0_IdealContinuous_Config';
catch
    % 少数旧版本不允许直接重命名活动配置；不复制第二个ConfigSet。
end

set_param(mdl,'InitFcn',[ ...
    'm0ModelDir=fileparts(get_param(bdroot,''FileName''));' ...
    'if ~isempty(m0ModelDir), addpath(m0ModelDir); end']);
set_param(mdl,'Description',[ ...
    '5 MW PMSG M0 ideal continuous average model. ' ...
    '23 explicit continuous states; no PWM, digital delay, limiter, ' ...
    'PLL/presynchronization, active damping, MPPT or pitch dynamics.']);

Simulink.Annotation(mdl,[ ...
    'M0：5 MW构网型PMSG理想连续平均模型' newline ...
    '三功率面：P_MSC,dc / P_GSC,dc / P_PCC；严格DC-link能量方程' newline ...
    '状态映射：m0_state_names.m；输出映射：m0_output_names.m']);

% 首次保存后再编译；若编译失败，仍只留下这一份可诊断模型。
if isfile(modelFile)
    save_system(mdl);
else
    save_system(mdl,modelFile);
end

compiled = false;
if ip.Results.Compile
    set_param(mdl,'SimulationCommand','update');
    compiled = true;
    save_system(mdl);
end

integrators = find_system(stateSS, ...
    'LookUnderMasks','all','FollowLinks','on', ...
    'BlockType','Integrator');
assert(numel(integrators)==23, ...
    '生成模型内Integrator数量不是23，而是%d。',numel(integrators));

info = struct();
info.model_name = mdl;
info.model_path = modelFile;
info.compiled = compiled;
info.integrator_count = numel(integrators);
info.state_count = numel(stateNames);
info.output_count = numel(outputNames);
info.parameter_count = numel(pvec);
info.max_normalized_equilibrium_residual = OP.max_normalized_residual;
info.dc_power_mismatch_W = OP.dc_power_mismatch_W;

fprintf(['M0 model: %s\nIntegrator count: %d\nCompiled: %d\n' ...
    'Equilibrium max normalized residual: %.3e\n'], ...
    modelFile,info.integrator_count,info.compiled, ...
    info.max_normalized_equilibrium_residual);
end

function buildStateIntegratorSubsystem(ss,stateNames)
% 23个标量Integrator确保每个物理状态在Simulink中显式可见。
n = numel(stateNames);
add_block('simulink/Sources/In1',[ss '/dx'], ...
    'Position',[25 45 55 65], ...
    'Port','1','PortDimensions',num2str(n));
add_block('simulink/Signal Routing/Demux',[ss '/dx_Demux'], ...
    'Position',[95 25 100 25+42*n], ...
    'Outputs',num2str(n));
add_block('simulink/Signal Routing/Mux',[ss '/x_Mux'], ...
    'Position',[430 25 435 25+42*n], ...
    'Inputs',num2str(n));
add_block('simulink/Sinks/Out1',[ss '/x'], ...
    'Position',[485 45 515 65],'Port','1');

add_line(ss,'dx/1','dx_Demux/1','autorouting','on');
for k = 1:n
    safeName = regexprep(char(stateNames(k)),'[^A-Za-z0-9_]','_');
    blockName = sprintf('x%02d_%s',k,safeName);
    y = 22+42*(k-1);
    add_block('simulink/Continuous/Integrator',[ss '/' blockName], ...
        'Position',[190 y 310 y+25], ...
        'InitialCondition',sprintf('M0_x0(%d)',k));
    add_line(ss,sprintf('dx_Demux/%d',k), ...
        [blockName '/1'],'autorouting','on');
    add_line(ss,[blockName '/1'], ...
        sprintf('x_Mux/%d',k),'autorouting','on');
end
add_line(ss,'x_Mux/1','x/1','autorouting','on');
end

function setMatlabFunctionScript(blockPath)
rhsCode = sprintf([ ...
    'function [dx,y] = M0_RHS(x,u,p)\n' ...
    '%%#codegen\n' ...
    'dx = zeros(23,1);\n' ...
    'y = zeros(29,1);\n' ...
    '[dx,y] = m0_nonlinear_dynamics(x,u,p);\n' ...
    'end\n']);
rt = sfroot;
chart = rt.find('-isa','Stateflow.EMChart','Path',blockPath);
assert(isscalar(chart), ...
    '无法唯一定位MATLAB Function块：%s。',blockPath);
chart.Script = rhsCode;
end

function clearSubsystemContents(sys)
% 原位清除一个Block Diagram或Subsystem内部的块和连线。
lines = find_system(sys,'FindAll','on','SearchDepth',1,'Type','line');
for k = 1:numel(lines)
    try
        delete_line(lines(k));
    catch
    end
end
blocks = find_system(sys,'SearchDepth',1,'Type','Block');
blocks(strcmp(blocks,sys)) = [];
for k = numel(blocks):-1:1
    delete_block(blocks{k});
end
annotations = find_system(sys,'FindAll','on','SearchDepth',1, ...
    'Type','annotation');
for k = 1:numel(annotations)
    try
        delete(annotations(k));
    catch
    end
end
end
