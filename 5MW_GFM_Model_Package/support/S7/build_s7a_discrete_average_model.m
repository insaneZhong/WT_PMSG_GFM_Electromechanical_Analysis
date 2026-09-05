function info=build_s7a_discrete_average_model(varargin)
%BUILD_S7A_DISCRETE_AVERAGE_MODEL  建立唯一的S7A离散平均Simulink副本。
%
% 模型名：S7A_DiscreteAvg_5MW
% 物理plant在采样周期内连续积分；控制器状态在采样时刻按
% S7_Controller_State_Audit.csv 中的Forward-Euler候选实现更新。
% 此模型不修改M0、不含PWM/开关/限幅/保护，专门用于Gate V2。

ip=inputParser;
ip.addParameter('Ts',[],@(x)isempty(x)||(isnumeric(x)&&isscalar(x)&&x>0));
ip.addParameter('Delay',[],@(x)isempty(x)||(isnumeric(x)&&isscalar(x)&&x>=0));
ip.addParameter('Compile',true,@(x)islogical(x)&&isscalar(x));
ip.parse(varargin{:}); o=ip.Results;

here=fileparts(mfilename('fullpath')); idealDir=fileparts(here); addpath(idealDir); addpath(here);
matFile=fullfile(idealDir,'M0_5MW_Aligned_Workpoint_and_SSM.mat');
S=load(matFile,'params','operating_point','state_names');
[pvec,~]=m0_pack_parameters(S.params,S.operating_point); xeq=S.operating_point.x0(:); u0=zeros(6,1);
Ts0=S.params.controller_Ts_s; if isempty(o.Ts), Ts=Ts0; else, Ts=o.Ts; end
if isempty(o.Delay), tau=Ts; else, tau=o.Delay; end
cmd0=s7a_discrete_average_core('commands',xeq,u0,pvec); z0=[xeq;cmd0;cmd0];

mdl='S7A_DiscreteAvg_5MW'; modelFile=fullfile(here,[mdl '.slx']);
if bdIsLoaded(mdl), close_system(mdl,0); end
if isfile(modelFile), load_system(modelFile); else, new_system(mdl,'Model'); end
clearContents(mdl);

add_block('simulink/Sources/In1',[mdl '/u'], 'Position',[35 180 65 210], ...
    'Port','1','PortDimensions','6','OutDataTypeStr','double','SampleTime','Ts_S7A');
add_block('simulink/User-Defined Functions/S-Function',[mdl '/S7A_DiscreteAverage'], ...
    'Position',[240 145 480 245],'FunctionName','s7a_discrete_average_sfun', ...
    'Parameters','S7A_pvec,S7A_z0,Ts_S7A,Tau_S7A','SFunctionModules','');
add_block('simulink/Sinks/Out1',[mdl '/y'], 'Position',[575 180 605 210], 'Port','1');
add_line(mdl,'u/1','S7A_DiscreteAverage/1','autorouting','on');
add_line(mdl,'S7A_DiscreteAverage/1','y/1','autorouting','on');

mws=get_param(mdl,'ModelWorkspace'); assignin(mws,'S7A_pvec',pvec(:)); assignin(mws,'S7A_z0',z0(:));
assignin(mws,'Ts_S7A',Ts); assignin(mws,'Tau_S7A',tau);
set_param(mdl,'SolverType','Fixed-step','Solver','FixedStepDiscrete','FixedStep','Ts_S7A', ...
    'StartTime','0','StopTime','0.5','SignalLogging','off','SaveOutput','on', ...
    'OutputSaveName','yout','SaveFormat','Dataset','SaveTime','on','TimeSaveName','tout', ...
    'ReturnWorkspaceOutputs','on','SaveState','off','UnconnectedInputMsg','error','UnconnectedOutputMsg','error');
set_param(mdl,'InitFcn',[ ...
    's7aModelDir=fileparts(get_param(bdroot,''FileName''));' ...
    'if ~isempty(s7aModelDir), addpath(s7aModelDir); end']);
set_param(mdl,'Description','S7A 5 MW reference discrete averaged model derived from M0 equations; no PWM, limits, protection or legacy C state.');
if isfile(modelFile), save_system(mdl); else, save_system(mdl,modelFile); end
compiled=false; if o.Compile, set_param(mdl,'SimulationCommand','update'); compiled=true; save_system(mdl); end
info=struct('model_name',mdl,'model_path',modelFile,'Ts_s',Ts,'Delay_s',tau,'compiled',compiled, ...
    'state_count',31,'output_count',29,'source_model',matFile,'status','REFERENCE_DIGITAL_AVERAGE_NOT_LEGACY_C');
fprintf('S7A离散平均模型：%s\nTs=%.9g s, delay=%.9g s, compiled=%d\n',modelFile,Ts,tau,compiled);
end

function clearContents(sys)
lines=find_system(sys,'FindAll','on','SearchDepth',1,'Type','line');
for k=1:numel(lines), try, delete_line(lines(k)); catch, end, end
blocks=find_system(sys,'SearchDepth',1,'Type','Block'); blocks(strcmp(blocks,sys))=[];
for k=numel(blocks):-1:1, delete_block(blocks{k}); end
end
