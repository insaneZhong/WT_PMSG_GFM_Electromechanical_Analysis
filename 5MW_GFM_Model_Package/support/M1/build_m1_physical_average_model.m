function info=build_m1_physical_average_model(varargin)
%BUILD_M1_PHYSICAL_AVERAGE_MODEL 从冻结M0生成唯一、可切换的M1物理平均VSC模型。
%
% VSC_DC_NORMALIZATION_MODE=1: M1-a 固定Vdc基准归一化
% VSC_DC_NORMALIZATION_MODE=2: M1-b 实时Udc前馈（默认，理论上回归M0）
%
% 不增加PWM、采样、延迟或限幅；只显式加入 vconv=0.5*Udc*m。
ip=inputParser; ip.addParameter('RunShortSimulation',true,@(x)islogical(x)&&isscalar(x)); ip.parse(varargin{:}); opt=ip.Results;
here=fileparts(mfilename('fullpath'));
m0Dir=fullfile(fileparts(here),'CurrentModel_Idealized');
src=fullfile(m0Dir,'Grid_Forming_PMSG5MW_TwoMass_Idealized.slx');
dst=fullfile(here,'Grid_Forming_PMSG5MW_TwoMass_M1_PhysicalAvg.slx');
tempDir=fullfile(here,'temp'); if ~isfolder(tempDir), mkdir(tempDir); end
tmp=fullfile(tempDir,'Grid_Forming_PMSG5MW_TwoMass_M1_PhysicalAvg_build.slx');
assert(isfile(src),'M0模型不存在：%s',src);
if bdIsLoaded('Grid_Forming_PMSG5MW_TwoMass_M1_PhysicalAvg_build'), close_system('Grid_Forming_PMSG5MW_TwoMass_M1_PhysicalAvg_build',0); end
copyfile(src,tmp,'f'); load_system(tmp); mdl=bdroot;

% M1仍复用冻结M0的公共plant参数与已验收控制平衡点求解器，不复制公共库。
oldInit=get_param(mdl,'InitFcn'); srcExpr="fileparts(get_param(bdroot,'FileName'))";
quoted=strrep(m0Dir,'\','/');
oldInit=strrep(oldInit,srcExpr,'M1_M0_SOURCE_DIR');
newInit=sprintf(['M1_M0_SOURCE_DIR=''%s'';\naddpath(M1_M0_SOURCE_DIR);\n' ...
    'if ~exist(''VSC_DC_NORMALIZATION_MODE'',''var''), VSC_DC_NORMALIZATION_MODE=2; end\n' ...
    'if ~exist(''VSC_DC_NOMINAL_V'',''var''), VSC_DC_NOMINAL_V=1500; end\n%s'],quoted,oldInit);
set_param(mdl,'InitFcn',newInit);

sides={'Ideal_GSC_AverageVSC','Ideal_MSC_AverageVSC'};
for k=1:numel(sides), addPhysicalDcActuation([mdl '/' sides{k}]); end

% 同一DC-link状态驱动两侧物理平均VSC。
add_line(mdl,'Ideal_DC_UdcState/1','Ideal_GSC_AverageVSC/3','autorouting','on');
add_line(mdl,'Ideal_DC_UdcState/1','Ideal_MSC_AverageVSC/3','autorouting','on');
set_param(mdl,'ShowPortDataTypes','on');
Simulink.BlockDiagram.arrangeSystem(mdl,'FullLayout','false');
set_param(mdl,'SimulationCommand','update');
compilePass=true; simPass=true; simMessage="";
if opt.RunShortSimulation
    try
        sim(mdl,'StopTime','0.002','ReturnWorkspaceOutputs','on');
    catch ME
        simPass=false; simMessage=string(ME.message);
    end
end
if ~simPass
    error('M1短时结构仿真失败：%s',simMessage);
end
save_system(mdl,dst); close_system(mdl,0);
if isfile(tmp), delete(tmp); end % 单个、明确路径的临时文件
info=struct('model',dst,'source_model',src,'compile_pass',compilePass,'short_simulation_pass',simPass, ...
    'mode_parameter','VSC_DC_NORMALIZATION_MODE','fixed_mode',1,'realtime_ff_mode',2);
fprintf('M1 model built and validated: %s\n',dst);
end

function addPhysicalDcActuation(parent)
% 在原u_alpha/u_beta和连续正则化环节之间插入显式物理平均VSC。
inU=[parent '/Udc']; act=[parent '/Physical_DC_Actuation'];
if ~isempty(find_system(parent,'SearchDepth',1,'Name','Physical_DC_Actuation')), return; end
add_block('simulink/Ports & Subsystems/In1',inU,'Port','3','Position',[-350 260 -320 280]);
add_block('simulink/Ports & Subsystems/Subsystem',act,'Position',[-260 225 -80 335]);
buildActuationSubsystem(act);
for pair={{'u_alpha','CmdAlphaDelay1',1},{'u_beta','CmdBetaDelay1',2}}
    src=[parent '/' pair{1}{1}]; dst=[parent '/' pair{1}{2}];
    lh=get_param(src,'LineHandles'); if ishandle(lh.Outport), delete_line(lh.Outport); end
    add_line(parent,[pair{1}{1} '/1'],sprintf('Physical_DC_Actuation/%d',pair{1}{3}),'autorouting','on');
    add_line(parent,sprintf('Physical_DC_Actuation/%d',pair{1}{3}),[pair{1}{2} '/1'],'autorouting','on');
end
add_line(parent,'Udc/1','Physical_DC_Actuation/3','autorouting','on');
end

function buildActuationSubsystem(s)
% 固定归一化支路：u*Udc/VdcN；实时前馈支路：0.5*Udc*(2*u/Udc)。
Simulink.SubSystem.deleteContents(s);
add_block('simulink/Ports & Subsystems/In1',[s '/u_alpha'],'Port','1','Position',[25 38 55 52]);
add_block('simulink/Ports & Subsystems/In1',[s '/u_beta'],'Port','2','Position',[25 118 55 132]);
add_block('simulink/Ports & Subsystems/In1',[s '/Udc'],'Port','3','Position',[25 198 55 212]);
add_block('simulink/Sources/Constant',[s '/Mode'],'Value','VSC_DC_NORMALIZATION_MODE','Position',[260 205 300 225]);
for q=1:2
    nm=iff(q==1,'alpha','beta'); y=25+(q-1)*80;
    add_block('simulink/Math Operations/Product',[s '/Fixed_Product_' nm],'Inputs','**','Position',[100 y 130 y+30]);
    add_block('simulink/Math Operations/Gain',[s '/Fixed_InvVdcN_' nm],'Gain','1/VSC_DC_NOMINAL_V','Position',[160 y 230 y+30]);
    add_block('simulink/Math Operations/Product',[s '/Realtime_Divide_' nm],'Inputs','*/','Position',[100 y+35 130 y+65]);
    add_block('simulink/Math Operations/Gain',[s '/Realtime_Two_' nm],'Gain','2','Position',[150 y+35 185 y+65]);
    add_block('simulink/Math Operations/Product',[s '/Realtime_Product_' nm],'Inputs','**','Position',[205 y+35 235 y+65]);
    add_block('simulink/Math Operations/Gain',[s '/Realtime_Half_' nm],'Gain','0.5','Position',[250 y+35 285 y+65]);
    add_block('simulink/Signal Routing/Switch',[s '/Select_' nm],'Threshold','1.5','Criteria','u2 >= Threshold','Position',[330 y+5 365 y+55]);
    add_block('simulink/Ports & Subsystems/Out1',[s '/v_' nm],'Port',num2str(q),'Position',[405 y+23 435 y+37]);
    u=sprintf('u_%s/1',nm);
    add_line(s,u,['Fixed_Product_' nm '/1']); add_line(s,'Udc/1',['Fixed_Product_' nm '/2'],'autorouting','on');
    add_line(s,['Fixed_Product_' nm '/1'],['Fixed_InvVdcN_' nm '/1']);
    add_line(s,u,['Realtime_Divide_' nm '/1']); add_line(s,'Udc/1',['Realtime_Divide_' nm '/2'],'autorouting','on');
    add_line(s,['Realtime_Divide_' nm '/1'],['Realtime_Two_' nm '/1']);
    add_line(s,['Realtime_Two_' nm '/1'],['Realtime_Product_' nm '/1']); add_line(s,'Udc/1',['Realtime_Product_' nm '/2'],'autorouting','on');
    add_line(s,['Realtime_Product_' nm '/1'],['Realtime_Half_' nm '/1']);
    add_line(s,['Realtime_Half_' nm '/1'],['Select_' nm '/1']); add_line(s,'Mode/1',['Select_' nm '/2'],'autorouting','on'); add_line(s,['Fixed_InvVdcN_' nm '/1'],['Select_' nm '/3']);
    add_line(s,['Select_' nm '/1'],['v_' nm '/1']);
end
Simulink.BlockDiagram.arrangeSystem(s,'FullLayout','true');
end
function y=iff(c,a,b),if c,y=a;else,y=b;end,end
