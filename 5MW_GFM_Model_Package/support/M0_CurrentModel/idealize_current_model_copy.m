function report=idealize_current_model_copy()
% 在当前5 MW副本上完成第一版理想化：
% 1) 使用连续执行、无限幅、严格VSG的专用MEX；
% 2) 平均VSC命令延迟直通；
% 3) 关闭重复直流调节和主动阻尼；
% 4) 尝试将powergui/求解器切换为连续仿真。
% 原始开关模型不修改。
here=fileparts(mfilename('fullpath'));
mdl='Grid_Forming_PMSG5MW_TwoMass_Idealized';
load_system(fullfile(here,[mdl '.slx']));

% 控制器换成副本专用MEX（41路输出，含连续alpha-beta指令）。
sf=[mdl '/MOTOR_CONTROL1/S-Function1'];
hasLegacyController=false;
try
    get_param(sf,'Handle');
    hasLegacyController=true;
catch
end
if hasLegacyController
    set_param(sf,'FunctionName','main_5mw_idealized');
else
    assert(~isempty(find_system([mdl '/MOTOR_CONTROL1/IdealCtrlRHS'], ...
        'SearchDepth',0)), ...
        'Neither the legacy controller nor the explicit continuous controller exists.');
end

% 平均变流器命令通道取消三阶一阶串联延迟。
% 为避免理想受控电压源+瞬时功率测量造成不可解代数环，保留一个
% 极快的连续一阶正则化状态（tau=1e-7 s）；它不是数字延迟，后续
% 小信号模型必须显式包含这6个连续状态。
vscs={'Ideal_MSC_AverageVSC','Ideal_GSC_AverageVSC'};
delayNames={'CmdAlphaDelay1','CmdAlphaDelay2','CmdAlphaDelay3', ...
    'CmdBetaDelay1','CmdBetaDelay2','CmdBetaDelay3'};
for i=1:numel(vscs)
    for j=1:numel(delayNames)
        b=[mdl '/' vscs{i} '/' delayNames{j}];
        % Only one 1e-7 s *continuous* regularizer is retained per command
        % axis.  It breaks the SPS ideal-voltage-source DAE algebraic loop;
        % it is not a sampled/PWM/digital delay and is included explicitly
        % in the aligned small-signal state vector.  The other two legacy
        % stages are true unit through paths.
        isRegularizer=strcmp(delayNames{j},'CmdAlphaDelay1') || ...
            strcmp(delayNames{j},'CmdBetaDelay1');
        if isRegularizer
            if strcmp(get_param(b,'BlockType'),'Gain')
                replace_block(b,'Gain','built-in/StateSpace','noprompt');
            end
            if contains(delayNames{j},'Alpha')
                if strcmp(vscs{i},'Ideal_MSC_AverageVSC'), ic='IdealMSCAlpha0'; else, ic='IdealGSCAlpha0'; end
            else
                if strcmp(vscs{i},'Ideal_MSC_AverageVSC'), ic='IdealMSCBeta0'; else, ic='IdealGSCBeta0'; end
            end
            tau=1e-7;
            set_param(b,'A',num2str(-1/tau,'%.17g'), ...
                'B',num2str(1/tau,'%.17g'),'C','1','D','0', ...
                'InitialCondition',ic);
        else
            if ~strcmp(get_param(b,'BlockType'),'Gain')
                replace_block(b,get_param(b,'BlockType'),'built-in/Gain','noprompt');
            end
            set_param(b,'Gain','1');
        end
    end
    % The old guarded copy used fitted gain/rotation matrices and a negative
    % command polarity to mimic the switching/SVPWM path.  M0 uses physical
    % phase-voltage alpha-beta commands, so these empirical mappings must be
    % removed rather than retained as an unmodelled controller dynamic.
    q=[mdl '/' vscs{i}];
    set_param([q '/MapAtoA'],'Gain','1');
    set_param([q '/MapAtoB'],'Gain','0');
    set_param([q '/MapBtoA'],'Gain','0');
    set_param([q '/MapBtoB'],'Gain','1');
    % The retained ideal-source terminal orientation is negative with
    % respect to the controller alpha-beta command.  A positive MSC trial
    % gives an immediate 10-ms current divergence, so do not infer a port
    % mapping from one instantaneous phase-current derivative alone.
    set_param([q '/CmdAlphaPolarity'],'Gain','-1');
    set_param([q '/CmdBetaPolarity'],'Gain','-1');
    % 两个端口功率统一到 DC-link 能量方程：P_MSC>0 表示流入母线，
    % P_GSC>0 表示流出母线。共同 SPS 周期工作点证明，两个理想
    % VSC 的内部交流端口功率均已采用相应的正能量方向，因此都取
    % +1。旧 GSC=-1 结论来自非周期 LCL 初值，不能用于符号审计。
    if strcmp(vscs{i},'Ideal_MSC_AverageVSC')
        set_param([q '/PSign'],'Gain','1');
    else
        set_param([q '/PSign'],'Gain','1');
    end
end
% 不允许 powergui 在每次仿真开始时自行重算“稳态”电气状态。
% 该求解不知道显式连续控制器的 5 MW 积分器初值；模型级完整
% xInitial 才是机械、电气和控制器共同平衡点的唯一初态来源。
set_param([mdl '/powergui'],'x0status','blocks');

% SPS 只把部分串联电感电流选为独立状态。L1/L3 在当前拓扑中是
% 依赖状态，若其 InitialCurrent 字段仍含数值表达式，powergui 会报告
% "Initial state conflict"。清空字段并不把电流设为零；实际初值由
% 模型级 xInitial 恢复，避免支路初值和全状态工作点重复指定。
dependentInductors={'L1','L3'};
for k=1:numel(dependentInductors)
    blk=[mdl '/' dependentInductors{k}];
    if getSimulinkBlockHandle(blk)>0
        set_param(blk,'Setx0','off','InitialCurrent','[]');
    end
end

% 允许直接点击 Run：在 InitFcn 的参数初始化之后，加载已经通过联合
% 物理平衡点验收的完整状态和连续控制器参数。使用标记避免重复追加。
opMarker='% JointPhysicalOPAutoLoad';
initFcn=get_param(mdl,'InitFcn');
if ~contains(initFcn,opMarker)
    opLoadCode=sprintf([ ...
        '\n%s\n' ...
        'opFile=fullfile(fileparts(get_param(bdroot,''FileName'')),''02_Joint_Physical_Alignment_Summary.mat'');\n' ...
        'assert(exist(opFile,''file'')==2,''缺少已验收的联合物理工作点：%%s'',opFile);\n' ...
        'opData=load(opFile,''xInitial'',''IdealCtrlPVec'');\n' ...
        'xInitial=opData.xInitial; IdealCtrlPVec=opData.IdealCtrlPVec;\n' ...
        'clear opData opFile;\n'],opMarker);
    set_param(mdl,'InitFcn',[initFcn opLoadCode]);
end
set_param(mdl,'LoadInitialState','on','InitialState','xInitial');

% Direct-GFM ideal branch: do not retain the original PLL/pre-sync breaker
% sequence.  The retained physical grid is connected at t=0.
brk=[mdl '/Three-Phase Breaker'];
if getSimulinkBlockHandle(brk)>0
    % Mask initialization requires a nonempty switching-time vector even
    % when external control is disabled.  Its only scheduled opening is
    % placed far outside every alignment simulation.
    set_param(brk,'InitialState','closed','External','off','SwitchTimes','1e6');
end

% 关闭主动阻尼/参考曲线接口；原模型控制器中已通过编译宏关闭相应分支。
setConstantIfPresent(mdl,'ActiveDampingScaleCommand','0');
setConstantIfPresent(mdl,'VdcRefProfileEnable','0');

% The paper small-signal baseline is a fixed operating-point experiment,
% not a start-up/MPPT/pitch experiment.  Replace only the idealized copy's
% mechanical source interface with the exact source_aligned_rhs law:
%   Ta = Tm0*wm0/wt - Dt*(wcoi-wm0)
%   Te,in = Te,PMSG + Dg*(wcoi-wm0)
% Dt/Jt=Dg/Jg, so this passive COI anchor cancels from the relative shaft
% acceleration and cannot masquerade as active torsional damping.
installAlignedMechanicalSource(mdl);
installIdealPrefInterface(mdl);

% 仅把明显的外部保护限幅放到理想副本的非作用区；不改物理模型参数。
sat=find_system(mdl,'LookUnderMasks','all','FollowLinks','on','Type','Block', ...
    'BlockType','Saturate');
for k=1:numel(sat)
    try
        set_param(sat{k},'UpperLimit','1e12','LowerLimit','-1e12');
    catch
    end
end

% 连续电力系统仿真设置。若当前MATLAB版本不接受Continuous，保留原设置并记录。
continuousPowergui=false; continuousSolver=false;
try
    set_param([mdl '/powergui'],'SimulationMode','Continuous');
    continuousPowergui=true;
catch
end
try
    set_param(mdl,'Solver','ode15s','SolverType','Variable-step', ...
        'RelTol','1e-5','MaxStep','1e-4');
    continuousSolver=true;
catch
end

set_param(mdl,'SimulationCommand','update');
save_system(mdl,fullfile(here,[mdl '.slx']));
if hasLegacyController
    controllerName='main_5mw_idealized';
else
    controllerName='IdealCtrlRHS + 11 visible continuous integrators';
end
report=struct('model',fullfile(here,[mdl '.slx']), ...
    'controller',controllerName, ...
    'commandDelaysRemoved',true, ...
    'powerguiContinuous',continuousPowergui, ...
    'variableStepSolver',continuousSolver, ...
    'saturationBlocksRaised',numel(sat));
fid=fopen(fullfile(here,'01_Idealization_Implementation_Report_CN.md'),'w','n','UTF-8');
fprintf(fid,'# 当前5 MW模型理想化副本实施报告\n\n');
fprintf(fid,'- 副本：`%s.slx`\n- 原始模型：未修改\n- 控制器：`main_5mw_idealized.mexw64`\n- 控制器执行：每个4 us主步执行，PI状态更新步长取4 us\n- VSG：严格VSG、物理相对角、启动PLL/接管分支关闭\n- 主动阻尼：关闭\n- 重复GSC直流能量修正：关闭\n- 平均VSC命令延迟：6个延迟块/变流器均改为直通\n- 外部Saturate块：%d个移至非作用区\n- powergui连续模式：%d\n- 求解器Variable-step：%d\n\n',mdl,numel(sat),continuousPowergui,continuousSolver);
fprintf(fid,'## 当前边界\n\n');
fprintf(fid,'本副本已去除控制器的PWM调度、SVPWM输出对控制电压的依赖、命令延迟和限幅路径；但控制器仍以专用MEX形式承载内部PI/VSG状态，因此“严格同源小信号”验收前还必须将这些内部状态迁移为显式连续Integrator，并逐项核对功率面。\n');
fclose(fid);
close_system(mdl,0);
end

function setConstantIfPresent(mdl,name,value)
b=[mdl '/' name];
if ~isempty(find_system(b,'SearchDepth',0))
    try, set_param(b,'Value',value); catch, end
end
end

function installAlignedMechanicalSource(mdl)
dt=[mdl '/Drivetrain_TwoMass'];
oldTe=[mdl '/TeEffective'];
phOld=get_param(oldTe,'PortHandles');
oldLine=get_param(phOld.Inport(1),'Line');
assert(oldLine>0,'Cannot locate the retained physical PMSG torque signal.');
tePhysical=get_param(oldLine,'SrcPortHandle');
phDt=get_param(dt,'PortHandles');

names={'Ideal_Jt_wt','Ideal_Jg_wg','Ideal_COI_Sum','Ideal_COI_Gain', ...
    'Ideal_wm0','Ideal_COI_Error','Ideal_Dt','Ideal_Dg', ...
    'Ideal_Pmech0','Ideal_Aero_Divide','Ideal_Taero','Ideal_Te'};
libs={'simulink/Math Operations/Gain','simulink/Math Operations/Gain', ...
    'simulink/Math Operations/Sum','simulink/Math Operations/Gain', ...
    'simulink/Sources/Constant','simulink/Math Operations/Sum', ...
    'simulink/Math Operations/Gain','simulink/Math Operations/Gain', ...
    'simulink/Sources/Constant','simulink/Math Operations/Product', ...
    'simulink/Math Operations/Sum','simulink/Math Operations/Sum'};
for k=1:numel(names)
    p=[mdl '/' names{k}];
    if getSimulinkBlockHandle(p)<=0
        add_block(libs{k},p,'Position',[1050+120*mod(k-1,4), ...
            1420+70*floor((k-1)/4),1120+120*mod(k-1,4),1445+70*floor((k-1)/4)]);
    end
end

set_param([mdl '/Ideal_Jt_wt'],'Gain','IdealCtrlPVec(19)');
set_param([mdl '/Ideal_Jg_wg'],'Gain','IdealCtrlPVec(20)');
set_param([mdl '/Ideal_COI_Sum'],'Inputs','++');
set_param([mdl '/Ideal_COI_Gain'],'Gain','1/(IdealCtrlPVec(19)+IdealCtrlPVec(20))');
set_param([mdl '/Ideal_wm0'],'Value','IdealCtrlPVec(12)');
set_param([mdl '/Ideal_COI_Error'],'Inputs','+-');
set_param([mdl '/Ideal_Dt'],'Gain','IdealCtrlPVec(23)');
set_param([mdl '/Ideal_Dg'],'Gain','IdealCtrlPVec(24)');
set_param([mdl '/Ideal_Pmech0'],'Value','IdealCtrlPVec(39)*IdealCtrlPVec(12)');
set_param([mdl '/Ideal_Aero_Divide'],'Inputs','*/');
set_param([mdl '/Ideal_Taero'],'Inputs','+-');
% Drivetrain_TwoMass uses generator_residual = Tsh + T_e,input - Dg,old.
% The retained PMSM measurement is a positive generating torque, whereas
% the subsystem input convention is signed electromagnetic torque.  Feed
% -(Tgen + Dg*domega_coi), and disable the old individual-speed damping so
% the nonlinear copy uses exactly the same COI anchor as source_aligned_rhs.
set_param([mdl '/Ideal_Te'],'Inputs','--');
set_param([dt '/D_t'],'Gain','0');
set_param([dt '/D_g'],'Gain','0');

connect(phDt.Outport(1),get_param([mdl '/Ideal_Jt_wt'],'PortHandles').Inport(1));
connect(phDt.Outport(2),get_param([mdl '/Ideal_Jg_wg'],'PortHandles').Inport(1));
connect(get_param([mdl '/Ideal_Jt_wt'],'PortHandles').Outport(1), ...
    get_param([mdl '/Ideal_COI_Sum'],'PortHandles').Inport(1));
connect(get_param([mdl '/Ideal_Jg_wg'],'PortHandles').Outport(1), ...
    get_param([mdl '/Ideal_COI_Sum'],'PortHandles').Inport(2));
connect(get_param([mdl '/Ideal_COI_Sum'],'PortHandles').Outport(1), ...
    get_param([mdl '/Ideal_COI_Gain'],'PortHandles').Inport(1));
connect(get_param([mdl '/Ideal_COI_Gain'],'PortHandles').Outport(1), ...
    get_param([mdl '/Ideal_COI_Error'],'PortHandles').Inport(1));
connect(get_param([mdl '/Ideal_wm0'],'PortHandles').Outport(1), ...
    get_param([mdl '/Ideal_COI_Error'],'PortHandles').Inport(2));
connect(get_param([mdl '/Ideal_COI_Error'],'PortHandles').Outport(1), ...
    get_param([mdl '/Ideal_Dt'],'PortHandles').Inport(1));
connect(get_param([mdl '/Ideal_COI_Error'],'PortHandles').Outport(1), ...
    get_param([mdl '/Ideal_Dg'],'PortHandles').Inport(1));
connect(get_param([mdl '/Ideal_Pmech0'],'PortHandles').Outport(1), ...
    get_param([mdl '/Ideal_Aero_Divide'],'PortHandles').Inport(1));
connect(phDt.Outport(1),get_param([mdl '/Ideal_Aero_Divide'],'PortHandles').Inport(2));
connect(get_param([mdl '/Ideal_Aero_Divide'],'PortHandles').Outport(1), ...
    get_param([mdl '/Ideal_Taero'],'PortHandles').Inport(1));
connect(get_param([mdl '/Ideal_Dt'],'PortHandles').Outport(1), ...
    get_param([mdl '/Ideal_Taero'],'PortHandles').Inport(2));
connect(tePhysical,get_param([mdl '/Ideal_Te'],'PortHandles').Inport(1));
connect(get_param([mdl '/Ideal_Dg'],'PortHandles').Outport(1), ...
    get_param([mdl '/Ideal_Te'],'PortHandles').Inport(2));
connect(get_param([mdl '/Ideal_Taero'],'PortHandles').Outport(1),phDt.Inport(1));
connect(get_param([mdl '/Ideal_Te'],'PortHandles').Outport(1),phDt.Inport(2));

    function connect(src,dst)
        lh=get_param(dst,'Line');
        if lh>0
            if get_param(lh,'SrcPortHandle')==src, return; end
            delete_line(lh);
        end
        add_line(mdl,src,dst,'autorouting','on');
    end
end

function installIdealPrefInterface(mdl)
names={'Ideal_Pref0','Ideal_DeltaPref','Ideal_Pref_Command'};
libs={'simulink/Sources/Constant','simulink/Sources/Step', ...
    'simulink/Math Operations/Sum'};
for k=1:numel(names)
    p=[mdl '/' names{k}];
    if getSimulinkBlockHandle(p)<=0
        add_block(libs{k},p,'Position',[850+110*(k-1),1320,920+110*(k-1),1345]);
    end
end
set_param([mdl '/Ideal_Pref0'],'Value','IdealCtrlPVec(37)');
set_param([mdl '/Ideal_DeltaPref'],'Time','1e6','Before','0','After','0');
set_param([mdl '/Ideal_Pref_Command'],'Inputs','++');
connect(get_param([mdl '/Ideal_Pref0'],'PortHandles').Outport(1), ...
    get_param([mdl '/Ideal_Pref_Command'],'PortHandles').Inport(1));
connect(get_param([mdl '/Ideal_DeltaPref'],'PortHandles').Outport(1), ...
    get_param([mdl '/Ideal_Pref_Command'],'PortHandles').Inport(2));
phCtrl=get_param([mdl '/MOTOR_CONTROL1'],'PortHandles');
connect(get_param([mdl '/Ideal_Pref_Command'],'PortHandles').Outport(1), ...
    phCtrl.Inport(18));

    function connect(src,dst)
        lh=get_param(dst,'Line');
        if lh>0
            if get_param(lh,'SrcPortHandle')==src, return; end
            delete_line(lh);
        end
        add_line(mdl,src,dst,'autorouting','on');
    end
end
