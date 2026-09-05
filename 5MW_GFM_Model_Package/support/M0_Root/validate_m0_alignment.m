function validation = validate_m0_alignment(varargin)
%VALIDATE_M0_ALIGNMENT 验证M0理想连续非线性模型与同源小信号模型严格对齐。
%
% validation = validate_m0_alignment()
%
% 本函数只在内存中返回validation结构体，不保存工作区、原始时序、图片
% 或中间MAT/CSV。验证链保持唯一：
%   init_m0_5mw_parameters -> solve_m0_equilibrium
%   -> m0_nonlinear_dynamics -> linearize_m0_equilibrium
%   -> M0_PMSG_GFM_5MW.slx
%
% 主要门槛：
%   1) 恰有23个连续Integrator；无离散、延迟、限幅、开关、PWM或S-Function；
%   2) 同一工作点满足状态残差、三功率面及DC-link/滤波能量平衡；
%   3) 同一RHS数值线性化后全部极点严格位于左半平面；
%   4) Simulink无扰动保持工作点；小dPref响应与同一A/B/C/D一致。

ip = inputParser;
ip.addParameter('ModelName','M0_PMSG_GFM_5MW', ...
    @(x)ischar(x)||isstring(x));
ip.addParameter('NoDisturbanceTime_s',0.5, ...
    @(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('StepTime_s',0.5, ...
    @(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('ComparisonTime_s',10, ...
    @(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('ComparisonSampleTime_s',1e-3, ...
    @(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('dPref_pu',1e-4, ...
    @(x)isnumeric(x)&&isscalar(x)&&x>0&&x<=1e-2);
ip.addParameter('ResidualTolerance_pu_per_s',1e-8, ...
    @(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('PowerBalanceTolerance_pu',1e-8, ...
    @(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('PoleMargin_per_s',1e-6, ...
    @(x)isnumeric(x)&&isscalar(x)&&x>=0);
ip.addParameter('NoDisturbanceDriftTolerance_pu',1e-7, ...
    @(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('NRMSETolerance',0.02, ...
    @(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('PeakErrorTolerance',0.02, ...
    @(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('FinalErrorTolerance',0.02, ...
    @(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.parse(varargin{:});
o = ip.Results;

mdl = char(string(o.ModelName));
modelFile = which([mdl '.slx']);
if isempty(modelFile)
    localFile = fullfile(fileparts(mfilename('fullpath')),[mdl '.slx']);
    assert(isfile(localFile),'未找到唯一M0模型：%s。',localFile);
    modelFile = localFile;
end

% 所有分析和仿真均使用同一参数对象、同一平衡点和同一RHS。
P = init_m0_5mw_parameters('ControllerProfile','AlignedStable');
[OP,P] = solve_m0_equilibrium(P);
lin = linearize_m0_equilibrium(P,OP);
[pvec,parameterNames] = m0_pack_parameters(P,OP);
stateNames = m0_state_names();
inputNames = lin.input_names;
outputNames = m0_output_names();
u0 = zeros(6,1);
[dx0,y0] = m0_nonlinear_dynamics(OP.x0,u0,pvec);

validation = struct();
validation.model = struct( ...
    'name',mdl,'file',modelFile,'parameter_profile',P.controller_profile, ...
    'state_count',numel(stateNames),'input_count',numel(inputNames), ...
    'output_count',numel(outputNames));
validation.names = struct('states',stateNames,'inputs',inputNames, ...
    'outputs',outputNames,'parameters',parameterNames);
validation.operating_point = OP;
validation.linear_model = lin;

%% Gate A：模型结构必须是唯一、纯连续、无隐藏非线性/离散环节
wasLoaded = bdIsLoaded(mdl);
if ~wasLoaded
    load_system(modelFile);
end
cleanupObj = onCleanup(@()closeIfOriginallyClosed(mdl,wasLoaded)); %#ok<NASGU>

mws = get_param(mdl,'ModelWorkspace');
assignin(mws,'M0_x0',OP.x0);
assignin(mws,'M0_pvec',pvec);

integrators = find_system(mdl,'LookUnderMasks','all','FollowLinks','on', ...
    'Type','Block','BlockType','Integrator');
forbiddenTypes = [ ...
    "S-Function","DiscreteIntegrator","Delay","UnitDelay", ...
    "TransportDelay","VariableTransportDelay","TappedDelay", ...
    "ZeroOrderHold","RateTransition","DiscreteFilter", ...
    "DiscreteTransferFcn","Saturate","Switch","MultiPortSwitch", ...
    "ManualSwitch","Relay","PulseGenerator"];
forbidden = strings(0,1);
for k = 1:numel(forbiddenTypes)
    hits = find_system(mdl,'LookUnderMasks','all','FollowLinks','on', ...
        'Type','Block','BlockType',char(forbiddenTypes(k)));
    if forbiddenTypes(k)=="S-Function"
        % MATLAB Function图在编译表示中含一个隐藏sf_sfun执行引擎；它不是
        % 旧控制器MEX，也不引入采样/离散状态。只豁免这种明确的内部块。
        keep = true(size(hits));
        for j = 1:numel(hits)
            keep(j) = ~isInternalMatlabFunctionSfun(hits{j});
        end
        hits = hits(keep);
    end
    forbidden = [forbidden; string(hits(:))]; %#ok<AGROW>
end
allBlocks = find_system(mdl,'LookUnderMasks','all','FollowLinks','on', ...
    'Type','Block');
names = string(allBlocks(:));
pwmMask = ~cellfun('isempty',regexpi(cellstr(names),'(^|[/_\- ])S?PWM($|[/_\- ])'));
forbidden = unique([forbidden; names(pwmMask)]);

structuralPass = numel(integrators)==23 && isempty(forbidden);
validation.structure = struct( ...
    'integrator_count',numel(integrators), ...
    'integrator_paths',{integrators}, ...
    'forbidden_block_count',numel(forbidden), ...
    'forbidden_block_paths',forbidden, ...
    'pass',structuralPass);

%% Gate B：严格平衡点、三个功率面和能量/转矩残差
idx = @(name)find(outputNames==string(name),1,'first');
Pmsc = y0(idx('P_MSC_dc_W'));
Pgsc = y0(idx('P_GSC_dc_W'));
Ppcc = y0(idx('P_PCC_W'));
Pgrid = y0(idx('P_Grid_W'));
dcResidual = y0(idx('DC_energy_residual_W'));
filterResidual = y0(idx('filter_power_balance_residual_W'));
gridLoss = 1.5*P.Rg_ohm*sum(OP.ig_dq_A.^2);
gridPowerResidual = Ppcc-Pgrid-gridLoss;
torqueResidual = OP.torque_mismatch_Nm;

powerScale = P.Sbase_W;
torqueScale = P.Sbase_W/P.omega_m0_radps;
powerAudit = struct( ...
    'P_MSC_dc_W',Pmsc,'P_GSC_dc_W',Pgsc, ...
    'P_PCC_W',Ppcc,'P_Grid_W',Pgrid, ...
    'dc_link_power_mismatch_W',Pmsc-Pgsc, ...
    'dc_link_energy_residual_W',dcResidual, ...
    'filter_power_balance_residual_W',filterResidual, ...
    'grid_copper_loss_W',gridLoss, ...
    'grid_power_balance_residual_W',gridPowerResidual, ...
    'torque_balance_residual_Nm',torqueResidual);
normalizedResiduals = [ ...
    abs(Pmsc-Pgsc)/powerScale; abs(dcResidual)/powerScale; ...
    abs(filterResidual)/powerScale; abs(gridPowerResidual)/powerScale; ...
    abs(torqueResidual)/torqueScale];
equilibriumPass = OP.max_normalized_residual <= ...
    o.ResidualTolerance_pu_per_s && ...
    all(normalizedResiduals <= o.PowerBalanceTolerance_pu);
validation.equilibrium = struct( ...
    'max_normalized_state_residual',OP.max_normalized_residual, ...
    'state_residual_norm',norm(dx0,2), ...
    'power_and_energy',powerAudit, ...
    'max_normalized_power_or_torque_residual',max(normalizedResiduals), ...
    'pass',equilibriumPass);

% 物理相对角必须对应正的稳态P-delta斜率，禁止翻转VSG符号来掩盖。
dP = 1e-4*P.Sbase_W;
Pminus = P; Pminus.Pref_W = P.Pref_W-dP;
Pplus = P;  Pplus.Pref_W = P.Pref_W+dP;
[OPminus,~] = solve_m0_equilibrium(Pminus);
[OPplus,~] = solve_m0_equilibrium(Pplus);
dDelta = unwrap([OPminus.delta_v_rad;OPplus.delta_v_rad]);
powerAngleSlope = (OPplus.P_PCC_W-OPminus.P_PCC_W)/ ...
    (dDelta(2)-dDelta(1));
validation.power_angle = struct( ...
    'dP_dDelta_W_per_rad',powerAngleSlope, ...
    'physical_vsg_power_error_sign',P.vsg_physical_power_error_sign, ...
    'pass',powerAngleSlope>0 && P.vsg_physical_power_error_sign==1);

%% Gate C：同源A/B/C/D的全部极点稳定
lambda = lin.eigenvalues;
polePass = all(isfinite(lambda)) && ...
    lin.max_real_part < -o.PoleMargin_per_s;
validation.poles = struct( ...
    'eigenvalues',lambda,'frequency_Hz',lin.frequency_Hz, ...
    'damping_ratio',lin.damping_ratio, ...
    'max_real_part_per_s',lin.max_real_part, ...
    'required_margin_per_s',o.PoleMargin_per_s,'pass',polePass);

%% Gate D：Simulink无扰动必须保持同一工作点
set_param(mdl,'SimulationCommand','update');
[tNo,xNo,yNo] = runModel(mdl,OP.x0,pvec,zeros(3,6), ...
    [0;o.NoDisturbanceTime_s/2;o.NoDisturbanceTime_s]);
xScale = localStateScale(P,OP);
yScale = localOutputScale(P,OP,y0);
stateDrift = max(abs(xNo-OP.x0.'),[],1)./xScale.';
outputDrift = max(abs(yNo-y0.'),[],1)./yScale.';
noDisturbancePass = all(isfinite(xNo(:))) && all(isfinite(yNo(:))) && ...
    max(stateDrift)<=o.NoDisturbanceDriftTolerance_pu && ...
    max(outputDrift)<=o.NoDisturbanceDriftTolerance_pu;
validation.no_disturbance = struct( ...
    'simulated_time_s',tNo(end)-tNo(1), ...
    'max_state_drift_pu',max(stateDrift), ...
    'max_output_drift_pu',max(outputDrift), ...
    'dc_energy_residual_max_pu', ...
        max(abs(yNo(:,idx('DC_energy_residual_W'))))/P.Sbase_W, ...
    'filter_energy_residual_max_pu', ...
        max(abs(yNo(:,idx('filter_power_balance_residual_W'))))/P.Sbase_W, ...
    'pass',noDisturbancePass);

%% Gate E：小dPref下，Simulink非线性模型与同源线性模型逐项比较
assert(o.ComparisonTime_s>o.StepTime_s, ...
    'ComparisonTime_s必须大于StepTime_s。');
dPref = o.dPref_pu*P.Sbase_W;
uStep = zeros(3,6); uStep(2:3,2) = dPref;
[tNL,~,yNL] = runModel(mdl,OP.x0,pvec,uStep, ...
    [0;o.StepTime_s;o.ComparisonTime_s]);

tCmp = (0:o.ComparisonSampleTime_s:o.ComparisonTime_s).';
if tCmp(end)<o.ComparisonTime_s
    tCmp(end+1,1)=o.ComparisonTime_s;
end
uCmp = zeros(numel(tCmp),6);
uCmp(tCmp>=o.StepTime_s,2)=dPref;
sys = ss(lin.A,lin.B,lin.C,lin.D);
% Simulink外部扰动明确采用ZOH。连续系统lsim默认会在采样点之间作
% 线性插值（FOH），对包含直接项D和快速LCL模态的阶跃会造成伪误差；
% 因此先以同一比较步长作精确ZOH离散化，再进行线性响应计算。
sysZoh = c2d(sys,o.ComparisonSampleTime_s,'zoh');
yLinDelta = lsim(sysZoh,uCmp,tCmp,zeros(23,1));
yNLinterp = interp1(tNL,yNL,tCmp,'linear','extrap');
yNLdelta = yNLinterp-y0.';

compareNames = ["P_PCC_W","Udc_V","Tgen_Nm","Tshaft_Nm", ...
    "omega_rel_radps","omega_vsg_radps"];
metrics = repmat(struct('signal',"",'output_index',0, ...
    'normalization_scale',NaN,'nrmse',NaN,'peak_error',NaN, ...
    'final_error',NaN,'pass',false),numel(compareNames),1);
for k = 1:numel(compareNames)
    j = idx(compareNames(k));
    nl = yNLdelta(:,j); li = yLinDelta(:,j);
    floorScale = 1e-9*yScale(j);
    scale = max([max(abs(li)),max(abs(nl)),floorScale,eps]);
    nrmse = sqrt(mean((nl-li).^2))/scale;
    peakError = abs(max(abs(nl))-max(abs(li)))/scale;
    finalError = abs(nl(end)-li(end))/scale;
    metrics(k) = struct('signal',compareNames(k),'output_index',j, ...
        'normalization_scale',scale,'nrmse',nrmse, ...
        'peak_error',peakError,'final_error',finalError, ...
        'pass',nrmse<=o.NRMSETolerance && ...
            peakError<=o.PeakErrorTolerance && ...
            finalError<=o.FinalErrorTolerance);
end
alignmentPass = all([metrics.pass]);
validation.small_signal_alignment = struct( ...
    'disturbance','dPref step', ...
    'dPref_pu',o.dPref_pu,'dPref_W',dPref, ...
    'step_time_s',o.StepTime_s, ...
    'comparison_time_s',o.ComparisonTime_s, ...
    'metrics',metrics, ...
    'thresholds',struct('nrmse',o.NRMSETolerance, ...
        'peak_error',o.PeakErrorTolerance, ...
        'final_error',o.FinalErrorTolerance), ...
    'pass',alignmentPass);

validation.gates = struct( ...
    'structure',structuralPass, ...
    'equilibrium_and_energy',equilibriumPass, ...
    'positive_power_angle_slope',validation.power_angle.pass, ...
    'all_poles_stable',polePass, ...
    'no_disturbance',noDisturbancePass, ...
    'nonlinear_linear_alignment',alignmentPass);
validation.overall_pass = all(struct2array(validation.gates));
validation.timestamp = datetime('now','TimeZone','Asia/Shanghai');
end

function [t,x,y] = runModel(mdl,x0,pvec,uData,uTime)
% 以阶跃外部输入运行根Inport u，并兼容根Outport Dataset输出。
mws = get_param(mdl,'ModelWorkspace');
assignin(mws,'M0_x0',x0);
assignin(mws,'M0_pvec',pvec);

% Simulink根ExternalInput在部分版本中会忽略timeseries的ZOH元数据，
% 直接在稀疏断点间线性插值。对每个跳变显式插入[t-eps,t]两点，
% 将爬坡压缩到1 ns，避免把本应在StepTime发生的阶跃变成全段斜坡。
[uTimeExpanded,uDataExpanded] = encodeNearIdealSteps(uTime,uData);
uTs = timeseries(uDataExpanded,uTimeExpanded);
simIn = Simulink.SimulationInput(mdl);
simIn = simIn.setExternalInput(uTs);
simIn = simIn.setModelParameter( ...
    'StartTime','0','StopTime',num2str(uTime(end),17), ...
    'SaveOutput','on','OutputSaveName','yout', ...
    'SaveTime','on','TimeSaveName','tout', ...
    'SaveFormat','Dataset','SignalLogging','off', ...
    'ReturnWorkspaceOutputs','on');
simOut = sim(simIn);

[tx,x] = extractRootOutput(simOut,'x',1);
[ty,y] = extractRootOutput(simOut,'y',2);
assert(size(x,2)==23,'Simulink根输出x维数应为23，实际为%d。',size(x,2));
assert(size(y,2)==29,'Simulink根输出y维数应为29，实际为%d。',size(y,2));

% 两个根输出理论上同一时间基准；若求解器返回方式不同则统一到y。
t = ty(:);
if numel(tx)~=numel(ty) || any(abs(tx(:)-ty(:))>1e-12)
    x = interp1(tx(:),x,t,'linear','extrap');
end
end

function [tOut,uOut] = encodeNearIdealSteps(tIn,uIn)
tIn = tIn(:);
assert(size(uIn,1)==numel(tIn),'外部输入断点数与数据行数不一致。');
tOut = tIn(1);
uOut = uIn(1,:);
for k = 2:numel(tIn)
    changed = any(uIn(k,:)~=uIn(k-1,:));
    if changed
        jumpWidth = min(1e-9,0.01*(tIn(k)-tIn(k-1)));
        tBefore = tIn(k)-jumpWidth;
        if tBefore>tOut(end)
            tOut(end+1,1) = tBefore; %#ok<AGROW>
            uOut(end+1,:) = uIn(k-1,:); %#ok<AGROW>
        end
    end
    tOut(end+1,1) = tIn(k); %#ok<AGROW>
    uOut(end+1,:) = uIn(k,:); %#ok<AGROW>
end
end

function [t,data] = extractRootOutput(simOut,signalName,fallbackIndex)
% 兼容Dataset、timeseries及传统Structure-with-time根输出格式。
try
    rootOut = simOut.get('yout');
catch
    rootOut = [];
end
if isa(rootOut,'Simulink.SimulationData.Dataset')
    element = [];
    try
        element = rootOut.getElement(signalName);
    catch
    end
    if isempty(element)
        element = rootOut.getElement(fallbackIndex);
    end
    values = element.Values;
elseif isa(rootOut,'timeseries')
    values = rootOut;
elseif isstruct(rootOut) && isfield(rootOut,'signals')
    sig = rootOut.signals(fallbackIndex);
    if isfield(rootOut,'time')
        t = rootOut.time(:);
    else
        t = simOut.get('tout');
    end
    data = orientTimeRows(sig.values,numel(t));
    return;
else
    % 某些配置会把命名Outport直接放入SimulationOutput。
    try
        values = simOut.get(signalName);
    catch
        error('无法从SimulationOutput提取根输出%s。',signalName);
    end
end

if isa(values,'timeseries')
    t = values.Time(:);
    data = orientTimeRows(values.Data,numel(t));
elseif istimetable(values)
    t = seconds(values.Properties.RowTimes-values.Properties.RowTimes(1));
    data = orientTimeRows(values.Variables,numel(t));
else
    try
        t = values.Time(:);
        data = orientTimeRows(values.Data,numel(t));
    catch
        error('根输出%s的Values不是可识别的timeseries/timetable。',signalName);
    end
end
end

function data = orientTimeRows(data,nTime)
data = squeeze(data);
if isvector(data)
    data = data(:);
end
if size(data,1)~=nTime && size(data,2)==nTime
    data = data.';
end
assert(size(data,1)==nTime,'输出数据的时间维无法识别。');
end

function scale = localStateScale(P,OP)
scale = [1;P.omega_m0_radps;P.omega_m0_radps; ...
    P.Iphase_peak_base_A;P.Iphase_peak_base_A; ...
    P.Iphase_peak_base_A;P.Vphase_peak_V;P.Vphase_peak_V; ...
    P.Vdc_ref_V;P.Sbase_W;P.Sbase_W;P.omega0_radps;1; ...
    P.Iphase_peak_base_A;P.Iphase_peak_base_A; ...
    P.Vphase_peak_V;P.Vphase_peak_V; ...
    P.Iphase_peak_base_A;P.Iphase_peak_base_A; ...
    P.Vphase_peak_V;P.Vphase_peak_V; ...
    P.Iphase_peak_base_A;P.Iphase_peak_base_A];
scale = max(scale,abs(OP.x0));
end

function scale = localOutputScale(P,OP,y0)
torqueBase = P.Sbase_W/P.omega_m0_radps;
scale = [P.Sbase_W;P.Sbase_W;P.Sbase_W;P.Sbase_W;P.Sbase_W; ...
    P.Vdc_ref_V;torqueBase;torqueBase;P.omega_m0_radps; ...
    P.omega_m0_radps;P.omega_m0_radps;P.omega0_radps;1; ...
    P.Vphase_peak_V;P.Iphase_peak_base_A;P.Iphase_peak_base_A; ...
    P.Sbase_W;P.Sbase_W;P.Vdc_ref_V;P.Sbase_W;P.Sbase_W; ...
    P.Vphase_peak_V;P.Iphase_peak_base_A;P.Iphase_peak_base_A; ...
    P.Sbase_W;P.Sbase_W;P.Vphase_peak_V;P.Sbase_W;P.Sbase_W];
scale = max(scale,abs(y0));
scale(8) = max(scale(8),abs(OP.Tgen_Nm));
end

function tf = isInternalMatlabFunctionSfun(blockPath)
tf = false;
try
    if ~strcmp(get_param(blockPath,'FunctionName'),'sf_sfun')
        return;
    end
    parentPath = get_param(blockPath,'Parent');
    tf = strcmp(get_param(parentPath,'SFBlockType'),'MATLAB Function');
catch
    tf = false;
end
end

function closeIfOriginallyClosed(mdl,wasLoaded)
if ~wasLoaded && bdIsLoaded(mdl)
    close_system(mdl,0);
end
end
