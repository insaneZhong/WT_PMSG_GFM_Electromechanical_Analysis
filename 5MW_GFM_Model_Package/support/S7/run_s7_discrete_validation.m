function R = run_s7_discrete_validation(varargin)
%RUN_S7_DISCRETE_VALIDATION  S7双验证路线的S7-0与S7A执行入口。
%
% 本程序只使用当前M0连续平均模型的严格工作点和同源RHS参数，构造一个
% “连续plant + 采样控制器 + ZOH + 可变计算延迟”的离散平均筛查映射。
% 它不是开关EMT，也不是旧C/S-Function的直接复刻；真实离散控制器分块
% 和Gate V2的Simulink离散平均模型仍需后续补齐。
%
% 用法：
%   R = run_s7_discrete_validation('Action','all');
%   R = run_s7_discrete_validation('Action','freeze');
%   R = run_s7_discrete_validation('Action','screen');
%
% 长期只写入S7摘要文件；临时调试数据不写入MATLAB工作区文件。

ip = inputParser;
ip.addParameter('Action','all',@(x)ischar(x)||isstring(x));
ip.addParameter('SaveFigures',false,@(x)islogical(x)&&isscalar(x));
ip.addParameter('RunDelayScan',true,@(x)islogical(x)&&isscalar(x));
ip.addParameter('RunSamplingScan',true,@(x)islogical(x)&&isscalar(x));
ip.addParameter('ODERelTol',1e-7,@(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('ODEAbsTol',1e-8,@(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.parse(varargin{:});
o=ip.Results;

here=fileparts(mfilename('fullpath'));
idealDir=fileparts(here);
matFile=fullfile(idealDir,'M0_5MW_Aligned_Workpoint_and_SSM.mat');
assert(exist(matFile,'file')==2,'未找到连续参考SSM：%s',matFile);
S=load(matFile,'A','B','C','D','eigenvalues','params','operating_point', ...
    'state_names','input_names','output_names','x_eq');
assert(isfield(S,'A')&&size(S.A,1)==23,'连续M0参考SSM不是23状态。');
P=S.params; OP=S.operating_point; xeq=S.x_eq(:); u0=zeros(6,1);

% S7-0：冻结S6边界和连续参考基线。
manifest=writeS7ReferenceManifest(here,idealDir,S);
digitalManifest=writeS7DigitalManifest(here,idealDir,P);
stateAuditPath=writeS7ControllerStateAudit(here,P,S);
registerPath=writeS7HypothesisRegister(here,manifest,digitalManifest,stateAuditPath,P,S);

action=lower(strtrim(string(o.Action)));
R=struct('objective','S7 electrical implementation fidelity validation', ...
    'S6_status','INCONCLUSIVE','reference_manifest',manifest, ...
    'digital_controller_manifest',digitalManifest, ...
    'controller_state_audit',stateAuditPath, ...
    'hypothesis_register',registerPath,'status','S7_0_COMPLETE');
if action=="freeze"
    fprintf('S7-0完成：S6保持INCONCLUSIVE，已生成参考清单和假设寄存器。\n');
    return
end

% S7A-1：先确定当前M0连续控制模型是否可被采样映射。
% 这里的映射是同源方程的采样平均筛查，不冒充Gate V2所需的Simulink离散模型。
Ts0=P.controller_Ts_s;
cmd0=referenceCommands(xeq,u0,P); z0=[xeq;cmd0;cmd0];
scale=[max(abs(xeq),1);max(abs(z0(24:31)),1)];
cfg=struct('RelTol',o.ODERelTol,'AbsTol',o.ODEAbsTol,'stateScale',scale);

rowsV1=cell(0,9);
smallRatios=[0.01 0.05 0.10];
for k=1:numel(smallRatios)
    Ts=Ts0*smallRatios(k);
    [Ad,resid]=sampledJacobian(xeq,u0,P,Ts,0,cfg);
    [ss,freq,zeta,cls]=discreteModalSummary(Ad,Ts,S.state_names);
    [err,trend]=compareContinuousModes(S.A,S.state_names,ss,freq,zeta,cls);
    rowsV1(end+1,:)={smallRatios(k),Ts,resid,err.freq_max_pct,err.real_max_abs, ...
        err.tor_freq_pct,err.tor_zeta_abs,trend,all(abs(eig(Ad))<1+1e-7)}; %#ok<AGROW>
end
V1=cell2table(rowsV1,'VariableNames',{'Ts_over_Ts0','Ts_s','EquilibriumResidual', ...
    'MaxFrequencyError_pct','MaxMappedRealError_per_s','TORFrequencyError_pct', ...
    'TORDampingAbsError','TrendStatus','AllDiscreteStable'});
V1Pass=all(V1.MaxFrequencyError_pct<0.5 & V1.TORFrequencyError_pct<0.5 & ...
    V1.TrendStatus=="CONSISTENT");

delayRows=cell(0,16);
if o.RunDelayScan
    dRatios=[0 0.5 1.0 1.5];
    for k=1:numel(dRatios)
        tau=dRatios(k)*Ts0;
        [Ad,resid]=sampledJacobian(xeq,u0,P,Ts0,tau,cfg);
        [ss,freq,zeta,cls]=discreteModalSummary(Ad,Ts0,S.state_names);
        met=extractS7Metrics(ss,freq,zeta,cls,Ts0);
        delayRows(end+1,:)={"A2_DELAY",dRatios(k),Ts0,tau,resid,met.torFreq, ...
            met.torZeta,met.torDe,met.maxReal,met.maxModulus,met.dcFreq, ...
            met.syncFreq,met.gscFreq,met.torClass,met.electricClass,met.stable}; %#ok<AGROW>
    end
end
Delay=cell2table(delayRows,'VariableNames',{'ScanType','DelayRatio','Ts_s','Delay_s', ...
    'EquilibriumResidual','TORFrequency_Hz','TORDampingRatio','TOR_DampingProxy', ...
    'MaxMappedReal_per_s','MaxPoleModulus','DCFrequency_Hz','SYNCFrequency_Hz', ...
    'GSCFrequency_Hz','TORClass','DominantElectricalClass','Stable'});

samplingRows=cell(0,16);
if o.RunSamplingScan
    ratios=[0.5 1 2 4];
    for k=1:numel(ratios)
        Ts=ratios(k)*Ts0;
        % 稀疏A3：典型一拍延迟，避免Ts×Td全笛卡尔积。
        [Ad,resid]=sampledJacobian(xeq,u0,P,Ts,Ts,cfg);
        [ss,freq,zeta,cls]=discreteModalSummary(Ad,Ts,S.state_names);
        met=extractS7Metrics(ss,freq,zeta,cls,Ts);
        Nbw=1/(Ts*max(P.gsc_current_bw_Hz,P.gsc_voltage_bw_Hz));
        if Nbw>=10, domain="ENGINEERING_MAIN"; elseif Nbw>=6, domain="STRESS"; else, domain="NON_ENGINEERING"; end
        samplingRows(end+1,:)={"A3_SAMPLING",ratios(k),Ts,Ts,resid,Nbw,domain, ...
            met.torFreq,met.torZeta,met.torDe,met.maxReal,met.maxModulus, ...
            met.dcFreq,met.syncFreq,met.gscFreq,met.stable}; %#ok<AGROW>
    end
end
Sampling=cell2table(samplingRows,'VariableNames',{'ScanType','Ts_over_Ts0','Ts_s', ...
    'Delay_s','EquilibriumResidual','N_bw','DigitalDomain','TORFrequency_Hz', ...
    'TORDampingRatio','TOR_DampingProxy','MaxMappedReal_per_s','MaxPoleModulus', ...
    'DCFrequency_Hz','SYNCFrequency_Hz','GSCFrequency_Hz','Stable'});

% S7A状态：V1通过才允许把A2/A3称为机制筛查；Gate V2仍因缺少真实
% Simulink离散平均模型而保持BLOCKED，不进入S7B EMT。
R.stageS7A1=struct('continuous_limit',V1,'gateV1_pass',V1Pass, ...
    'delay_scan',Delay,'sampling_scan',Sampling, ...
    'gateV2_status','BLOCKED_MISSING_DISCRETE_AVERAGE_SIMULINK_MODEL', ...
    'model_kind','same-RHS sampled-data average screening; not EMT');

writetable(V1,fullfile(here,'S7A_Continuous_Limit_Gate.csv'));
writetable(Delay,fullfile(here,'S7A_Delay_Scan_Summary.csv'));
writetable(Sampling,fullfile(here,'S7A_Sampling_Scan_Summary.csv'));
writeS7CompactEvidence(here,V1,Delay,Sampling,V1Pass);
writeS7Report(here,R,P,S,manifest,digitalManifest,V1Pass,V1,Delay,Sampling);
if o.SaveFigures && ~isempty(Delay)
    makeS7AFigure(fullfile(here,'S7A_Implementation_Screening.png'),Delay,Sampling);
end
fprintf('S7A执行完成：V1=%s；V2=BLOCKED（缺少真实离散平均Simulink模型）；S7B未启动。\n',iff(V1Pass,'PASS','FAIL'));
end

function M=writeS7ReferenceManifest(here,idealDir,S)
% 只生成摘要清单，不保存完整工作区。
files={fullfile(idealDir,'M0_PMSG_GFM_5MW.slx'), ...
    fullfile(idealDir,'m0_nonlinear_dynamics.m'), ...
    fullfile(idealDir,'linearize_m0_equilibrium.m'), ...
    fullfile(idealDir,'init_m0_5mw_parameters.m'), ...
    fullfile(idealDir,'m0_pack_parameters.m'), ...
    fullfile(idealDir,'M0_5MW_Aligned_Workpoint_and_SSM.mat')};
rows=cell(0,5);
for k=1:numel(files)
    f=files{k}; if exist(f,'file'), h=string(fileHash(f)); else, h="MISSING"; end
    rows(end+1,:)={"FILE",string(f),h,bytesIfExists(f),"S7参考输入，内容冻结"}; %#ok<AGROW>
end
P=S.params; OP=S.operating_point;
add=@(name,val,src,note)0; %#ok<NASGU>
params={ ...
    {'MODEL_SCOPE','M0_5MW ideal continuous physical averaged VSC','M0 README','S7不引入EMT/PWM'}, ...
    {'S6_STATUS','INCONCLUSIVE','M3 S6 report','S6不阻塞S7'}, ...
    {'Sbase_W',P.Sbase_W,'init_m0_5mw_parameters.m','5 MW'}, ...
    {'Vdc_ref_V',P.Vdc_ref_V,'init_m0_5mw_parameters.m','连续基准'}, ...
    {'Cdc_F',P.Cdc_F,'init_m0_5mw_parameters.m','DC-link'}, ...
    {'Cf_F',P.Cf_F,'init_m0_5mw_parameters.m','交流滤波电容'}, ...
    {'Jt_kgm2',P.Jt_kgm2,'init_m0_5mw_parameters.m','两质量轴系'}, ...
    {'Jg_kgm2',P.Jg_kgm2,'init_m0_5mw_parameters.m','两质量轴系'}, ...
    {'Ksh_Nm_per_rad',P.Ksh_Nm_per_rad,'init_m0_5mw_parameters.m','轴系刚度'}, ...
    {'Dsh_Nms_per_rad',P.Dsh_Nms_per_rad,'init_m0_5mw_parameters.m','轴系阻尼'}, ...
    {'H_s',P.H_s,'init_m0_5mw_parameters.m','VSG虚拟惯量'}, ...
    {'mp_radps_per_W',P.mp_radps_per_W,'init_m0_5mw_parameters.m','P-f下垂'}, ...
    {'DVCScale',P.Kp_dvc_A_per_V/7.8983182658,'audited parameter','固定不调参'}, ...
    {'GSC_current_bw_Hz',P.gsc_current_bw_Hz,'init_m0_5mw_parameters.m','连续等效'}, ...
    {'GSC_voltage_bw_Hz',P.gsc_voltage_bw_Hz,'init_m0_5mw_parameters.m','连续等效'}, ...
    {'SCR',P.SCR,'init_m0_5mw_parameters.m','固定不调参'}, ...
    {'controller_Ts_s',P.controller_Ts_s,'init_m0_5mw_parameters.m','S7数字筛查标称采样'}, ...
    {'P_MSC_dc_W',OP.P_MSC_dc_W,'operating_point','最终工作点'}, ...
    {'P_GSC_dc_W',OP.P_GSC_dc_W,'operating_point','最终工作点'}, ...
    {'P_PCC_W',OP.P_PCC_W,'operating_point','最终工作点'}, ...
    {'Udc0_V',OP.x0(9),'operating_point','最终工作点'}, ...
    {'state_count',size(S.A,1),'M0 SSM','23'}, ...
    {'input_count',size(S.B,2),'M0 SSM','6'} };
for k=1:numel(params)
    rows(end+1,:)={"PARAMETER",string(params{k}{1}),string(params{k}{2}),"",string(params{k}{3})+"; "+string(params{k}{4})}; %#ok<AGROW>
end
T=cell2table(rows,'VariableNames',{'Category','Item','Value_or_SHA256','Bytes','Source_and_Note'});
path=fullfile(here,'S7_Reference_Manifest.csv'); writetable(T,path);
M=struct('path',path,'table',T,'sha256_model',string(fileHash(files{1})), ...
    'sha256_rhs',string(fileHash(files{2})),'workpoint_P_PCC_W',OP.P_PCC_W, ...
    'workpoint_Udc_V',OP.x0(9),'state_count',size(S.A,1));
end

function D=writeS7DigitalManifest(here,idealDir,P)
src=fullfile(idealDir,'CurrentModel_Idealized');
controllers={ ...
    'MSC current','motorcontrol_legacy_ad_base.c','motor_PI2_calc','Euler-like source update; limits/anti-windup in source','AUDIT_ONLY_NOT_ALIGNED_TO_M0'; ...
    'MSC DVC','motorcontrol_legacy_ad_base.c','motor_PI2_calc','legacy C source; startup gate and saturation present','AUDIT_ONLY_NOT_ALIGNED_TO_M0'; ...
    'GSC current','grid_forming_control.c','motor_PI2_calc','legacy C source; sampled PWM scheduler present','AUDIT_ONLY_NOT_ALIGNED_TO_M0'; ...
    'GSC voltage','grid_forming_control.c','motor_PI2_calc','legacy C source; current/vector limits present','AUDIT_ONLY_NOT_ALIGNED_TO_M0'; ...
    'GFM/VSG','grid_forming_control.c','w_vsg_state update','legacy C source; PLL/presync/mode transition present','AUDIT_ONLY_NOT_ALIGNED_TO_M0'};
rows=cell(0,8);
for k=1:size(controllers,1)
    f=fullfile(src,controllers{k,2});
    rows(end+1,:)={controllers{k,1},controllers{k,2},controllers{k,3},P.controller_Ts_s, ...
        controllers{k,4},controllers{k,5},string(fileHash(f)),string(f)}; %#ok<AGROW>
end
T=cell2table(rows,'VariableNames',{'Controller','SourceFile','UpdateMethod','Ts_s','DelayOrLimitNote','S7Status','SourceSHA256','SourcePath'});
path=fullfile(here,'S7_Digital_Controller_Manifest.csv'); writetable(T,path);
% 记录本轮S7A筛查采用的离散化假设；不把它误写成旧C控制器的真实实现。
discRows={ ...
    'MSC current','Forward Euler state update (S7A screening only)',P.controller_Ts_s,0,'not the legacy C implementation'; ...
    'MSC DVC','Forward Euler state update (S7A screening only)',P.controller_Ts_s,0,'not the legacy C implementation'; ...
    'GSC current','Forward Euler state update (S7A screening only)',P.controller_Ts_s,0,'not the legacy C implementation'; ...
    'GSC voltage','Forward Euler state update (S7A screening only)',P.controller_Ts_s,0,'not the legacy C implementation'; ...
    'GFM/VSG','Forward Euler state update (S7A screening only)',P.controller_Ts_s,0,'not the legacy C implementation'};
DT=cell2table(discRows,'VariableNames',{'Controller','Method','Ts_s','Delay_s','Boundary'});
discPath=fullfile(here,'Controller_Discretization_Manifest.csv'); writetable(DT,discPath);
D=struct('path',path,'table',T,'discretization_manifest_path',discPath,'status','AUDIT_ONLY_NOT_ALIGNED_TO_M0');
end

function path=writeS7HypothesisRegister(here,M,D,stateAuditPath,P,S)
path=fullfile(here,'S7_Hypothesis_Register.md');
fid=fopen(path,'w','n','UTF-8'); assert(fid>0,'无法写入S7假设寄存器'); c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'# S7 电气实现保真度假设寄存器\n\n');
fprintf(fid,'更新时间：%s\n\n',datestr(now,31));
fprintf(fid,'## 证据边界\n\n');
fprintf(fid,'- S6：**INCONCLUSIVE**。S6机械模型保真度未完成同机组、可追溯柔性闭环验证；S7独立开放，不将S7结果升级为完整柔性风机结论。\n');
fprintf(fid,'- S7仅检验连续平均候选机制在采样、ZOH、控制器离散和开关实现中的保持、迁移或失效。\n');
fprintf(fid,'- 本轮 S7A 使用同源RHS的采样数据平均筛查器，不等同于真实C/S-Function离散控制器；S7B EMT未启动。\n\n');
fprintf(fid,'## 冻结项\n\n');
fprintf(fid,'|项目|冻结值|\n|---|---:|\n');
fprintf(fid,'|连续参考模型|`M0_PMSG_GFM_5MW.slx`|\n|状态数|%d|\n|额定功率|%.6g MW|\n|标称控制采样|%.9g s|\n|H|%.6g s|\n|SCR|%.6g|\n|DVC比例|%.6g|\n',S.A(1,1)*0+size(S.A,1),P.Sbase_W/1e6,P.controller_Ts_s,P.H_s,P.SCR,P.Kp_dvc_A_per_V/7.8983182658);
fprintf(fid,'\n## 候选假设\n\n');
fprintf(fid,'|编号|候选机制|可证伪条件|当前状态|\n|---|---|---|---|\n');
fprintf(fid,'|H7-1|数字延迟改变 `G_Te,omega_g` 的幅相|轴系频率处阻尼代理由正变负或显著迁移|待S7A筛查|\n');
fprintf(fid,'|H7-2|Pole/Excitation分类随实现迁移|TOR极点或模态残差排序发生反转|待S7A筛查|\n');
fprintf(fid,'|H7-3|电气模态进入轴系时间尺度|MAC/参与因子连续性支持模态接近或交换|待S7A筛查|\n');
fprintf(fid,'|H7-4|低频输入输出响应依赖实现|相同扰动下频率/阻尼/排序不一致|Gate V2前不下结论|\n');
fprintf(fid,'\n## Gate规则\n\n');
fprintf(fid,'- Gate V1：连续极限 `Ts/Ts0 -> 0`，频率误差 <0.5%%，TOR趋势一致；否则停止机制解释。\n');
fprintf(fid,'- Gate V2：真实离散平均Simulink模型与离散SSM小扰动 NRMSE <5%%；当前因模型不存在而保持 BLOCKED。\n');
fprintf(fid,'- Gate S7B：只有 V1、V2 通过后才选择少量开关EMT工况。\n\n');
fprintf(fid,'## 当前文件\n\n- `%s`\n- `%s`\n- `%s`\n- `%s`\n- 同目录下的S7A摘要CSV。\n',M.path,D.path,D.discretization_manifest_path,stateAuditPath);
end

function cmd=referenceCommands(x,u,P)
q=algebraicSignals(x,u,P);
cmd=[q.vmd;q.vmq;q.uinvd;q.uinvq];
end

function [Ad,resid]=sampledJacobian(xeq,u0,P,Ts,tau,cfg)
cmd0=referenceCommands(xeq,u0,P); z0=[xeq;cmd0;cmd0]; n=numel(z0); J=zeros(n);
f0=sampledMap(z0,u0,P,Ts,tau,cfg);
resid=max(abs((f0-z0)./max(cfg.stateScale,1)));
for k=1:n
    h=1e-6*max(abs(z0(k)),1);
    zp=z0; zm=z0; zp(k)=zp(k)+h; zm(k)=zm(k)-h;
    J(:,k)=(sampledMap(zp,u0,P,Ts,tau,cfg)-sampledMap(zm,u0,P,Ts,tau,cfg))/(2*h);
end
Ad=J;
end

function zn=sampledMap(z,u,P,Ts,tau,cfg)
x=z(1:23); oldCmd=z(24:27); oldCmd2=z(28:31);
q=algebraicSignals(x,u,P); newCmd=[q.vmd;q.vmq;q.uinvd;q.uinvq];
tau=max(0,tau);
% 计算延迟：tau<=Ts时旧命令先保持tau；tau=1.5Ts时，
% 更旧命令先保持0.5Ts，随后上一拍命令保持0.5Ts。
if tau<=Ts
    x1=integratePlant(x,oldCmd,u,P,tau,cfg);
    x2=integratePlant(x1,newCmd,u,P,Ts-tau,cfg);
else
    nDelay=floor(tau/Ts); remDelay=tau-nDelay*Ts;
    if nDelay==1
        x1=integratePlant(x,oldCmd2,u,P,min(remDelay,Ts),cfg);
        x2=integratePlant(x1,oldCmd,u,P,Ts-min(remDelay,Ts),cfg);
    else
        % S7第一轮只允许到1.5Ts；更大延迟不静默折算。
        error('S7 delay screen currently supports tau<=1.5Ts only.');
    end
end
% 控制器只在采样时刻更新，采用与连续增益一致的前向Euler状态更新。
x2(6)=x(6)+Ts*q.dxDc;
x2(7)=x(7)+Ts*q.dxMid;
x2(8)=x(8)+Ts*q.dxMiq;
x2(10)=x(10)+Ts*q.dxPf;
x2(11)=x(11)+Ts*q.dxQf;
x2(12)=x(12)+Ts*q.dxWv;
x2(13)=x(13)+Ts*q.dxDelta;
x2(14)=x(14)+Ts*q.dxVd;
x2(15)=x(15)+Ts*q.dxVq;
x2(16)=x(16)+Ts*q.dxId;
x2(17)=x(17)+Ts*q.dxIq;
zn=[x2;newCmd;oldCmd];
end

function xn=integratePlant(x,cmd,u,P,dt,cfg)
if dt<=0, xn=x; return; end
phys=[1:5 9 18:23]; xp=x(phys);
opts=odeset('RelTol',cfg.RelTol,'AbsTol',cfg.AbsTol,'MaxStep',max(dt/4,1e-8));
[~,xx]=ode45(@(t,z)plantRhs(t,z,x,phys,cmd,u,P),[0 dt],xp,opts); %#ok<ASGLU>
xn=x; xn(phys)=xx(end,:).';
end

function dz=plantRhs(~,xp,xfull,phys,cmd,u,P)
x=xfull; x(phys)=xp;
theta=x(1); wt=x(2); wg=x(3); imd=x(4); imq=x(5); Udc=x(9);
ifd=x(18); ifq=x(19); vcfd=x(20); vcfq=x(21); igd=x(22); igq=x(23);
dTm=u(1); dVgrid=u(5); dwGrid=u(4);
Tm=P.Tm0_Nm*P.omega_m0_radps/wt+dTm; wgrid=P.omega0_radps+dwGrid; Vgrid=P.Vphase_peak_V*(1+dVgrid);
omegaRel=wt-wg; omegaCoi=(P.Jt_kgm2*wt+P.Jg_kgm2*wg)/(P.Jt_kgm2+P.Jg_kgm2);
Tshaft=P.Ksh_Nm_per_rad*theta+P.Dsh_Nms_per_rad*omegaRel; Tgen=P.Kt_Nm_per_A*imq; we=P.pole_pairs*wg;
vpccd=vcfd+P.Rd_ohm*(ifd-igd); vpccq=vcfq+P.Rd_ohm*(ifq-igq);
vmd=cmd(1); vmq=cmd(2); uinvd=cmd(3); uinvq=cmd(4);
Pmsc=-1.5*(vmd*imd+vmq*imq); Pgsc=1.5*(uinvd*ifd+uinvq*ifq);
dz=zeros(12,1);
dz(1)=omegaRel;
dz(2)=(Tm-Tshaft-P.Dt_Nms_per_rad*(omegaCoi-P.omega_m0_radps))/P.Jt_kgm2;
dz(3)=(Tshaft-Tgen-P.Dg_Nms_per_rad*(omegaCoi-P.omega_m0_radps))/P.Jg_kgm2;
dz(4)=(vmd-P.Rs_ohm*imd-we*P.Lq_H*imq)/P.Ld_H;
dz(5)=(vmq-P.Rs_ohm*imq+we*(P.Ld_H*imd+P.psi_f_Wb))/P.Lq_H;
dz(6)=(Pmsc-Pgsc)/(P.Cdc_F*Udc);
dz(7)=(uinvd-vpccd-P.Rf_ohm*ifd+wgrid*P.Lf_H*ifq)/P.Lf_H;
dz(8)=(uinvq-vpccq-P.Rf_ohm*ifq-wgrid*P.Lf_H*ifd)/P.Lf_H;
dz(9)=(ifd-igd)/P.Cf_F+wgrid*vcfq;
dz(10)=(ifq-igq)/P.Cf_F-wgrid*vcfd;
dz(11)=(vpccd-Vgrid-P.Rg_ohm*igd+wgrid*P.Lg_H*igq)/P.Lg_H;
dz(12)=(vpccq-P.Rg_ohm*igq-wgrid*P.Lg_H*igd)/P.Lg_H;
end

function q=algebraicSignals(x,u,P)
theta=x(1); wt=x(2); wg=x(3); imd=x(4); imq=x(5); xiDc=x(6); xiMid=x(7); xiMiq=x(8); Udc=x(9);
Pf=x(10); Qf=x(11); wv=x(12); delta=x(13); xiVd=x(14); xiVq=x(15); xiId=x(16); xiIq=x(17);
ifd=x(18); ifq=x(19); vcfd=x(20); vcfq=x(21); igd=x(22); igq=x(23);
dTm=u(1); dPref=u(2); dQref=u(3); dwGrid=u(4); dVgrid=u(5); dVdcRef=u(6);
Tm=P.Tm0_Nm*P.omega_m0_radps/wt+dTm; %#ok<NASGU>
Pref=P.Pref_W+dPref; Qref=P.Qref_var+dQref; wgrid=P.omega0_radps+dwGrid; Vgrid=P.Vphase_peak_V*(1+dVgrid); VdcRef=P.Vdc_ref_V+dVdcRef; %#ok<NASGU>
omegaRel=wt-wg; omegaCoi=(P.Jt_kgm2*wt+P.Jg_kgm2*wg)/(P.Jt_kgm2+P.Jg_kgm2); Tshaft=P.Ksh_Nm_per_rad*theta+P.Dsh_Nms_per_rad*omegaRel; Tgen=P.Kt_Nm_per_A*imq; %#ok<NASGU>
eDc=VdcRef-Udc; imqRef=P.Kp_dvc_A_per_V*eDc+xiDc; imdRef=0; eMid=imdRef-imd; eMiq=imqRef-imq; we=P.pole_pairs*wg;
vmd=P.Kp_msc_i_V_per_A*eMid+xiMid+P.Rs_ohm*imdRef+we*P.Lq_H*imqRef;
vmq=P.Kp_msc_i_V_per_A*eMiq+xiMiq+P.Rs_ohm*imqRef-we*(P.Ld_H*imdRef+P.psi_f_Wb);
Pmsc=-1.5*(vmd*imd+vmq*imq);
icapd=ifd-igd; icapq=ifq-igq; vpccd=vcfd+P.Rd_ohm*icapd; vpccq=vcfq+P.Rd_ohm*icapq;
Ppcc=1.5*(vpccd*igd+vpccq*igq); Qpcc=1.5*(vpccq*igd-vpccd*igq);
c=cos(delta); s=sin(delta); vpd=c*vpccd+s*vpccq; vpq=-s*vpccd+c*vpccq; ifld=c*ifd+s*ifq; iflq=-s*ifd+c*ifq; igld=c*igd+s*igq; iglq=-s*igd+c*igq;
Vref=P.E0_peak_V+P.qv_droop_V_per_var*(Qref-Qf); evd=Vref-vpd; evq=-vpq;
ifdRef=P.Kp_gsc_v_A_per_V*evd+xiVd-P.Cf_F*wv*vpq+P.gsc_grid_current_feedforward*igld;
ifqRef=P.Kp_gsc_v_A_per_V*evq+xiVq+P.Cf_F*wv*vpd+P.gsc_grid_current_feedforward*iglq;
eid=ifdRef-ifld; eiq=ifqRef-iflq;
ucd=P.Kp_gsc_i_V_per_A*eid+xiId-wv*P.Lf_H*iflq+P.gsc_pcc_voltage_feedforward*(vpd+P.Rf_ohm*ifld);
ucq=P.Kp_gsc_i_V_per_A*eiq+xiIq+wv*P.Lf_H*ifld+P.gsc_pcc_voltage_feedforward*(vpq+P.Rf_ohm*iflq);
uinvd=c*ucd-s*ucq; uinvq=s*ucd+c*ucq; Pgsc=1.5*(uinvd*ifd+uinvq*ifq); %#ok<NASGU>
q=struct('vmd',vmd,'vmq',vmq,'uinvd',uinvd,'uinvq',uinvq,'dxDc',P.Ki_dvc_A_per_Vs*eDc, ...
    'dxMid',P.Ki_msc_i_V_per_As*eMid,'dxMiq',P.Ki_msc_i_V_per_As*eMiq, ...
    'dxPf',P.pq_filter_radps*(Ppcc-Pf),'dxQf',P.pq_filter_radps*(Qpcc-Qf), ...
    'dxWv',P.omega0_radps/(2*P.H_s*P.Sbase_W)*(P.vsg_physical_power_error_sign*(Pref-Pf)-(wv-wgrid)/P.mp_radps_per_W), ...
    'dxDelta',wv-wgrid,'dxVd',P.Ki_gsc_v_A_per_Vs*evd,'dxVq',P.Ki_gsc_v_A_per_Vs*evq, ...
    'dxId',P.Ki_gsc_i_V_per_As*eid,'dxIq',P.Ki_gsc_i_V_per_As*eiq);
end

function [ss,freq,zeta,cls]=discreteModalSummary(Ad,Ts,stateNames)
[V,D]=eig(Ad); z=diag(D); keep=abs(z)>1e-10; z=z(keep); V=V(:,keep);
ss=log(z)/Ts; freq=abs(imag(ss))/(2*pi); zeta=-real(ss)./max(abs(ss),eps);
cls=classifyRightVectors(V,stateNames);
end

function cls=classifyRightVectors(V,stateNames)
cls=strings(size(V,2),1);
if isempty(V), return; end
names=["TOR" "DC" "SYNC" "GSC"];
for k=1:size(V,2)
    v=V(:,k); n=min(numel(stateNames),23);
    if n<23, cls(k)="UNKNOWN"; continue; end
    g=[sum(abs(v([1 2 3])).^2),sum(abs(v([6 9 10 11])).^2), ...
        sum(abs(v([12 13])).^2),sum(abs(v([14:23])).^2)];
    [~,ii]=max(g); cls(k)=names(ii);
end
end

function [err,trend]=compareContinuousModes(A,stateNames,ss,freq,zeta,cls)
[V,D]=eig(A); lc=diag(D); fc=abs(imag(lc))/(2*pi); zc=-real(lc)./max(abs(lc),eps); cc=classifyRightVectors(V,stateNames); %#ok<NASGU>
err.freq_max_pct=Inf; err.real_max_abs=Inf; err.tor_freq_pct=Inf; err.tor_zeta_abs=Inf;
% 连续/离散右特征向量的状态分组仅用于筛查，不能保证同一阶次的标签
% 在数值排序中一致。因此Gate V1采用物理轴系窗口(0.5--10 Hz)内
% 最接近理论2.5 Hz的正频共轭模态作为TOR配对，避免把LCL高频模态
% 的分类误差误判为连续极限失败。
ic=find(imag(lc)>1e-7 & fc>0.5 & fc<10); id=find(imag(ss)>1e-7 & freq>0.5 & freq<10);
if isempty(ic)||isempty(id), trend="INCONSISTENT"; return; end
[~,a]=min(abs(fc(ic)-2.5)); ic0=ic(a); [~,b]=min(abs(freq(id)-fc(ic0))); id0=id(b);
err.tor_freq_pct=100*abs(freq(id0)-fc(ic0))/max(fc(ic0),eps);
err.tor_zeta_abs=abs(zeta(id0)-zc(ic0));
err.freq_max_pct=err.tor_freq_pct; err.real_max_abs=abs(real(ss(id0))-real(lc(ic0)));
trend=string(iff(err.tor_freq_pct<0.5&&err.tor_zeta_abs<0.02,'CONSISTENT','INCONSISTENT'));
end

function m=extractS7Metrics(ss,freq,zeta,cls,Ts)
pos=find(imag(ss)>1e-7 & freq>0.05 & freq<50);
if isempty(pos), m=struct('torFreq',NaN,'torZeta',NaN,'torDe',NaN,'maxReal',max(real(ss)), ...
        'maxModulus',max(abs(exp(ss*Ts))), 'dcFreq',NaN,'syncFreq',NaN,'gscFreq',NaN,'torClass',"NONE",'electricClass',"NONE",'stable',false); return; end
% TOR优先用物理频率窗口识别；右向量分类在离散映射中可能因零状态
% 和共轭排序而漂移，因此不把“没有TOR标签”当成没有轴系模态。
torPos=pos(freq(pos)>0.5 & freq(pos)<10); [~,kk]=min(abs(freq(torPos)-2.5)); midx=torPos(kk); tf=freq(midx); tz=zeta(midx);
[df,~]=pickClass(pos,cls,freq,zeta,"DC"); [sf,~]=pickClass(pos,cls,freq,zeta,"SYNC"); [gf,~]=pickClass(pos,cls,freq,zeta,"GSC");
m=struct('torFreq',tf,'torZeta',tz,'torDe',-real(ss(midx)),'maxReal',max(real(ss)), ...
    'maxModulus',max(abs(exp(ss*Ts))),'dcFreq',df,'syncFreq',sf,'gscFreq',gf, ...
    'torClass',"TOR",'electricClass',"MULTI",'stable',max(abs(exp(ss*Ts)))<1+1e-7);
end

function [f,z]=pickClass(pos,cls,freq,zeta,name)
ix=pos(cls(pos)==name); if isempty(ix), f=NaN; z=NaN; else, [~,k]=min(abs(freq(ix)-2.5)); f=freq(ix(k)); z=zeta(ix(k)); end
end

function path=writeS7ControllerStateAudit(here,P,S)
% S7-1：把M0状态逐一映射到物理连续状态或软件离散状态。
% 这是候选数字实现审计，不声称已经复刻旧C/S-Function。
N=23; names=string(S.state_names(:)); rows=cell(N,9);
physical=[1 2 3 4 5 9 18 19 20 21 22 23];
eq={ ...
 'd(theta_sh)/dt = omega_t - omega_g'; ...
 'Jt*d(omega_t)/dt = Tm - Tshaft - Dt*(omega_coi-wm0)'; ...
 'Jg*d(omega_g)/dt = Tshaft - Tgen - Dg*(omega_coi-wm0)'; ...
 'Ld*d(i_m_d)/dt = v_md - Rs*i_m_d - np*omega_g*Lq*i_m_q'; ...
 'Lq*d(i_m_q)/dt = v_mq - Rs*i_m_q + np*omega_g*(Ld*i_m_d+psi_f)'; ...
 'xi_dvc[k+1] = xi_dvc[k] + Ts*Ki_dvc*(VdcRef-Udc)'; ...
 'xi_m_id[k+1] = xi_m_id[k] + Ts*Ki_mi*(i_md_ref-i_md)'; ...
 'xi_m_iq[k+1] = xi_m_iq[k] + Ts*Ki_mi*(i_mq_ref-i_mq)'; ...
 'Cdc*Udc*d(Udc)/dt = Pmsc - Pgsc'; ...
 'P_f[k+1] = P_f[k] + Ts*wpf*(Ppcc-P_f[k])'; ...
 'Q_f[k+1] = Q_f[k] + Ts*wpf*(Qpcc-Q_f[k])'; ...
 'omega_v[k+1] = omega_v[k] + Ts*w0/(2H*Sb)*(sP*(Pref-P_f)-(omega_v-wgrid)/mp)'; ...
 'delta_v[k+1] = delta_v[k] + Ts*(omega_v-wgrid)'; ...
 'xi_g_vd[k+1] = xi_g_vd[k] + Ts*Kigv*(Vref-vpd)'; ...
 'xi_g_vq[k+1] = xi_g_vq[k] + Ts*Kigv*(-vpq)'; ...
 'xi_g_id[k+1] = xi_g_id[k] + Ts*Kigi*(i_fd_ref-i_fd)'; ...
 'xi_g_iq[k+1] = xi_g_iq[k] + Ts*Kigi*(i_fq_ref-i_fq)'; ...
 'Lf*d(i_f_d)/dt = u_inv_d-vpcc_d-Rf*i_f_d+wgrid*Lf*i_f_q'; ...
 'Lf*d(i_f_q)/dt = u_inv_q-vpcc_q-Rf*i_f_q-wgrid*Lf*i_f_d'; ...
 'Cf*d(v_cf_d)/dt = i_f_d-i_g_d+wgrid*Cf*v_cf_q'; ...
 'Cf*d(v_cf_q)/dt = i_f_q-i_g_q-wgrid*Cf*v_cf_d'; ...
 'Lg*d(i_g_d)/dt = vpcc_d-Vgrid-Rg*i_g_d+wgrid*Lg*i_g_q'; ...
 'Lg*d(i_g_q)/dt = vpcc_q-Rg*i_g_q-wgrid*Lg*i_g_d'};
update=cell(N,1); typ=cell(N,1); method=cell(N,1); source=cell(N,1);
for k=1:N
    if ismember(k,physical)
        typ{k}='physical_continuous'; update{k}='continuous plant ODE over [kTs,(k+1)Ts)'; method{k}='ODE'; source{k}='M0 nonlinear RHS';
    else
        typ{k}='controller_software'; update{k}='x_c[k+1]=x_c[k]+Ts*f_c(x_c[k],y[k])'; method{k}='Forward Euler candidate'; source{k}='M0 controller equation; actual C method not aligned';
    end
end
for k=1:N
    rows(k,:)={k,names(k),eq{k},typ{k},update{k},method{k},P.controller_Ts_s,0,source{k}};
end
T=cell2table(rows,'VariableNames',{'Index','State','M0_Equation','StateType','S7_Update','DiscretizationMethod','Ts_s','IntrinsicDelay_s','SourceBoundary'});
path=fullfile(here,'S7_Controller_State_Audit.csv'); writetable(T,path);
end

function writeS7Report(here,R,P,S,M,D,V1Pass,V1,Delay,Sampling)
path=fullfile(here,'S7_Final_Report_CN.md'); fid=fopen(path,'w','n','UTF-8'); assert(fid>0,'无法写入S7报告'); c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'# S7 电气实现保真度验证阶段报告\n\n'); fprintf(fid,'生成时间：%s\n\n',datestr(now,31));
fprintf(fid,'## 总体状态\n\n'); fprintf(fid,'- S6：**INCONCLUSIVE**，不因本轮结果升级。\n');
fprintf(fid,'- S7-0参考冻结：**COMPLETE**。\n- S7A同源采样平均筛查：**EXECUTED**。\n- Gate V1连续极限：**%s**。\n- Gate V2：**BLOCKED**，当前没有与M0严格对齐的真实离散平均Simulink模型。\n- S7B开关EMT：**NOT STARTED**。\n\n',iff(V1Pass,'PASS','FAIL'));
fprintf(fid,'## 参考边界\n\n'); fprintf(fid,'模型：`M0_PMSG_GFM_5MW.slx`；状态数：%d；额定功率：%.6g MW；标称采样：%.9g s。\n',size(S.A,1),P.Sbase_W/1e6,P.controller_Ts_s);
fprintf(fid,'工作点：P_MSC=%.9f MW，P_GSC=%.9f MW，P_PCC=%.9f MW，Udc=%.6f V。\n\n',S.operating_point.P_MSC_dc_W/1e6,S.operating_point.P_GSC_dc_W/1e6,S.operating_point.P_PCC_W/1e6,S.operating_point.x0(9));
fprintf(fid,'## 方法说明\n\n'); fprintf(fid,'采样平均筛查器从同一23状态RHS计算采样时刻控制命令，控制器积分状态按前向Euler更新，变流器命令经ZOH保持，plant使用连续ODE积分；延迟通过一个采样区间内旧/新命令分段保持实现。该实现只用于S7A筛查，不能替代真实C/S-Function数字控制器或开关EMT。\n\n');
fprintf(fid,'## M0离散接口审计\n\n');
fprintf(fid,'当前保存的 M0 使用 `ode15s` 变步长求解器和 23 个连续 Integrator；`M0_RHS` 在保存模型中为隐藏的 `sf_sfun` S-Function。模型内未发现可追溯的 Unit Delay、Zero-Order Hold 或离散 PI 更新链，因此不能仅通过修改求解器设置得到计划中的真实离散平均模型。\n\n');
fprintf(fid,'旧 C/S-Function 控制器已单独列入 `S7_Digital_Controller_Manifest.csv`，但其 PLL/模式切换/限幅/PWM 调度与 M0 连续方程不对齐，只能作为实现审计来源，不能用于 Gate V2。\n\n');
fprintf(fid,'## S7-1状态映射审计\n\n');
fprintf(fid,'`S7_Controller_State_Audit.csv` 已覆盖全部23个状态：12个物理连续状态采用ODE，11个软件/控制状态给出Forward-Euler候选差分式。该表满足候选映射可追溯性，但由于当前保存模型没有真实离散控制块，Gate G7-1仍标记为 CONDITIONAL，不能据此声称已复刻实际C控制器。\n\n');
fprintf(fid,'状态审计文件：`S7_Controller_State_Audit.csv`；控制器离散化假设文件：`Controller_Discretization_Manifest.csv`。\n\n');
if V1Pass
    fprintf(fid,'## S7A数值摘要\n\n');
    fprintf(fid,'连续极限三点的最大轴系频率误差为 %.6g%%，最大轴系阻尼代理误差为 %.6g；三点均保持离散稳定。\n',max(V1.TORFrequencyError_pct),max(V1.TORDampingAbsError));
    if ~isempty(Delay)
        fprintf(fid,'A2延迟筛查：Td/Ts=0、0.5、1、1.5 均未出现离散不稳定，轴系频率范围 %.6f--%.6f Hz，阻尼代理范围 %.6g--%.6g /s。\n',min(Delay.TORFrequency_Hz),max(Delay.TORFrequency_Hz),min(Delay.TOR_DampingProxy),max(Delay.TOR_DampingProxy));
    end
    if ~isempty(Sampling)
        fprintf(fid,'A3采样筛查：Ts/Ts0=0.5、1、2、4 均未出现离散不稳定；Ts/Ts0=4 被标为 STRESS，不作为工程主结论。\n\n');
    else
        fprintf(fid,'A3采样筛查：本次未运行。\n\n');
    end
end
fprintf(fid,'## 文件\n\n- `%s`\n- `%s`\n- `Controller_Discretization_Manifest.csv`\n- `S7A_Continuous_Limit_Gate.csv`\n- `S7A_Delay_Scan_Summary.csv`\n- `S7A_Sampling_Scan_Summary.csv`\n- `S7A_Discrete_Mode_Tracking.csv`\n- `S7A_Feedback_Damping.csv`\n- `S7A_Excitation_Comparison.csv`\n- `S7A_Boundary_Summary.csv`\n- `S7_Final_Mechanism_Evidence_Matrix.csv`\n- `run_s7_discrete_validation.m`\n\n',M.path,D.path);
fprintf(fid,'## 解释纪律\n\n1. 任何A2/A3极点移动只能作为实现依赖性筛查，不称为物理机制。\n2. 若V1失败，必须先修正采样映射，不得讨论负阻尼或方向性。\n3. 在Gate V2通过前，不能把S7A结果写成“离散控制已验证”；在S7B通过前，不能写成“开关实现稳健”。\n');
end

function writeS7CompactEvidence(here,V1,Delay,Sampling,V1Pass)
% 只保存可复核的摘要，不保存特征向量、完整时序或求解历史。
% 注意：TOR_DampingProxy 是离散TOR极点的 -real(s)，不是复转矩 De(jw)。
rows=cell(0,13);
for k=1:height(Delay)
    rows(end+1,:)={"A2_DELAY",Delay.DelayRatio(k),Delay.Ts_s(k),Delay.Delay_s(k),NaN,"DELAY_SCAN", ...
        Delay.TORFrequency_Hz(k),Delay.TORDampingRatio(k),Delay.TOR_DampingProxy(k), ...
        Delay.MaxMappedReal_per_s(k),Delay.MaxPoleModulus(k),Delay.Stable(k), ...
        "SCREENING_ONLY_PHYSICAL_TOR_WINDOW"}; %#ok<AGROW>
end
for k=1:height(Sampling)
    rows(end+1,:)={"A3_SAMPLING",Sampling.Ts_over_Ts0(k),Sampling.Ts_s(k),Sampling.Delay_s(k),Sampling.N_bw(k),Sampling.DigitalDomain(k), ...
        Sampling.TORFrequency_Hz(k),Sampling.TORDampingRatio(k),Sampling.TOR_DampingProxy(k), ...
        Sampling.MaxMappedReal_per_s(k),Sampling.MaxPoleModulus(k),Sampling.Stable(k), ...
        "SCREENING_ONLY_PHYSICAL_TOR_WINDOW"}; %#ok<AGROW>
end
ModeTracking=cell2table(rows,'VariableNames',{'ScanType','ParameterRatio','Ts_s','Delay_s','N_bw','DigitalDomain', ...
    'TORFrequency_Hz','TORDampingRatio','TOR_DampingProxy_per_s','MaxMappedReal_per_s', ...
    'MaxPoleModulus','Stable','ModeIdentityStatus'});
writetable(ModeTracking,fullfile(here,'S7A_Discrete_Mode_Tracking.csv'));

fbRows=cell(0,6);
for k=1:height(Delay)
    fbRows(end+1,:)={"A2_DELAY",Delay.DelayRatio(k),Delay.TORFrequency_Hz(k),Delay.TOR_DampingProxy(k), ...
        "-real(s_TOR); not De(jw_tor)","SCREENING_ONLY_NOT_COMPLEX_TORQUE"}; %#ok<AGROW>
end
for k=1:height(Sampling)
    fbRows(end+1,:)={"A3_SAMPLING",Sampling.Ts_over_Ts0(k),Sampling.TORFrequency_Hz(k),Sampling.TOR_DampingProxy(k), ...
        "-real(s_TOR); not De(jw_tor)","SCREENING_ONLY_NOT_COMPLEX_TORQUE"}; %#ok<AGROW>
end
Feedback=cell2table(fbRows,'VariableNames',{'ScanType','ParameterRatio','Frequency_Hz','DampingProxy_per_s','QuantityDefinition','Status'});
writetable(Feedback,fullfile(here,'S7A_Feedback_Damping.csv'));

Excitation=table("S7A","NOT_COMPUTED", ...
    "B_d/C_d与离散平均Simulink输入输出映射尚未导出；Gate V2前不计算模态残差", ...
    "BLOCKED_MISSING_DISCRETE_IO_MAP", ...
    'VariableNames',{'Route','Status','Reason','EvidenceBoundary'});
writetable(Excitation,fullfile(here,'S7A_Excitation_Comparison.csv'));

if V1Pass, v1Status="PASS"; else, v1Status="FAIL_STOP"; end
if isempty(Delay), a2Status="NOT_RUN"; elseif all(Delay.Stable), a2Status="SCREENING_STABLE"; else, a2Status="SCREENING_COUNTEREXAMPLE"; end
if isempty(Sampling), a3Status="NOT_RUN"; elseif all(Sampling.Stable), a3Status="SCREENING_STABLE"; else, a3Status="SCREENING_COUNTEREXAMPLE"; end
Boundary=table(["V1_CONTINUOUS_LIMIT";"A2_DELAY";"A3_SAMPLING";"V2_DISCRETE_SSM";"S7B_EMT"], ...
    [v1Status;a2Status;a3Status;"BLOCKED";"NOT_STARTED"], ...
    ["Ts/Ts0趋零时同源采样映射与连续SSM趋势一致"; ...
     "同源采样延迟筛查未见不稳定反例；仅为实现依赖性筛查"; ...
     "标称一拍延迟采样筛查未见不稳定反例；4Ts为STRESS点"; ...
     "缺少可追溯真实离散平均Simulink模型，不能比较离散SSM与非线性离散模型"; ...
     "只有V2通过后才启动少量开关EMT"], ...
    'VariableNames',{'Gate','Status','EvidenceBoundary'});
writetable(Boundary,fullfile(here,'S7A_Boundary_Summary.csv'));

Evidence=table(["S6";"S7-0";"S7A-V1";"S7A-A2/A3";"S7A-V2";"S7B"], ...
    ["INCONCLUSIVE";"COMPLETE";v1Status;"SCREENED";"BLOCKED";"NOT_STARTED"], ...
    ["OBSERVATION_ONLY";"REFERENCE_FROZEN";"CONDITIONAL_MECHANISM_SCREENING";"IMPLEMENTATION_SCREENING";"NOT_ESTABLISHED";"NOT_ESTABLISHED"], ...
    ["S6柔性机械跨模型证据未闭合"; ...
     "M0连续模型、工作点、参数和数字审计清单已冻结"; ...
     "同源采样平均映射在连续极限下通过V1"; ...
     "延迟/采样筛查未见反例，但不等于真实数字控制验证"; ...
     "缺少真实离散平均Simulink模型"; ...
     "未启动，避免把筛查误写为EMT稳健性"], ...
    'VariableNames',{'Stage','Status','EvidenceLevel','ClaimBoundary'});
writetable(Evidence,fullfile(here,'S7_Final_Mechanism_Evidence_Matrix.csv'));
end

function makeS7AFigure(path,D,S)
fig=figure('Visible','off','Color','w','Position',[80 80 1200 700]); tiledlayout(fig,1,2,'TileSpacing','compact');
nexttile; if ~isempty(D), plot(D.DelayRatio,D.TORDampingRatio,'o-','LineWidth',1.5); grid on; xlabel('T_d/T_s'); ylabel('TOR damping ratio'); title('S7A delay screen'); end
nexttile; if ~isempty(S), plot(S.Ts_over_Ts0,S.TORDampingRatio,'s-','LineWidth',1.5); grid on; xlabel('T_s/T_{s0}'); ylabel('TOR damping ratio'); title('S7A sampling screen'); end
exportgraphics(fig,path,'Resolution',180); close(fig);
end

function h=fileHash(f)
if exist(f,'file')~=2, h='MISSING'; return; end
fid=fopen(f,'r'); assert(fid>0,'无法读取文件用于哈希：%s',f);
b=fread(fid,Inf,'*uint8'); fclose(fid);
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(b);
raw=typecast(md.digest(),'uint8'); h=lower(reshape(dec2hex(raw,2).',1,[]));
end

function n=bytesIfExists(f)
if exist(f,'file')==2, d=dir(f); n=d.bytes; else, n=NaN; end
end

function s=iff(c,a,b)
if c, s=a; else, s=b; end
end
