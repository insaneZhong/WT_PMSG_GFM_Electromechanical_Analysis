function R = run_mechanism_closure_analysis(varargin)
%RUN_MECHANISM_CLOSURE_ANALYSIS
% 理想连续5 MW模型的机制闭合主入口：多模态叠加、H非单调、SCR/H/DVC机制区、
% Grid<->Machine方向依赖传递，以及少量代表点非线性验证。
% 默认不保存原始时序、workspace或求解历史；图件统一保存到一个目录。
ip=inputParser; ip.addParameter('SaveFigures',true,@(x)islogical(x)&&isscalar(x)); ip.addParameter('RunNonlinearValidation',true,@(x)islogical(x)&&isscalar(x)); ip.parse(varargin{:}); o=ip.Results;
here=fileparts(mfilename('fullpath')); figDir=fullfile(here,'Figures_Mechanism_Closure'); if ~exist(figDir,'dir'), mkdir(figDir); end
try
    [models,base]=prepare_multimode_models(); %#ok<ASGLU>
    super=analyze_minimal_modal_superposition(models,here); assert(super.gate,'Gate 1 FAIL：最小多模态重构未通过。');
    H=analyze_H_nonmonotonic_modal_mechanism(here);
    transition=analyze_mechanism_transition_parameter_scans(H.summary,here);
    bidir=analyze_bidirectional_disturbance_transfer(here);
    region=build_mechanism_region_map(here);
    if o.RunNonlinearValidation, validation=localValidateRepresentatives(region,base,models); else, validation=struct([]); end
    gate1=super.gate; gate2=localGate2(region,validation);
    R=struct('objective','GFM风机多模态扰动塑形、双向机电传播及机制区域分析','gate1',gate1,'gate2',gate2, ...
        'modal_superposition',super,'H_mechanism',H,'transition',transition,'bidirectional',bidir,'region',region,'validation',validation);
    localWriteSummary(R,here);
    localWriteNovelty(R,here);
    if o.SaveFigures, make_mechanism_closure_figures(R,figDir); end
    latest=fullfile(here,'latest_failed_case.mat'); if exist(latest,'file'), delete(latest); end
    fprintf('MECHANISM_CLOSURE_GATE_1=%d\n',gate1); fprintf('MECHANISM_CLOSURE_GATE_2=%d\n',gate2);
catch ME
    latest_failed_case=struct('case_name','mechanism closure','message',ME.message,'stack',{ME.stack}); save(fullfile(here,'latest_failed_case.mat'),'latest_failed_case'); rethrow(ME);
end
end

function V=localValidateRepresentatives(region,base,models)
% 每一机制类最多一个代表点；使用0.005 Hz输入，保持严格小信号区。
classes=["PATH_SHAPING_DOMINATED","JOINT_POLE_PATH_SHAPING","WEAK_CONTROL_EFFECT"]; T=region.summary; iv=find(strcmpi(base.models,'VSG'),1); p0=base.parameter_vector; xSeed=models{iv}.x0; scr0=4;
V=struct('name',{},'classification',{},'map',{},'scr',{},'control_value',{},'output',{},'t',{},'nl',{},'ssm',{},'minimal',{},'nrmse_full',{},'corr_full',{},'peak_full',{},'nrmse_min',{},'corr_min',{},'peak_min',{},'pass',{});
for q=1:numel(classes)
    ix=find(T.Status=="PASS" & T.MechanismClass==classes(q),1); if isempty(ix), continue, end
    row=T(ix,:); p=p0; p(9)=p0(9)*scr0/row.SCR; p(10)=p0(10)*scr0/row.SCR;
    if row.Map=="SCR_H", p(33)=row.ControlValue; else, p(25)=p0(25)*row.ControlValue; p(26)=p0(26)*row.ControlValue; end
    [x,~]=solve_multimode_control_equilibrium(xSeed,p,'VSG',struct); L=multimode_linearize_control(x,p,'VSG',struct); M=multimode_modal_data(L.A,L.state_names);
    d=zeros(4,1); d(4)=2*pi*.005; [t,~,Y]=multimode_simulate_linear_step(L,d,.10,10,5001); Yn=localNonlinear(x,p,d,t);
    for out=string({'omega_sh','T_sh'})
        iy=find(strcmp(L.output_names,out),1); full=Y(:,iy); nl=Yn(:,iy); miny=localMinimal(M,L,iy,d,t,.10); [n1,c1,p1]=localMetric(nl,full); [n2,c2,p2]=localMetric(nl,miny);
        V(end+1)=struct('name',sprintf('%s: SCR=%g, value=%g',char(row.Map),row.SCR,row.ControlValue),'classification',char(row.MechanismClass),'map',char(row.Map),'scr',row.SCR,'control_value',row.ControlValue,'output',char(out),'t',t,'nl',nl,'ssm',full,'minimal',miny,'nrmse_full',n1,'corr_full',c1,'peak_full',p1,'nrmse_min',n2,'corr_min',c2,'peak_min',p2,'pass',n1<.02 && c1>.98 && n2<.05 && c2>.98); %#ok<AGROW>
    end
end
end

function Y=localNonlinear(x0,p,d,t)
i=find(t>=.10,1); opt=odeset('RelTol',1e-8,'AbsTol',1e-9,'MaxStep',.01); [t1,x1]=ode15s(@(~,x)source_aligned_rhs_control(x,p,'VSG',zeros(4,1),struct),t(1:i),x0,opt); [t2,x2]=ode15s(@(~,x)source_aligned_rhs_control(x,p,'VSG',d,struct),t(i:end),x1(end,:).',opt); tt=[t1;t2(2:end)]; xx=[x1;x2(2:end,:)]; [y0,names]=source_aligned_internal_outputs_control(x0,p,'VSG',zeros(4,1),struct); Y=zeros(numel(tt),numel(y0));
for k=1:numel(tt), dd=zeros(4,1); if tt(k)>=.10, dd=d; end, Y(k,:)=source_aligned_internal_outputs_control(xx(k,:).',p,'VSG',dd,struct).'-y0.'; end
end

function y=localMinimal(M,L,iy,d,t,tStep)
cand=find((imag(M.lambda)>1e-8)|(abs(imag(M.lambda))<=1e-8 & abs(M.lambda)>1e-6)); score=zeros(numel(cand),1); R=zeros(numel(cand),1);
for q=1:numel(cand), k=cand(q); R(q)=L.C(iy,:)*M.V(:,k)*(M.W(:,k)'*L.B*d); score(q)=abs(R(q)/M.lambda(k))*(1+(imag(M.lambda(k))>1e-8)); end
[~,o]=sort(score,'descend'); o=o(1:min(5,numel(o))); tau=max(t-tStep,0); y=zeros(size(t)); ix=t>=tStep;
for q=1:numel(o), lam=M.lambda(cand(o(q))); term=R(o(q))/lam*(exp(lam*tau(ix))-1); if imag(lam)>1e-8, y(ix)=y(ix)+2*real(term); else, y(ix)=y(ix)+real(term); end, end
end
function [n,c,p]=localMetric(a,b), n=norm(a-b)/max(norm(a-mean(a)),eps); aa=a-mean(a); bb=b-mean(b); c=(aa'*bb)/max(norm(aa)*norm(bb),eps); p=100*abs(max(abs(a))-max(abs(b)))/max(max(abs(a)),eps); end
function ok=localGate2(region,V)
classes=unique(region.summary.MechanismClass(region.summary.Status=="PASS")); required=["PATH_SHAPING_DOMINATED","JOINT_POLE_PATH_SHAPING","WEAK_CONTROL_EFFECT"]; classCoverage=all(ismember(required,classes)); if isempty(V), nl=true; else, nl=all([V.pass]); end; ok=classCoverage && nl;
end
function localWriteSummary(R,outDir)
rows=struct('Gate1',R.gate1,'Gate2',R.gate2,'HMechanism',R.H_mechanism.mechanism,'HPeak_s',R.H_mechanism.peak_H,'HInputPeak_s',R.H_mechanism.input_peak_H,'HPoleRelativeChange',R.H_mechanism.pole_change,'BidirectionalRows',height(R.bidirectional.summary),'RegionRows',height(R.region.summary));
T=struct2table(rows); writetable(T,fullfile(outDir,'Mechanism_Closure_Run_Summary.csv'));
fid=fopen(fullfile(outDir,'Mechanism_Closure_Report_CN.md'),'w'); assert(fid>0,'无法写入机制闭合报告。');
fprintf(fid,'# GFM风机多模态扰动塑形、双向机电传播及机制区域分析\n\n');
fprintf(fid,'## 结论边界\n\n本报告仅使用已对齐的5 MW理想连续平均模型及同源23状态小信号模型；未加入EMT、PWM、离散PI、限幅、启动或保护。\n\n');
fprintf(fid,'## Gate结果\n\n- Gate 1（3–5主导模态阶跃重构）：%s。\n- Gate 2（机制区域类别和少量NL代表点）：%s。\n\n',localPass(R.gate1),localPass(R.gate2));
fprintf(fid,'## H非单调机理\n\nTOR残差峰值 H=%.3g s，TOR输入投影峰值 H=%.3g s，当前分类为 **%s**；TOR阻尼最大相对变化 %.3f%%。\n\n',R.H_mechanism.peak_H,R.H_mechanism.input_peak_H,R.H_mechanism.mechanism,100*R.H_mechanism.pole_change);
fprintf(fid,'## 双向传播\n\n详见 `Bidirectional_Disturbance_Transfer_Summary.csv` 与 `Bidirectional_Coupling_Matrix_Summary.csv`。使用“方向依赖扰动传递”，不把非共轭输入输出的差异误称为非互易性。\n\n');
fprintf(fid,'## 机制区域\n\n连续指标采用 I_pole=|zeta_tor-zeta_ref|/zeta_ref 与 I_path=|log10(Gamma_path)|；5%%和20%%仅作分类可视化阈值，不是稳定边界。\n\n');
if ~isempty(R.validation)
    fprintf(fid,'## 理想连续非线性代表点\n\n'); fprintf(fid,'|Case|Class|Output|Full SSM NRMSE|Minimal NRMSE|Full peak error %%|PASS|\n|---|---|---|---:|---:|---:|---|\n');
    for k=1:numel(R.validation), v=R.validation(k); fprintf(fid,'|%s|%s|%s|%.4g|%.4g|%.4g|%d|\n',v.name,v.classification,v.output,v.nrmse_full,v.nrmse_min,v.peak_full,v.pass); end
end
fclose(fid);
end
function localWriteNovelty(R,outDir)
fid=fopen(fullfile(outDir,'Novelty_Evidence_Matrix.md'),'w'); assert(fid>0,'无法写入证据矩阵。');
fprintf(fid,'# 实验事实与后续论文支撑矩阵\n\n| Dimension | Existing-style analysis | Current work evidence |\n|---|---|---|\n');
fprintf(fid,'| Torsional pole | eigenvalue/damping | 同一或近似极点下仍可出现不同的扰动响应；采用 I_pole 与 I_path 区分。 |\n');
fprintf(fid,'| Complex torque | G_Te,omega_g | 当前架构复转矩近似相同，仍通过残差和多模态叠加解释响应差异。 |\n');
fprintf(fid,'| DVC | damping effect | 扫描同时记录DVC通道增益、极点指标和扰动残差。 |\n');
fprintf(fid,'| GFM architecture | controller comparison | GWT在MSC iq*节点截断，MWT保留Udc->iq*->Te路径。 |\n');
fprintf(fid,'| Modal analysis | torsional mode | 3–5个TOR/DC/SYNC/GSC/SPEED模态重构时域响应。 |\n');
fprintf(fid,'| Coupling direction | machine/grid interaction | Grid->Machine 与 Machine->Grid 的方向依赖传递、PCC可观测性摘要。 |\n');
fprintf(fid,'| Grid strength | damping sensitivity | SCR-H/SCR-DVC机制区域区分路径塑形和共同塑形。 |\n');
fprintf(fid,'\n本文档仅记录当前实验事实，不宣称“首次”或“文献从未提出”。\n'); fclose(fid);
end
function s=localPass(x), if x, s='PASS'; else, s='REVIEW'; end, end
