function E=run_5mw_mechanism_evidence(varargin)
%RUN_5MW_MECHANISM_EVIDENCE
% 5 MW IdealAvg 对齐模型的机理证据主程序。
%
% 本程序只使用当前唯一的 Idealized SLX 和同源 23 状态方程：
% 1) 0.1/0.2/0.4% 机械转矩扰动线性区验收；
% 2) 机械反馈开环的 G_Te,wg(jw) 复转矩频率扫描；
% 3) 同一工作点的线性通道消融；
% 4) 中心差分与复步长 Jacobian 一致性检查。
% 不复制模型、不保存原始时序、不保存图片。

ip=inputParser;
ip.addParameter('StopTime_s',8,@(x)isnumeric(x)&&isscalar(x)&&x>3);
ip.addParameter('StepTime_s',1,@(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('SaveSummary',true,@(x)islogical(x)&&isscalar(x));
ip.addParameter('UseExistingSummary',false,@(x)islogical(x)&&isscalar(x));
ip.parse(varargin{:}); o=ip.Results;

here=fileparts(mfilename('fullpath'));
addpath(here,fullfile(here,'temp'),fileparts(here));
if o.UseExistingSummary
    q=load(fullfile(here,'03_Mechanism_Evidence_Summary.mat'),'E');
    E=refreshMechanismSummary(q.E);
    save(fullfile(here,'03_Mechanism_Evidence_Summary.mat'),'E','-v7');
    writeReport(fullfile(here,'03_Mechanism_Evidence_Report_CN.md'),E);
    fprintf('SUMMARY_REFRESHED_FROM_EXISTING_DATA=1\n');
    return
end

% 以同一联合物理平衡点为唯一基准；该函数不写文件。
J=solve_joint_physical_periodic_operating_point();
O=J.source_aligned; p=J.pvec(:); x=stateVector(O); n=numel(x);

% ---- A. 解析状态方程的线性化和数值一致性 ----
[Afd,~,~,~]=linearizeFD(x,p);
Acs=linearizeCS(x,p);
jacobianError=norm(Afd-Acs,'fro')/max(norm(Acs,'fro'),eps);
ev=eig(Afd); [shaftEig,shaftHz,shaftZeta]=pickShaft(ev,Afd);

% ---- B. 机械侧 0.1/0.2/0.4% 小转矩扰动 ----
levels=[0.001 0.002 0.004];
S0=probe_short_joint_transient(o.StopTime_s,J.x0,p,J.source_aligned,1e6,0);
torqueRows=table('Size',[numel(levels) 10], ...
    'VariableTypes',{'double','double','double','double','double','double','double','double','double','double'}, ...
    'VariableNames',{'Level_pu','DeltaTorque_Nm','NL_Frequency_Hz','NL_Damping','FreqError_pct','DampingError_pct','ScaleNRMSE','PeakOmega_rel','Stable','FitResidual'});
baseNorm=[];
for k=1:numel(levels)
    dT=levels(k)*O.Tgen_Nm;
    Sk=probe_short_joint_transient(o.StopTime_s,J.x0,p,J.source_aligned,1e6,0, ...
        'TorqueStepTime_s',o.StepTime_s,'DeltaTorque_Nm',dT);
    [t0,y0]=relativeSpeed(S0); [tk,yk]=relativeSpeed(Sk);
    y=interp1(tk,yk,t0,'linear','extrap')-y0;
    fit=fitShaftDecay(t0,y,o.StepTime_s,shaftHz,shaftZeta);
    if k==1, baseNorm=max(norm(y),eps); else, baseNorm=max(baseNorm,eps); end
    if k==1, yBase=y/levels(k); end
    scaleNrmse=norm(y/levels(k)-yBase)/max(norm(yBase),eps);
    torqueRows.Level_pu(k)=levels(k);
    torqueRows.DeltaTorque_Nm(k)=dT;
    torqueRows.NL_Frequency_Hz(k)=fit.frequency_Hz;
    torqueRows.NL_Damping(k)=fit.damping_ratio;
    torqueRows.FreqError_pct(k)=100*abs(fit.frequency_Hz-shaftHz)/shaftHz;
    torqueRows.DampingError_pct(k)=100*abs(fit.damping_ratio-shaftZeta)/max(shaftZeta,eps);
    torqueRows.ScaleNRMSE(k)=scaleNrmse;
    torqueRows.PeakOmega_rel(k)=max(abs(y));
    torqueRows.Stable(k)=fit.sigma<0;
    torqueRows.FitResidual(k)=fit.residual;
end

% ---- C. 机械反馈开环的复转矩 ----
freq_Hz=linspace(0.2,10,200).'; w=2*pi*freq_Hz;
idxE=4:23; Cte=zeros(1,numel(idxE)); Cte(2)=p(18); % Tgen=Kt*imq
Bwg=Afd(idxE,3); Aee=Afd(idxE,idxE);
G=zeros(numel(w),1);
for k=1:numel(w)
    G(k)=Cte*((1i*w(k)*eye(numel(idxE))-Aee)\Bwg);
end
De=real(G); Ke=-w.*imag(G);
[~,iTor]=min(abs(freq_Hz-shaftHz));
complexAtShaft=table(freq_Hz(iTor),real(G(iTor)),imag(G(iTor)),De(iTor),Ke(iTor), ...
    'VariableNames',{'Frequency_Hz','G_Re','G_Im','D_e_Nms_per_rad','K_e_Nm_per_rad'});
freqScan=table(freq_Hz,real(G),imag(G),De,Ke, ...
    'VariableNames',{'Frequency_Hz','G_Re','G_Im','D_e_Nms_per_rad','K_e_Nm_per_rad'});

% ---- D. 同一平衡点线性通道消融 ----
% 累积闭合：机械 -> PMSG -> MSC -> DVC/DC -> GSC内环 -> GFM。
stageNames={'Mechanical only';'+ PMSG speed coupling';'+ MSC finite-bandwidth states'; ...
    '+ DVC and DC-link';'+ GSC inner/LCL';'+ GFM'};
active={1:3,1:5,1:8,1:11,[1:11 14:23],1:23};
ablationRows=table('Size',[numel(active) 6], ...
    'VariableTypes',{'string','double','double','double','double','double'}, ...
    'VariableNames',{'Configuration','ShaftFrequency_Hz','ShaftDamping','MaxRealPole','D_e_at_tor','K_e_at_tor'});
for k=1:numel(active)
    Ak=closeByStateGroups(Afd,active{k});
    [lamk,fk,zk]=pickShaft(eig(Ak),Ak);
    [Gk,dek,kek]=teSpeedResponse(Ak,idxE,Cte,w,Bwg);
    ablationRows.Configuration(k)=stageNames{k};
    ablationRows.ShaftFrequency_Hz(k)=fk;
    ablationRows.ShaftDamping(k)=zk;
    ablationRows.MaxRealPole(k)=max(real(eig(Ak)));
    ablationRows.D_e_at_tor(k)=dek(iTor);
    ablationRows.K_e_at_tor(k)=kek(iTor);
end

% 针对直接闭环通道的旁路消融：同一工作点，不重新配平。
cutNames={'Full';'DVC bypass';'DC-link bypass';'GSC/GFM power fixed'};
cutMats={Afd,cutDVC(Afd),cutDC(Afd),cutGFM(Afd)};
cutRows=table('Size',[numel(cutMats) 6], ...
    'VariableTypes',{'string','double','double','double','double','double'}, ...
    'VariableNames',{'Configuration','ShaftFrequency_Hz','ShaftDamping','MaxRealPole','D_e_at_tor','K_e_at_tor'});
for k=1:numel(cutMats)
    Ak=cutMats{k}; [lamk,fk,zk]=pickShaft(eig(Ak),Ak); %#ok<ASGLU>
    [~,dek,kek]=teSpeedResponse(Ak,idxE,Cte,w,Afd(idxE,3));
    cutRows.Configuration(k)=cutNames{k}; cutRows.ShaftFrequency_Hz(k)=fk;
    cutRows.ShaftDamping(k)=zk; cutRows.MaxRealPole(k)=max(real(eig(Ak)));
    cutRows.D_e_at_tor(k)=dek(iTor); cutRows.K_e_at_tor(k)=kek(iTor);
end

E=struct('model',J.model,'operating_point',J.source_aligned, ...
    'small_signal',struct('A',Afd,'eigenvalues',ev,'max_real_pole',max(real(ev)), ...
    'shaft_eigenvalue',shaftEig,'shaft_frequency_Hz',shaftHz,'shaft_damping_ratio',shaftZeta), ...
    'jacobian',struct('central_vs_complex_step_relative_error',jacobianError), ...
    'torque_linearity',torqueRows,'complex_torque_at_shaft',complexAtShaft, ...
    'complex_torque_scan',freqScan,'cumulative_ablation',ablationRows,'cut_ablation',cutRows, ...
    'settings',o);
E=refreshMechanismSummary(E);

if o.SaveSummary
    save(fullfile(here,'03_Mechanism_Evidence_Summary.mat'),'E','-v7');
    writeReport(fullfile(here,'03_Mechanism_Evidence_Report_CN.md'),E);
end
fprintf('\n=== 5 MW机理证据阶段 ===\n');
fprintf('shaft=%.6f Hz, zeta=%.6f%%, maxRe=%.6g 1/s\n',shaftHz,100*shaftZeta,max(real(ev)));
fprintf('Jacobian central-vs-complex relative error=%.3e\n',jacobianError);
disp(torqueRows); disp(complexAtShaft); disp(cutRows);
end

function x=stateVector(O)
c=O.controller_x0; x=[O.theta_tw0;O.omega0;O.omega0;O.pmsg_id0;O.pmsg_iq0; ...
    c(1:3);O.pvec(2);c(4:11);O.if_grid_dq_A;O.vcap_grid_dq_V;O.ig_grid_dq_A];
end

function [A,B,C,D]=linearizeFD(x,p)
n=numel(x); A=zeros(n); C=zeros(6,n);
for k=1:n
    h=1e-6*max(abs(x(k)),1); xp=x; xm=x; xp(k)=xp(k)+h; xm(k)=xm(k)-h;
    A(:,k)=(source_aligned_rhs(xp,p)-source_aligned_rhs(xm,p))/(2*h);
    C(:,k)=(source_aligned_outputs(xp,p)-source_aligned_outputs(xm,p))/(2*h);
end
hp=10; pp=p; pm=p; pp(37)=pp(37)+hp; pm(37)=pm(37)-hp;
B=(source_aligned_rhs(x,pp)-source_aligned_rhs(x,pm))/(2*hp);
D=(source_aligned_outputs(x,pp)-source_aligned_outputs(x,pm))/(2*hp);
end

function A=linearizeCS(x,p)
n=numel(x); A=zeros(n); h=1e-25;
for k=1:n
    xc=x; xc(k)=xc(k)+1i*h;
    A(:,k)=imag(source_aligned_rhs(xc,p))/h;
end
end

function [t,y]=relativeSpeed(S)
t=S.omega_t.t(:); y=S.omega_t.y(:)-interp1(S.omega_g.t(:),S.omega_g.y(:),t,'linear','extrap');
end

function fit=fitShaftDecay(t,y,t0,f0,z0)
idx=t>=t0+0.05 & t<=min(t0+4,t(end)); tt=t(idx)-t0; yy=y(idx);
if numel(tt)<20, fit=struct('frequency_Hz',NaN,'damping_ratio',NaN,'sigma',NaN,'residual',Inf); return; end
q0=[f0,-z0*2*pi*f0]; opts=optimset('Display','off','MaxIter',300,'TolX',1e-7,'TolFun',1e-10);
q=fminsearch(@(q)decayCost(q,tt,yy),q0,opts); f=q(1); sigma=q(2);
M=[ones(size(tt)),tt,exp(sigma*tt).*cos(2*pi*f*tt),exp(sigma*tt).*sin(2*pi*f*tt)]; b=M\yy;
fit=struct('frequency_Hz',abs(f),'damping_ratio',-sigma/sqrt(sigma^2+(2*pi*f)^2), ...
    'sigma',sigma,'residual',norm(M*b-yy)/max(norm(yy),eps));
end

function c=decayCost(q,t,y)
f=q(1); sigma=q(2);
if f<1 || f>5 || sigma<-10 || sigma>1, c=1e6+(abs(f)+abs(sigma))*1e3; return; end
M=[ones(size(t)),t,exp(sigma*t).*cos(2*pi*f*t),exp(sigma*t).*sin(2*pi*f*t)];
c=norm(M*(M\y)-y)^2/max(norm(y)^2,eps);
end

function [lam,f,zeta]=pickShaft(ev,A)
V=[]; %#ok<NASGU>
[V,~]=eig(A); %#ok<ASGLU>
cand=find(imag(ev)>0 & abs(imag(ev))/(2*pi)>1 & abs(imag(ev))/(2*pi)<5);
if isempty(cand), [~,ii]=min(abs(real(ev))); lam=ev(ii); f=abs(imag(lam))/(2*pi); zeta=-real(lam)/max(abs(lam),eps); return; end
% Eigenvector mechanical participation; eig(A) ordering is recomputed here.
[V,D]=eig(A); ee=diag(D); cand=find(imag(ee)>0 & abs(imag(ee))/(2*pi)>1 & abs(imag(ee))/(2*pi)<5);
[~,ii]=max(sum(abs(V(1:3,cand)).^2,1)); lam=ee(cand(ii)); f=abs(imag(lam))/(2*pi); zeta=-real(lam)/max(abs(lam),eps);
end

function A2=closeByStateGroups(A,active)
inactive=setdiff(1:size(A,1),active); A2=A; A2(active,inactive)=0; A2(inactive,active)=0;
end

function A2=cutDVC(A)
A2=A; A2(4:9,[6 9])=0; A2(6,:)=0; A2(6,6)=0;
end

function A2=cutDC(A)
A2=A; A2(9,:)=0; A2(:,9)=0;
end

function A2=cutGFM(A)
A2=A; A2(9,10:23)=0; A2(12:23,9)=0;
end

function [G,De,Ke]=teSpeedResponse(A,idxE,Cte,w,Bwg)
Aee=A(idxE,idxE); B=A(idxE,3); G=zeros(numel(w),1);
for k=1:numel(w), G(k)=Cte*((1i*w(k)*eye(numel(idxE))-Aee)\B); end
De=real(G); Ke=-w.*imag(G);
end

function E=refreshMechanismSummary(E)
% 用已保存的同源 A 矩阵补齐精确轴系频率点与 gamma 通道扫描。
% 该后处理不运行模型、不生成时序，仅更新唯一汇总 MAT/报告。
A=E.small_signal.A; op=E.operating_point; p=op.pvec(:);
idxE=4:23; Cte=zeros(1,numel(idxE)); Cte(2)=p(18);
Bwg=A(idxE,3); wtor=2*pi*E.small_signal.shaft_frequency_Hz;
Gtor=Cte*((1i*wtor*eye(numel(idxE))-A(idxE,idxE))\Bwg);
E.complex_torque_at_shaft=table(E.small_signal.shaft_frequency_Hz,real(Gtor),imag(Gtor),real(Gtor),-wtor*imag(Gtor), ...
    'VariableNames',{'Frequency_Hz','G_Re','G_Im','D_e_Nms_per_rad','K_e_Nm_per_rad'});
E.gamma_scan=gammaScan(A,p(18),E.small_signal.shaft_frequency_Hz);
E=refreshAblationAtExactFrequency(E);
end

function E=refreshAblationAtExactFrequency(E)
% 将历史表中的频响数值统一改为精确轴系频率点，避免与扫描最近网格点混淆。
A=E.small_signal.A; p=E.operating_point.pvec(:); idxE=4:23;
Cte=zeros(1,numel(idxE)); Cte(2)=p(18);
wtor=2*pi*E.small_signal.shaft_frequency_Hz;
active={1:3,1:5,1:8,1:11,[1:11 14:23],1:23};
for k=1:numel(active)
    Ak=closeByStateGroups(A,active{k});
    [~,fk,zk]=pickShaft(eig(Ak),Ak);
    G=Cte*((1i*wtor*eye(numel(idxE))-Ak(idxE,idxE))\Ak(idxE,3));
    E.cumulative_ablation.ShaftFrequency_Hz(k)=fk;
    E.cumulative_ablation.ShaftDamping(k)=zk;
    E.cumulative_ablation.MaxRealPole(k)=max(real(eig(Ak)));
    E.cumulative_ablation.D_e_at_tor(k)=real(G);
    E.cumulative_ablation.K_e_at_tor(k)=-wtor*imag(G);
end
cuts={A,cutDVC(A),cutDC(A),cutGFM(A)};
for k=1:numel(cuts)
    Ak=cuts{k}; [~,fk,zk]=pickShaft(eig(Ak),Ak);
    G=Cte*((1i*wtor*eye(numel(idxE))-Ak(idxE,idxE))\Ak(idxE,3));
    E.cut_ablation.ShaftFrequency_Hz(k)=fk;
    E.cut_ablation.ShaftDamping(k)=zk;
    E.cut_ablation.MaxRealPole(k)=max(real(eig(Ak)));
    E.cut_ablation.D_e_at_tor(k)=real(G);
    E.cut_ablation.K_e_at_tor(k)=-wtor*imag(G);
end
end

function T=gammaScan(A,Kt,shaftHz)
% 通道耦合强度 gamma=0..1 的同一工作点线性消融。
% gamma=0 为旁路，gamma=1 为完整闭环；不重新配平。
channels={'DVC';'DC-link';'GFM'};
gammas=[0 0.25 0.5 0.75 1].';
T=table('Size',[numel(channels)*numel(gammas) 7], ...
    'VariableTypes',{'string','double','double','double','double','double','double'}, ...
    'VariableNames',{'Channel','Gamma','ShaftFrequency_Hz','ShaftDamping','MaxRealPole','D_e_Nms_per_rad','K_e_Nm_per_rad'});
idxE=4:23; Cte=zeros(1,numel(idxE)); Cte(2)=Kt; Bwg=A(idxE,3); wtor=2*pi*shaftHz;
r=0;
for ic=1:numel(channels)
    for ig=1:numel(gammas)
        r=r+1; Ag=applyGamma(A,channels{ic},gammas(ig));
        ev=eig(Ag); [~,fk,zk]=pickShaft(ev,Ag);
        G=Cte*((1i*wtor*eye(numel(idxE))-Ag(idxE,idxE))\Bwg);
        T.Channel(r)=channels{ic}; T.Gamma(r)=gammas(ig);
        T.ShaftFrequency_Hz(r)=fk; T.ShaftDamping(r)=zk;
        T.MaxRealPole(r)=max(real(ev)); T.D_e_Nms_per_rad(r)=real(G);
        T.K_e_Nm_per_rad(r)=-wtor*imag(G);
    end
end
end

function Ag=applyGamma(A,channel,gamma)
Ag=A; n=size(A,1);
switch channel
    case 'DVC'
        % DVC 状态为 6，DVC/DC-link 反馈出口位于状态 4:9。
        Ag(4:9,[6 9])=gamma*Ag(4:9,[6 9]);
        Ag(6,:)=gamma*Ag(6,:);
    case 'DC-link'
        % 保留 DC-link 自身极点，仅缩放其与其余状态的双向耦合。
        r=setdiff(1:n,9);
        Ag(9,r)=gamma*Ag(9,r); Ag(r,9)=gamma*Ag(r,9);
    case 'GFM'
        % GFM 对 DC-link 的有功闭环耦合；保持 GFM 内部状态自身动态。
        Ag(9,10:23)=gamma*Ag(9,10:23);
        Ag(12:23,9)=gamma*Ag(12:23,9);
    otherwise
        error('Unknown gamma channel: %s',channel);
end
end

function writeReport(file,E)
fid=fopen(file,'w','n','UTF-8'); assert(fid>0,'Cannot write report.'); c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'# 5 MW构网型PMSG机理证据阶段报告\n\n');
fprintf(fid,'## 基准封账\n\n');
fprintf(fid,'当前唯一基准模型：`%s.slx`。工作点为 %.6f MW PCC、MSC/GSC端口 %.6f MW。\n\n',E.model,E.operating_point.P_pcc_measurement_W/1e6,E.operating_point.P_gsc_W/1e6);
fprintf(fid,'小信号最大极点实部 %.6g 1/s；轴系模态 %.6f Hz，阻尼比 %.6f%%。\n\n',E.small_signal.max_real_pole,E.small_signal.shaft_frequency_Hz,100*E.small_signal.shaft_damping_ratio);
fprintf(fid,'中心差分与复步长Jacobian相对误差：%.3e。\n\n',E.jacobian.central_vs_complex_step_relative_error);
fprintf(fid,'## 机械小扰动线性区\n\n'); writeTableMD(fid,E.torque_linearity); fprintf(fid,'\n');
fprintf(fid,'## 轴系频率处复转矩\n\n'); fprintf(fid,'定义 G_Te,wg=DeltaTe/Deltaomegag，D_e=Re(G)，K_e=-omega Im(G)。\n\n'); writeTableMD(fid,E.complex_torque_at_shaft); fprintf(fid,'\n');
fprintf(fid,'该频响在机械反馈开环下计算，频率为轴系特征值的精确频率点；正负号按发电机方程 J_g*domega_g=T_sh-T_gen 解释。\n\n');
if isfield(E,'gamma_scan')
    fprintf(fid,'## 通道 gamma 消融（同一工作点、不重新配平）\n\n');
    fprintf(fid,'gamma=0 表示该通道旁路，gamma=1 表示完整闭环。该表用于量化通道耦合趋势，不把多闭环贡献误写成严格线性相加。\n\n');
    writeTableMD(fid,E.gamma_scan); fprintf(fid,'\n');
end
fprintf(fid,'## 累积通道闭合\n\n'); writeTableMD(fid,E.cumulative_ablation); fprintf(fid,'\n');
fprintf(fid,'## 直接旁路消融\n\n'); writeTableMD(fid,E.cut_ablation); fprintf(fid,'\n');
fprintf(fid,'## 解释边界\n\n');
fprintf(fid,'上述消融均在同一工作点的线性化方程上完成，不重新配平。当前 GFM gamma 扫描基本不改变轴系极点，说明在这个理想连续对齐模型中，GFM暂未形成独立的轴系自治反馈通道；后续应把它作为激励/模态残差问题继续验证，不能直接表述为“GFM必然降低轴系阻尼”。\n');
end

function writeTableMD(fid,T)
vars=T.Properties.VariableNames;
fprintf(fid,'|'); for k=1:numel(vars), fprintf(fid,'%s|',vars{k}); end; fprintf(fid,'\n|');
for k=1:numel(vars), fprintf(fid,'---|'); end; fprintf(fid,'\n');
for r=1:height(T)
    fprintf(fid,'|');
    for k=1:numel(vars)
        v=T{r,k};
        if iscell(v), v=v{1}; end
        if isstring(v) || ischar(v), s=char(string(v));
        elseif isnumeric(v) || islogical(v), s=num2str(v,8);
        else, s=char(string(v)); end
        fprintf(fid,'%s|',s);
    end
    fprintf(fid,'\n');
end
end
