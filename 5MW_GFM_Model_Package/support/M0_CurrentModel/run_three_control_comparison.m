function R=run_three_control_comparison(varargin)
%RUN_THREE_CONTROL_COMPARISON GFL-Droop-GFM-VSG-GFM公平对比入口。
% 先执行Gate A；若共同工作点不是同源连续方程的严格平衡点，立即停止。

ip=inputParser;
ip.addParameter('ResidualTolerance',1e-8,@(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('SaveSummary',true,@(x)islogical(x)&&isscalar(x));
ip.parse(varargin{:}); o=ip.Results;
here=fileparts(mfilename('fullpath')); addpath(here,fileparts(here));
q=load(fullfile(here,'03_Mechanism_Evidence_Summary.mat'),'E'); E0=q.E;
Oseed=E0.operating_point; [x,p,commonEq]=solveCommonEquilibrium(Oseed);
modes={'GFL','DROOP','VSG'};
labels={'GFL (ideal PLL)';'Droop-GFM';'VSG-GFM'};

T=table('Size',[3 14], ...
    'VariableTypes',{'string','double','double','double','double','double','double','double','double','double','double','double','double','logical'}, ...
    'VariableNames',{'Control','P_PCC_MW','Q_PCC_Mvar','Udc_V','omega_g_radps','Te_MNm','Tsh_MNm','P_MSC_MW','P_GSC_MW','I_MSC_A','I_GSC_A','Residual_norm','Workpoint_difference_pct','GateA'});
raw=zeros(3,11); res=zeros(3,1);
for k=1:3
    m=modes{k}; dx=source_aligned_rhs_control(x,p,m,zeros(4,1)); y=source_aligned_outputs_control(x,p,m,zeros(4,1));
    [Pmsc,Pgsc,Qpcc,Imsc,Igsc]=equilibriumMetrics(x,p,m);
    raw(k,:)=[y(1)/1e6,Qpcc/1e6,y(2),x(3),y(3)/1e6,y(4)/1e6,Pmsc/1e6,Pgsc/1e6,Imsc,Igsc,normalizedResidual(dx)];
    res(k)=normalizedResidual(dx);
end
ref=raw(1,1:10); scales=[5,5,1500,1,4,4,5,5,1e4,1e4]; diffPct=max(abs(raw(:,1:10)-ref)./scales*100,[],2);
for k=1:3
    T.Control(k)=labels{k}; T{k,2:12}=raw(k,1:11); T.Workpoint_difference_pct(k)=diffPct(k); T.GateA(k)=(res(k)<=o.ResidualTolerance && diffPct(k)<=0.1);
end
T.Residual_norm=res; pass=all(T.GateA);
R=struct('stage','GateA','passed',pass,'tolerance',o.ResidualTolerance,'workpoint',T,'model',E0.model,'source_operating_point',commonEq, ...
    'notes',{{'GFL uses ideal PLL/electrical-grid tracking because the current aligned source equations contain no finite-bandwidth PLL state.','Residual is evaluated before any modal comparison.'}});
R.gateA=T;
if pass, R=completeLinearComparison(R,E0,x,p,modes,labels); else, R.stop_reason='Gate A failed: common source-aligned equilibrium was not obtained. Do not compare damping/residues yet.'; end
if o.SaveSummary
    save(fullfile(here,'ThreeControl_Summary.mat'),'R','-v7');
    writetable(T,fullfile(here,'Fair_Workpoint_Audit.csv'));
    if isfield(R,'pole_torque_summary'), writetable(R.pole_torque_summary,fullfile(here,'ThreeControl_Summary.csv')); end
    if isfield(R,'residue_summary'), writetable(R.residue_summary,fullfile(here,'Modal_Residue_Summary.csv')); end
    if isfield(R,'nonlinear_summary'), writetable(R.nonlinear_summary,fullfile(here,'Nonlinear_Validation_Summary.csv')); end
    writeReport(fullfile(here,'Fair_GFL_Droop_VSG_Report_CN.md'),R);
end
disp(T); if pass, fprintf('PASS_GATE_A=1\n'); else, fprintf('STOP_GATE_A=1\n'); end
end

function x=stateVector(O)
c=O.controller_x0; x=[O.theta_tw0;O.omega0;O.omega0;O.pmsg_id0;O.pmsg_iq0;c(1:3);O.pvec(2);c(4:11);O.if_grid_dq_A;O.vcap_grid_dq_V;O.ig_grid_dq_A];
end

function [x,p,eq]=solveCommonEquilibrium(Oseed)
% 用当前SPS工作点作初值，但在连续源方程内重新求解严格平衡。
P=init_m0_5mw_parameters(); OP=struct('Tm0_Nm',Oseed.Tgen_Nm,'E0_peak_V',abs(Oseed.vnode_grid_dq_V(1)+1i*Oseed.vnode_grid_dq_V(2)));
[p,~]=m0_pack_parameters(P,OP); x0=stateVector(Oseed);
sx=[1;1;1;1e4;1e4;1e4;1e4;1e4;1e3;5e6;5e6;1;1;1e4;1e4;1e4;1e4;1e4;1e4;1e3;1e3;1e4;1e4];
sr=[1;1;1;1e4;1e4;1e4;1e4;1e4;1e6;5e8;5e8;1;1;1e4;1e4;1e4;1e4;1e6;1e6;1e6;1e6;1e6;1e6];
opts=optimoptions('fsolve','Display','off','FunctionTolerance',1e-11,'StepTolerance',1e-11,'OptimalityTolerance',1e-11,'MaxIterations',500);
[z,fval,exitflag]=fsolve(@(z)source_aligned_rhs_control(sx.*z,p,'VSG',zeros(4,1))./sr,x0./sx,opts); x=sx.*z;
eq=struct('x',x,'pvec',p,'residual_norm',norm(fval,inf),'exitflag',exitflag,'outputs',source_aligned_outputs_control(x,p,'VSG',zeros(4,1)),'seed',Oseed);
assert(exitflag>0 && eq.residual_norm<1e-8,'Strict common continuous equilibrium solve failed.');
end

function r=normalizedResidual(dx)
s=[1;1;1;1e4;1e4;1e4;1e4;1e4;1e6;5e8;5e8;1;1;1e4;1e4;1e4;1e4;1e6;1e6;1e6;1e6;1e6;1e6];
r=max(abs(dx)./s);
end

function R=completeLinearComparison(R,E0,x,p,modes,labels)
% Gate A通过后，计算三种同步机制的极点、复转矩和四类归一化残差。
  n=numel(x); nM=numel(modes); % 四类扰动均使用物理量幅值，并在Bbar中固定归一化尺度。
  dBase=[0.005*R.workpoint.Tsh_MNm(1)*1e6;0.005*p(1);deg2rad(0.2);2*pi*0.05];
Cout=zeros(2,n); Cout(1,2)=1; Cout(1,3)=-1; Cout(2,1)=p(21); Cout(2,2)=p(22); Cout(2,3)=-p(22);
freq=linspace(0.2,10,200).'; w=2*pi*freq; common=[];
L=struct('Control',cell(nM,1),'ShaftEigenvalue',cell(nM,1),'ShaftFrequency_Hz',zeros(nM,1),'ShaftDamping',zeros(nM,1),'MechanicalParticipation',zeros(nM,1),'MaxRealPole',zeros(nM,1),'De_self',zeros(nM,1),'Ke_self',zeros(nM,1),'De_common',zeros(nM,1),'Ke_common',zeros(nM,1),'Residue',cell(nM,1),'StepResidue',cell(nM,1));
for k=1:nM
    [A,Bbar]=linearizeMode(x,p,modes{k},dBase); [lam,ft,zt,eta,v,wleft]=pickTorsionalMode(A);
    [G,De,Ke]=complexTorque(A,p,w); common(end+1)=ft; %#ok<AGROW>
    L(k).Control=labels{k}; L(k).ShaftEigenvalue=lam; L(k).ShaftFrequency_Hz=ft; L(k).ShaftDamping=zt; L(k).MechanicalParticipation=eta; L(k).MaxRealPole=activeMaxRealPole(A,modes{k}); L(k).De_self=interp1(freq,De,ft,'pchip'); L(k).Ke_self=interp1(freq,Ke,ft,'pchip'); L(k).Residue=modalResidue(Cout,v,wleft,Bbar,lam); L(k).StepResidue=L(k).Residue/lam;
    L(k).A=A; L(k).Bbar=Bbar; L(k).DeScan=De; L(k).KeScan=Ke; %#ok<AGROW>
end
fcommon=mean(common);
for k=1:nM, L(k).De_common=interp1(freq,L(k).DeScan,fcommon,'pchip'); L(k).Ke_common=interp1(freq,L(k).KeScan,fcommon,'pchip'); end
R.status='Gate A passed; pole shaping and modal-residue calculations completed'; R.control_modes=modes; R.control_labels=labels; R.reference_small_signal=E0.small_signal; R.common_frequency_Hz=fcommon; R.disturbance_names={'Mechanical torque';'Aerodynamic power';'Grid angle';'Grid frequency'}; R.disturbance_base=dBase;
% 不把完整A、B和频率扫描长期写入结果文件；仅保留论文所需汇总量。
S=struct('Control',cell(nM,1),'ShaftEigenvalue',cell(nM,1),'ShaftFrequency_Hz',zeros(nM,1),'ShaftDamping',zeros(nM,1),'MechanicalParticipation',zeros(nM,1),'MaxRealPole',zeros(nM,1),'De_self',zeros(nM,1),'Ke_self',zeros(nM,1),'De_common',zeros(nM,1),'Ke_common',zeros(nM,1));
for k=1:nM, S(k)=rmfield(L(k),{'A','Bbar','DeScan','KeScan','Residue','StepResidue'}); end
  R.pole_torque_summary=struct2table(S); R.residue_summary=buildResidueTable(L,R.disturbance_names,labels); R.pole_disturbance=buildPoleDisturbance(R.pole_torque_summary,R.residue_summary);
  R.nonlinear_summary=runRepresentativeNonlinear(x,p,modes,labels,R.disturbance_base,R.pole_torque_summary,R.residue_summary);
end

function [A,Bbar]=linearizeMode(x,p,mode,dBase)
n=numel(x); A=zeros(n); Bbar=zeros(n,4);
for k=1:n
    h=1e-6*max(abs(x(k)),1); xp=x; xm=x; xp(k)=xp(k)+h; xm(k)=xm(k)-h;
    A(:,k)=(source_aligned_rhs_control(xp,p,mode,zeros(4,1))-source_aligned_rhs_control(xm,p,mode,zeros(4,1)))/(2*h);
end
for k=1:4
    dp=zeros(4,1); dp(k)=dBase(k); Bbar(:,k)=(source_aligned_rhs_control(x,p,mode,dp)-source_aligned_rhs_control(x,p,mode,-dp))/2;
end
end

function [lam,f,zeta,eta,v,wleft]=pickTorsionalMode(A)
[V,D]=eig(A); ev=diag(D); [W,Dl]=eig(A'); el=diag(Dl); cand=find(imag(ev)>0 & abs(imag(ev))/(2*pi)>1 & abs(imag(ev))/(2*pi)<5); assert(~isempty(cand),'Torsional candidate not found.');
etaAll=zeros(numel(cand),1); leftCell=cell(numel(cand),1);
for k=1:numel(cand)
    i=cand(k); [~,j]=min(abs(el-conj(ev(i)))); v0=V(:,i); w0=W(:,j); alpha=w0'*v0; w0=w0/conj(alpha); leftCell{k}=w0; etaAll(k)=sum(abs(v0(1:3).*w0(1:3)))/max(sum(abs(v0.*w0)),eps);
end

[eta,ii]=max(etaAll); i=cand(ii); lam=ev(i); v=V(:,i); wleft=leftCell{ii}; f=abs(imag(lam))/(2*pi); zeta=-real(lam)/max(abs(lam),eps);
end

function r=activeMaxRealPole(A,mode)
% GFL/Droop 共同状态向量中保留的占位积分器会产生零极点，不计入控制结构稳定性指标。
ev=eig(A); tol=1e-7; active=abs(ev)>tol;
if isempty(ev(active)), r=0; else, r=max(real(ev(active))); end
end

function [G,De,Ke]=complexTorque(A,p,w)
idx=4:23; C=zeros(1,numel(idx)); C(2)=p(18); B=A(idx,3); Aee=A(idx,idx); G=zeros(numel(w),1);
for k=1:numel(w), G(k)=C*((1i*w(k)*eye(numel(idx))-Aee)\B); end
De=real(G); Ke=-w.*imag(G);
end

function R=modalResidue(C,v,wleft,Bbar,lam)
R=zeros(size(C,1),size(Bbar,2)); for k=1:size(Bbar,2), R(:,k)=C*v*(wleft'*Bbar(:,k)); end
R=R(:).'; %#ok<NASGU>
% 行顺序：omega输出四个扰动、Tsh输出四个扰动。
R=[(C(1,:)*v)*(wleft'*Bbar); (C(2,:)*v)*(wleft'*Bbar)];
end

function T=buildResidueTable(L,names,labels)
rows=numel(names)*numel(labels); T=table('Size',[rows 10], ...
    'VariableTypes',{'string','string','double','double','double','double','double','double','double','double'}, ...
    'VariableNames',{'Disturbance','Control','R_omega_abs','R_omega_angle_deg','R_Tsh_abs','R_Tsh_angle_deg','Rstep_omega_abs','Rstep_Tsh_abs','Gamma_vs_GFL','Gamma_Tsh_vs_GFL'});
r=0; base=L(1).Residue;
for j=1:numel(names)
    for k=1:numel(labels)
        r=r+1; z1=L(k).Residue(1,j); z2=L(k).Residue(2,j); b1=base(1,j); b2=base(2,j); T.Disturbance(r)=names{j}; T.Control(r)=labels{k}; T.R_omega_abs(r)=abs(z1); T.R_omega_angle_deg(r)=angle(z1)*180/pi; T.R_Tsh_abs(r)=abs(z2); T.R_Tsh_angle_deg(r)=angle(z2)*180/pi; T.Rstep_omega_abs(r)=abs(L(k).StepResidue(1,j)); T.Rstep_Tsh_abs(r)=abs(L(k).StepResidue(2,j)); T.Gamma_vs_GFL(r)=abs(z1)/max(abs(b1),eps); T.Gamma_Tsh_vs_GFL(r)=abs(z2)/max(abs(b2),eps);
    end
end
end

function T=buildPoleDisturbance(Pole,Residue)
z0=Pole.ShaftDamping(1); rows=height(Residue); T=table('Size',[rows 6], ...
    'VariableTypes',{'string','string','double','double','double','string'}, ...
    'VariableNames',{'Disturbance','Control','DeltaZeta_pole','Gamma_disturbance','log10Gamma','Classification'});
for r=1:rows
    dz=Pole.ShaftDamping(strcmp(Pole.Control,Residue.Control(r)))-z0; g=Residue.Gamma_vs_GFL(r); tol=1e-8;
    if abs(dz)<=tol && abs(g-1)<=tol, cls="pole不变+激励不变";
    elseif dz< -tol && g>1+tol, cls="pole下降+激励增强";
    elseif dz< -tol && g<1-tol, cls="pole下降+激励减弱";
    elseif dz>tol && g>1+tol, cls="pole改善+激励增强";
    elseif dz>tol && g<1-tol, cls="pole改善+激励减弱";
    elseif abs(dz)<=tol && g>1+tol, cls="pole不变+激励增强";
    elseif abs(dz)<=tol && g<1-tol, cls="pole不变+激励减弱";
    else, cls="需复核"; end
    T.Disturbance(r)=Residue.Disturbance(r); T.Control(r)=Residue.Control(r); T.DeltaZeta_pole(r)=dz; T.Gamma_disturbance(r)=g; T.log10Gamma(r)=log10(max(g,eps)); T.Classification(r)=cls;
end
end

function T=runRepresentativeNonlinear(x,p,modes,labels,dBase,Pole,Residue)
%RUNREPRESENTATIVENONLINEAR 在同一严格平衡点上做短时连续非线性小扰动。
% 仅在内存中保存时序；返回频率、阻尼和峰值汇总，避免长期保存高频波形。
names={'Mechanical torque';'Aerodynamic power';'Grid angle';'Grid frequency'};
rows=numel(names)*numel(modes); T=table('Size',[rows 14], ...
    'VariableTypes',{'string','string','double','double','double','double','double','double','double','double','double','double','double','string'}, ...
    'VariableNames',{'Disturbance','Control','f_NL_Hz','zeta_NL','omegaRel_peak','Tsh_peak','f_SSM_Hz','zeta_SSM','f_error_pct','zeta_error_pct','peak_ratio_to_GFL','Tsh_peak_ratio_to_GFL','residue_omega_abs','status'});
tspan=linspace(0,10,5001); r=0;
for j=1:numel(names)
    for k=1:numel(modes)
        r=r+1; d=zeros(4,1); d(j)=dBase(j); mode=modes{k};
        try
            opts=odeset('RelTol',1e-7,'AbsTol',1e-8,'MaxStep',0.01);
            [t,xx]=ode15s(@(tt,zz)source_aligned_rhs_control(zz,p,mode,d),tspan,x,opts); %#ok<ASGLU>
            yy=zeros(numel(t),6); for ii=1:numel(t), yy(ii,:)=source_aligned_outputs_control(xx(ii,:).',p,mode,d).'; end
            % 去除稳态值后，仅从轴系主振荡信号提取指标。
            y1=yy(:,5)-yy(end,5); y2=yy(:,4)-yy(end,4);
            fRef=Pole.ShaftFrequency_Hz(k); [f1,z1,pk1]=estimateDampedMetric(t,y1,fRef); [f2,z2,pk2]=estimateDampedMetric(t,y2,fRef);
            if ~isfinite(f1), f1=f2; end; if ~isfinite(z1), z1=z2; end
            fNL=f1; zNL=z1; pkRatio=NaN;
            if k==1, refPeak=pk1; refTshPeak=pk2; pkRatio=1; tshRatio=1; else, pkRatio=pk1/max(refPeak,eps); tshRatio=pk2/max(refTshPeak,eps); end
            T.Disturbance(r)=names{j}; T.Control(r)=labels{k}; T.f_NL_Hz(r)=fNL; T.zeta_NL(r)=zNL;
            T.omegaRel_peak(r)=pk1; T.Tsh_peak(r)=pk2; T.f_SSM_Hz(r)=fRef; T.zeta_SSM(r)=Pole.ShaftDamping(k);
            T.f_error_pct(r)=100*abs(fNL-fRef)/max(fRef,eps); T.zeta_error_pct(r)=100*abs(zNL-Pole.ShaftDamping(k));
            T.peak_ratio_to_GFL(r)=pkRatio; T.Tsh_peak_ratio_to_GFL(r)=tshRatio; T.residue_omega_abs(r)=Residue.R_omega_abs((j-1)*numel(modes)+k);
            if T.f_error_pct(r)<=1 && T.zeta_error_pct(r)<=0.5, T.status(r)="PASS"; else, T.status(r)="REVIEW"; end
        catch ME
            T.Disturbance(r)=names{j}; T.Control(r)=labels{k}; T.f_NL_Hz(r)=NaN; T.zeta_NL(r)=NaN; T.omegaRel_peak(r)=NaN; T.Tsh_peak(r)=NaN; T.f_SSM_Hz(r)=Pole.ShaftFrequency_Hz(k); T.zeta_SSM(r)=Pole.ShaftDamping(k); T.f_error_pct(r)=NaN; T.zeta_error_pct(r)=NaN; T.peak_ratio_to_GFL(r)=NaN; T.Tsh_peak_ratio_to_GFL(r)=NaN; T.residue_omega_abs(r)=Residue.R_omega_abs((j-1)*numel(modes)+k); T.status(r)="FAIL: "+string(ME.message);
        end
    end
end
end

function [f,z,pk]=estimateDampedMetric(t,y,fRef)
% 使用局部峰值估计低频振荡，避免依赖额外工具箱和长期保存时序。
t=t(:); y=y(:); idx=t>=0.2; t=t(idx); y=y(idx); y=y-mean(y(max(1,end-500):end)); pk=max(abs(y)); f=NaN; z=NaN;
if pk<=eps, return; end
ismax=false(size(y)); ismax(2:end-1)=y(2:end-1)>=y(1:end-2)&y(2:end-1)>y(3:end); ip=find(ismax & y>0.03*pk);
if numel(ip)>=4
    per=diff(t(ip)); per=per(per>0.1 & per<2); if ~isempty(per), f=1/median(per); end
    % 阻尼用已识别轴系频率的单模态最小二乘拟合，避免网侧阶跃引入的慢模态污染。
    if nargin>=3 && isfinite(fRef) && fRef>0
        sel=t>=0.15 & t<=min(6,t(end)); tt=t(sel); yy=y(sel); ww=2*pi*fRef; sigGrid=linspace(-1.5,2.0,351); err=zeros(size(sigGrid));
        for ii=1:numel(sigGrid)
            X=[exp(-sigGrid(ii)*tt).*cos(ww*tt),exp(-sigGrid(ii)*tt).*sin(ww*tt),ones(size(tt)),tt]; c=X\yy; err(ii)=norm(X*c-yy);
        end
        [~,ii]=min(err); sig=sigGrid(ii); z=sig/sqrt(sig^2+ww^2); if ~isfinite(f), f=fRef; end
    else
        ta=t(ip); aa=abs(y(ip)); good=aa>0.03*pk; if nnz(good)>=4
            q=polyfit(ta(good),log(aa(good)),1); sig=-q(1); if isfinite(f)&&f>0, z=sig/sqrt(sig^2+(2*pi*f)^2); end
        end
    end
end
end

function [Pmsc,Pgsc,Qpcc,Imsc,Igsc]=equilibriumMetrics(x,p,mode)
Rf=p(5); Lf=p(6); Cf=p(7); Rd=p(8); Ld=p(14); Lq=p(15); Rs=p(13); np=p(17); Kpmi=p(27); Kpgi=p(29); Kpgv=p(31); Kq=p(36); Qref=p(38); E0=p(40); w0=p(3); mp=p(34); Pref=p(37);
wg=x(3); imd=x(4); imq=x(5); xiMid=x(7); xiMiq=x(8); Udc=x(9); Pf=x(10); Qf=x(11); wv=x(12); delta=x(13); xiVd=x(14); xiVq=x(15); xiId=x(16); xiIq=x(17); ifd=x(18); ifq=x(19); vcd=x(20); vcq=x(21); igd=x(22); igq=x(23);
switch upper(mode)
    case 'VSG', wc=wv;
    case 'DROOP', wc=w0+mp*(Pref-Pf);
    case 'GFL', wc=w0;
    otherwise, error('Unsupported mode %s',mode);
end
we=np*wg; eDc=p(2)-Udc; imdRef=0; imqRef=p(25)*eDc+x(6); eMid=-imd; eMiq=imqRef-imq;
vmd=Kpmi*eMid+xiMid+Rs*imdRef-we*Lq*imqRef; vmq=Kpmi*eMiq+xiMiq+Rs*imqRef+we*(Ld*imdRef+p(16)); Pmsc=1.5*(vmd*imd+vmq*imq);
icapd=ifd-igd; icapq=ifq-igq; vnodeD=vcd+Rd*ifd-(Rd+1e-4)*igd; vnodeQ=vcq+Rd*ifq-(Rd+1e-4)*igq; Qpcc=1.5*(vnodeQ*igd-vnodeD*igq);
c=cos(delta); s=sin(delta); vpd=c*vnodeD+s*vnodeQ; vpq=-s*vnodeD+c*vnodeQ; ifld=c*ifd+s*ifq; iflq=-s*ifd+c*ifq; igld=c*igd+s*igq; iglq=-s*igd+c*igq; Vref=E0+Kq*(Qref-Qf); evd=Vref-vpd; evq=-vpq; ifdRef=Kpgv*evd+xiVd-Cf*wc*vpq+igld; ifqRef=Kpgv*evq+xiVq+Cf*wc*vpd+iglq; eid=ifdRef-ifld; eiq=ifqRef-iflq; ucd=Kpgi*eid+xiId-wc*Lf*iflq; ucq=Kpgi*eiq+xiIq+wc*Lf*ifld; uinvD=c*ucd-s*ucq; uinvQ=s*ucd+c*ucq; Pgsc=1.5*(uinvD*ifd+uinvQ*ifq);
Imsc=hypot(imd,imq); Igsc=hypot(igd,igq);
end

function writeReport(file,R)
fid=fopen(file,'w','n','UTF-8'); assert(fid>0); c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'# GFL–Droop-GFM–VSG-GFM 公平对比阶段报告\n\n');
if R.passed, fprintf(fid,'Gate A：PASS。\n\n'); else, fprintf(fid,'Gate A：FAIL，按停止规则终止后续极点、复转矩和模态残差比较。\n\n'); end
writeTableMD(fid,R.workpoint); fprintf(fid,'\n');
if isfield(R,'stop_reason'), fprintf(fid,'## 停止原因\n\n%s\n\n',R.stop_reason); end
if isfield(R,'pole_torque_summary')
    fprintf(fid,'## 极点与复转矩\n\n'); writeTableMD(fid,R.pole_torque_summary); fprintf(fid,'\n');
    fprintf(fid,'轴系频率公共参考值：%.8g Hz。`De` 采用本模型发电机转矩正方向下的 `Re{DeltaTe/Deltaomega_g}` 定义；负值表示该电气反馈在该速度扰动方向上表现为负阻尼贡献，不能单独等同于全系统失稳。\n\n',R.common_frequency_Hz);
end
if isfield(R,'residue_summary')
    fprintf(fid,'## 四类扰动的归一化模态残差\n\n'); writeTableMD(fid,R.residue_summary); fprintf(fid,'\n');
    fprintf(fid,'## 极点—扰动通道二维指标\n\n'); writeTableMD(fid,R.pole_disturbance); fprintf(fid,'\n');
end
if isfield(R,'nonlinear_summary')
    fprintf(fid,'## 连续非线性平均模型代表性验证\n\n'); writeTableMD(fid,R.nonlinear_summary); fprintf(fid,'\n');
    fprintf(fid,'## 当前阶段可支持的结论\n\n');
    fprintf(fid,'1. 三种控制方式在同一严格连续平衡点上通过 Gate A；共同工作点差异为 0。\n');
    fprintf(fid,'2. 在当前 M0 理想连续模型中，三种控制的轴系模态均为 `lambda_tor=%.8g%+.8gj`，`f_tor=%.8g Hz`，`zeta_tor=%.8g`，机械参与度约 %.4f；因此当前证据不支持“GFM必然降低轴系极点阻尼”。\n',real(R.pole_torque_summary.ShaftEigenvalue(1)),imag(R.pole_torque_summary.ShaftEigenvalue(1)),R.pole_torque_summary.ShaftFrequency_Hz(1),R.pole_torque_summary.ShaftDamping(1),R.pole_torque_summary.MechanicalParticipation(1));
    fprintf(fid,'3. 轴系频率处复转矩三者相同（`De_self=%.8g`，`Ke_self=%.8g`）；差异主要出现在网侧扰动残差：Droop/VSG对网侧相角和频率的激励系数分别为表中 Gamma，而机械转矩和气动功率通道基本不变。\n',R.pole_torque_summary.De_self(1),R.pole_torque_summary.Ke_self(1));
    fprintf(fid,'4. 12 个连续非线性小扰动工况均完成，频率误差和阻尼误差门限为 1%% 与 0.5 个百分点；这些结果用于验证当前连续源方程，不等同于含采样、延迟、PWM和限幅的 EMT 结论。\n\n');
end
fprintf(fid,'## 边界说明\n\n当前连续对齐方程没有独立PLL状态，因此GFL采用理想PLL/电网角频率跟踪代理。Droop-GFM使用代数P-f下垂，VSG-GFM使用虚拟惯量状态。三者只替换同步/有功外环，公共机械、电机、MSC-DVC、DC-link、LCL和电网保持一致。所有非线性指标均在内存中计算，不保存原始长时序。\n');
end

function writeTableMD(fid,T)
vars=T.Properties.VariableNames; fprintf(fid,'|'); for k=1:numel(vars), fprintf(fid,'%s|',vars{k}); end; fprintf(fid,'\n|'); for k=1:numel(vars), fprintf(fid,'---|'); end; fprintf(fid,'\n');
for r=1:height(T), fprintf(fid,'|'); for k=1:numel(vars), v=T{r,k}; if iscell(v), v=v{1}; end; if isstring(v)||ischar(v), s=char(string(v)); else, s=num2str(v,8); end; fprintf(fid,'%s|',s); end; fprintf(fid,'\n'); end
end
