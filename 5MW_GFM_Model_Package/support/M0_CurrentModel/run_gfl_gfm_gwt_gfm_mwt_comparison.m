function R=run_gfl_gfm_gwt_gfm_mwt_comparison(varargin)
%RUN_GFL_GFM_GWT_GFM_MWT_COMPARISON
% 在同一个5 MW连续平均M0状态方程中比较：
%   GFL       : 理想PLL/电网角频率跟踪；MSC-DVC；
%   GFM-GWT   : GSC-DVC；MSC采用工作点固定MPPT/定转矩电流；
%   GFM-MWT   : MSC-DVC；GSC采用VSG。
%
% 不复制SLX、不恢复PWM/离散/延迟/限幅。所有时序仅在内存中使用。

ip=inputParser;
ip.addParameter('SaveResults',true,@(v)islogical(v)&&isscalar(v));
ip.addParameter('RunNonlinear',true,@(v)islogical(v)&&isscalar(v));
ip.addParameter('StepAngleDeg',0.2,@(v)isnumeric(v)&&isscalar(v));
ip.addParameter('StopTime',10,@(v)isnumeric(v)&&isscalar(v)&&v>1);
ip.addParameter('KpGscDvc',5e3,@(v)isnumeric(v)&&isscalar(v)&&v>0);
ip.addParameter('KiGscDvc',5e2,@(v)isnumeric(v)&&isscalar(v)&&v>=0);
ip.addParameter('MpGwt',5e-7,@(v)isnumeric(v)&&isscalar(v)&&v>0);
ip.parse(varargin{:}); opt=ip.Results;

here=fileparts(mfilename('fullpath')); addpath(here,fileparts(here));
q=load(fullfile(here,'03_Mechanism_Evidence_Summary.mat'),'E'); E=q.E; O=E.operating_point;
P=init_m0_5mw_parameters();
OP=struct('Tm0_Nm',O.Tgen_Nm,'E0_peak_V',abs(O.vnode_grid_dq_V(1)+1i*O.vnode_grid_dq_V(2)));
[p,~]=m0_pack_parameters(P,OP); xseed=stateVector(O);

modes={'GFL','GFMGWT','VSG'};
labels=[ "GFL"; "GFM-GWT (GSC-DVC + MSC-MPPT/转矩)"; "GFM-MWT (MSC-DVC + GSC-VSG)" ];
flags={struct,struct('imqRef0',O.pmsg_iq0,'KpGscDvc',opt.KpGscDvc,'KiGscDvc',opt.KiGscDvc,'mpGwt',opt.MpGwt),struct};
n=numel(xseed); nm=numel(modes);
sx=[1;1;1;1e4;1e4;1e4;1e4;1e4;1e3;5e6;5e6;1;1;1e4;1e4;1e4;1e4;1e4;1e4;1e3;1e3;1e4;1e4];
sr=[1;1;1;1e4;1e4;1e4;1e4;1e4;1e6;5e8;5e8;1;1;1e4;1e4;1e4;1e4;1e6;1e6;1e6;1e6;1e6;1e6];

WP=table('Size',[nm 14],...
    'VariableTypes',{'string','double','double','double','double','double','double','double','double','double','double','double','double','logical'},...
    'VariableNames',{'Architecture','P_PCC_MW','Q_PCC_Mvar','Udc_V','omega_g_radps','Te_MNm','Tsh_MNm','P_MSC_MW','P_GSC_MW','P_Mismatch_W','Torque_Mismatch_Nm','Residual_norm','Max_abs_dx','GateA'});
PO=table('Size',[nm 13],...
    'VariableTypes',{'string','double','double','double','double','double','double','double','double','double','double','double','double'},...
    'VariableNames',{'Architecture','ShaftEigenvalueReal','ShaftEigenvalueImag','f_tor_Hz','zeta_tor','MechanicalParticipation','MaxRealPole','GTeOmegaReal','GTeOmegaImag','Ke_at_ftor','TeThetaGain_at_ftor','TeFreqGain_at_ftor','dP_dDelta_W_per_rad'});
DR=table('Size',[nm 10],...
    'VariableTypes',{'string','double','double','double','double','double','double','double','double','string'},...
    'VariableNames',{'Architecture','GridAngle_deg','omegaRelPeak','TshPeak','TePeak','PpccPeak_MW','f_est_Hz','zeta_est','peak_ratio_to_GFL','Status'});

X=zeros(n,nm); Aall=cell(nm,1); Ball=cell(nm,1); responses=cell(nm,1); eqInfo=cell(nm,1);
[xVsgEq,eqVsg]=solveMode(xseed,p,'VSG',flags{3},sx,sr);
for k=1:nm
    x0=xseed;
    if strcmp(modes{k},'GFMGWT'), x0(6)=0; x0(12)=p(3); x0(13)=xseed(13); end
    if strcmp(modes{k},'GFL')
        % GFL的理想PLL代理没有独立PLL状态；采用同一物理工作点，
        % 仅替换同步环，避免把未定义的占位状态当成新的平衡方程。
        x=xVsgEq; eq=eqVsg;
    elseif strcmp(modes{k},'VSG')
        x=xVsgEq; eq=eqVsg;
    else
        [x,eq]=solveMode(x0,p,modes{k},flags{k},sx,sr);
    end
    X(:,k)=x; eqInfo{k}=eq;
    [Pmsc,Pgsc,Qpcc]=equilibriumMetrics(x,p,modes{k},flags{k});
    y=source_aligned_outputs_control(x,p,modes{k},zeros(4,1),flags{k});
    dx=source_aligned_rhs_control(x,p,modes{k},zeros(4,1),flags{k});
    WP(k,:)={labels(k),y(1)/1e6,Qpcc/1e6,y(2),x(3),y(3)/1e6,y(4)/1e6,Pmsc/1e6,Pgsc/1e6,Pmsc-Pgsc,y(3)-y(4),norm(dx,inf),max(abs(dx)),norm(dx,inf)<=1e-7};
    [A,Bbar]=linearizeMode(x,p,modes{k},flags{k}); Aall{k}=A; Ball{k}=Bbar;
    [lam,ft,zeta,eta]=pickTorsionalMode(A); [G,~,~]=complexTorque(A,p,ft); Gtheta=G*1i*2*pi*ft;
    dPdd=angleSensitivity(x,p,modes{k},flags{k});
    PO(k,:)={labels(k),real(lam),imag(lam),ft,zeta,eta,max(real(eig(A))),real(G),imag(G),-ft*2*pi*imag(G),real(Gtheta),abs(G),dPdd};
    if opt.RunNonlinear
        responses{k}=runAngleStep(x,p,modes{k},flags{k},opt.StepAngleDeg,opt.StopTime,PO.f_tor_Hz(k));
        DR(k,1:8)={labels(k),opt.StepAngleDeg,responses{k}.omegaPeak,responses{k}.tshPeak,responses{k}.tePeak,responses{k}.pPeak/1e6,responses{k}.fEst,responses{k}.zetaEst};
        if k==1, refPeak=responses{k}.omegaPeak; end
        DR.peak_ratio_to_GFL(k)=responses{k}.omegaPeak/max(refPeak,eps);
        if responses{k}.status=="PASS", DR.Status(k)="PASS"; else, DR.Status(k)=responses{k}.status; end
    else
        DR(k,1:8)={labels(k),opt.StepAngleDeg,NaN,NaN,NaN,NaN,NaN,NaN}; DR.peak_ratio_to_GFL(k)=NaN; DR.Status(k)="SKIPPED";
    end
end

R=struct('objective','GFL-GFM-GWT-GFM-MWT公平对比','passed',all(WP.GateA),'workpoint',WP,'pole_summary',PO,'disturbance_summary',DR,'states',X,'equilibrium',{eqInfo},'responses',{responses},'models',{modes},'labels',{labels},'parameter_vector',p);
figDir=fullfile(here,'Figures_Disturbance_Path'); if ~exist(figDir,'dir'), mkdir(figDir); end
makeFigures(PO,DR,responses,labels,figDir);
if opt.SaveResults
    writetable(WP,fullfile(here,'Architecture_Workpoint_Summary.csv'));
    writetable(PO,fullfile(here,'Architecture_Pole_Torque_Summary.csv'));
    writetable(DR,fullfile(here,'Architecture_Disturbance_Response_Summary.csv'));
    save(fullfile(here,'Architecture_Comparison_Summary.mat'),'R','WP','PO','DR','-v7');
    writeReport(fullfile(here,'GFL_GFMGWT_GFMMWT_Architecture_Report_CN.md'),R);
end
disp(WP); disp(PO); disp(DR);
end

function x=stateVector(O)
c=O.controller_x0; x=[O.theta_tw0;O.omega0;O.omega0;O.pmsg_id0;O.pmsg_iq0;c(1:3);O.pvec(2);c(4:11);O.if_grid_dq_A;O.vcap_grid_dq_V;O.ig_grid_dq_A];
end

function [x,eq]=solveMode(x0,p,mode,flags,sx,sr)
opts=optimoptions('fsolve','Display','off','Algorithm','levenberg-marquardt','FunctionTolerance',1e-13,'StepTolerance',1e-13,'OptimalityTolerance',1e-13,'MaxIterations',3000,'MaxFunctionEvaluations',50000);
if strcmp(mode,'GFMGWT')
    % xiDc和wv在GFM-GWT中不是独立动态：平衡点要求Udc=Vdc0、xiDc=0，
    % wv为公共占位状态。去掉这两个恒等方程后求解其余物理平衡方程，
    % 避免由占位积分器造成的Jacobian秩亏。
    active=[1:6 7:11 13:23]; residualIdx=[1:6 7:12 13:23]; xwork=x0; xwork(12)=p(3);
    [z,fval,exitflag]=fsolve(@(z)reducedResidual(z,xwork,active,residualIdx,p,mode,flags,sx,sr),xwork(active)./sx(active),opts);
    x=xwork; x(active)=sx(active).*z;
else
    [z,fval,exitflag]=fsolve(@(z)source_aligned_rhs_control(sx.*z,p,mode,zeros(4,1),flags)./sr,x0./sx,opts);
    x=sx.*z;
end
dx=source_aligned_rhs_control(x,p,mode,zeros(4,1),flags);
eq=struct('exitflag',exitflag,'residual_norm',norm(fval,inf),'max_abs_dx',max(abs(dx)));
assert(exitflag>0 && eq.residual_norm<1e-8,'Equilibrium solve failed for %s: residual %.4g',mode,eq.residual_norm);
end

function r=reducedResidual(z,x,active,residualIdx,p,mode,flags,sx,sr)
x(active)=sx(active).*z; dx=source_aligned_rhs_control(x,p,mode,zeros(4,1),flags); r=dx(residualIdx)./sr(residualIdx);
end

function [A,Bbar]=linearizeMode(x,p,mode,flags)
n=numel(x); A=zeros(n); Bbar=zeros(n,4);
for k=1:n
    h=1e-6*max(abs(x(k)),1); xp=x; xm=x; xp(k)=xp(k)+h; xm(k)=xm(k)-h;
    A(:,k)=(source_aligned_rhs_control(xp,p,mode,zeros(4,1),flags)-source_aligned_rhs_control(xm,p,mode,zeros(4,1),flags))/(2*h);
end
dScale=[0.005*2.573e6;0.005*p(1);deg2rad(0.2);2*pi*0.05];
for k=1:4
    d=zeros(4,1); d(k)=dScale(k);
    Bbar(:,k)=(source_aligned_rhs_control(x,p,mode,d,flags)-source_aligned_rhs_control(x,p,mode,-d,flags))/2;
end
end

function [lam,f,zeta,eta]=pickTorsionalMode(A)
[V,D]=eig(A); ev=diag(D); [W,Dl]=eig(A'); el=diag(Dl); cand=find(imag(ev)>0 & abs(imag(ev))/(2*pi)>1 & abs(imag(ev))/(2*pi)<5); assert(~isempty(cand),'No torsional pole found.');
etaAll=zeros(numel(cand),1); left=cell(numel(cand),1);
for k=1:numel(cand)
    i=cand(k); [~,j]=min(abs(el-conj(ev(i)))); v=V(:,i); w=W(:,j); a=w'*v; w=w/conj(a); left{k}=w; etaAll(k)=sum(abs(v(1:3).*w(1:3)))/max(sum(abs(v.*w)),eps);
end
[eta,j]=max(etaAll); i=cand(j); lam=ev(i); f=abs(imag(lam))/(2*pi); zeta=-real(lam)/max(abs(lam),eps);
end

function [G,De,Ke]=complexTorque(A,p,ft)
idx=4:23; C=zeros(1,numel(idx)); C(2)=p(18); B=A(idx,3); Ae=A(idx,idx); w=2*pi*ft; G=C*((1i*w*eye(numel(idx))-Ae)\B); De=real(G); Ke=-w*imag(G);
end

function kdelta=angleSensitivity(x,p,mode,flags)
h=1e-6; xp=x; xm=x; xp(13)=xp(13)+h; xm(13)=xm(13)-h; [~,pgp,~]=equilibriumMetrics(xp,p,mode,flags); [~,pgm,~]=equilibriumMetrics(xm,p,mode,flags); kdelta=(pgp-pgm)/(2*h);
end

function [Pmsc,Pgsc,Qpcc]=equilibriumMetrics(x,p,mode,flags)
Vdc0=p(2); w0=p(3); Rf=p(5); Lf=p(6); Cf=p(7); Rd=p(8); Rs=p(13); Ld=p(14); Lq=p(15); psi=p(16); np=p(17); Kpdc=p(25); Kpmi=p(27); Kpgi=p(29); Kpgv=p(31); Kq=p(36); Qref=p(38); E0=p(40); ffIg=p(42); ffVpcc=p(43); mp=p(34); Pref=p(37);
imd=x(4); imq=x(5); xiDc=x(6); Udc=x(9); Pf=x(10); Qf=x(11); wv=x(12); delta=x(13); xiVd=x(14); xiVq=x(15); xiId=x(16); xiIq=x(17); ifd=x(18); ifq=x(19); vcd=x(20); vcq=x(21); igd=x(22); igq=x(23); wg=x(3);
if strcmp(mode,'GFMGWT')
    wctrl=wv; imqRef=flags.imqRef0;
elseif strcmp(mode,'VSG')
    wctrl=wv; imqRef=Kpdc*(Vdc0-Udc)+xiDc;
elseif strcmp(mode,'GFL')
    wctrl=w0; imqRef=Kpdc*(Vdc0-Udc)+xiDc;
else
    wctrl=w0+mp*(Pref-Pf); imqRef=Kpdc*(Vdc0-Udc)+xiDc;
end
we=np*wg; imdRef=0; vmd=Kpmi*(-imd)+x(7)+Rs*imdRef-we*Lq*imqRef; vmq=Kpmi*(imqRef-imq)+x(8)+Rs*imqRef+we*(Ld*imdRef+psi); Pmsc=1.5*(vmd*imd+vmq*imq);
vnodeD=vcd+Rd*ifd-(Rd+1e-4)*igd; vnodeQ=vcq+Rd*ifq-(Rd+1e-4)*igq; Qpcc=1.5*(vnodeQ*igd-vnodeD*igq);
c=cos(delta); s=sin(delta); vpd=c*vnodeD+s*vnodeQ; vpq=-s*vnodeD+c*vnodeQ; ifld=c*ifd+s*ifq; iflq=-s*ifd+c*ifq; igld=c*igd+s*igq; iglq=-s*igd+c*igq; Vref=E0+Kq*(Qref-Qf); ifdRef=Kpgv*(Vref-vpd)+xiVd-Cf*wctrl*vpq+ffIg*igld; ifqRef=Kpgv*(-vpq)+xiVq+Cf*wctrl*vpd+ffIg*iglq; ucd=Kpgi*(ifdRef-ifld)+xiId-wctrl*Lf*iflq+ffVpcc*(vpd+Rf*ifld); ucq=Kpgi*(ifqRef-iflq)+xiIq+wctrl*Lf*ifld+ffVpcc*(vpq+Rf*iflq); uinvD=c*ucd-s*ucq; uinvQ=s*ucd+c*ucq; Pgsc=1.5*(uinvD*ifd+uinvQ*ifq);
end

function out=runAngleStep(x,p,mode,flags,angleDeg,stopTime,fRef)
ang=deg2rad(angleDeg); t1=0.1; opts=odeset('RelTol',1e-7,'AbsTol',1e-8,'MaxStep',0.01);
[ta,xa]=ode15s(@(t,z)source_aligned_rhs_control(z,p,mode,zeros(4,1),flags),[0 t1],x,opts);
d=[0;0;ang;0]; [tb,xb]=ode15s(@(t,z)source_aligned_rhs_control(z,p,mode,d,flags),[t1 stopTime],xa(end,:).',opts);
t=[ta;tb(2:end)]; xx=[xa;xb(2:end,:)]; yy=zeros(numel(t),6);
for i=1:numel(t), yy(i,:)=source_aligned_outputs_control(xx(i,:).',p,mode,[0;0;(t(i)>=t1)*ang;0],flags).'; end
y=yy(:,5)-yy(end,5); ts=t(t>=t1); ys=y(t>=t1); tsh=yy(t>=t1,4)-yy(end,4); te=yy(t>=t1,3)-yy(end,3); pp=yy(t>=t1,1)-yy(end,1);
[fEst,zEst,pk]=estimateDampedMetric(ts,ys,fRef); out=struct('t',t,'omega_rel',yy(:,5),'Tsh',yy(:,4),'Te',yy(:,3),'Ppcc',yy(:,1),'omegaPeak',pk,'tshPeak',max(abs(tsh)),'tePeak',max(abs(te)),'pPeak',max(abs(pp)),'fEst',fEst,'zetaEst',zEst,'status',"PASS"); if pk<1e-8, out.fEst=fRef; out.zetaEst=NaN; out.status="NO_TORSIONAL_EXCITATION"; elseif ~isfinite(fEst), out.status="REVIEW"; end
end

function [f,z,pk]=estimateDampedMetric(t,y,fRef)
t=t(:); y=y(:); y=y-mean(y(max(1,end-500):end)); pk=max(abs(y)); f=NaN; z=NaN; if pk<=eps, return; end
ismax=false(size(y)); ismax(2:end-1)=y(2:end-1)>=y(1:end-2)&y(2:end-1)>y(3:end); ip=find(ismax & y>0.03*pk);
if numel(ip)>=4
    per=diff(t(ip)); per=per(per>0.1 & per<2); if ~isempty(per), f=1/median(per); end
    sel=t>=0.15 & t<=min(6,t(end)); tt=t(sel); yy=y(sel); ww=2*pi*fRef; sigGrid=linspace(-1.5,2,351); err=zeros(size(sigGrid));
    for j=1:numel(sigGrid), X=[exp(-sigGrid(j)*tt).*cos(ww*tt),exp(-sigGrid(j)*tt).*sin(ww*tt),ones(size(tt)),tt]; c=X\yy; err(j)=norm(X*c-yy); end
    [~,j]=min(err); sig=sigGrid(j); z=sig/sqrt(sig^2+ww^2); if ~isfinite(f), f=fRef; end
end
end

function makeFigures(PO,DR,responses,labels,figDir)
f=figure('Visible','off','Color','w'); bar(PO.zeta_tor*100); set(gca,'XTick',1:height(PO),'XTickLabel',{'GFL','GFM-GWT','GFM-MWT'}); ylabel('\zeta_{tor} (%)'); title('三种架构轴系模态阻尼比'); grid on; saveas(f,fullfile(figDir,'Fig09_Architecture_Torsional_Damping.png')); print(f,fullfile(figDir,'Fig09_Architecture_Torsional_Damping.pdf'),'-dpdf'); close(f);
if all(~cellfun(@isempty,responses))
    f=figure('Visible','off','Color','w'); hold on; cc=lines(3); for k=1:3, plot(responses{k}.t,responses{k}.omega_rel,'Color',cc(k,:),'LineWidth',1.2); end; xline(0.1,'k:'); xlabel('t (s)'); ylabel('\Delta\omega_{sh} (rad/s)'); title('0.2°网侧相角阶跃下轴系响应'); legend(cellstr(labels),'Location','best'); grid on; saveas(f,fullfile(figDir,'Fig10_Architecture_GridAngle_Response.png')); print(f,fullfile(figDir,'Fig10_Architecture_GridAngle_Response.pdf'),'-dpdf'); close(f);
end
end

function writeReport(file,R)
fid=fopen(file,'w','n','UTF-8'); assert(fid>0); c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'# GFL–GFM-GWT–GFM-MWT 公平对比报告\n\n');
fprintf(fid,'## 目标与结构\n\n本阶段仅使用同一个5 MW理想连续平均M0状态方程，不恢复PWM、离散、数字延迟、限幅或EMT。三种架构保持机械、PMSG、MSC/GSC内环、DC-link、LCL和电网参数一致，仅改变同步与有功控制分配。\n\n');
fprintf(fid,'## Gate A 工作点\n\n'); writeTableMD(fid,R.workpoint); fprintf(fid,'\n## 轴系极点与复转矩\n\n'); writeTableMD(fid,R.pole_summary); fprintf(fid,'\n## 网侧相角小扰动\n\n'); writeTableMD(fid,R.disturbance_summary);
fprintf(fid,'\n## 文献对应关系\n\nLiu等人的对照将GFM-GWT（GSC-DVC、MSC-MPPT/转矩）与GFM-MWT（MSC-DVC、GSC-GFM）区分开来；本报告正是对这两个架构在同一连续M0中的复现层。若两者轴系阻尼不同，应归因于DVC位置和DC-link能量通道，而不能只归因于“GFM”三个字。\n\n');
fprintf(fid,'## 当前结论\n\n工作点Gate A=%d。GFM-GWT与GFM-MWT的极点、轴系频率和电气转矩反馈已被分开记录；最终结论以表格和图10的网侧扰动响应为准。当前结果属于理想连续模型证据，不等同于开关EMT结论。\n',R.passed);
end

function writeTableMD(fid,T)
v=T.Properties.VariableNames; fprintf(fid,'|'); for j=1:numel(v), fprintf(fid,'%s|',v{j}); end; fprintf(fid,'\n|'); for j=1:numel(v), fprintf(fid,'---|'); end; fprintf(fid,'\n');
for i=1:height(T), fprintf(fid,'|'); for j=1:numel(v), a=T{i,j}; if iscell(a), a=a{1}; end; if ismissing(a), s=''; else, s=char(string(a)); end; fprintf(fid,'%s|',s); end; fprintf(fid,'\n'); end
end
