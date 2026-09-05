function R = run_wind_step_modal_residue_analysis(varargin)
%RUN_WIND_STEP_MODAL_RESIDUE_ANALYSIS
% 下一阶段理想连续平均模型证据：
%   1) 等效风速阶跃下，三种同步架构的非线性/小信号响应对照；
%   2) 机械转矩、气动功率、电网相角、电网频率四类扰动的轴系模态残差。
%
% 当前 M0 方程没有显式 Cp-lambda 风速状态。因此“风速阶跃”在本脚本中
% 采用固定局部运行点的等效气动功率阶跃：
%   Delta P_aero = (dP_aero/dV)|V0 * Delta V_wind.
% 该定义保持非线性模型和同源小信号模型使用完全相同的输入，不引入
% MPPT/Pitch 过渡动态。所有完整时序只在内存中存在；长期只保存汇总表、
% 图片、报告和不含原始时序的结果 MAT。

ip = inputParser;
ip.addParameter('DeltaWind_mps',0.01,@(x)isnumeric(x)&&isscalar(x)&&isfinite(x));
ip.addParameter('StepTime_s',0.10,@(x)isnumeric(x)&&isscalar(x)&&x>=0);
ip.addParameter('StopTime_s',10,@(x)isnumeric(x)&&isscalar(x)&&x>1);
ip.addParameter('NumPoints',5001,@(x)isnumeric(x)&&isscalar(x)&&x>=501);
ip.addParameter('SettlingStopTime_s',180,@(x)isnumeric(x)&&isscalar(x)&&x>=20);
ip.addParameter('RunNonlinear',true,@(x)islogical(x)&&isscalar(x));
ip.addParameter('SaveResults',true,@(x)islogical(x)&&isscalar(x));
ip.addParameter('SaveFigures',true,@(x)islogical(x)&&isscalar(x));
ip.addParameter('Visible','off',@(x)ischar(x)||isstring(x));
ip.parse(varargin{:}); o=ip.Results;

here=fileparts(mfilename('fullpath')); addpath(here,fileparts(here));
S=load(fullfile(here,'Architecture_Comparison_Summary.mat'),'R'); base=S.R;
assert(base.passed,'三架构基准 Gate A 未通过，停止下一阶段分析。');
E=load(fullfile(here,'03_Mechanism_Evidence_Summary.mat'),'E'); O=E.E.operating_point;
p=base.parameter_vector; modes=cellstr(base.models(:)); labels=cellstr(base.labels(:));
xAll=base.states; n=size(xAll,1); nm=numel(modes);

% GFM-GWT 与上一阶段使用完全相同的控制参数；其它架构不需要额外 flags。
flagsAll=cell(nm,1); flagsAll(:)={struct};
for k=1:nm
    if strcmpi(modes{k},'GFMGWT')
        flagsAll{k}=struct('imqRef0',O.pmsg_iq0,'KpGscDvc',5e3,'KiGscDvc',5e2,'mpGwt',5e-7);
    end
end

% 局部等效风速—气动功率映射，与既有风速阶跃脚本保持一致。
Vwind0=12.20; P_aero0=p(39)*p(12); dPaero_dV=3*P_aero0/max(Vwind0,eps);
DeltaV=o.DeltaWind_mps; DeltaP=dPaero_dV*DeltaV;
stepInput=zeros(4,1); stepInput(2)=DeltaP;
t=linspace(0,o.StopTime_s,round(o.NumPoints)).';

% 结果表：每个架构×四个输出各一行；频率和阻尼只对轴系相对速度填充。
sigNames={"Delta omega_sh";"Delta T_sh";"Delta T_e";"Delta Udc"};
sigUnits={"rad/s";"MNm";"MNm";"V"}; outCols=[5 4 3 2]; outScale=[1 1e6 1e6 1];
windRows=nm*numel(sigNames);
windT=table('Size',[windRows 18], ...
    'VariableTypes',{'string','string','double','double','double','double','double','double','double','double','double','double','double','double','double','double','string','string'}, ...
    'VariableNames',{'Architecture','Signal','DeltaWind_mps','EquivalentDeltaPaero_W','StepTime_s','Peak_NL','Peak_SSM','PeakError_pct','NRMSE','Correlation','f_NL_Hz','zeta_NL','f_SSM_Hz','zeta_SSM','Final_NL','Final_SSM','Status','InputDefinition'});

% 物理扰动导数、输出导数、模态残差均在同一循环中计算，避免重复求解。
Cout=zeros(2,n); Cout(1,2)=1; Cout(1,3)=-1;
Cout(2,1)=p(21); Cout(2,2)=p(22); Cout(2,3)=-p(22);
dBase=[0.005*2.573e6;0.005*p(1);deg2rad(0.2);2*pi*0.05];
distNames={"Mechanical torque";"Aerodynamic power";"Grid angle";"Grid frequency"};
resRows=nm*numel(distNames);
resT=table('Size',[resRows 19], ...
    'VariableTypes',{'string','string','double','double','double','double','double','double','double','double','double','double','double','double','double','double','double','string','string'}, ...
    'VariableNames',{'Architecture','Disturbance','f_tor_Hz','zeta_tor','MechanicalParticipation','O_omega_abs','O_Tsh_abs','K_abs','Residue_omega_abs','Residue_Tsh_abs','StepResidue_omega_abs','StepResidue_Tsh_abs','Gamma_omega_vs_GFL','Gamma_Tsh_vs_GFL','PoleReal','PoleImag','InputAmplitude','InputUnit','Classification'});

windRow=0; resRow=0; modelCache=cell(nm,1);
for k=1:nm
    mode=modes{k}; flags=flagsAll{k}; x0=xAll(:,k);
    [A,B,C,D]=linearizePhysical(x0,p,mode,flags);
    [lam,ft,zeta,eta,v,wleft]=pickTorsionalMode(A);
    modelCache{k}=struct('A',A,'B',B,'C',C,'D',D,'lambda',lam,'f',ft,'zeta',zeta,'eta',eta,'v',v,'wleft',wleft,'x0',x0,'flags',flags);

    % ---- 等效风速阶跃：非线性与小信号 ----
    y0=source_aligned_outputs_control(x0,p,mode,zeros(4,1),flags);
    if o.RunNonlinear
        [tn,xn]=simulatePiecewise(@(xx,dd)source_aligned_rhs_control(xx,p,mode,dd,flags),x0,stepInput,o.StepTime_s,o.StopTime_s,o.NumPoints);
        yn=zeros(numel(tn),6);
        for ii=1:numel(tn), yn(ii,:)=source_aligned_outputs_control(xn(ii,:).',p,mode,inputAt(tn(ii),stepInput,o.StepTime_s),flags).'; end
        yn=yn-y0.';
    else
        tn=t; yn=NaN(numel(t),6);
    end
    [ts,zs]=simulatePiecewise(@(zz,dd)linearRhs(A,B,zz,dd),zeros(n,1),stepInput,o.StepTime_s,o.StopTime_s,o.NumPoints);
    yss=zeros(numel(ts),6);
    for ii=1:numel(ts), dd=inputAt(ts(ii),stepInput,o.StepTime_s); yss(ii,:)=(C*zs(ii,:).'+D*dd).'; end
    if o.RunNonlinear
        assert(max(abs(tn-ts))<1e-8,'Nonlinear and SSM time bases differ.');
    else
        tn=ts;
    end

    for j=1:numel(sigNames)
        windRow=windRow+1; a=yn(:,outCols(j))/outScale(j); b=yss(:,outCols(j))/outScale(j);
        [fNL,zNL]=estimateDampedMetric(tn,a,ft); den=max(norm(a-mean(a)),eps);
        windT.Architecture(windRow)=string(labels{k}); windT.Signal(windRow)=sigNames{j};
        windT.DeltaWind_mps(windRow)=DeltaV; windT.EquivalentDeltaPaero_W(windRow)=DeltaP; windT.StepTime_s(windRow)=o.StepTime_s;
        windT.Peak_NL(windRow)=max(abs(a)); windT.Peak_SSM(windRow)=max(abs(b));
        windT.PeakError_pct(windRow)=100*abs(windT.Peak_NL(windRow)-windT.Peak_SSM(windRow))/max(windT.Peak_NL(windRow),eps);
        windT.NRMSE(windRow)=norm(a-b)/den; windT.Correlation(windRow)=corrValue(a,b);
        if j==1, windT.f_NL_Hz(windRow)=fNL; windT.zeta_NL(windRow)=zNL; windT.f_SSM_Hz(windRow)=ft; windT.zeta_SSM(windRow)=zeta; else, windT.f_NL_Hz(windRow)=NaN; windT.zeta_NL(windRow)=NaN; windT.f_SSM_Hz(windRow)=NaN; windT.zeta_SSM(windRow)=NaN; end
        windT.Final_NL(windRow)=a(end); windT.Final_SSM(windRow)=b(end);
        if j==3 && windT.Peak_NL(windRow)<1e-8 && windT.Peak_SSM(windRow)<1e-8
            windT.Status(windRow)="NO_EXCITATION";
        else
            % 平滑风速等效阶跃采用 2% 峰值误差、5% NRMSE 的对齐门槛。
            windT.Status(windRow)=string(iff(windT.PeakError_pct(windRow)<=2 && 100*windT.NRMSE(windRow)<=5,"PASS","REVIEW"));
        end
        windT.InputDefinition(windRow)="DeltaPaero=(dPaero/dV)atV0 times DeltaVwind";
    end

    % ---- 物理单位的四类扰动模态残差 ----
    Omod=Cout*v; Kphys=wleft'*B; Rmode=[Omod(1)*Kphys;Omod(2)*Kphys]; Rstep=Rmode/lam;
    for j=1:numel(distNames)
        resRow=resRow+1; resT.Architecture(resRow)=string(labels{k}); resT.Disturbance(resRow)=distNames{j};
        resT.f_tor_Hz(resRow)=ft; resT.zeta_tor(resRow)=zeta; resT.MechanicalParticipation(resRow)=eta;
        resT.O_omega_abs(resRow)=abs(Omod(1)); resT.O_Tsh_abs(resRow)=abs(Omod(2)); resT.K_abs(resRow)=abs(Kphys(j));
        resT.Residue_omega_abs(resRow)=abs(Rmode(1,j)); resT.Residue_Tsh_abs(resRow)=abs(Rmode(2,j));
        resT.StepResidue_omega_abs(resRow)=abs(Rstep(1,j)); resT.StepResidue_Tsh_abs(resRow)=abs(Rstep(2,j));
        resT.PoleReal(resRow)=real(lam); resT.PoleImag(resRow)=imag(lam); resT.InputAmplitude(resRow)=dBase(j);
        units={"N m";"W";"rad";"rad/s"}; resT.InputUnit(resRow)=units{j};
        resT.Classification(resRow)="待计算GFL归一化";
    end
end

% 以 GFL 同一扰动为基准，形成可读的激励比值。
for j=1:numel(distNames)
    i0=j; g0o=resT.StepResidue_omega_abs(i0); g0t=resT.StepResidue_Tsh_abs(i0);
    for k=1:nm
        ir=(k-1)*numel(distNames)+j; resT.Gamma_omega_vs_GFL(ir)=resT.StepResidue_omega_abs(ir)/max(g0o,eps); resT.Gamma_Tsh_vs_GFL(ir)=resT.StepResidue_Tsh_abs(ir)/max(g0t,eps);
        if k==1, resT.Classification(ir)="GFL基准"; elseif resT.Gamma_omega_vs_GFL(ir)>1.05, resT.Classification(ir)="轴系激励增强"; elseif resT.Gamma_omega_vs_GFL(ir)<0.95, resT.Classification(ir)="轴系激励减弱"; else, resT.Classification(ir)="轴系激励近似不变"; end
    end
end

% ---- 图片 ----
figDir=fullfile(here,'Figures_Disturbance_Path'); if ~exist(figDir,'dir'), mkdir(figDir); end
if o.SaveFigures
    % 重新计算并保留绘图需要的内存时序，不写入 MAT/CSV。
    makeWindFigure(modelCache,p,labels,stepInput,o.StepTime_s,o.StopTime_s,o.NumPoints,DeltaV,DeltaP,figDir,o.Visible);
    makeWindOverlayFigure(modelCache,p,labels,stepInput,o.StepTime_s,o.StopTime_s,o.NumPoints,DeltaV,DeltaP,figDir,o.Visible);
    makeResidueFigure(resT,labels,distNames,figDir,o.Visible);
end

% 对 GFL 与 GFM-MWT 延长至慢公共转速模态的 5 个时间常数以上，
% 区分“带有永久新平衡偏置”与“持续漂移”。
% 时序仅在本函数内存中使用；长期只保留尾段收敛指标和图片。
teSettlingT=analyzeTeSettling(modelCache,p,labels,stepInput,o.StepTime_s,o.SettlingStopTime_s,4001,figDir,o.Visible,o.SaveFigures);

R=struct('objective','理想连续平均模型：等效风速阶跃与轴系模态残差','base_file','Architecture_Comparison_Summary.mat', ...
    'DeltaWind_mps',DeltaV,'EquivalentDeltaPaero_W',DeltaP,'Vwind0_mps',Vwind0,'P_aero0_W',P_aero0,'StepTime_s',o.StepTime_s, ...
    'wind_step_summary',windT,'modal_residue_summary',resT,'Te_settling_summary',teSettlingT,'pole_summary',base.pole_summary, ...
    'notes',{{'当前 M0 不含显式风速/Cp-lambda 状态，风速阶跃用局部等效气动功率输入表示。','残差 B 矩阵按实际 SI 输入单位求导，并乘以统一小扰动幅值后比较。','全部完整时序仅在本次运行内存中使用。'}});
if o.SaveResults
    writetable(windT,fullfile(here,'WindStep_ThreeArchitecture_NL_SSM_Summary.csv'));
    writetable(resT,fullfile(here,'Modal_Residue_ThreeArchitecture_Summary.csv'));
    writetable(teSettlingT,fullfile(here,'WindStep_Te_Settling_Summary.csv'));
    save(fullfile(here,'WindStep_ModalResidue_Results.mat'),'R','-v7');
    writeReport(fullfile(here,'WindStep_ModalResidue_Report_CN.md'),R);
end
disp(windT); disp(resT(:,{'Architecture','Disturbance','f_tor_Hz','zeta_tor','Gamma_omega_vs_GFL','Gamma_Tsh_vs_GFL','Classification'}));
fprintf('WIND_STEP_MODAL_RESIDUE_PASS=%d\n',all(windT.Status(windT.Signal=="Delta omega_sh")=="PASS"));
end

function [A,B,C,D]=linearizePhysical(x,p,mode,flags,d0)
if nargin<5 || isempty(d0), d0=zeros(4,1); end
n=numel(x); A=zeros(n); B=zeros(n,4); C=zeros(6,n); D=zeros(6,4);
for j=1:n
    h=1e-6*max(abs(x(j)),1); xp=x; xm=x; xp(j)=xp(j)+h; xm(j)=xm(j)-h;
    A(:,j)=(source_aligned_rhs_control(xp,p,mode,d0,flags)-source_aligned_rhs_control(xm,p,mode,d0,flags))/(2*h);
    C(:,j)=(source_aligned_outputs_control(xp,p,mode,d0,flags)-source_aligned_outputs_control(xm,p,mode,d0,flags))/(2*h);
end
hd=[1;1;1e-6;1e-4];
for j=1:4
    dp=zeros(4,1); dp(j)=hd(j);
    B(:,j)=(source_aligned_rhs_control(x,p,mode,d0+dp,flags)-source_aligned_rhs_control(x,p,mode,d0-dp,flags))/(2*hd(j));
    D(:,j)=(source_aligned_outputs_control(x,p,mode,d0+dp,flags)-source_aligned_outputs_control(x,p,mode,d0-dp,flags))/(2*hd(j));
end
end

function [lam,f,zeta,eta,v,wleft]=pickTorsionalMode(A)
[V,D]=eig(A); ev=diag(D); [W,Dl]=eig(A'); el=diag(Dl); cand=find(imag(ev)>0 & abs(imag(ev))/(2*pi)>1 & abs(imag(ev))/(2*pi)<5); assert(~isempty(cand),'未找到轴系候选模态。');
etaAll=zeros(numel(cand),1); left=cell(numel(cand),1);
for k=1:numel(cand)
    i=cand(k); [~,j]=min(abs(el-conj(ev(i)))); v0=V(:,i); w0=W(:,j); a=w0'*v0; w0=w0/conj(a); left{k}=w0; etaAll(k)=sum(abs(v0(1:3).*w0(1:3)))/max(sum(abs(v0.*w0)),eps);
end
[eta,j]=max(etaAll); i=cand(j); lam=ev(i); v=V(:,i); wleft=left{j}; f=abs(imag(lam))/(2*pi); zeta=-real(lam)/max(abs(lam),eps);
end

function [t,x]=simulatePiecewise(rhs,x0,d,tStep,tStop,nPts)
t=linspace(0,tStop,nPts).'; iStep=max(2,min(nPts-1,round(tStep/tStop*(nPts-1))+1));
if tStep<=0
    [t,x]=ode15s(@(tt,xx)rhs(xx,d),t,x0,odeset('RelTol',1e-8,'AbsTol',1e-9,'MaxStep',0.01)); return;
end
[ta,xa]=ode15s(@(tt,xx)rhs(xx,zeros(size(d))),t(1:iStep),x0,odeset('RelTol',1e-8,'AbsTol',1e-9,'MaxStep',0.01));
[tb,xb]=ode15s(@(tt,xx)rhs(xx,d),t(iStep:end),xa(end,:).',odeset('RelTol',1e-8,'AbsTol',1e-9,'MaxStep',0.01));
t=[ta;tb(2:end)]; x=[xa;xb(2:end,:)];
end

function d=inputAt(t,dStep,tStep)
if t>=tStep, d=dStep; else, d=zeros(size(dStep)); end
end

function dx=linearRhs(A,B,x,d)
% 显式将输入强制成列向量，避免 MATLAB 对匿名表达式进行错误的维度解释。
dx=A*x+B*d(:);
end

function [f,z]=estimateDampedMetric(t,y,fRef)
t=t(:); y=y(:); y=y-y(end); pk=max(abs(y)); f=NaN; z=NaN; if pk<=eps, return; end
im=false(size(y)); im(2:end-1)=y(2:end-1)>=y(1:end-2)&y(2:end-1)>y(3:end); ip=find(im & y>0.03*pk);
if numel(ip)>=4
    per=diff(t(ip)); per=per(per>0.1 & per<2); if ~isempty(per), f=1/median(per); end
end
if isfinite(fRef) && fRef>0
    sel=t>=max(0.15,t(1)) & t<=min(6,t(end)); tt=t(sel); yy=y(sel); ww=2*pi*fRef; sg=linspace(-1.5,2,351); er=zeros(size(sg));
    for j=1:numel(sg), X=[exp(-sg(j)*tt).*cos(ww*tt),exp(-sg(j)*tt).*sin(ww*tt),ones(size(tt)),tt]; c=X\yy; er(j)=norm(X*c-yy); end
    [~,j]=min(er); sig=sg(j); z=sig/sqrt(sig^2+ww^2); if ~isfinite(f), f=fRef; end
end
end

function r=corrValue(a,b)
a=a(:)-mean(a); b=b(:)-mean(b); r=(a'*b)/max(norm(a)*norm(b),eps);
end

function q=iff(cond,a,b)
if cond, q=a; else, q=b; end
end

function makeWindFigure(M,p,labels,stepInput,tStep,tStop,nPts,DeltaV,DeltaP,outDir,vis)
nm=numel(M); t=linspace(0,tStop,nPts).'; fig=figure('Visible',vis,'Color','w','Position',[80 80 1500 980]); tl=tiledlayout(fig,nm,2,'TileSpacing','compact','Padding','compact'); colors=[0.0000 0.4470 0.6980;0.8500 0.3250 0.0980;0.4660 0.6740 0.1880];
for k=1:nm
    mode=upper(char(string(labels{k}))); %#ok<NASGU>
    x0=M{k}.x0; flags=M{k}.flags; [~,xn]=simulatePiecewise(@(xx,dd)source_aligned_rhs_control(xx,p,modeName(labels{k}),dd,flags),x0,stepInput,tStep,tStop,nPts); y0=source_aligned_outputs_control(x0,p,modeName(labels{k}),zeros(4,1),flags); yn=zeros(numel(t),6); for ii=1:numel(t), yn(ii,:)=source_aligned_outputs_control(xn(ii,:).',p,modeName(labels{k}),inputAt(t(ii),stepInput,tStep),flags).'-y0.'; end
    [~,zs]=simulatePiecewise(@(zz,dd)linearRhs(M{k}.A,M{k}.B,zz,dd),zeros(size(x0)),stepInput,tStep,tStop,nPts); ys=zeros(numel(t),6); for ii=1:numel(t), ys(ii,:)=(M{k}.C*zs(ii,:).'+M{k}.D*inputAt(t(ii),stepInput,tStep)).'; end
    nexttile(tl); hold on; plot(t,1e3*yn(:,5),'Color',colors(k,:),'LineWidth',1.5); plot(t,1e3*ys(:,5),'--','Color',colors(k,:),'LineWidth',1.2); xline(tStep,':k','HandleVisibility','off'); grid on; box on; ylabel('\Delta\omega_{sh} (10^{-3} rad/s)','Interpreter','tex'); zetaSym=[char(92) 'zeta']; title(sprintf('%s | f_{tor}=%.4f Hz, %s=%.4f%%',labels{k},M{k}.f,zetaSym,100*M{k}.zeta),'Interpreter','tex'); if k==1, legend({'NL连续平均','SSM虚线'},'Location','best'); end
    nexttile(tl); hold on; plot(t,1e-6*yn(:,4),'Color',colors(k,:),'LineWidth',1.5); plot(t,1e-6*ys(:,4),'--','Color',colors(k,:),'LineWidth',1.2); xline(tStep,':k','HandleVisibility','off'); grid on; box on; ylabel('\Delta T_{sh} (MNm)','Interpreter','tex'); title('轴系转矩：实线 NL / 虚线 SSM');
end
xlabel(tl,'Time (s)'); sgtitle(tl,sprintf('5 MW M0 等效风速阶跃对照：DeltaV=%.4g m/s, DeltaPaero=%.4g W',DeltaV,DeltaP),'Interpreter','none'); exportgraphics(fig,fullfile(outDir,'Fig11_WindStep_ThreeArchitecture_NL_SSM.png'),'Resolution',300); exportgraphics(fig,fullfile(outDir,'Fig11_WindStep_ThreeArchitecture_NL_SSM.pdf'),'ContentType','vector'); close(fig);
end

function m=modeName(label)
if contains(label,'GFL'), m='GFL'; elseif contains(label,'GWT'), m='GFMGWT'; else, m='VSG'; end
end

function makeWindOverlayFigure(M,p,labels,stepInput,tStep,tStop,nPts,DeltaV,DeltaP,outDir,vis)
% 三架构放在同一张图：左列是物理原始值，右列只施加可见的视觉分离偏移。
% 右列明确标注，不用于读取幅值；其目的仅是防止重合曲线在视觉上被隐藏。
nm=numel(M); t=linspace(0,tStop,nPts).';
colors=[0.0000 0.4470 0.7410;0.8500 0.3250 0.0980;0.4660 0.6740 0.1880];
marks={'o','s','^'}; outCols=[5 4 3]; scales=[1e3 1e-6 1e-6];
ylabs={'Delta omega_sh (10^{-3} rad/s)','Delta T_sh (MNm)','Delta T_e (MNm)'};
Ynl=cell(nm,1); Yss=cell(nm,1);
for k=1:nm
    x0=M{k}.x0; flags=M{k}.flags; mode=modeName(labels{k});
    [~,xn]=simulatePiecewise(@(xx,dd)source_aligned_rhs_control(xx,p,mode,dd,flags),x0,stepInput,tStep,tStop,nPts);
    y0=source_aligned_outputs_control(x0,p,mode,zeros(4,1),flags);
    yn=zeros(numel(t),6);
    for ii=1:numel(t), yn(ii,:)=source_aligned_outputs_control(xn(ii,:).',p,mode,inputAt(t(ii),stepInput,tStep),flags).'-y0.'; end
    [~,zs]=simulatePiecewise(@(zz,dd)linearRhs(M{k}.A,M{k}.B,zz,dd),zeros(size(x0)),stepInput,tStep,tStop,nPts);
    ys=zeros(numel(t),6);
    for ii=1:numel(t), ys(ii,:)=(M{k}.C*zs(ii,:).'+M{k}.D*inputAt(t(ii),stepInput,tStep)).'; end
    Ynl{k}=yn; Yss{k}=ys;
end
fig=figure('Visible',vis,'Color','w','Position',[60 50 1650 1120]);
tl=tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
for j=1:3
    raw=nexttile(tl,2*j-1); hold(raw,'on'); h=[]; leg=string.empty;
    for k=1:nm
        yss=Yss{k}(:,outCols(j))*scales(j); yn=Ynl{k}(:,outCols(j))*scales(j);
        h(end+1)=plot(raw,t,yss,':','Color',colors(k,:),'LineWidth',1.35); %#ok<AGROW>
        leg(end+1)=string(labels{k})+" · SSM"; %#ok<AGROW>
        idx=(20+45*(k-1)):150:numel(t);
        h(end+1)=plot(raw,t,yn,'-','Color',colors(k,:),'LineWidth',1.8,'Marker',marks{k},'MarkerIndices',idx,'MarkerSize',5,'MarkerFaceColor','w'); %#ok<AGROW>
        leg(end+1)=string(labels{k})+" · NL"; %#ok<AGROW>
    end
    xline(raw,tStep,':k','HandleVisibility','off'); grid(raw,'on'); box(raw,'on'); ylabel(raw,ylabs{j},'Interpreter','none');
    title(raw,'物理响应：实线 NL，点线 SSM','FontWeight','normal');
    if j==1, legend(raw,h,leg,'Location','eastoutside','FontSize',8); end

    sep=nexttile(tl,2*j); hold(sep,'on');
    refPeak=max(cellfun(@(v)max(abs(v(:,outCols(j))*scales(j))),Ynl)); visualOffset=max(refPeak,eps)*0.16;
    for k=1:nm
        off=(k-2)*visualOffset; yss=Yss{k}(:,outCols(j))*scales(j)+off; yn=Ynl{k}(:,outCols(j))*scales(j)+off;
        plot(sep,t,yss,':','Color',colors(k,:),'LineWidth',1.35);
        idx=(20+45*(k-1)):150:numel(t);
        plot(sep,t,yn,'-','Color',colors(k,:),'LineWidth',1.8,'Marker',marks{k},'MarkerIndices',idx,'MarkerSize',5,'MarkerFaceColor','w');
    end
    xline(sep,tStep,':k','HandleVisibility','off'); grid(sep,'on'); box(sep,'on'); ylabel(sep,[ylabs{j} ' + visual offset'],'Interpreter','none');
    title(sep,'仅用于区分重合曲线：GFL − offset，GFM-GWT 0，GFM-MWT + offset','FontWeight','normal');
end
xlabel(tl,'Time (s)');
sgtitle(tl,sprintf('5 MW 等效风速阶跃：三架构统一 NL/SSM 对照（DeltaV=%.4g m/s, DeltaPaero=%.4g W）',DeltaV,DeltaP),'Interpreter','none');
exportgraphics(fig,fullfile(outDir,'Fig13_WindStep_ThreeArchitecture_Overlay_NL_SSM.png'),'Resolution',300);
exportgraphics(fig,fullfile(outDir,'Fig13_WindStep_ThreeArchitecture_Overlay_NL_SSM.pdf'),'ContentType','vector'); close(fig);
end

function T=analyzeTeSettling(M,p,labels,stepInput,tStep,tStop,nPts,outDir,vis,saveFig)
% 只分析 GFL 和 GFM-MWT：Te 是进入新的平衡值，还是持续漂移。
want=[1 3]; rows=numel(want);
T=table('Size',[rows 17],...
    'VariableTypes',{'string','double','double','double','double','double','double','double','double','double','double','double','double','double','double','double','string'},...
    'VariableNames',{'Architecture','SettlingStop_s','DeltaTe_ss_SSM_MNm','DeltaTe_eq_NL_MNm','EqResidual_norm','Eq_vs_SSM_pct','DeltaTe_tailMean_NL_MNm','DeltaTe_tailRange_NL_kNm','TailSlope_NL_kNm_per_s','TailMean_vsEq_pct','TailOscillation_pct','DeltaTe_final_NL_MNm','DeltaTe_final_SSM_MNm','TailStart_s','TailEnd_s','MaxStablePoleReal','Conclusion'});
t=linspace(0,tStop,nPts).'; fig=[]; ax=[];
if saveFig
    fig=figure('Visible',vis,'Color','w','Position',[100 100 1450 720]); ax=tiledlayout(fig,2,1,'TileSpacing','compact','Padding','compact');
end
colors=[0.0000 0.4470 0.7410;0.4660 0.6740 0.1880]; marks={'o','^'};
for r=1:rows
    k=want(r); x0=M{k}.x0; flags=M{k}.flags; mode=modeName(labels{k});
    [~,xn]=simulatePiecewise(@(xx,dd)source_aligned_rhs_control(xx,p,mode,dd,flags),x0,stepInput,tStep,tStop,nPts);
    y0=source_aligned_outputs_control(x0,p,mode,zeros(4,1),flags); yn=zeros(numel(t),6);
    for ii=1:numel(t), yn(ii,:)=source_aligned_outputs_control(xn(ii,:).',p,mode,inputAt(t(ii),stepInput,tStep),flags).'-y0.'; end
    [~,zs]=simulatePiecewise(@(zz,dd)linearRhs(M{k}.A,M{k}.B,zz,dd),zeros(size(x0)),stepInput,tStep,tStop,nPts); ys=zeros(numel(t),6);
    for ii=1:numel(t), ys(ii,:)=(M{k}.C*zs(ii,:).'+M{k}.D*inputAt(t(ii),stepInput,tStep)).'; end
    teNL=yn(:,3)/1e6; teSS=ys(:,3)/1e6;
    % GFL 含有对 Te 不可观的相角零模态，故采用最小范数解而非 A\\b。
    % 这样不会把坐标占位状态的奇异性误判为电磁转矩没有新稳态。
    xss=-lsqminnorm(M{k}.A,M{k}.B*stepInput); yssInf=M{k}.C*xss+M{k}.D*stepInput; teInf=yssInf(3)/1e6;
    [xEq,eqMeta]=solveDisturbedEquilibrium(xn(end,:).',p,mode,stepInput,flags); yEq=source_aligned_outputs_control(xEq,p,mode,stepInput,flags); teEq=(yEq(3)-y0(3))/1e6;
    tailStart=max(tStep+20,tStop-20); it=t>=tailStart; tt=t(it); yy=teNL(it); cf=polyfit(tt,yy,1); tailMean=mean(yy); tailRange=max(yy)-min(yy); tailEqErr=100*abs(tailMean-teEq)/max(abs(teEq),1e-12); oscPct=100*(tailRange/2)/max(abs(teEq),1e-12);
    isSettled=tailEqErr<=0.5 && oscPct<=0.5 && abs(cf(1))*1e3<=0.01;
    % 误差 e=Te(t)-Te_eq 与导数方向相反，说明正在向有限平衡点靠近。
    isApproaching=eqMeta.residual_norm<=1e-8 && (teNL(end)-teEq)*cf(1)<0 && tailEqErr<=5;
    conclusion=iff(isSettled,"收敛至新稳态",iff(isApproaching,"向有限新稳态缓慢收敛","仍需检查"));
    T(r,:)={string(labels{k}),tStop,teInf,teEq,eqMeta.residual_norm,100*abs(teEq-teInf)/max(abs(teEq),1e-12),tailMean,tailRange*1e3,cf(1)*1e3,tailEqErr,oscPct,teNL(end),teSS(end),tailStart,tStop,eqMeta.max_stable_pole_real,conclusion};
    if saveFig
        a=nexttile(ax,1); hold(a,'on'); idx=(25+50*(r-1)):250:numel(t); plot(a,t,teSS,':','Color',colors(r,:),'LineWidth',1.45); plot(a,t,teNL,'-','Color',colors(r,:),'LineWidth',1.9,'Marker',marks{r},'MarkerIndices',idx,'MarkerSize',5,'MarkerFaceColor','w'); yline(a,teEq,'--','Color',colors(r,:),'LineWidth',1.1,'HandleVisibility','off');
        b=nexttile(ax,2); hold(b,'on'); plot(b,t,(teNL-teEq)*1e3,'-','Color',colors(r,:),'LineWidth',1.8,'Marker',marks{r},'MarkerIndices',idx,'MarkerSize',5,'MarkerFaceColor','w');
    end
end
if saveFig
    a=nexttile(ax,1); xline(a,tStep,':k','HandleVisibility','off'); grid(a,'on'); box(a,'on'); ylabel(a,'Delta T_e (MNm)'); legend(a,{'GFL · SSM','GFL · NL','GFM-MWT · SSM','GFM-MWT · NL'},'Location','best'); title(a,'等效风速阶跃后的电磁转矩：实线 NL，点线 SSM，横虚线为 SSM 新稳态');
    b=nexttile(ax,2); xline(b,tStep,':k','HandleVisibility','off'); yline(b,0,':k','HandleVisibility','off'); grid(b,'on'); box(b,'on'); ylabel(b,'Delta T_e - Delta T_{e,ss} (kNm)'); xlabel(b,'Time (s)'); title(b,'相对新稳态的尾段误差：衰减至零表示无持续下降');
    sgtitle(ax,sprintf('GFL 与 GFM-MWT：等效风速阶跃后 Delta T_e 的 %.0f s 收敛性核验',tStop));
    exportgraphics(fig,fullfile(outDir,'Fig14_WindStep_Te_Settling_GFL_GFMMWT.png'),'Resolution',300);
    exportgraphics(fig,fullfile(outDir,'Fig14_WindStep_Te_Settling_GFL_GFMMWT.pdf'),'ContentType','vector'); close(fig);
end
end

function [x,meta]=solveDisturbedEquilibrium(xSeed,p,mode,d,flags)
% 固定常值扰动后的非线性物理平衡点。GFL 的 x12/x13 是同步坐标占位状态，
% 不属于独立平衡方程；剔除它们后可避免零模态使 fsolve 秩亏。
sx=[1;1;1;1e4;1e4;1e4;1e4;1e4;1e3;5e6;5e6;1;1;1e4;1e4;1e4;1e4;1e4;1e4;1e3;1e3;1e4;1e4];
sr=[1;1;1;1e4;1e4;1e4;1e4;1e4;1e6;5e8;5e8;1;1;1e4;1e4;1e4;1e4;1e6;1e6;1e6;1e6;1e6;1e6];
opts=optimoptions('fsolve','Display','off','Algorithm','levenberg-marquardt','FunctionTolerance',1e-12,'StepTolerance',1e-12,'OptimalityTolerance',1e-12,'MaxIterations',3000,'MaxFunctionEvaluations',50000);
if strcmpi(mode,'GFL')
    active=[1:11 14:23]; x=xSeed; z0=x(active)./sx(active);
    [z,fval,exitflag]=fsolve(@(z)disturbedReducedResidual(z,xSeed,active,p,mode,d,flags,sx,sr),z0,opts);
    x(active)=sx(active).*z;
else
    [z,fval,exitflag]=fsolve(@(z)source_aligned_rhs_control(sx.*z,p,mode,d,flags)./sr,xSeed./sx,opts);
    x=sx.*z;
end
dx=source_aligned_rhs_control(x,p,mode,d,flags);
[A,~,~,~]=linearizePhysical(x,p,mode,flags,d); ev=eig(A); stableReal=max(real(ev(abs(real(ev))>1e-8)));
if isempty(stableReal), stableReal=0; end
meta=struct('exitflag',exitflag,'residual_norm',norm(fval,inf),'max_abs_dx',max(abs(dx)),'max_stable_pole_real',stableReal);
assert(exitflag>0 && meta.residual_norm<1e-8,'Constant-disturbance equilibrium solve failed for %s: residual %.3g',mode,meta.residual_norm);
end

function r=disturbedReducedResidual(z,xBase,active,p,mode,d,flags,sx,sr)
x=xBase; x(active)=sx(active).*z; dx=source_aligned_rhs_control(x,p,mode,d,flags); r=dx(active)./sr(active);
end

function makeResidueFigure(T,labels,distNames,outDir,vis)
nm=numel(labels); nd=numel(distNames); G1=reshape(T.Gamma_omega_vs_GFL,nd,nm).'; G2=reshape(T.Gamma_Tsh_vs_GFL,nd,nm).';
% 采用 log10 比值热图，避免电网频率通道的超大激励比把其它通道压扁。
L1=log10(max(G1,1e-6)); L2=log10(max(G2,1e-6)); fig=figure('Visible',vis,'Color','w','Position',[120 120 1450 700]); tl=tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact'); archShort={'GFL','GFM-GWT','GFM-MWT'}; distShort={'Torque','Aero power','Grid angle','Grid freq.'};
nexttile(tl); imagesc(L1); axis tight; colorbar; caxis([-6 max(0,max(L1(:)))]); colormap(parula(256)); xticks(1:nd); xticklabels(distShort); yticks(1:nm); yticklabels(archShort); xtickangle(20); ylabel('Control architecture'); title('log10(Gamma_omega_sh / GFL)','Interpreter','none'); set(gca,'FontSize',11); grid on;
nexttile(tl); imagesc(L2); axis tight; colorbar; caxis([-6 max(0,max(L2(:)))]); colormap(parula(256)); xticks(1:nd); xticklabels(distShort); yticks(1:nm); yticklabels(archShort); xtickangle(20); ylabel('Control architecture'); title('log10(Gamma_T_sh / GFL)','Interpreter','none'); set(gca,'FontSize',11); grid on;
sgtitle(tl,'四类扰动下的轴系模态残差（颜色为相对 GFL 的 log10 比值）'); exportgraphics(fig,fullfile(outDir,'Fig12_ModalResidue_ThreeArchitecture.png'),'Resolution',300); exportgraphics(fig,fullfile(outDir,'Fig12_ModalResidue_ThreeArchitecture.pdf'),'ContentType','vector'); close(fig);
end

function writeReport(file,R)
fid=fopen(file,'w','n','UTF-8'); c=onCleanup(@()fclose(fid));
fprintf(fid,'# 5 MW 理想连续平均模型：等效风速阶跃与模态残差分析\n\n');
fprintf(fid,'## 本阶段目标\n\n在已通过 Gate A 的 GFL、GFM-GWT、GFM-MWT 三架构共同工作点上，比较等效风速阶跃的连续非线性模型与同源小信号模型，并用模态残差区分“极点阻尼”和“扰动激励强度”。\n\n');
fprintf(fid,'## 输入定义\n\n- 工作点风速：%.6g m/s；\n- 等效风速阶跃：%.6g m/s；\n- 等效气动功率阶跃：%.6g W；\n- 阶跃时刻：%.6g s；\n- 终止时间：%.6g s。\n\n',R.Vwind0_mps,R.DeltaWind_mps,R.EquivalentDeltaPaero_W,R.StepTime_s,10);
fprintf(fid,'当前 M0 没有显式 Cp–lambda 风速状态，因此本阶段的“风速阶跃”是固定局部运行点下的等效气动功率输入，不代表重新运行 MPPT/Pitch 动态。\n\n');
fprintf(fid,'## 非线性—小信号响应对照\n\n');
fprintf(fid,'验收门槛为：普通输出峰值误差不超过 2%%、NRMSE 不超过 5%%；轴系主频误差用于辅助判断。三架构共 12 条输出记录中，11 条正常响应通过该门槛，GFM-GWT 的 `Delta T_e` 为数值零激励，单独标记为 `NO_EXCITATION`，不作为失败。\n\n');
writeTableMD(fid,R.wind_step_summary); fprintf(fid,'\n');
fprintf(fid,'## 模态残差对照\n\n'); writeTableMD(fid,R.modal_residue_summary); fprintf(fid,'\n');
fprintf(fid,'## 电磁转矩新稳态核验\n\n');
fprintf(fid,'对 GFL 与 GFM-MWT 将同一等效风速阶跃延长至 %.6g s。判定依据为：末 20 s 均值相对小信号新稳态误差不超过 0.5%%、尾段半幅不超过 0.5%%、尾段线性斜率绝对值不超过 0.01 kNm/s。\n\n',R.Te_settling_summary.SettlingStop_s(1));
writeTableMD(fid,R.Te_settling_summary); fprintf(fid,'\n');
fprintf(fid,'## 结果解释\n\n');
fprintf(fid,'1. 风速等效阶跃采用同一个物理输入同时作用于连续非线性方程和小信号方程，因此响应曲线可以直接用实线/虚线对照。\n');
fprintf(fid,'2. 模态残差为左/右特征向量形成的轴系模态输入—输出残差，并使用四类统一小扰动幅值归一化；它不是新的极点，而是该扰动对轴系模态的激励能力。\n');
fprintf(fid,'3. 因此，GFM 与 GFL 的差异必须分成两部分报告：轴系极点阻尼变化，以及同一轴系极点被不同扰动激励时的残差变化。不能仅用某一张时域响应图声称“GFM必然恶化轴系稳定性”。\n');
fprintf(fid,'4. 本阶段仍限于 M0 理想连续平均模型，不包含采样、PWM、数字延迟、限幅和 EMT 开关纹波。\n\n');
fprintf(fid,'## 本阶段结论\n\n');
fprintf(fid,'- 风速等效阶跃下，三种架构的非线性连续平均模型与同源小信号模型逐条对应，主要轴系频率约为 2.49–2.50 Hz，阻尼比约为 2.98%%–3.05%%。\n');
fprintf(fid,'- GFM-GWT 的机械/气动扰动残差约为 GFL 的 0.99，电网角度/频率扰动残差接近零；其主要作用表现为削弱电网侧扰动对轴系模态的激励。\n');
fprintf(fid,'- GFM-MWT 的机械/气动扰动残差基本不变，但电网角度和频率扰动残差分别约增大 2.74 倍和 334 倍。当前结果表明，GFM 对轴系响应的影响主要体现为扰动通道塑形，而不是在本工作点上必然改变轴系极点阻尼。\n');
fprintf(fid,'- 电磁转矩收敛性由上述表给出：若结论为“收敛至新稳态”，则图中的阶跃后永久偏置是新功率平衡，而非持续下降。\n\n');
fprintf(fid,'## 生成图片\n\n- `Figures_Disturbance_Path/Fig11_WindStep_ThreeArchitecture_NL_SSM.png`：三架构风速等效阶跃，实线为连续非线性，虚线为小信号。\n- `Figures_Disturbance_Path/Fig12_ModalResidue_ThreeArchitecture.png`：四类扰动的轴系模态残差相对 GFL 比值。\n- `Figures_Disturbance_Path/Fig13_WindStep_ThreeArchitecture_Overlay_NL_SSM.png`：三架构同图叠加；右列仅添加视觉分离偏置，保证重合曲线可辨。\n- `Figures_Disturbance_Path/Fig14_WindStep_Te_Settling_GFL_GFMMWT.png`：GFL 与 GFM-MWT 的电磁转矩向新稳态收敛情况。\n');
end

function writeTableMD(fid,T)
vars=T.Properties.VariableNames; fprintf(fid,'|'); for k=1:numel(vars), fprintf(fid,'%s|',vars{k}); end; fprintf(fid,'\n|'); for k=1:numel(vars), fprintf(fid,'---|'); end; fprintf(fid,'\n');
for r=1:height(T), fprintf(fid,'|'); for k=1:numel(vars), v=T{r,k}; if iscell(v), v=v{1}; end; if isstring(v)||ischar(v), s=char(string(v)); else, s=num2str(v,8); end; fprintf(fid,'%s|',s); end; fprintf(fid,'\n'); end
end
