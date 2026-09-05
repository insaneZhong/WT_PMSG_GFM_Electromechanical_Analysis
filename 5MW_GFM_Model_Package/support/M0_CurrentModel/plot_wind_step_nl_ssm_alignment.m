function R = plot_wind_step_nl_ssm_alignment(varargin)
%PLOT_WIND_STEP_NL_SSM_ALIGNMENT
% 在冻结的 5 MW M0 工作点施加局部等效风速阶跃，比较：
%   1) 连续非线性平均模型（实线）；
%   2) 同源小信号模型（虚线）。
%
% 说明：当前 M0 源方程的外部机械扰动接口是 Delta P_aero，而不是
% 原始风速状态。因此本脚本在固定转速/桨距的局部邻域内采用：
%   Delta P_aero = (dP_aero/dV)|0 * Delta V_wind
% 将风速阶跃严格映射为同一个功率扰动，避免引入 MPPT/Pitch 过渡动态。
% 不保存原始长时序，只保存精简指标 CSV 和论文图片。

ip=inputParser;
ip.addParameter('Control','VSG',@(x)ischar(x)||isstring(x));
ip.addParameter('DeltaWind_mps',0.01,@(x)isnumeric(x)&&isscalar(x)&&isfinite(x));
ip.addParameter('StepTime_s',0.10,@(x)isnumeric(x)&&isscalar(x)&&x>=0);
ip.addParameter('StopTime_s',10,@(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('NumPoints',5001,@(x)isnumeric(x)&&isscalar(x)&&x>=100);
ip.addParameter('Visible','off',@(x)ischar(x)||isstring(x));
ip.addParameter('SaveFigure',true,@(x)islogical(x)&&isscalar(x));
ip.addParameter('SaveSummary',true,@(x)islogical(x)&&isscalar(x));
ip.parse(varargin{:}); o=ip.Results;

here=fileparts(mfilename('fullpath')); addpath(here,fileparts(here));
M=analyze_modal_residue_decomposition('SaveSummary',false);
mode=upper(char(string(o.Control)));
idx=find(strcmpi({'GFL','DROOP','VSG'},mode),1);
assert(~isempty(idx),'Control must be GFL, DROOP or VSG.');
Q=M.models{idx}; x0=Q.x; p=Q.p; n=numel(x0);

% 当前 M0 平衡点的局部风速—气动功率映射。
P_aero0=p(39)*p(12); Vwind0=12.20;
dPaero_dV=3*P_aero0/max(Vwind0,eps);
DeltaV=o.DeltaWind_mps; DeltaP=dPaero_dV*DeltaV;

% 统一的阶跃定义：在 StepTime_s 之后施加正的气动功率增量。
dStep=zeros(4,1); dStep(2)=DeltaP;
t=linspace(0,o.StopTime_s,round(o.NumPoints)).';

% 原始工作点输出，用于将两种模型都写成增量响应。
y0=source_aligned_outputs_control(x0,p,mode,zeros(4,1));

% 连续非线性 M0。
odeOpts=odeset('RelTol',1e-8,'AbsTol',1e-9,'MaxStep',0.01);
[tn,xn]=ode15s(@(tt,xx)rhsStepNL(tt,xx,p,mode,dStep,o.StepTime_s),t,x0,odeOpts);
yn=zeros(numel(tn),6);
for k=1:numel(tn)
    yn(k,:)=source_aligned_outputs_control(xn(k,:).',p,mode,rhsInput(tn(k),dStep,o.StepTime_s)).';
end
yn=yn-y0.';

% 同源小信号模型：数值 Jacobian，输入和输出均采用实际 SI 单位导数。
[A,B,C,D]=linearizeSource(x0,p,mode);
[ts,zs]=ode15s(@(tt,zz)rhsStepSSM(tt,zz,A,B,dStep,o.StepTime_s),t,zeros(n,1),odeOpts);
ys=(C*zs.' + D*inputMatrix(ts,dStep,o.StepTime_s)).';

% 四个论文主变量：轴系相对速度、轴转矩、电磁转矩、直流电压。
% 机械侧风速扰动不一定直接改变瞬时 PCC 有功，因此这里避免把
% P_PCC 的近零响应误判为模型不对应。
sigNames={"Delta omega_sh";"Delta T_sh";"Delta T_e";"Delta Udc"};
sigUnits={"rad/s";"MNm";"MNm";"V"};
cols=[5 4 3 2]; scale=[1 1e6 1e6 1];
Ynl=zeros(numel(t),4); Yssm=zeros(numel(t),4);
for j=1:4
    Ynl(:,j)=yn(:,cols(j))/scale(j);
    Yssm(:,j)=ys(:,cols(j))/scale(j);
end

% 只保存精简指标，不保存完整时序。
summary=table('Size',[4 12], ...
    'VariableTypes',{'string','string','double','double','double','double','double','double','double','double','double','string'}, ...
    'VariableNames',{'Control','Signal','DeltaWind_mps','EquivalentDeltaPaero_W','StepTime_s','Peak_NL','Peak_SSM','PeakError_pct','NRMSE','Correlation','Final_NL','Final_SSM'});
for j=1:4
    a=Ynl(:,j); b=Yssm(:,j); den=max(norm(a-mean(a)),eps);
    summary.Control(j)=string(mode); summary.Signal(j)=string(sigNames{j});
    summary.DeltaWind_mps(j)=DeltaV; summary.EquivalentDeltaPaero_W(j)=DeltaP;
    summary.StepTime_s(j)=o.StepTime_s; summary.Peak_NL(j)=max(abs(a)); summary.Peak_SSM(j)=max(abs(b));
    summary.PeakError_pct(j)=100*abs(summary.Peak_NL(j)-summary.Peak_SSM(j))/max(summary.Peak_NL(j),eps);
    summary.NRMSE(j)=norm(a-b)/den; summary.Correlation(j)=corrValue(a,b);
    summary.Final_NL(j)=a(end); summary.Final_SSM(j)=b(end);
end

% 统一图：实线为连续非线性，虚线为小信号。
fig=figure('Visible',o.Visible,'Color','w','Position',[100 100 1500 900]);
tl=tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
colors=[0.0000 0.4470 0.6980;0.8353 0.3686 0.0000;0.0000 0.6196 0.4509;0.4940 0.1840 0.5560];
for j=1:4
    nexttile(tl); hold on;
    plot(t,Ynl(:,j),'Color',colors(j,:),'LineWidth',1.6,'LineStyle','-');
    plot(t,Yssm(:,j),'Color',colors(j,:),'LineWidth',1.4,'LineStyle','--');
    xline(o.StepTime_s,':k','LineWidth',0.9,'HandleVisibility','off');
    grid on; box on; xlabel('Time (s)'); ylabel(sprintf('%s (%s)',char(sigNames{j}),char(sigUnits{j})));
    title(sprintf('%s: NL vs SSM, peak error %.3f%%',char(sigNames{j}),summary.PeakError_pct(j)),'Interpreter','none');
    if j==1, legend({'连续非线性平均模型','小信号模型'},'Location','best'); end
end
sgtitle(tl,sprintf('5 MW M0 wind-speed-equivalent step alignment (%s, DeltaV = %.3f m/s)',mode,DeltaV),'Interpreter','none');

outDir=fullfile(here,'Figures_Disturbance_Path');
if o.SaveFigure
    if ~exist(outDir,'dir'), mkdir(outDir); end
    exportgraphics(fig,fullfile(outDir,'Fig08_WindStep_NL_SSM_Alignment.png'),'Resolution',300);
    exportgraphics(fig,fullfile(outDir,'Fig08_WindStep_NL_SSM_Alignment.pdf'),'ContentType','vector');
end
if o.SaveSummary
    writetable(summary,fullfile(here,'WindStep_NL_SSM_Comparison_Summary.csv'));
end
close(fig);

R=struct('Control',mode,'DeltaWind_mps',DeltaV,'EquivalentDeltaPaero_W',DeltaP, ...
    'StepTime_s',o.StepTime_s,'Vwind0_mps',Vwind0,'P_aero0_W',P_aero0, ...
    'Summary',summary,'Figure',fullfile(outDir,'Fig08_WindStep_NL_SSM_Alignment.png'));
fprintf('WIND_STEP_ALIGNMENT control=%s DeltaV=%.6g m/s DeltaPaero=%.6g W maxPeakErr=%.6g%% maxNRMSE=%.6g\n', ...
    mode,DeltaV,DeltaP,max(summary.PeakError_pct),max(summary.NRMSE));
end

function dx=rhsStepNL(t,x,p,mode,dStep,tStep)
dx=source_aligned_rhs_control(x,p,mode,rhsInput(t,dStep,tStep));
end

function d=rhsInput(t,dStep,tStep)
if t>=tStep, d=dStep; else, d=zeros(4,1); end
end

function D=inputMatrix(t,dStep,tStep)
D=zeros(4,numel(t));
for k=1:numel(t), D(:,k)=rhsInput(t(k),dStep,tStep); end
end

function dx=rhsStepSSM(t,z,A,B,dStep,tStep)
dx=A*z+B*rhsInput(t,dStep,tStep);
end

function [A,B,C,D]=linearizeSource(x,p,mode)
n=numel(x); A=zeros(n); B=zeros(n,4); C=zeros(6,n); D=zeros(6,4);
for j=1:n
    h=1e-6*max(abs(x(j)),1); xp=x; xm=x; xp(j)=xp(j)+h; xm(j)=xm(j)-h;
    A(:,j)=(source_aligned_rhs_control(xp,p,mode,zeros(4,1))-source_aligned_rhs_control(xm,p,mode,zeros(4,1)))/(2*h);
    C(:,j)=(source_aligned_outputs_control(xp,p,mode,zeros(4,1))-source_aligned_outputs_control(xm,p,mode,zeros(4,1)))/(2*h);
end
hd=[1;1;1e-6;1e-4];
for j=1:4
    dp=zeros(4,1); dp(j)=hd(j);
    B(:,j)=(source_aligned_rhs_control(x,p,mode,dp)-source_aligned_rhs_control(x,p,mode,-dp))/(2*hd(j));
    D(:,j)=(source_aligned_outputs_control(x,p,mode,dp)-source_aligned_outputs_control(x,p,mode,-dp))/(2*hd(j));
end
end

function r=corrValue(a,b)
a=a(:); b=b(:); aa=a-mean(a); bb=b-mean(b); r=(aa'*bb)/max(norm(aa)*norm(bb),eps);
end
