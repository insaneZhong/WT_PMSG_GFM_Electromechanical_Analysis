function outFile = plot_wind_step_ssm_only(varargin)
%PLOT_WIND_STEP_SSM_ONLY
% 只绘制同一 5 MW 平衡点上三种架构的小信号响应。
%
% 目的：不再叠加连续非线性曲线，直接比较 GFL、GFM-GWT 和
% GFM-MWT 在同一等效风速阶跃下的线性响应。左列为物理原值；
% 右列仅添加明确说明的视觉分离偏置，以便曲线重合时仍可辨认。
% 该函数不保存时序或新的 MAT/CSV，只输出一张 PNG 和一张 PDF。

ip=inputParser;
ip.addParameter('DeltaWind_mps',0.01,@(x)isnumeric(x)&&isscalar(x)&&isfinite(x));
ip.addParameter('StepTime_s',0.10,@(x)isnumeric(x)&&isscalar(x)&&x>=0);
ip.addParameter('StopTime_s',10,@(x)isnumeric(x)&&isscalar(x)&&x>1);
ip.addParameter('NumPoints',5001,@(x)isnumeric(x)&&isscalar(x)&&x>=501);
ip.addParameter('Visible','off',@(x)ischar(x)||isstring(x));
ip.parse(varargin{:}); o=ip.Results;

here=fileparts(mfilename('fullpath')); addpath(here,fileparts(here));
S=load(fullfile(here,'Architecture_Comparison_Summary.mat'),'R'); base=S.R;
assert(base.passed,'三架构共同工作点未通过 Gate A，停止绘图。');
p=base.parameter_vector; modes=cellstr(base.models(:)); labels=cellstr(base.labels(:));
xAll=base.states; nm=numel(modes);

% 与主分析脚本完全一致的局部风速—气动功率映射。
Vwind0=12.20; P_aero0=p(39)*p(12);
deltaPaero=(3*P_aero0/max(Vwind0,eps))*o.DeltaWind_mps;
dStep=zeros(4,1); dStep(2)=deltaPaero;
t=linspace(0,o.StopTime_s,round(o.NumPoints)).';

% 输出顺序来自 source_aligned_outputs_control：
% [P_pcc, Udc, Te, Tsh, omega_sh, omega_vsg]。
outCols=[5 4 3]; scales=[1e3 1e-6 1e-6];
ylabs={'Delta omega_sh (10^{-3} rad/s)','Delta T_sh (MNm)','Delta T_e (MNm)'};
Y=cell(nm,1);
for k=1:nm
    flags=localFlags(modes{k},base);
    [A,B,C,D]=localLinearize(xAll(:,k),p,modes{k},flags);
    [~,z]=localStepSim(A,B,zeros(size(xAll(:,k))),dStep,o.StepTime_s,o.StopTime_s,o.NumPoints);
    y=zeros(numel(t),6);
    for ii=1:numel(t)
        d=localInputAt(t(ii),dStep,o.StepTime_s);
        y(ii,:)=(C*z(ii,:).'+D*d).';
    end
    Y{k}=y;
end

colors=[0.0000 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.4660 0.6740 0.1880];
marks={'o','s','^'};
fig=figure('Visible',o.Visible,'Color','w','Position',[70 50 1650 1120]);
tl=tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
for j=1:3
    % 左列：真实小信号物理响应，直接用于读取幅值。
    ax=nexttile(tl,2*j-1); hold(ax,'on'); h=gobjects(nm,1);
    for k=1:nm
        y=Y{k}(:,outCols(j))*scales(j);
        idx=(25+45*(k-1)):150:numel(t);
        h(k)=plot(ax,t,y,'-','Color',colors(k,:),'LineWidth',1.8, ...
            'Marker',marks{k},'MarkerIndices',idx,'MarkerSize',5,'MarkerFaceColor','w');
    end
    xline(ax,o.StepTime_s,':k','HandleVisibility','off'); grid(ax,'on'); box(ax,'on');
    ylabel(ax,ylabs{j},'Interpreter','none');
    title(ax,'小信号物理响应','FontWeight','normal');
    if j==1, legend(ax,h,labels,'Location','eastoutside','FontSize',9); end

    % 右列：只为可见性添加垂直偏移，不用于读取任何物理幅值。
    ax=nexttile(tl,2*j); hold(ax,'on');
    peak=max(cellfun(@(v)max(abs(v(:,outCols(j))*scales(j))),Y));
    visualOffset=max(peak,eps)*0.16;
    for k=1:nm
        y=Y{k}(:,outCols(j))*scales(j)+(k-2)*visualOffset;
        idx=(25+45*(k-1)):150:numel(t);
        plot(ax,t,y,'-','Color',colors(k,:),'LineWidth',1.8, ...
            'Marker',marks{k},'MarkerIndices',idx,'MarkerSize',5,'MarkerFaceColor','w');
    end
    xline(ax,o.StepTime_s,':k','HandleVisibility','off'); grid(ax,'on'); box(ax,'on');
    ylabel(ax,[ylabs{j} ' + visual offset'],'Interpreter','none');
    title(ax,'仅用于区分重合曲线：GFL − offset，GFM-GWT 0，GFM-MWT + offset', ...
        'FontWeight','normal');
end
xlabel(tl,'Time (s)');
sgtitle(tl,sprintf(['5 MW 等效风速阶跃：三架构小信号响应（DeltaV = %.4g m/s，' ...
    'DeltaPaero = %.4g W）'],o.DeltaWind_mps,deltaPaero),'Interpreter','none');

figDir=fullfile(here,'Figures_Disturbance_Path'); if ~exist(figDir,'dir'), mkdir(figDir); end
outFile=fullfile(figDir,'Fig15_WindStep_ThreeArchitecture_SSMOnly.png');
exportgraphics(fig,outFile,'Resolution',300);
exportgraphics(fig,fullfile(figDir,'Fig15_WindStep_ThreeArchitecture_SSMOnly.pdf'),'ContentType','vector');
close(fig);
fprintf('SSM_ONLY_FIGURE=%s\n',outFile);
end

function flags=localFlags(mode,base)
flags=struct;
if strcmpi(mode,'GFMGWT')
    O=load(fullfile(fileparts(mfilename('fullpath')),'03_Mechanism_Evidence_Summary.mat'),'E');
    flags=struct('imqRef0',O.E.operating_point.pmsg_iq0,'KpGscDvc',5e3,'KiGscDvc',5e2,'mpGwt',5e-7);
end
% 参数 base 在此保留为输入，明确所有架构均来自同一个 Gate A 基准。
assert(base.passed,'Gate A 状态异常。');
end

function [A,B,C,D]=localLinearize(x,p,mode,flags)
n=numel(x); A=zeros(n); B=zeros(n,4); C=zeros(6,n); D=zeros(6,4);
for j=1:n
    h=1e-6*max(abs(x(j)),1); e=zeros(n,1); e(j)=h;
    A(:,j)=(source_aligned_rhs_control(x+e,p,mode,zeros(4,1),flags)- ...
        source_aligned_rhs_control(x-e,p,mode,zeros(4,1),flags))/(2*h);
    C(:,j)=(source_aligned_outputs_control(x+e,p,mode,zeros(4,1),flags)- ...
        source_aligned_outputs_control(x-e,p,mode,zeros(4,1),flags))/(2*h);
end
hd=[1;1;1e-6;1e-4]; % [T_m, P_aero, theta_grid, omega_grid] 的 SI 数值差分步长。
for j=1:4
    h=hd(j); e=zeros(4,1); e(j)=h;
    B(:,j)=(source_aligned_rhs_control(x,p,mode,e,flags)- ...
        source_aligned_rhs_control(x,p,mode,-e,flags))/(2*h);
    D(:,j)=(source_aligned_outputs_control(x,p,mode,e,flags)- ...
        source_aligned_outputs_control(x,p,mode,-e,flags))/(2*h);
end
end

function [t,x]=localStepSim(A,B,x0,dStep,tStep,tStop,nPts)
t=linspace(0,tStop,nPts).'; iStep=max(2,min(nPts-1,round(tStep/tStop*(nPts-1))+1));
[t1,x1]=ode15s(@(tt,xx)A*xx,t(1:iStep),x0,odeset('RelTol',1e-8,'AbsTol',1e-9,'MaxStep',0.01));
[t2,x2]=ode15s(@(tt,xx)A*xx+B*dStep,t(iStep:end),x1(end,:).',odeset('RelTol',1e-8,'AbsTol',1e-9,'MaxStep',0.01));
t=[t1;t2(2:end)]; x=[x1;x2(2:end,:)];
end

function d=localInputAt(t,dStep,tStep)
if t>=tStep, d=dStep; else, d=zeros(size(dStep)); end
end
