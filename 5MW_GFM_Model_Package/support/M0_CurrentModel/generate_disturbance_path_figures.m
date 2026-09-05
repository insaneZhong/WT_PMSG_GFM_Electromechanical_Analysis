function outDir=generate_disturbance_path_figures(varargin)
%GENERATE_DISTURBANCE_PATH_FIGURES
% 生成 GFL/Droop-GFM/VSG-GFM 扰动通道机理对照图。
%
% 输入只使用当前目录的精简汇总 CSV，不读取或保存原始时序：
%   ThreeControl_Summary.csv
%   Modal_Residue_Decomposition_Summary.csv
%   Disturbance_Path_Summary.csv
%   Nonlinear_Validation_Summary.csv
%
% 输出统一保存到 Figures_Disturbance_Path，PNG 为 300 dpi，另存 PDF。

ip=inputParser;
ip.addParameter('Visible','off',@(x)ischar(x)||isstring(x));
ip.addParameter('SavePDF',true,@(x)islogical(x)&&isscalar(x));
ip.parse(varargin{:}); opt=ip.Results;

here=fileparts(mfilename('fullpath'));
outDir=fullfile(here,'Figures_Disturbance_Path');
if ~exist(outDir,'dir'), mkdir(outDir); end

oldVis=get(0,'DefaultFigureVisible');
cleanup=onCleanup(@()set(0,'DefaultFigureVisible',oldVis));
set(0,'DefaultFigureVisible',char(opt.Visible));

% Okabe-Ito 色盲友好配色：GFL、Droop-GFM、VSG-GFM。
colors=[0.0000 0.4470 0.6980; 0.0000 0.6196 0.4509; 0.8353 0.3686 0.0000];
controls=["GFL (ideal PLL)" "Droop-GFM" "VSG-GFM"];
shortNames={"GFL","Droop-GFM","VSG-GFM"};

Tctrl=readtable(fullfile(here,'ThreeControl_Summary.csv'),'VariableNamingRule','preserve');
Tres=readtable(fullfile(here,'Modal_Residue_Decomposition_Summary.csv'),'VariableNamingRule','preserve');
Tpath=readtable(fullfile(here,'Disturbance_Path_Summary.csv'),'VariableNamingRule','preserve');
Tnl=readtable(fullfile(here,'Nonlinear_Validation_Summary.csv'),'VariableNamingRule','preserve');

% 图1：轴系极点对应的频率和阻尼。
fig=figure('Color','w','Position',[100 100 1100 430]);
tl=tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
f=zeros(1,3); z=zeros(1,3);
for k=1:3
    ix=strcmp(string(Tctrl.Control),controls(k));
    f(k)=Tctrl.ShaftFrequency_Hz(ix);
    z(k)=100*Tctrl.ShaftDamping(ix);
end
ax=nexttile(tl,1); b=bar(ax,f,0.62); b.FaceColor='flat'; b.CData=colors;
set(ax,'XTick',1:3,'XTickLabel',shortNames); ylabel(ax,'Torsional frequency (Hz)');
title(ax,'Axis mode frequency'); grid(ax,'on'); box(ax,'off');
ylim(ax,[min(f)-0.01 max(f)+0.01]);
for k=1:3, text(ax,k,f(k)+0.002,sprintf('%.4f',f(k)),'HorizontalAlignment','center','FontSize',9); end
ax=nexttile(tl,2); b=bar(ax,z,0.62); b.FaceColor='flat'; b.CData=colors;
set(ax,'XTick',1:3,'XTickLabel',shortNames); ylabel(ax,'Damping ratio (%)');
title(ax,'Axis mode damping'); grid(ax,'on'); box(ax,'off');
ylim(ax,[0 max(z)*1.18]);
for k=1:3, text(ax,k,z(k)+max(z)*0.03,sprintf('%.3f',z(k)),'HorizontalAlignment','center','FontSize',9); end
sgtitle(tl,'GFL–Droop-GFM–VSG-GFM: shaft pole comparison');
saveFig(fig,fullfile(outDir,'Fig01_Pole_Damping_Comparison'),opt.SavePDF);
close(fig);

% 图2：四类扰动的轴系速度残差相对 GFL 比值，采用对数坐标突出激励能力差异。
disturbances=["Mechanical torque" "Aerodynamic power" "Grid angle" "Grid frequency"];
distLabels={'Mechanical torque','Aerodynamic power','Grid angle','Grid frequency'};
fig=figure('Color','w','Position',[100 100 1100 800]);
tl=tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
for j=1:4
    ax=nexttile(tl,j); vals=zeros(1,3); baseVal=NaN;
    for k=1:3
        ix=strcmp(string(Tres.Control),controls(k)) & strcmp(string(Tres.Disturbance),disturbances(j));
        vals(k)=Tres.R_omega_abs(ix);
        if k==1, baseVal=vals(k); end
    end
    vals=vals/max(baseVal,eps);
    b=bar(ax,vals,0.62); b.FaceColor='flat'; b.CData=colors;
    set(ax,'XTick',1:3,'XTickLabel',shortNames);
    % 对接近 1 的比值采用线性坐标，避免 log 轴将 1 显示成一长串
    % 9/0；只有跨越数量级的网侧扰动保留 log 坐标以突出差异。
    if max(vals)/max(min(vals),eps) < 2
        set(ax,'YScale','linear');
        ylim(ax,[0.8 1.2]);
        textY=vals+0.035;
    else
        set(ax,'YScale','log');
        ylim(ax,[max(min(vals)*0.7,1e-3) max(vals)*1.8]);
        textY=vals*1.25;
    end
    ylabel(ax,'Residue ratio to GFL'); title(ax,distLabels{j}); grid(ax,'on'); box(ax,'off');
    for k=1:3, text(ax,k,textY(k),sprintf('%.3g',vals(k)),'HorizontalAlignment','center','FontSize',8); end
end
sgtitle(tl,'Four disturbance classes: torsional-mode residue comparison');
saveFig(fig,fullfile(outDir,'Fig02_Four_Disturbance_Residue_Comparison'),opt.SavePDF);
close(fig);

% 图3：轴系频率处逐级传递的幅值/相位（可视作离散频点 Bode 对照）。
vars=["P_GSC" "Udc" "iq_MSC_ref" "Te" "omega_sh"];
varLabels={'P_{GSC}','U_{dc}','i_{q,MSC}^{*}','T_e','omega_{sh}'};
gridDist=["Grid angle" "Grid frequency"];
fig=figure('Color','w','Position',[100 100 1200 800]);
tl=tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
for j=1:2
    ax=nexttile(tl,2*j-1); hold(ax,'on');
    for k=1:3
        vals=zeros(1,5);
        for q=1:5
            ix=strcmp(string(Tpath.Control),controls(k)) & strcmp(string(Tpath.Disturbance),gridDist(j)) & strcmp(string(Tpath.Variable),vars(q));
            vals(q)=Tpath.Gain(ix);
        end
        semilogy(ax,1:5,vals,'-o','Color',colors(k,:),'LineWidth',1.5,'MarkerSize',5,'DisplayName',shortNames{k});
    end
    set(ax,'XTick',1:5,'XTickLabel',varLabels); xtickangle(ax,25);
    ylabel(ax,'Gain at f_{tor}'); title(ax,[char(gridDist(j)) ' disturbance: gain']);
    grid(ax,'on'); box(ax,'off'); legend(ax,'Location','best','Box','off'); hold(ax,'off');
    ax=nexttile(tl,2*j); hold(ax,'on');
    for k=1:3
        vals=zeros(1,5);
        for q=1:5
            ix=strcmp(string(Tpath.Control),controls(k)) & strcmp(string(Tpath.Disturbance),gridDist(j)) & strcmp(string(Tpath.Variable),vars(q));
            vals(q)=Tpath.Phase_deg(ix);
        end
        plot(ax,1:5,vals,'-o','Color',colors(k,:),'LineWidth',1.5,'MarkerSize',5,'DisplayName',shortNames{k});
    end
    h0=yline(ax,0,'k:'); h0.HandleVisibility='off'; set(ax,'XTick',1:5,'XTickLabel',varLabels); xtickangle(ax,25);
    ylabel(ax,'Phase (deg)'); title(ax,[char(gridDist(j)) ' disturbance: phase']);
    grid(ax,'on'); box(ax,'off'); legend(ax,'Location','best','Box','off'); hold(ax,'off');
end
sgtitle(tl,'Disturbance-path gain and phase at the torsional frequency');
saveFig(fig,fullfile(outDir,'Fig03_Disturbance_Path_Gain_Phase'),opt.SavePDF);
close(fig);

% 图4：连续非线性模型中的峰值响应相对 GFL 的直观对照。
fig=figure('Color','w','Position',[100 100 1100 430]);
tl=tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
for q=1:2
    ax=nexttile(tl,q); vals=zeros(4,3); metric='peak_ratio_to_GFL';
    if q==2, metric='Tsh_peak_ratio_to_GFL'; end
    for j=1:4
        for k=1:3
            ix=strcmp(string(Tnl.Control),controls(k)) & strcmp(string(Tnl.Disturbance),disturbances(j));
            vals(j,k)=Tnl.(metric)(ix);
        end
    end
    b=bar(ax,vals,'grouped');
    for k=1:3, b(k).FaceColor=colors(k,:); end
    set(ax,'XTick',1:4,'XTickLabel',distLabels); xtickangle(ax,20);
    ylabel(ax,'Ratio to GFL'); title(ax,ternary(q==1,'Relative-speed peak','Shaft-torque peak'));
    h1=yline(ax,1,'k:'); h1.HandleVisibility='off'; grid(ax,'on'); box(ax,'off'); legend(ax,shortNames,'Location','best','Box','off');
end
sgtitle(tl,'Continuous nonlinear small-disturbance response comparison');
saveFig(fig,fullfile(outDir,'Fig04_Nonlinear_Response_Ratios'),opt.SavePDF);
close(fig);

% 写一个极简图片目录说明，便于后续实验沿用。
fid=fopen(fullfile(outDir,'README_Figures_CN.md'),'w');
fprintf(fid,'# GFM 扰动通道图片目录\n\n');
fprintf(fid,'本目录图片均由 `generate_disturbance_path_figures.m` 从精简 CSV 自动生成，不依赖原始时序。\n\n');
fprintf(fid,'- Fig01：三种控制的轴系频率与阻尼比。\n');
fprintf(fid,'- Fig02：机械转矩、气动功率、网侧相角、网侧频率四类扰动残差。\n');
fprintf(fid,'- Fig03：轴系固有频率处的逐级扰动路径增益和相位。\n');
fprintf(fid,'- Fig04：连续非线性模型的相对速度峰值和轴转矩峰值相对 GFL 的比例。\n');
fprintf(fid,'\nPNG 为 300 dpi；同时提供 PDF 矢量版本。\n');
fclose(fid);
fprintf('FIGURE_OUTPUT_DIR=%s\n',outDir);
end

function saveFig(fig,base,savePDF)
exportgraphics(fig,[base '.png'],'Resolution',300);
if savePDF
    exportgraphics(fig,[base '.pdf'],'ContentType','vector');
end
end

function s=ternary(cond,a,b)
if cond, s=a; else, s=b; end
end
