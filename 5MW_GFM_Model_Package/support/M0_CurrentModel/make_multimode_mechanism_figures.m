function make_multimode_mechanism_figures(chain,details,scr,H,DVC,V,outDir)
%MAKE_MULTIMODE_MECHANISM_FIGURES
% 只保存三张可直接用于论文梳理的汇总图；所有原始时序仍只驻留内存。
figDir=fullfile(outDir,'Figures_Multimode_Mechanism'); if ~exist(figDir,'dir'), mkdir(figDir); end
localTransferFigure(chain,figDir); localReconstructionFigure(details,figDir); localScanValidationFigure(scr,H,DVC,V,figDir);
end

function localTransferFigure(T,outDir)
signals={'P_GSC','Udc','iq_MSC_ref','iq_MSC','T_e','omega_sh','T_sh'}; dists={'Grid angle','Grid frequency'}; modes={'GFMGWT','VSG'};
fig=figure('Visible','off','Color','w','Position',[80 80 1500 720]); tl=tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
for d=1:2
    ax=nexttile(tl); A=zeros(numel(signals),2);
    for m=1:2
        for s=1:numel(signals)
            r=T(T.Mode==string(modes{m}) & T.Disturbance==string(dists{d}) & T.Signal==string(signals{s}) & T.FrequencyReference=="Common",:);
            A(s,m)=log10(max(r.GainRatio_vs_GFL(1),1e-12));
        end
    end
    bar(ax,A); grid(ax,'on'); yline(ax,0,'k-'); yline(ax,log10(1.5),':k'); yline(ax,log10(.67),':k');
    xticks(ax,1:numel(signals)); xticklabels(ax,signals); xtickangle(ax,25); ylabel(ax,'log_{10}(|G| / |G_{GFL}|)'); title(ax,[dists{d} '，f = 2.4942 Hz']); legend(ax,{'GFM-GWT','GFM-MWT'},'Location','best');
end
sgtitle(tl,'网侧扰动逐级传递差异：GSC—DC-link—MSC—轴系');
exportgraphics(fig,fullfile(outDir,'Fig20_GridToShaft_Transfer_Chain.png'),'Resolution',300); exportgraphics(fig,fullfile(outDir,'Fig20_GridToShaft_Transfer_Chain.pdf'),'ContentType','vector'); close(fig);
end

function localReconstructionFigure(details,outDir)
pick=[];
for k=1:numel(details)
    if strcmp(details(k).disturbance,'Grid frequency') && (contains(details(k).architecture,'GFL') || contains(details(k).architecture,'MWT'))
        pick(end+1)=k; %#ok<AGROW>
    end
end
fig=figure('Visible','off','Color','w','Position',[80 80 1400 850]); tl=tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
for q=1:min(4,numel(pick))
    D=details(pick(q)); ax=nexttile(tl); plot(ax,D.t,D.full,'k-','LineWidth',1.8); hold(ax,'on'); plot(ax,D.t,D.minimal,'--','Color',[.85 .33 .10],'LineWidth',1.8); xline(ax,.1,':k'); grid(ax,'on'); ylabel(ax,D.output,'Interpreter','none'); title(ax,[D.architecture ' · ' D.output],'Interpreter','none'); legend(ax,{'Full SSM','Minimal multimode'},'Location','best');
end
xlabel(tl,'Time (s)'); sgtitle(tl,'grid-frequency阶跃：完整SSM与最小多模态重构');
exportgraphics(fig,fullfile(outDir,'Fig21_Minimal_Multimode_Reconstruction.png'),'Resolution',300); exportgraphics(fig,fullfile(outDir,'Fig21_Minimal_Multimode_Reconstruction.pdf'),'ContentType','vector'); close(fig);
end

function localScanValidationFigure(scr,H,DVC,V,outDir)
fig=figure('Visible','off','Color','w','Position',[80 80 1500 980]); tl=tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
ax=nexttile(tl); im=contains(scr.Architecture,'MWT'); plot(ax,scr.SCR(im),scr.Gamma_frequency_vs_GFL(im),'o-','LineWidth',1.7); grid(ax,'on'); xlabel(ax,'SCR'); ylabel(ax,'Gamma_f'); title(ax,'SCR对频率扰动轴系残差比的影响');
ax=nexttile(tl); plot(ax,H.H_s,H.Rtor_frequency,'o-','LineWidth',1.7); grid(ax,'on'); xlabel(ax,'H (s)'); ylabel(ax,'|R_{tor,f}|'); title(ax,'H对轴系模态频率输入残差的影响');
ax=nexttile(tl); plot(ax,DVC.DVC_Scale,DVC.G_Te_frequency,'o-','LineWidth',1.7); hold(ax,'on'); plot(ax,DVC.DVC_Scale,DVC.Rtor_frequency,'s--','LineWidth',1.7); grid(ax,'on'); xlabel(ax,'DVC PI scale'); ylabel(ax,'magnitude'); title(ax,'DVC带宽对 Udc→Te→轴系路径的影响'); legend(ax,{'|G_{Te,f}|','|R_{tor,f}|'},'Location','best');
ax=nexttile(tl); hold(ax,'on'); cols=lines(6);
for k=1:2:numel(V)
    c=cols((k+1)/2,:); plot(ax,V(k).t,V(k).nl,'-','Color',c,'LineWidth',1.2); plot(ax,V(k).t,V(k).minimal,'--','Color',c,'LineWidth',1.2);
end
grid(ax,'on'); xlabel(ax,'Time (s)'); ylabel(ax,'Delta omega_sh (rad/s)'); title(ax,'代表点：理想非线性与最小多模态重构');
sgtitle(tl,'参数规律与代表性理想连续非线性验证');
exportgraphics(fig,fullfile(outDir,'Fig22_ParameterScan_and_NonlinearValidation.png'),'Resolution',300); exportgraphics(fig,fullfile(outDir,'Fig22_ParameterScan_and_NonlinearValidation.pdf'),'ContentType','vector'); close(fig);
end
