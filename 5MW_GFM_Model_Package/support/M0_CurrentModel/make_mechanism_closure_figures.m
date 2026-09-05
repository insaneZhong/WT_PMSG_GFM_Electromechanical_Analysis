function make_mechanism_closure_figures(R,figDir)
%MAKE_MECHANISM_CLOSURE_FIGURES 统一输出论文候选图，不保存原始时序。
if ~exist(figDir,'dir'), mkdir(figDir); end
% Fig 23: H机制与最小模态叠加
f=figure('Visible','off','Color','w','Position',[100 100 1300 620]); tiledlayout(1,2,'Padding','compact','TileSpacing','compact');
nexttile; H=R.H_mechanism.summary; hold on; for c=unique(H.TrackedMode,'stable').', ix=H.TrackedMode==c; plot(H.H_s(ix),H.ResidueMagnitude(ix),'o-','LineWidth',1.5,'DisplayName',char(c)); end, grid on; xlabel('H (s)'); ylabel('|R_{k,f}|'); title('H扫描：各跟踪模态的网侧频率残差'); legend('Location','best');
nexttile; S=R.modal_superposition.summary; ix=S.Output=="omega_sh" & S.Disturbance=="Grid frequency" & S.Architecture~="GFM-GWT (GSC-DVC + MSC-MPPT/转矩)"; B=S(ix,:); bar(1:height(B),B.ContributionRatio); xticks(1:height(B)); xticklabels(strcat(B.Architecture,"/",B.ModeID,"/",B.PhysicalClass)); grid on; ylabel('|y_k(t_{peak})| / |y(t_{peak})|'); title('网侧频率阶跃：峰值处模态贡献'); xtickangle(25);
localSave(f,fullfile(figDir,'Fig23_H_and_Modal_Superposition')); close(f);
% Fig 24: 双向传递
f=figure('Visible','off','Color','w','Position',[100 100 1300 620]); tiledlayout(1,2,'Padding','compact','TileSpacing','compact');
nexttile; T=R.bidirectional.summary; ix=T.Direction=="GRID_TO_MACHINE" & T.Output=="omega_sh"; localBar(T(ix,:),"Disturbance",'Grid-to-shaft |G_{mg}|'); set(gca,'YScale','log');
nexttile; ix=T.Direction=="MACHINE_TO_GRID" & T.Output=="P_PCC"; localBar(T(ix,:),"Disturbance",'Machine-to-PCC |G_{gm}|'); set(gca,'YScale','log');
localSave(f,fullfile(figDir,'Fig24_Bidirectional_Disturbance_Transfer')); close(f);
% Fig 25: 机制区域与代表点对齐
f=figure('Visible','off','Color','w','Position',[100 100 1300 620]); tiledlayout(1,2,'Padding','compact','TileSpacing','compact');
nexttile; T=R.region.summary; localRegionScatter(T(T.Map=="SCR_H",:),'SCR-H mechanism map');
nexttile; T=R.region.summary; localRegionScatter(T(T.Map=="SCR_DVC",:),'SCR-DVC mechanism map');
localSave(f,fullfile(figDir,'Fig25_Mechanism_Region_Maps')); close(f);
% Fig 26: 非线性代表点，仅画omega_sh，不输出原始时序文件
if ~isempty(R.validation)
    f=figure('Visible','off','Color','w','Position',[100 100 1300 620]); tiledlayout(1,2,'Padding','compact','TileSpacing','compact'); outs={"omega_sh","T_sh"};
    for q=1:2
        nexttile; hold on;
        for k=1:numel(R.validation)
            v=R.validation(k);
            if string(v.output)~=outs{q}, continue, end
            plot(v.t,v.nl,'LineWidth',1.5,'DisplayName',[v.classification ' NL']);
            plot(v.t,v.ssm,'--','LineWidth',1,'HandleVisibility','off');
        end
        grid on; xlabel('t (s)'); ylabel(char(outs{q}));
        title([char(outs{q}) ': solid NL, dashed full SSM']); legend('Location','best');
    end
    localSave(f,fullfile(figDir,'Fig26_Representative_Nonlinear_Alignment')); close(f);
end
end
function localBar(T,groupName,ttl)
arch=unique(T.Architecture,'stable'); dst=unique(T.(groupName),'stable'); V=nan(numel(dst),numel(arch)); for i=1:numel(dst), for j=1:numel(arch), ix=T.(groupName)==dst(i)&T.Architecture==arch(j); V(i,j)=T.Magnitude(find(ix,1)); end,end
bar(categorical(dst),V); grid on; ylabel('Magnitude'); title(ttl); legend(arch,'Location','best');
end
function localRegionScatter(T,ttl)
cls=unique(T.MechanismClass,'stable'); hold on; mk={'o','s','^','x'};
for k=1:numel(cls)
    ix=T.MechanismClass==cls(k); label=strrep(char(cls(k)),'_',' ');
    scatter(T.SCR(ix),T.ControlValue(ix),65,mk{min(k,numel(mk))},'filled','DisplayName',label);
end
grid on; xlabel('SCR'); ylabel('Control value'); title(ttl); legend('Location','best','Interpreter','none');
end
function localSave(f,path), exportgraphics(f,[path '.png'],'Resolution',180); exportgraphics(f,[path '.pdf'],'ContentType','vector'); end
