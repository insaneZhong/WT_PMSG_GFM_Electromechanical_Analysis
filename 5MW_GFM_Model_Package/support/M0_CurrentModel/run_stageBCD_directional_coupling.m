function R = run_stageBCD_directional_coupling(outDir)
%RUN_STAGEBCD_DIRECTIONAL_COUPLING 阶段B-D统一执行入口。
% B: 无量纲双向扰动矩阵；C: alpha_dc责任连续分配；
% D: SCR/H/DVC的Pole-Path-Direction统一指标。
% 只保存三张摘要CSV、一份中文报告和三张机制图，不保存时序或模型副本。

if nargin<1 || isempty(outDir), outDir=fileparts(mfilename('fullpath')); end
figDir=fullfile(outDir,'StageBCD_Figures'); if ~isfolder(figDir), mkdir(figDir); end

% 阶段A是B-D的硬前置Gate。
stageAFile=fullfile(outDir,'StageA_ControlEquation_Path_Classification.csv');
assert(isfile(stageAFile),'缺少阶段A结果。'); Aaudit=readtable(stageAFile,'TextType','string');
assert(all(Aaudit.ConsistencyPASS),'阶段A一致性门未通过，禁止执行B-D。');

[models,base]=prepare_multimode_models();
fref=referenceTorsionFrequency(models); wref=2*pi*fref;
bases=normalizationBases(base.parameter_vector);

%% Gate B：同一无量纲定义下的2x2双向矩阵
B=buildStageB(models,wref,bases);
gateB=gateStageB(B,Aaudit);
writetable(B,fullfile(outDir,'StageB_Normalized_Bidirectional_Matrix.csv'),'Encoding','UTF-8');
makeStageBFigure(models,B,bases,fref,figDir);
if ~gateB.pass
    writeReport(fullfile(outDir,'StageBCD_Directional_Coupling_Report_CN.md'),Aaudit,B,table,table,gateB,struct,struct,bases,fref);
    error('StageB:GateFailed','Gate B失败：%s',gateB.reason);
end

%% Gate C：只连续分配DC-link调节责任，公共plant和GFM参数保持不变
[C,gateC,alphaModels]=buildStageC(models,base,wref,bases); %#ok<ASGLU>
writetable(C,fullfile(outDir,'StageC_AlphaDC_Directional_Transition.csv'),'Encoding','UTF-8');
makeStageCFigure(C,figDir);
if ~gateC.pass
    writeReport(fullfile(outDir,'StageBCD_Directional_Coupling_Report_CN.md'),Aaudit,B,C,table,gateB,gateC,struct,bases,fref);
    error('StageC:GateFailed','Gate C失败：%s',gateC.reason);
end

%% Gate D：统一Pole/Path/Direction指标
D=buildStageD(models,base,bases);
gateD=gateStageD(D);
writetable(D,fullfile(outDir,'StageD_Pole_Path_Direction_Summary.csv'),'Encoding','UTF-8');
makeStageDFigure(D,figDir);
writeReport(fullfile(outDir,'StageBCD_Directional_Coupling_Report_CN.md'),Aaudit,B,C,D,gateB,gateC,gateD,bases,fref);

R=struct('passed',gateB.pass&&gateC.pass&&gateD.pass,'gateB',gateB,'gateC',gateC,'gateD',gateD, ...
    'stageB',B,'stageC',C,'stageD',D,'normalization',bases,'reference_frequency_Hz',fref,'figure_directory',figDir);
end

function b=normalizationBases(p)
% 输入：机械转矩/Tbase，电网频率/omega0；输出：轴系相对转速/omega_m0，PCC功率/Sbase。
b=struct('Sbase_W',p(1),'Vdcbase_V',p(2),'omega_grid_base_radps',p(3), ...
    'omega_machine_base_radps',p(12),'torque_base_Nm',p(1)/p(12));
end

function f=referenceTorsionFrequency(models)
L=models{1}; M=multimode_modal_data(L.A,L.state_names); it=multimode_pick_torsion_mode(M);
f=abs(imag(M.lambda(it)))/(2*pi);
end

function T=buildStageB(models,w,b)
n=numel(models); T=table('Size',[n 16], ...
 'VariableTypes',{'string','double','double','double','double','double','double','double','double','double','double','double','double','string','string','logical'}, ...
 'VariableNames',{'Architecture','Frequency_Hz','Gmm_Re','Gmm_Im','Gmg_Re','Gmg_Im','Ggm_Re','Ggm_Im','Ggg_Re','Ggg_Im','C_GridToMachine','C_MachineToGrid','DirectionalIndex','DominantDirection','Normalization','FinitePASS'});
for k=1:n
    L=models{k}; G=normalizedMatrix(L,w,b); cmg=abs(G(1,2)); cgm=abs(G(2,1)); d=(cmg-cgm)/(cmg+cgm+eps);
    T.Architecture(k)=string(L.label); T.Frequency_Hz(k)=w/(2*pi);
    T{k,3:10}=[real(G(1,1)),imag(G(1,1)),real(G(1,2)),imag(G(1,2)),real(G(2,1)),imag(G(2,1)),real(G(2,2)),imag(G(2,2))];
    T.C_GridToMachine(k)=cmg; T.C_MachineToGrid(k)=cgm; T.DirectionalIndex(k)=d;
    T.DominantDirection(k)=directionLabel(d,cmg,cgm); T.Normalization(k)="dm=DeltaTm/Tb,dg=Deltaomega_grid/omega0,ym=Deltaomega_sh/omega_m0,yg=DeltaP_PCC/Sb";
    T.FinitePASS(k)=all(isfinite([real(G(:));imag(G(:))]));
end
end

function g=gateStageB(T,Aaudit)
igwt=find(contains(T.Architecture,'GWT'),1); imwt=find(contains(T.Architecture,'MWT'),1);
r1=T.C_GridToMachine(igwt)/max(T.C_GridToMachine(imwt),eps);
r2=T.C_MachineToGrid(imwt)/max(T.C_MachineToGrid(igwt),eps);
requiredA=all(Aaudit.ConsistencyPASS);
pass=all(T.FinitePASS)&&requiredA&&r1<1e-6&&r2<1e-6&&T.C_MachineToGrid(igwt)>1e-8&&T.C_GridToMachine(imwt)>1e-8;
reason=sprintf('GWT Grid->Machine/MWT=%.3g; MWT Machine->Grid/GWT=%.3g',r1,r2);
g=struct('pass',pass,'ratio_gwt_grid_to_machine',r1,'ratio_mwt_machine_to_grid',r2,'reason',reason);
end

function [T,g,cache]=buildStageC(models,base,w,b)
p=base.parameter_vector; alphas=unique([0,logspace(-4,-1,13),.2:.1:1]).'; n=numel(alphas);
imwt=find(cellfun(@(q)strcmpi(q.mode,'VSG'),models),1); igwt=find(cellfun(@(q)strcmpi(q.mode,'GFMGWT'),models),1);
Lmwt=models{imwt}; Lgwt=models{igwt}; x=Lmwt.x0;
% 为只研究责任分配而不同时改变同步环，整个alpha族统一采用已稳定的GWT mp。
% 这不会修改公共基准；alpha=1的参照也是“相同mp下的MWT责任极限”。
commonMp=Lgwt.flags.mpGwt; pC=p; pC(34)=commonMp; imq0=x(5); xiBias=x(6);
baseFlags=struct('imqRef0',imq0,'xiDcBias',xiBias,'KpGscDvc',5e3,'KiGscDvc',5e2,'mpGwt',commonMp);
% 责任分配端点的公平参照：统一H/mp，只改变DVC所在变流器。
gwtFlags=Lgwt.flags; gwtFlags.mpGwt=commonMp;
L0=multimode_linearize_control(Lgwt.x0,pC,'GFMGWT',gwtFlags);
L1=multimode_linearize_control(Lmwt.x0,pC,'VSG',struct);
T=table('Size',[n 14], ...
 'VariableTypes',{'double','double','double','double','double','double','double','double','double','double','double','double','string','logical'}, ...
 'VariableNames',{'alpha_dc','EquilibriumResidual','MaxRealPole','f_tor_Hz','zeta_tor','C_GridToMachine','C_MachineToGrid','TotalCrossCoupling','TwoWayCoupling','DirectionalIndex','EndpointMatrixError','EndpointPoleError','DominantDirection','Stable'});
cache=cell(n,1);
for k=1:n
    flags=baseFlags; flags.alphaDc=alphas(k); dx=source_aligned_rhs_control(x,pC,'ALPHADC',zeros(4,1),flags);
    L=multimode_linearize_control(x,pC,'ALPHADC',flags); cache{k}=L; S=modalSummary(L); G=normalizedMatrix(L,w,b);
    cmg=abs(G(1,2)); cgm=abs(G(2,1)); d=(cmg-cgm)/(cmg+cgm+eps);
    T.alpha_dc(k)=alphas(k); T.EquilibriumResidual(k)=normalizedResidualLocal(dx); T.MaxRealPole(k)=S.max_real;
    T.f_tor_Hz(k)=S.f; T.zeta_tor(k)=S.zeta; T.C_GridToMachine(k)=cmg; T.C_MachineToGrid(k)=cgm;
    T.TotalCrossCoupling(k)=hypot(cmg,cgm); T.TwoWayCoupling(k)=sqrt(cmg*cgm); T.DirectionalIndex(k)=d;
    T.EndpointMatrixError(k)=NaN; T.EndpointPoleError(k)=NaN; T.DominantDirection(k)=directionLabel(d,cmg,cgm); T.Stable(k)=S.max_real<0;
end
G0=normalizedMatrix(L0,w,b); G1=normalizedMatrix(L1,w,b); S0=modalSummary(L0); S1=modalSummary(L1);
G0cross=[0,abs(G0(1,2));abs(G0(2,1)),0]; G1cross=[0,abs(G1(1,2));abs(G1(2,1)),0];
T.EndpointMatrixError(1)=norm(matrixFromRow(T(1,:))-G0cross,'fro')/max(norm(G0cross,'fro'),eps);
T.EndpointMatrixError(end)=norm(matrixFromRow(T(end,:))-G1cross,'fro')/max(norm(G1cross,'fro'),eps);
T.EndpointPoleError(1)=abs(T.f_tor_Hz(1)-S0.f)/S0.f+abs(T.zeta_tor(1)-S0.zeta)/max(abs(S0.zeta),eps);
T.EndpointPoleError(end)=abs(T.f_tor_Hz(end)-S1.f)/S1.f+abs(T.zeta_tor(end)-S1.zeta)/max(abs(S1.zeta),eps);
crosses=any(T.DirectionalIndex(1:end-1).*T.DirectionalIndex(2:end)<=0);
step=max(abs(diff(T.DirectionalIndex)));
pass=max(T.EquilibriumResidual)<1e-8&&all(T.Stable)&&max(T.EndpointMatrixError([1 end]))<1e-5&&max(T.EndpointPoleError([1 end]))<1e-4&&crosses&&step<0.75;
g=struct('pass',pass,'common_mp',commonMp,'endpoint_matrix_error',max(T.EndpointMatrixError([1 end])), ...
    'endpoint_pole_error',max(T.EndpointPoleError([1 end])),'direction_crosses_zero',crosses,'max_direction_step',step, ...
    'reason',sprintf('endpoint matrix %.3g, endpoint pole %.3g, direction step %.3g',max(T.EndpointMatrixError([1 end])),max(T.EndpointPoleError([1 end])),step));
end

function T=buildStageD(models,base,b)
p0=base.parameter_vector; rows={};
% SCR：三架构；每个点重新求严格平衡。
scrs=[2 3 4 6 8 10];
for s=scrs
    p=p0; p(9)=p0(9)*4/s; p(10)=p0(10)*4/s;
    for k=1:numel(models)
        L0=models{k}; flags=L0.flags;
        try
            [xeq,meta]=solve_multimode_control_equilibrium(L0.x0,p,L0.mode,flags); L=multimode_linearize_control(xeq,p,L0.mode,flags);
            rows(end+1,:)=caseRow('SCR',L0.label,s,s/4,L,meta.residual_norm,b); %#ok<AGROW>
        catch ME
            rows(end+1,:)=failedRow('SCR',L0.label,s,s/4,ME.message); %#ok<AGROW>
        end
    end
end
% H：只对两种GFM架构有物理意义。
hf=[.5 .75 1 1.5 2];
for f=hf
    p=p0; p(33)=p0(33)*f;
    for k=1:numel(models)
        if strcmpi(models{k}.mode,'GFL'), continue; end
        L0=models{k}; L=multimode_linearize_control(L0.x0,p,L0.mode,L0.flags);
        rows(end+1,:)=caseRow('H',L0.label,p(33),f,L,normalizedResidualLocal(source_aligned_rhs_control(L0.x0,p,L0.mode,zeros(4,1),L0.flags)),b); %#ok<AGROW>
    end
end
% DVC：GFL/MWT缩放MSC-DVC，GWT缩放GSC-DVC。
df=[.5 .75 1 1.25 1.5 2];
for f=df
    for k=1:numel(models)
        L0=models{k}; p=p0; flags=L0.flags;
        if strcmpi(L0.mode,'GFMGWT')
            flags.KpGscDvc=L0.flags.KpGscDvc*f; flags.KiGscDvc=L0.flags.KiGscDvc*f;
        else
            p(25)=p0(25)*f; p(26)=p0(26)*f;
        end
        L=multimode_linearize_control(L0.x0,p,L0.mode,flags);
        rows(end+1,:)=caseRow('DVC',L0.label,f,f,L,normalizedResidualLocal(source_aligned_rhs_control(L0.x0,p,L0.mode,zeros(4,1),flags)),b); %#ok<AGROW>
    end
end
T=cell2table(rows,'VariableNames',{'Parameter','Architecture','Value','Factor','Status','EquilibriumResidual','f_tor_Hz','zeta_tor','PoleReal','MaxRealPole','C_GridToMachine','C_MachineToGrid','TotalCrossCoupling','DirectionalIndex','PoleIndex_pct','PathIndex_pct','DirectionChange','Mechanism'});
T.Parameter=string(T.Parameter); T.Architecture=string(T.Architecture); T.Status=string(T.Status); T.Mechanism=string(T.Mechanism);
% 相对于每个“参数-架构”的Factor=1基准统一计算三类指标。
groups=unique(T(:,{'Parameter','Architecture'}),'rows','stable');
for g=1:height(groups)
    ix=T.Parameter==groups.Parameter(g)&T.Architecture==groups.Architecture(g)&T.Status=="PASS";
    ib=find(ix&abs(T.Factor-1)<1e-12,1); if isempty(ib), continue; end
    z0=T.zeta_tor(ib); c0=T.TotalCrossCoupling(ib); d0=T.DirectionalIndex(ib);
    jj=find(ix).'; for r=jj
        T.PoleIndex_pct(r)=100*abs(T.zeta_tor(r)/z0-1); T.PathIndex_pct(r)=100*abs(T.TotalCrossCoupling(r)/max(c0,eps)-1); T.DirectionChange(r)=abs(T.DirectionalIndex(r)-d0);
        T.Mechanism(r)=mechanismLabel(T.PoleIndex_pct(r),T.PathIndex_pct(r),T.DirectionChange(r));
    end
end
T.SystemStable=T.MaxRealPole<0;
for r=1:height(T)
    if T.Status(r)=="PASS" && ~T.SystemStable(r), T.Mechanism(r)="ELECTRICAL_INSTABILITY + "+T.Mechanism(r); end
end
end

function row=caseRow(param,label,val,factor,L,res,b)
S=modalSummary(L); G=normalizedMatrix(L,2*pi*S.f,b); cmg=abs(G(1,2)); cgm=abs(G(2,1)); d=(cmg-cgm)/(cmg+cgm+eps);
row={char(param),char(label),val,factor,'PASS',res,S.f,S.zeta,S.pole_real,S.max_real,cmg,cgm,hypot(cmg,cgm),d,NaN,NaN,NaN,''};
end
function row=failedRow(param,label,val,factor,msg)
row={char(param),char(label),val,factor,['FAIL: ' char(msg)],NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,'FAILED'};
end

function g=gateStageD(T)
pass=all(T.Status=="PASS")&&all(isfinite(T.PoleIndex_pct(T.Status=="PASS")))&&all(isfinite(T.PathIndex_pct(T.Status=="PASS")))&&all(isfinite(T.DirectionChange(T.Status=="PASS")));
g=struct('pass',pass,'rows',height(T),'failed_rows',nnz(T.Status~="PASS"),'unstable_rows',nnz(T.Status=="PASS"&~T.SystemStable), ...
    'reason',sprintf('%d rows, %d failed, %d electrically unstable',height(T),nnz(T.Status~="PASS"),nnz(T.Status=="PASS"&~T.SystemStable)));
end

function G=normalizedMatrix(L,w,b)
iyM=find(strcmp(L.output_names,'omega_sh'),1); iyG=find(strcmp(L.output_names,'P_PCC'),1);
H=L.C*((1i*w*eye(size(L.A))-L.A)\L.B)+L.D;
% [ym;yg]=Gbar[dm;dg]，机械输入为DeltaTm，电网输入为Deltaomega_grid。
G=[H(iyM,1)*b.torque_base_Nm/b.omega_machine_base_radps, H(iyM,4)*b.omega_grid_base_radps/b.omega_machine_base_radps; ...
   H(iyG,1)*b.torque_base_Nm/b.Sbase_W, H(iyG,4)*b.omega_grid_base_radps/b.Sbase_W];
end

function S=modalSummary(L)
M=multimode_modal_data(L.A,L.state_names); it=multimode_pick_torsion_mode(M); lam=M.lambda(it); ev=eig(L.A); active=abs(ev)>1e-7;
S=struct('f',abs(imag(lam))/(2*pi),'zeta',-real(lam)/abs(lam),'pole_real',real(lam),'max_real',max(real(ev(active))));
end

function G=matrixFromRow(r)
% alpha表只存交叉项；端点Gate比较完整矩阵时对角项由0占位不合适，
% 因而本函数只构造交叉矩阵，调用方也应使用交叉参照。
G=[0,r.C_GridToMachine; r.C_MachineToGrid,0];
end

function r=normalizedResidualLocal(dx)
s=[1;1;1;1e4;1e4;1e4;1e4;1e4;1e6;5e8;5e8;1;1;1e4;1e4;1e4;1e4;1e6;1e6;1e6;1e6;1e6;1e6]; r=max(abs(dx)./s);
end

function s=directionLabel(d,cmg,cgm)
if max(cmg,cgm)<1e-10, s="BOTH_WEAK"; elseif d>0.2, s="GRID_TO_MACHINE_DOMINANT"; elseif d<-.2, s="MACHINE_TO_GRID_DOMINANT"; else, s="BALANCED_BIDIRECTIONAL"; end
end
function s=mechanismLabel(ipathPole,ipath,idir)
pole=ipathPole>5; path=ipath>5; dir=idir>.1;
if dir&&pole&&path, s="JOINT_POLE_PATH_DIRECTION";
elseif dir&&path, s="PATH_AND_DIRECTION_SHAPING";
elseif dir&&pole, s="POLE_AND_DIRECTION_SHAPING";
elseif dir, s="DIRECTIONAL_COUPLING_SHAPING";
elseif pole&&path, s="JOINT_POLE_PATH_SHAPING";
elseif pole, s="POLE_SHAPING_DOMINATED";
elseif path, s="PATH_SHAPING_DOMINATED";
else, s="BASELINE_OR_WEAK_CHANGE"; end
end

function makeStageBFigure(models,T,b,fref,dirOut)
f=figure('Visible','off','Color','w','Position',[100 100 1250 720]); tl=tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
for k=1:numel(models)
    nexttile; M=[abs(complex(T.Gmm_Re(k),T.Gmm_Im(k))),T.C_GridToMachine(k);T.C_MachineToGrid(k),abs(complex(T.Ggg_Re(k),T.Ggg_Im(k)))];
    imagesc(log10(max(M,1e-12))); axis image; colorbar; caxis([-12 3]); title(shortArch(T.Architecture(k)),'Interpreter','none');
    set(gca,'XTick',1:2,'XTickLabel',{'d_m','d_g'},'YTick',1:2,'YTickLabel',{'y_m','y_g'}); for i=1:2, for j=1:2, text(j,i,sprintf('%.2e',M(i,j)),'HorizontalAlignment','center','Color','w','FontWeight','bold'); end, end
end
freq=logspace(log10(.2),log10(10),250); cols=lines(numel(models));
nexttile([1 3]); hold on;
for k=1:numel(models)
    cmg=zeros(size(freq)); cgm=cmg; for q=1:numel(freq), G=normalizedMatrix(models{k},2*pi*freq(q),b); cmg(q)=abs(G(1,2)); cgm(q)=abs(G(2,1)); end
    loglog(freq,max(cmg,1e-14),'-','Color',cols(k,:),'LineWidth',1.8,'DisplayName',shortArch(models{k}.label)+" G->M");
    loglog(freq,max(cgm,1e-14),'--','Color',cols(k,:),'LineWidth',1.8,'DisplayName',shortArch(models{k}.label)+" M->G");
end
xline(fref,':k','f_{tor}','HandleVisibility','off'); set(gca,'XScale','log','YScale','log'); xlim([.2 10]); ylim([1e-14 1e3]); grid on; xlabel('Frequency (Hz)'); ylabel('Normalized coupling magnitude'); legend('Location','eastoutside','Interpreter','none'); title('Normalized directional transfer');
title(tl,'Stage B: normalized bidirectional disturbance matrix'); exportgraphics(f,fullfile(dirOut,'StageB_Normalized_Bidirectional_Matrix.png'),'Resolution',180); close(f);
end

function makeStageCFigure(T,dirOut)
f=figure('Visible','off','Color','w','Position',[100 100 1150 720]); tl=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
xplot=max(T.alpha_dc,1e-4);
nexttile; loglog(xplot,max(T.C_GridToMachine,1e-14),'-o','LineWidth',1.8); hold on; loglog(xplot,max(T.C_MachineToGrid,1e-14),'--s','LineWidth',1.8); grid on; xlabel('\alpha_{dc} (0 shown at 10^{-4})'); ylabel('Normalized coupling'); legend('Grid->Machine','Machine->Grid','Location','best');
nexttile; semilogx(xplot,T.DirectionalIndex,'-o','LineWidth',1.8); yline(0,':k'); ylim([-1.05 1.05]); grid on; xlabel('\alpha_{dc} (0 shown at 10^{-4})'); ylabel('Directional index');
nexttile; plot(T.alpha_dc,100*T.zeta_tor,'-o','LineWidth',1.8); grid on; xlabel('\alpha_{dc}'); ylabel('\zeta_{tor} (%)');
nexttile; plot(T.alpha_dc,T.MaxRealPole,'-o','LineWidth',1.8); yline(0,':r'); grid on; xlabel('\alpha_{dc}'); ylabel('Maximum real pole (1/s)');
title(tl,'Stage C: continuous DC-link regulation responsibility allocation'); exportgraphics(f,fullfile(dirOut,'StageC_AlphaDC_Directional_Transition.png'),'Resolution',180); close(f);
end

function makeStageDFigure(T,dirOut)
params=["SCR","H","DVC"]; f=figure('Visible','off','Color','w','Position',[60 60 1450 950]); tl=tiledlayout(3,3,'TileSpacing','compact','Padding','compact');
for i=1:3
    S=T(T.Parameter==params(i)&T.Status=="PASS",:); arch=unique(S.Architecture,'stable');
    nexttile; hold on; for k=1:numel(arch), Q=S(S.Architecture==arch(k),:); [x,ix]=sort(Q.Value); plot(x,Q.PoleIndex_pct(ix),'-o','Color',archColor(arch(k)),'LineWidth',1.5,'DisplayName',shortArch(arch(k))); end; grid on; ylabel('Pole index (%)'); title(params(i)); legend('Location','best','Interpreter','none');
    nexttile; hold on; for k=1:numel(arch), Q=S(S.Architecture==arch(k),:); [x,ix]=sort(Q.Value); plot(x,Q.PathIndex_pct(ix),'-o','Color',archColor(arch(k)),'LineWidth',1.5); end; grid on; ylabel('Path index (%)'); title(params(i));
    nexttile; hold on; for k=1:numel(arch), Q=S(S.Architecture==arch(k),:); [x,ix]=sort(Q.Value); plot(x,Q.DirectionalIndex(ix),'-o','Color',archColor(arch(k)),'LineWidth',1.5); end; yline(0,':k'); ylim([-1.05 1.05]); grid on; ylabel('Directional index'); title(params(i));
end
title(tl,'Stage D: Pole-Path-Direction unified parameter effects'); exportgraphics(f,fullfile(dirOut,'StageD_Pole_Path_Direction_Map.png'),'Resolution',180); close(f);
end

function writeReport(file,A,B,C,D,gB,gC,gD,b,fref)
fid=fopen(file,'w','n','UTF-8'); assert(fid>0); c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'# 阶段B–D：双向耦合、DC-link责任分配与统一机制报告\n\n');
fprintf(fid,'## 边界\n\n只使用已对齐的理想连续NL/SSM；公共plant和基准工作点未修改。没有引入EMT、PWM、离散、延迟、限幅、LVRT或MPPT/Pitch动态。阶段A的%d条解析/频响一致性检查全部通过。\n\n',height(A));
fprintf(fid,'## 归一化定义\n\n$\\bar d_m=\\Delta T_m/T_b$，$\\bar d_g=\\Delta\\omega_g/\\omega_0$，$\\bar y_m=\\Delta\\omega_{sh}/\\omega_{m0}$，$\\bar y_g=\\Delta P_{PCC}/S_b$。其中 $T_b=%.8g$ N·m，评价频率为 %.8g Hz。\n\n',b.torque_base_Nm,fref);
fprintf(fid,'双向矩阵为 $[\\bar y_m,\\bar y_g]^T=\\mathbf G_{bi}[\\bar d_m,\\bar d_g]^T$；方向指标 $D_{dir}=(|G_{mg}|-|G_{gm}|)/(|G_{mg}|+|G_{gm}|)$。正值表示Grid→Machine占优，负值表示Machine→Grid占优；本文不使用“non-reciprocity”表述。\n\n');
fprintf(fid,'## Gate B：%s\n\n%s。\n\n',passfail(gB.pass),gB.reason); writeTable(fid,B); fprintf(fid,'\n');
if ~isempty(C)
    across=interp1(C.DirectionalIndex,C.alpha_dc,0,'linear','extrap');
    fprintf(fid,'## Gate C：%s\n\n%s。$\\alpha_{dc}=0$为统一GFM参数下的GWT责任极限，$\\alpha_{dc}=1$为MWT责任极限；中间值只分配DC-link调节作用，不插值plant。方向指标过零点约为 $\\alpha_{dc}=%.5g$，全扫描区间均稳定。\n\n',passfail(gC.pass),gC.reason,across); writeTable(fid,C); fprintf(fid,'\n');
end
if ~isempty(D)
    fprintf(fid,'## Gate D：%s\n\n%s。Pole与Path变化阈值均取5%%，方向指标变化阈值取0.1；这些阈值用于机制分区，不是稳定性标准。\n\n',passfail(gD.pass),gD.reason);
    fprintf(fid,'机制计数：\n\n'); U=groupsummary(D(D.Status=="PASS",:),{'Parameter','Mechanism'}); writeTable(fid,U); fprintf(fid,'\n');
end
fprintf(fid,'## 结论边界\n\n1. `STRUCTURAL_ZERO`只用于阶段A已由方程、可达性和偏移工作点共同确认的路径。\n2. 阶段B–D的数值描述是方向依赖扰动传递，不代表互易网络意义上的非互易性。\n3. $\\alpha_{dc}$是机理参数，不是已经可直接部署的控制器；若中间区出现右半平面极点，应报告稳定区间而不是强行整定。\n4. SCR、H和DVC可以同时影响Pole、Path和Direction，但控制架构首先决定通道是否可达，参数主要在可达通道内改变幅值、相位和模态投影。\n');
end

function writeTable(fid,T)
vars=T.Properties.VariableNames; fprintf(fid,'|'); for k=1:numel(vars), fprintf(fid,'%s|',vars{k}); end; fprintf(fid,'\n|'); for k=1:numel(vars), fprintf(fid,'---|'); end; fprintf(fid,'\n');
for r=1:height(T), fprintf(fid,'|'); for k=1:numel(vars), v=T{r,k}; if iscell(v),v=v{1};end; if isstring(v)||ischar(v)||islogical(v),s=char(string(v));else,s=num2str(v,7);end; fprintf(fid,'%s|',s); end; fprintf(fid,'\n'); end
end
function s=passfail(v), if v,s='PASS';else,s='FAIL';end,end
function s=shortArch(a)
a=string(a); if contains(a,'GWT'),s="GFM-GWT";elseif contains(a,'MWT'),s="GFM-MWT";else,s="GFL";end
end
function c=archColor(a)
s=shortArch(a); if s=="GFL",c=[0 .447 .741];elseif s=="GFM-GWT",c=[.85 .325 .098];else,c=[.929 .694 .125];end
end
