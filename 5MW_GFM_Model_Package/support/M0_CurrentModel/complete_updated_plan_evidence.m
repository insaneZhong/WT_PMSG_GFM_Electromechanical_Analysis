function out=complete_updated_plan_evidence(varargin)
%COMPLETE_UPDATED_PLAN_EVIDENCE
% 按调整后的理想连续平均模型计划，补齐三项证据：
% 1) G_Te,omega_g(jw) 反馈复转矩扫描；
% 2) 极点变化与网侧扰动激励比的二维归类；
% 3) 轴系共轭极点的单模态响应重构。
%
% 只读取/生成精简汇总文件和图片，不保存任何原始长时序。
% EMT、PWM、离散控制和限幅不在本入口内执行。

ip=inputParser;
ip.addParameter('SaveFigures',true,@(x)islogical(x)&&isscalar(x));
ip.addParameter('SaveSummary',true,@(x)islogical(x)&&isscalar(x));
ip.addParameter('RunNonlinearReconstruction',true,@(x)islogical(x)&&isscalar(x));
ip.addParameter('ReconstructionScale',0.1,@(x)isnumeric(x)&&isscalar(x)&&x>0&&x<=1);
ip.addParameter('Visible','off',@(x)ischar(x)||isstring(x));
ip.parse(varargin{:}); opt=ip.Results;

here=fileparts(mfilename('fullpath')); addpath(here);
figDir=fullfile(here,'Figures_Disturbance_Path');
if ~exist(figDir,'dir'), mkdir(figDir); end
oldVis=get(0,'DefaultFigureVisible'); cleanup=onCleanup(@()set(0,'DefaultFigureVisible',oldVis)); %#ok<NASGU>
set(0,'DefaultFigureVisible',char(opt.Visible));

% 重新在内存中取得同一严格工作点及三种控制的 A/B/模态数据。
M=analyze_modal_residue_decomposition('SaveSummary',false);
controls={"GFL (ideal PLL)","Droop-GFM","VSG-GFM"};
colors=[0.0000 0.4470 0.6980; 0.0000 0.6196 0.4509; 0.8353 0.3686 0.0000];

%% A. 反馈复转矩 G_Te,omega_g(jw)
freq=logspace(log10(0.2),log10(10),240).'; w=2*pi*freq;
records=struct('Control',{},'Frequency_Hz',{},'Gain',{},'Gain_dB',{},'Phase_deg',{},'De',{},'Ke',{});
feedback=cell(3,1);
for k=1:3
    A=M.models{k}.A; p=M.models{k}.p;
    [G,De,Ke]=feedbackTorqueScan(A,p,w);
    feedback{k}=struct('G',G,'De',De,'Ke',Ke);
    for q=1:numel(freq)
        records(end+1)=struct('Control',controls{k},'Frequency_Hz',freq(q), ...
            'Gain',abs(G(q)),'Gain_dB',20*log10(max(abs(G(q)),eps)), ...
            'Phase_deg',angle(G(q))*180/pi,'De',De(q),'Ke',Ke(q)); %#ok<AGROW>
    end
end
feedbackTable=struct2table(records);
if opt.SaveSummary, writetable(feedbackTable,fullfile(here,'Feedback_Torque_Scan.csv')); end

% 轴系频率处单点表，明确把反馈复转矩与外部扰动传递分开。
ftor=mean([M.models{1}.f M.models{2}.f M.models{3}.f]);
feedbackPoint=struct('Control',{},'f_tor_Hz',{},'G_Te_omega_g_real',{}, ...
    'G_Te_omega_g_imag',{},'De',{},'Ke',{});
for k=1:3
    Gk=feedback{k}.G; Dek=feedback{k}.De; Kek=feedback{k}.Ke;
    feedbackPoint(end+1)=struct('Control',controls{k},'f_tor_Hz',ftor, ...
        'G_Te_omega_g_real',interp1(freq,real(Gk),ftor,'pchip'), ...
        'G_Te_omega_g_imag',interp1(freq,imag(Gk),ftor,'pchip'), ...
        'De',interp1(freq,Dek,ftor,'pchip'),'Ke',interp1(freq,Kek,ftor,'pchip')); %#ok<AGROW>
end
feedbackPointTable=struct2table(feedbackPoint);
if opt.SaveSummary, writetable(feedbackPointTable,fullfile(here,'Feedback_Torque_At_Torsional_Frequency.csv')); end

if opt.SaveFigures
    fig=figure('Color','w','Position',[80 80 1180 800]); tl=tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
    ax=nexttile(tl,1); hold(ax,'on');
    for k=1:3, semilogx(ax,freq,20*log10(max(abs(feedback{k}.G),eps)),'Color',colors(k,:),'LineWidth',1.5,'DisplayName',controls{k}); end
    xline(ax,ftor,'k:','HandleVisibility','off'); ylabel(ax,'Magnitude (dB)'); title(ax,'Feedback transfer magnitude'); grid(ax,'on'); legend(ax,'Location','best','Box','off'); hold(ax,'off');
    ax=nexttile(tl,2); hold(ax,'on');
    for k=1:3, semilogx(ax,freq,angle(feedback{k}.G)*180/pi,'Color',colors(k,:),'LineWidth',1.5,'DisplayName',controls{k}); end
    xline(ax,ftor,'k:','HandleVisibility','off'); ylabel(ax,'Phase (deg)'); title(ax,'Feedback transfer phase'); grid(ax,'on'); legend(ax,'Location','best','Box','off'); hold(ax,'off');
    ax=nexttile(tl,3); hold(ax,'on');
    for k=1:3, semilogx(ax,freq,feedback{k}.De,'Color',colors(k,:),'LineWidth',1.5,'DisplayName',controls{k}); end
    yline(ax,0,'k:','HandleVisibility','off'); xline(ax,ftor,'k:','HandleVisibility','off'); xlabel(ax,'Frequency (Hz)'); ylabel(ax,'D_e'); title(ax,'Real feedback coefficient'); grid(ax,'on'); legend(ax,'Location','best','Box','off'); hold(ax,'off');
    ax=nexttile(tl,4); hold(ax,'on');
    for k=1:3, semilogx(ax,freq,feedback{k}.Ke,'Color',colors(k,:),'LineWidth',1.5,'DisplayName',controls{k}); end
    xline(ax,ftor,'k:','HandleVisibility','off'); xlabel(ax,'Frequency (Hz)'); ylabel(ax,'K_e'); title(ax,'Equivalent synchronizing coefficient'); grid(ax,'on'); legend(ax,'Location','best','Box','off'); hold(ax,'off');
    sgtitle(tl,'Feedback torque transfer versus external disturbance transfer');
    saveFigure(fig,fullfile(figDir,'Fig05_Feedback_Torque_Bode'),true); close(fig);
end

%% B. 极点—扰动通道二维归类（使用已有一维扫描汇总，不重新扫参）
S=readtable(fullfile(here,'Parameter_Sensitivity_Summary.csv'),'VariableNamingRule','preserve');
rec=struct('Parameter',{},'Value',{},'SCR',{},'P_MW',{},'Control',{},'GridDisturbance',{}, ...
    'DeltaZeta_pole',{},'Gamma_grid',{},'log10Gamma_grid',{},'Region',{});
for r=1:height(S)
    for k=2:3 % 只比较两个 GFM，相对同一行 GFL
        if k==2, ctrl=controls{2}; ga=S.Gamma_angle_Droop(r); gf=S.Gamma_freq_Droop(r); zg=S.zeta_Droop(r);
        else, ctrl=controls{3}; ga=S.Gamma_angle_VSG(r); gf=S.Gamma_freq_VSG(r); zg=S.zeta_VSG(r); end
        dz=zg-S.zeta_GFL(r);
        rec(end+1)=makeClassRecord(S,r,ctrl,"Grid angle",dz,ga); %#ok<AGROW>
        rec(end+1)=makeClassRecord(S,r,ctrl,"Grid frequency",dz,gf); %#ok<AGROW>
    end
end
classTable=struct2table(rec);
if opt.SaveSummary, writetable(classTable,fullfile(here,'Pole_Excitation_Classification.csv')); end

if opt.SaveFigures
    fig=figure('Color','w','Position',[100 100 1180 520]); tl=tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
    for j=1:2
        ax=nexttile(tl,j); hold(ax,'on');
        % 采用明确字符串，避免不同 MATLAB 版本对逻辑字符串拼接的差异。
        if j==1, dname="Grid angle"; else, dname="Grid frequency"; end
        for k=2:3
            ix=classTable.Control==controls{k} & classTable.GridDisturbance==dname;
            scatter(ax,100*classTable.DeltaZeta_pole(ix),classTable.log10Gamma_grid(ix),45, ...
                'MarkerFaceColor',colors(k,:),'MarkerEdgeColor','k','DisplayName',controls{k});
        end
        xline(ax,0,'k:','HandleVisibility','off'); xlabel(ax,'\Delta\zeta_{pole} (percentage points)'); ylabel(ax,'log_{10} \Gamma_{grid}'); title(ax,dname); grid(ax,'on'); legend(ax,'Location','best','Box','off'); hold(ax,'off');
    end
    sgtitle(tl,'Pole shaping versus disturbance-channel shaping');
    saveFigure(fig,fullfile(figDir,'Fig06_Pole_Excitation_Classification'),true); close(fig);
end

%% C. 轴系共轭极点单模态响应重构，并检查低频多模态缺口
if opt.RunNonlinearReconstruction
    names={"Grid angle","Grid frequency"}; modeNames={'GFL','DROOP','VSG'}; dIndex=[3 4]; tspan=linspace(0,10,5001); opts=odeset('RelTol',1e-7,'AbsTol',1e-8,'MaxStep',0.01); dScale=opt.ReconstructionScale;
    rec=struct('Control',{},'Disturbance',{},'DisturbanceScale',{},'f_tor_Hz',{},'NL_peak_omega',{},'AxisMode_peak_omega',{},'SlowModes_peak_omega',{},'FullSSM_peak_omega',{}, ...
        'NL_peak_Tsh',{},'AxisMode_peak_Tsh',{},'SlowModes_peak_Tsh',{},'FullSSM_peak_Tsh',{}, ...
        'AxisMode_NRMSE_omega',{},'SlowModes_NRMSE_omega',{},'FullSSM_NRMSE_omega',{}, ...
        'AxisMode_NRMSE_Tsh',{},'SlowModes_NRMSE_Tsh',{},'FullSSM_NRMSE_Tsh',{}, ...
        'AxisMode_Correlation_omega',{},'SlowModes_Correlation_omega',{},'FullSSM_Correlation_omega',{}, ...
        'AxisMode_Correlation_Tsh',{},'SlowModes_Correlation_Tsh',{},'FullSSM_Correlation_Tsh',{},'Status',{});
    plotData=cell(2,3);
    Cw=zeros(1,23); Cw(2)=1; Cw(3)=-1; Ct=zeros(1,23); Ct(1)=M.models{1}.p(21); Ct(2)=M.models{1}.p(22); Ct(3)=-M.models{1}.p(22);
    for j=1:2
        d=zeros(4,1); d(dIndex(j))=dScale*M.dBase(dIndex(j));
        for k=1:3
            Q=M.models{k};
            [t,xx]=ode15s(@(tt,zz)source_aligned_rhs_control(zz,Q.p,modeNames{k},d),tspan,Q.x,opts); %#ok<ASGLU>
            yy=zeros(numel(t),6); for ii=1:numel(t), yy(ii,:)=source_aligned_outputs_control(xx(ii,:).',Q.p,modeNames{k},d).'; end
            yn=yy(:,5)-yy(end,5); yt=yy(:,4)-yy(end,4);
            bStep=dScale*Q.Bbar(:,dIndex(j));
            [yAxisW,yAxisT]=modalResponse(Q.A,bStep,Cw,Ct,t,'axis',Q.lambda);
            [ySlowW,ySlowT]=modalResponse(Q.A,bStep,Cw,Ct,t,'slow',5);
            [~,dxFull]=ode15s(@(tt,zz)Q.A*zz+bStep,tspan,zeros(23,1),opts);
            yFullW=dxFull*Cw.'; yFullT=dxFull*Ct.'; yFullW=yFullW-yFullW(end); yFullT=yFullT-yFullT(end);
            eAxisW=norm(yn-yAxisW)/max(norm(yn),eps); eSlowW=norm(yn-ySlowW)/max(norm(yn),eps); eFullW=norm(yn-yFullW)/max(norm(yn),eps);
            eAxisT=norm(yt-yAxisT)/max(norm(yt),eps); eSlowT=norm(yt-ySlowT)/max(norm(yt),eps); eFullT=norm(yt-yFullT)/max(norm(yt),eps);
            cAxisW=safeCorr(yn,yAxisW); cSlowW=safeCorr(yn,ySlowW); cFullW=safeCorr(yn,yFullW); cAxisT=safeCorr(yt,yAxisT); cSlowT=safeCorr(yt,ySlowT); cFullT=safeCorr(yt,yFullT);
            if eFullW<0.1 && eFullT<0.1 && cFullW>0.95 && cFullT>0.95, status="PASS(full SSM)"; else, status="REVIEW"; end
            rec(end+1)=struct('Control',controls{k},'Disturbance',names{j},'DisturbanceScale',dScale,'f_tor_Hz',Q.f, ...
                'NL_peak_omega',max(abs(yn)),'AxisMode_peak_omega',max(abs(yAxisW)),'SlowModes_peak_omega',max(abs(ySlowW)),'FullSSM_peak_omega',max(abs(yFullW)), ...
                'NL_peak_Tsh',max(abs(yt)),'AxisMode_peak_Tsh',max(abs(yAxisT)),'SlowModes_peak_Tsh',max(abs(ySlowT)),'FullSSM_peak_Tsh',max(abs(yFullT)), ...
                'AxisMode_NRMSE_omega',eAxisW,'SlowModes_NRMSE_omega',eSlowW,'FullSSM_NRMSE_omega',eFullW, ...
                'AxisMode_NRMSE_Tsh',eAxisT,'SlowModes_NRMSE_Tsh',eSlowT,'FullSSM_NRMSE_Tsh',eFullT, ...
                'AxisMode_Correlation_omega',cAxisW,'SlowModes_Correlation_omega',cSlowW,'FullSSM_Correlation_omega',cFullW, ...
                'AxisMode_Correlation_Tsh',cAxisT,'SlowModes_Correlation_Tsh',cSlowT,'FullSSM_Correlation_Tsh',cFullT,'Status',status); %#ok<AGROW>
            plotData{j,k}=struct('t',t,'yn',yn,'yAxisW',yAxisW,'ySlowW',ySlowW,'yFullW',yFullW,'yt',yt,'yAxisT',yAxisT,'ySlowT',ySlowT,'yFullT',yFullT);
        end
    end
    singleTable=struct2table(rec);
    if opt.SaveSummary, writetable(singleTable,fullfile(here,'Single_Mode_Reconstruction_Summary.csv')); end
    if opt.SaveFigures
        fig=figure('Color','w','Position',[80 80 1200 780]); tl=tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
        for j=1:2
            ax=nexttile(tl,j); hold(ax,'on');
            for k=1:3, q=plotData{j,k}; plot(ax,q.t,q.yn,'Color',colors(k,:),'LineWidth',1.1,'DisplayName',[char(controls{k}) ' NL']); plot(ax,q.t,q.yAxisW,'--','Color',colors(k,:),'LineWidth',1.0,'HandleVisibility','off'); plot(ax,q.t,q.ySlowW,'-.','Color',colors(k,:),'LineWidth',1.0,'HandleVisibility','off'); plot(ax,q.t,q.yFullW,':','Color',colors(k,:),'LineWidth',1.0,'HandleVisibility','off'); end
            xlabel(ax,'Time (s)'); ylabel(ax,'\Delta\omega_{sh} (rad/s)'); title(ax,char(names{j})); grid(ax,'on'); legend(ax,'Location','best','Box','off'); hold(ax,'off');
        end
        for j=1:2
            ax=nexttile(tl,2+j); hold(ax,'on');
            for k=1:3, q=plotData{j,k}; plot(ax,q.t,q.yt,'Color',colors(k,:),'LineWidth',1.1,'DisplayName',[char(controls{k}) ' NL']); plot(ax,q.t,q.yAxisT,'--','Color',colors(k,:),'LineWidth',1.0,'HandleVisibility','off'); plot(ax,q.t,q.ySlowT,'-.','Color',colors(k,:),'LineWidth',1.0,'HandleVisibility','off'); plot(ax,q.t,q.yFullT,':','Color',colors(k,:),'LineWidth',1.0,'HandleVisibility','off'); end
            xlabel(ax,'Time (s)'); ylabel(ax,'\Delta T_{sh} (N m)'); title(ax,char(names{j})); grid(ax,'on'); legend(ax,'Location','best','Box','off'); hold(ax,'off');
        end
        sgtitle(tl,sprintf('NL (solid), axis mode (dashed), slow modes (dash-dot), full SSM (dotted), disturbance scale %.2g',dScale));
        saveFigure(fig,fullfile(figDir,'Fig07_Single_Mode_Reconstruction'),true); close(fig);
    end
else
    singleTable=table();
end

out=struct('feedback_scan',feedbackTable,'feedback_at_torsional_frequency',feedbackPointTable, ...
    'pole_excitation_classification',classTable,'single_mode_reconstruction',singleTable);
fprintf('UPDATED_PLAN_FEEDBACK_ROWS=%d\n',height(feedbackTable));
fprintf('UPDATED_PLAN_CLASSIFICATION_ROWS=%d\n',height(classTable));
if opt.RunNonlinearReconstruction, fprintf('UPDATED_PLAN_SINGLE_MODE_ROWS=%d\n',height(singleTable)); end
end

function [G,De,Ke]=feedbackTorqueScan(A,p,w)
idx=4:23; C=zeros(1,numel(idx)); C(2)=p(18); B=A(idx,3); Aee=A(idx,idx); G=zeros(numel(w),1);
for k=1:numel(w), G(k)=C*((1i*w(k)*eye(numel(idx))-Aee)\B); end
De=real(G); Ke=-w.*imag(G);
end

function [yw,yt]=modalResponse(A,b,Cw,Ct,t,kind,lambdaAxis)
% 用左右特征向量重构阶跃响应的模态分量；b 已包含实际扰动幅值。
[V,D]=eig(A); ev=diag(D); [W,Dl]=eig(A'); el=diag(Dl); yw=zeros(numel(t),1); yt=zeros(numel(t),1);
for i=1:numel(ev)
    lam=ev(i); include=false;
    if strcmp(kind,'axis')
        include=abs(lam-lambdaAxis)<1e-6 || abs(lam-conj(lambdaAxis))<1e-6;
    else
        if abs(imag(lam))<1e-7
            include=abs(real(lam))<=100 && real(lam)<-1e-8;
        elseif imag(lam)>1e-7
            include=abs(imag(lam))/(2*pi)<=5;
        end
    end
    if ~include || abs(lam)<1e-8, continue; end
    [~,j]=min(abs(el-conj(lam))); v=V(:,i); w0=W(:,j); a=w0'*v; w0=w0/conj(a); rW=(Cw*v)*(w0'*b); rT=(Ct*v)*(w0'*b);
    if imag(lam)>1e-7
        yw=yw+2*real((rW/lam).*exp(lam*t)); yt=yt+2*real((rT/lam).*exp(lam*t));
    elseif abs(imag(lam))<1e-7
        yw=yw+real((rW/lam).*exp(lam*t)); yt=yt+real((rT/lam).*exp(lam*t));
    end
end
end

function s=makeClassRecord(T,r,ctrl,dname,dz,gamma)
if abs(dz)<=1e-8 && gamma>1+1e-8, region="I: pole unchanged + excitation changed";
elseif abs(dz)<=1e-8 && gamma<1-1e-8, region="I-: pole unchanged + excitation reduced";
elseif dz< -1e-8 && gamma>1+1e-8, region="II: pole damping reduced + excitation increased";
elseif dz< -1e-8 && gamma<1-1e-8, region="III: pole damping reduced + excitation reduced";
elseif dz>1e-8 && gamma>1+1e-8, region="IV: pole damping improved + excitation increased";
elseif dz>1e-8 && gamma<1-1e-8, region="V: pole damping improved + excitation reduced";
elseif abs(dz)<=1e-8 && abs(gamma-1)<=1e-8, region="0: unchanged";
else, region="Review"; end
s=struct('Parameter',string(T.Parameter(r)),'Value',T.Value(r),'SCR',T.SCR(r),'P_MW',T.P_MW(r), ...
    'Control',ctrl,'GridDisturbance',dname,'DeltaZeta_pole',dz,'Gamma_grid',gamma, ...
    'log10Gamma_grid',log10(max(gamma,eps)),'Region',region);
end

function c=safeCorr(a,b)
cc=corrcoef(a(:),b(:)); if numel(cc)>=4 && isfinite(cc(1,2)), c=cc(1,2); else, c=NaN; end
end

function saveFigure(fig,base,savePDF)
exportgraphics(fig,[base '.png'],'Resolution',300);
if savePDF, exportgraphics(fig,[base '.pdf'],'ContentType','vector'); end
end
