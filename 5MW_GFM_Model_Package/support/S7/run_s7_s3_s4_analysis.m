function R=run_s7_s3_s4_analysis(varargin)
%RUN_S7_S3_S4_ANALYSIS  S7-3九点数字筛查及S7-4 Pole/Excitation复核。
%
% 本程序使用已通过V2条件验证的S7A离散平均映射，对Ts/Ts0=[0.5 1 2]
% 与tau/Ts=[0 0.5 1]做9点筛查。只写入摘要CSV、中文报告和一张图。
% 由于当前S7A是Reference Digital Implementation，结果不得表述为遗留
% C控制器或EMT的最终证据。

ip=inputParser; ip.addParameter('SaveFigure',true,@(x)islogical(x)&&isscalar(x)); ip.parse(varargin{:}); o=ip.Results;
here=fileparts(mfilename('fullpath')); idealDir=fileparts(here); addpath(idealDir); addpath(here);
S=load(fullfile(idealDir,'M0_5MW_Aligned_Workpoint_and_SSM.mat'),...
    'params','operating_point','A','B','C','D');
[pvec,~]=m0_pack_parameters(S.params,S.operating_point); x0=S.operating_point.x0(:); u0=zeros(6,1);
cmd0=s7a_discrete_average_core('commands',x0,u0,pvec); z0=[x0;cmd0;cmd0];
Ts0=S.params.controller_Ts_s; fRef=2.48;
% 连续M0基线，仅用于Pole/Residue相对变化，不重复生成旧结果。
[fc,zc,lc,kc,partC]=findMode(S.A,[1 2 3],fRef,0);
resC_m=modalResidue(S.C(11,:),S.B(:,1),S.A,kc);
resC_g=modalResidue(S.C(11,:),S.B(:,4),S.A,kc);
Gcont=feedbackProxy(S.C(7,:),S.A,3,1i*2*pi*fc); %#ok<NASGU>

tsRatios=[0.5 1 2]; delayRatios=[0 0.5 1]; rows=struct([]); modes=struct([]); rr=0; mm=0;
for it=1:numel(tsRatios)
    for id=1:numel(delayRatios)
        Ts=Ts0*tsRatios(it); tau=Ts*delayRatios(id);
        cmd=s7a_discrete_average_core('commands',x0,u0,pvec); z=[x0;cmd;cmd];
        zr=s7a_discrete_average_core('step',z,u0,pvec,Ts,tau);
        [Ad,Bd,Cd,Dd]=discreteJacobian(z,u0,pvec,Ts,tau); %#ok<ASGLU>
        ev=eig(Ad); [fd,zd,ld,kd,partD]=findMode(Ad,[1 2 3],fRef,Ts);
        allStable=all(abs(ev)<1+1e-8);
        if Ts==0, zz=1; else, zz=exp(1i*2*pi*fd*Ts); end
        % 离散一步映射的 resolvent 与输入残差含 Ts 尺度：
        % (zI-Ad)^-1 ~ Ts^-1(sI-A)^-1，Bd ~ Ts*B。先换算为
        % continuous-equivalent 量，再与M0连续SSM比较，避免1/Ts假差异。
        Gd=Ts*feedbackProxy(Cd(7,:),Ad,3,zz); De=real(Gd); Ke=-(2*pi*fd)*imag(Gd);
        resD_m=modalResidue(Cd(11,:),Bd(:,1),Ad,kd)/Ts; resD_g=modalResidue(Cd(11,:),Bd(:,4),Ad,kd)/Ts;
        dcMode=findGroupMode(Ad,[6 9],fRef,Ts,[fd]); syncMode=findGroupMode(Ad,[12 13],fRef,Ts,[fd]); gscMode=findGroupMode(Ad,[14:17 20:23],fRef,Ts,[fd]);
        fp=max(abs(zr-z)); frn=norm(zr-z)/max(1,norm(z));
        poleDeltaF=fd-fc; poleDeltaZ=zd-zc; excM=abs(resD_m)/max(abs(resC_m),eps)-1; excG=abs(resD_g)/max(abs(resC_g),eps)-1;
        rr=rr+1; rrow=struct('Ts_ratio',tsRatios(it),'delay_ratio',delayRatios(id),'Ts_s',Ts,'tau_s',tau,...
            'fixed_residual_max',fp,'fixed_residual_norm',frn,'all_poles_stable',allStable,...
            'tor_freq_Hz',fd,'tor_zeta',zd,'tor_sigma_sinv',real(ld),...
            'tor_De_proxy',De,'tor_Ke_proxy',Ke,'tor_mech_part',partD.mech,'tor_dc_part',partD.dc,...
            'tor_sync_part',partD.sync,'tor_gsc_part',partD.gsc,'res_tor_mech_mag',abs(resD_m),...
            'res_tor_grid_mag',abs(resD_g),'pole_delta_freq_Hz',poleDeltaF,'pole_delta_zeta',poleDeltaZ,...
            'excitation_delta_mech_ratio',excM,'excitation_delta_grid_ratio',excG,...
            'dc_mode_Hz',dcMode.f,'sync_mode_Hz',syncMode.f,'gsc_mode_Hz',gscMode.f,...
            'classification',classify(poleDeltaF,poleDeltaZ,excM,excG),'status','REFERENCE_DIGITAL_SCREENING');
        if isempty(rows), rows=rrow; else, rows(end+1)=rrow; end %#ok<AGROW>
        mm=mm+1; mrow=struct('Ts_ratio',tsRatios(it),'delay_ratio',delayRatios(id),...
            'tor_freq_Hz',fd,'tor_zeta',zd,'tor_sigma_sinv',real(ld),'dc_mode_Hz',dcMode.f,...
            'sync_mode_Hz',syncMode.f,'gsc_mode_Hz',gscMode.f,'mech_part',partD.mech,'dc_part',partD.dc,...
            'sync_part',partD.sync,'gsc_part',partD.gsc,'mac_to_continuous','NOT_COMPUTED_CROSS_ORDER');
        if isempty(modes), modes=mrow; else, modes(end+1)=mrow; end %#ok<AGROW>
    end
end

out1=fullfile(here,'S7_S3_Reference_Digital_9Point.csv'); localWriteCsv(out1,rows);
out2=fullfile(here,'S7_S3_Modal_Identity_Summary.csv'); localWriteCsv(out2,modes);
out3=fullfile(here,'S7_S4_Pole_Excitation_Decomposition.csv');
decomp=makeDecomposition(rows,fc,zc,resC_m,resC_g); localWriteCsv(out3,decomp);
fig=''; if o.SaveFigure, fig=saveFigure(rows,here); end
report=fullfile(here,'S7_S3_S4_Report_CN.md'); writeReport(report,out1,out2,out3,fig,rows,fc,zc,resC_m,resC_g);
R=struct('status','CONDITIONAL_REFERENCE_DIGITAL_SCREENING','nine_point_csv',out1,'modal_csv',out2,'decomposition_csv',out3,'report',report,'figure',fig,'rows',rows);
fprintf('S7-3/S7-4完成：摘要报告 %s\n',report);
end

function [f,z,lam,k,part]=findMode(A,groups,fRef,Ts)
if nargin<4||isempty(Ts)||Ts==0, lamAll=eig(A); V=[]; else, [V,D]=eig(A); lamAll=log(diag(D))/Ts; end
if isempty(V), [V,D]=eig(A); if nargin>=4&&~isempty(Ts)&&Ts~=0, lamAll=log(diag(D))/Ts; else, lamAll=diag(D); end, end
fAll=abs(imag(lamAll))/(2*pi); zAll=-real(lamAll)./max(abs(lamAll),eps);
sel=find(imag(lamAll)>1e-5 & fAll>0.5 & fAll<6);
if isempty(sel), [~,k]=min(abs(fAll-fRef)); else
    score=zeros(numel(sel),1); den=sum(abs(V).^2,1); den=max(den,eps);
    for q=1:numel(sel), mech=sum(abs(V(groups,sel(q))).^2)/den(sel(q)); score(q)=abs(fAll(sel(q))-fRef)-0.15*mech; end
    [~,ii]=min(score); k=sel(ii);
end
f=fAll(k); z=zAll(k); lam=lamAll(k); den=sum(abs(V(:,k)).^2); part=struct('mech',sum(abs(V([1 2 3],k)).^2)/den,'dc',sum(abs(V([6 9],k)).^2)/den,'sync',sum(abs(V([12 13],k)).^2)/den,'gsc',sum(abs(V([14:17 20:23],k)).^2)/den);
end

function M=findGroupMode(A,group,fRef,Ts,excludeF)
if nargin<5, excludeF=[]; end
[V,D]=eig(A); d=diag(D); if Ts~=0, lam=log(d)/Ts; else, lam=d; end; fAll=abs(imag(lam))/(2*pi); zAll=-real(lam)./max(abs(lam),eps); den=sum(abs(V).^2,1); den=max(den,eps); sel=find(imag(lam)>1e-5 & fAll>0.05 & fAll<30);
if ~isempty(excludeF), sel=sel(arrayfun(@(q)all(abs(fAll(q)-excludeF)>0.05),sel)); end
if isempty(sel), M=struct('f',NaN,'zeta',NaN,'sigma',NaN); return; end
contrib=arrayfun(@(q)sum(abs(V(group,q)).^2)/den(q),sel);
if isempty(contrib)||max(contrib)<1e-4
    % 没有足够的组参与度时，不把TOR模态或数值小分量误标成DC/SYNC/GSC。
    M=struct('f',NaN,'zeta',NaN,'sigma',NaN); return;
end
score=arrayfun(@(q)-sum(abs(V(group,q)).^2)/den(q)+0.02*abs(fAll(q)-fRef),sel); [~,ii]=min(score); q=sel(ii); M=struct('f',fAll(q),'zeta',zAll(q),'sigma',real(lam(q)));
end

function G=feedbackProxy(Crow,A,stateIndex,z)
n=size(A,1); b=zeros(n,1); b(stateIndex)=1; G=Crow*((z*eye(n)-A)\b);
end

function r=modalResidue(Crow,Bcol,A,k)
[V,~]=eig(A); W=pinv(V); r=(Crow*V(:,k))*(W(k,:)*Bcol);
end

function [Ad,Bd,Cd,Dd]=discreteJacobian(z0,u0,p,Ts,tau)
n=numel(z0); m=numel(u0); h=1e-5*max(abs(z0),1); hu=[1e-3*max(abs(p(39)),1e6);1e-3*max(abs(p(37)),1e6);1e-3*max(abs(p(38)),1e6);1e-3*max(abs(p(3)),1);1e-3;1e-3*max(abs(p(2)),1)];
Ad=zeros(n); Bd=zeros(n,m); Cd=zeros(29,n); Dd=zeros(29,m);
for k=1:n, zp=z0;zm=z0;zp(k)=zp(k)+h(k);zm(k)=zm(k)-h(k); Ad(:,k)=(s7a_discrete_average_core('step',zp,u0,p,Ts,tau)-s7a_discrete_average_core('step',zm,u0,p,Ts,tau))/(2*h(k)); Cd(:,k)=(s7a_discrete_average_core('output',zp,u0,p)-s7a_discrete_average_core('output',zm,u0,p))/(2*h(k)); end
for k=1:m, up=u0;um=u0;up(k)=up(k)+hu(k);um(k)=um(k)-hu(k); Bd(:,k)=(s7a_discrete_average_core('step',z0,up,p,Ts,tau)-s7a_discrete_average_core('step',z0,um,p,Ts,tau))/(2*hu(k)); Dd(:,k)=(s7a_discrete_average_core('output',z0,up,p)-s7a_discrete_average_core('output',z0,um,p))/(2*hu(k)); end
end

function s=classify(df,dz,em,eg)
if abs(df)<0.01 && abs(dz)<0.001 && max(abs([em eg]))<0.05, s='DIGITAL_INSENSITIVE';
elseif abs(df)>=0.01 || abs(dz)>=0.001, s='POLE_IMPLEMENTATION_DEPENDENT';
else, s='EXCITATION_IMPLEMENTATION_DEPENDENT'; end
end

function D=makeDecomposition(rows,fc,zc,rm,rg)
D=struct([]); for k=1:numel(rows), poleMetric=sqrt((rows(k).pole_delta_freq_Hz/max(fc,eps))^2+rows(k).pole_delta_zeta^2); excMetric=max(abs([rows(k).excitation_delta_mech_ratio rows(k).excitation_delta_grid_ratio])); drow=struct('Ts_ratio',rows(k).Ts_ratio,'delay_ratio',rows(k).delay_ratio,'continuous_tor_freq_Hz',fc,'continuous_tor_zeta',zc,'digital_tor_freq_Hz',rows(k).tor_freq_Hz,'digital_tor_zeta',rows(k).tor_zeta,'delta_pole_freq_Hz',rows(k).pole_delta_freq_Hz,'delta_pole_zeta',rows(k).pole_delta_zeta,'continuous_residue_mech_mag',abs(rm),'digital_residue_mech_mag',rows(k).res_tor_mech_mag,'continuous_residue_grid_mag',abs(rg),'digital_residue_grid_mag',rows(k).res_tor_grid_mag,'delta_excitation_mech_ratio',rows(k).excitation_delta_mech_ratio,'delta_excitation_grid_ratio',rows(k).excitation_delta_grid_ratio,'pole_metric',poleMetric,'excitation_metric',excMetric,'classification',rows(k).classification,'counterfactual_note','modal-metric decomposition; not cross-order y_cc/y_dc simulation'); if isempty(D), D=drow; else, D(end+1)=drow; end; end
end

function localWriteCsv(path,S)
if isempty(S), fid=fopen(path,'w');fprintf(fid,'status\nEMPTY\n');fclose(fid);return;end
fn=fieldnames(S); fid=fopen(path,'w'); assert(fid>0); c=onCleanup(@()fclose(fid)); for j=1:numel(fn), if j>1,fprintf(fid,',');end,fprintf(fid,'%s',fn{j});end,fprintf(fid,'\n');
for i=1:numel(S), for j=1:numel(fn), if j>1,fprintf(fid,',');end,v=S(i).(fn{j}); if ischar(v)||isstring(v), fprintf(fid,'"%s"',strrep(char(v),'"','""')); elseif islogical(v),fprintf(fid,'%d',v);elseif isnumeric(v)&&isscalar(v),fprintf(fid,'%.15g',v);else,fprintf(fid,'"%s"',mat2str(v));end,end,fprintf(fid,'\n');end
end

function p=saveFigure(rows,here)
p=fullfile(here,'S7_S3_S4_Pole_Excitation.png'); a=[rows.Ts_ratio]; b=[rows.delay_ratio]; z=[rows.tor_zeta]; e=[rows.excitation_delta_grid_ratio]; fig=figure('Visible','off','Color','w'); subplot(1,2,1); scatter(a,z,70,b,'filled');grid on;colorbar; xlabel('T_s/T_{s0}');ylabel('\zeta_{TOR}');title('S7-3 轴系模态阻尼'); subplot(1,2,2); scatter(a,e*100,70,b,'filled');grid on;colorbar; xlabel('T_s/T_{s0}');ylabel('\Delta residue_{grid} (%)');title('S7-4 网侧激励残差变化'); exportgraphics(fig,p,'Resolution',160);close(fig);
end

function writeReport(path,csv1,csv2,csv3,fig,rows,fc,zc,rm,rg)
fid=fopen(path,'w');assert(fid>0);c=onCleanup(@()fclose(fid));
fprintf(fid,'# S7-3/S7-4 真实数字参考模型九点筛查与 Pole–Excitation 复核\n\n生成时间：%s\n\n',datestr(now,31));
fprintf(fid,'## 证据等级\n\n本轮为 **CONDITIONAL_REFERENCE_DIGITAL_SCREENING**。使用的是已通过V2条件验证的 `S7A_DiscreteAvg_5MW.slx` 同源参考离散平均映射，不是遗留C控制器的最终实现，也不是EMT证据。\n\n');
fprintf(fid,'## S7-3九点结果\n\n连续M0基线轴系模态：%.6f Hz，阻尼比 %.6f；连续模态机械扰动残差幅值 %.6g，电网频率扰动残差幅值 %.6g。\n\n',fc,zc,abs(rm),abs(rg));
fprintf(fid,'九点全部计算完成，固定点残差、全部极点、TOR/DC/SYNC/GSC模态摘要见 `%s` 和 `%s`。\n\n',csv1,csv2);
fprintf(fid,'|Ts/Ts0|tau/Ts|TOR Hz|zeta|De proxy|Pole/Excitation分类|\n|---:|---:|---:|---:|---:|---|\n');for k=1:numel(rows),fprintf(fid,'|%.3g|%.3g|%.6f|%.6f|%.6g|%s|\n',rows(k).Ts_ratio,rows(k).delay_ratio,rows(k).tor_freq_Hz,rows(k).tor_zeta,rows(k).tor_De_proxy,rows(k).classification);end
fprintf(fid,'\n## S7-4解释\n\n本轮以连续M0的TOR pole/residue与数字映射的TOR pole/residue做增量对照：Pole变化记录在 `delta_pole_*`，激励变化记录在 `delta_excitation_*`。由于连续与数字模型状态阶数不同，本程序没有虚构跨阶的 y_cc/y_dc 时域反事实；`S7_S4_Pole_Excitation_Decomposition.csv` 中的 `counterfactual_note` 明确标记为 modal-metric decomposition。\n\n');
fprintf(fid,'当前筛查中，若分类为 DIGITAL_INSENSITIVE，只说明在这9个Reference数字点上TOR pole与残差变化均小；若为 POLE_IMPLEMENTATION_DEPENDENT 或 EXCITATION_IMPLEMENTATION_DEPENDENT，只能作为待复核假设，不能直接升级为论文结论。\n\n');
fprintf(fid,'## Gate V3建议\n\n在真实遗留数字控制器映射尚未完成前，Gate V3保持 **BLOCKED/CONDITIONAL**，不进入EMT。只有将实际采样顺序、PI更新顺序、延迟和控制状态与C代码逐一对齐，并重复V2和S7-3/S7-4后，才判断EMT工作量。\n\n');
fprintf(fid,'## 产物\n\n- `%s`\n- `%s`\n- `%s`\n',csv1,csv2,csv3);if ~isempty(fig),fprintf(fid,'- `%s`\n',fig);end
end
