function R=run_s7_v2_validation(varargin)
%RUN_S7_V2_VALIDATION  S7-2离散平均模型的固定点、离散SSM和时域对照。
%
% 本程序只使用 M0 的23状态连续物理方程，并把11个控制/软件状态按
% S7_Controller_State_Audit.csv 中的候选Forward-Euler映射实现为离散
% 平均模型。它不调用旧C控制器，不含PWM、开关、限幅或保护。
% 输出仅保存摘要、报告和一张综合图，不保存完整时序。

ip=inputParser;
ip.addParameter('Cases',{'D1','D2','D3'});
ip.addParameter('Disturbances',{'mechanical','grid_frequency'});
ip.addParameter('StopTime',0.4,@(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('StepTime',0.02,@(x)isnumeric(x)&&isscalar(x)&&x>=0);
ip.addParameter('StepMagnitudePu',0.005,@(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('SaveFigures',true,@(x)islogical(x)&&isscalar(x));
ip.parse(varargin{:}); o=ip.Results;

here=fileparts(mfilename('fullpath')); idealDir=fileparts(here);
addpath(idealDir); addpath(here);
matFile=fullfile(idealDir,'M0_5MW_Aligned_Workpoint_and_SSM.mat');
S=load(matFile,'params','operating_point','state_names');
[pvec,~]=m0_pack_parameters(S.params,S.operating_point);
x0=S.operating_point.x0(:); u0=zeros(6,1); P=S.params;
Ts0=P.controller_Ts_s; w0=P.omega0_radps; Tm0=P.Tm0_Nm;

% S7-2代表工况：D1基准、D2一周期半延迟、D3两倍采样周期压力测试。
caseTable=struct(...
    'name',{'D1','D2','D3'},...
    'Ts',{Ts0,Ts0,2*Ts0},...
    'tau',{Ts0,1.5*Ts0,Ts0},...
    'description',{'Ts=Ts0, tau=Ts0','Ts=Ts0, tau=1.5Ts0','Ts=2Ts0, tau=Ts0'});

results=struct([]); caseResults=struct([]); figData=struct([]); row=0;
for ic=1:numel(o.Cases)
    ci=find(strcmpi({caseTable.name},o.Cases{ic}),1);
    if isempty(ci), error('Unknown S7-2 case: %s',o.Cases{ic}); end
    C=caseTable(ci);
    % 建立/编译同一个唯一S7A模型，避免生成多份模型文件。
    build_s7a_discrete_average_model('Ts',C.Ts,'Delay',C.tau,'Compile',true);
    cmd0=s7a_discrete_average_core('commands',x0,u0,pvec); z0=[x0;cmd0;cmd0];
    zFixed=s7a_discrete_average_core('step',z0,u0,pvec,C.Ts,C.tau);
    fixedResidual=max(abs(zFixed-z0)); fixedNorm=norm(zFixed-z0)/max(1,norm(z0));
    [Ad,Bd,Cd,Dd]=localDiscreteJacobian(z0,u0,pvec,C.Ts,C.tau);
    ev=eig(Ad); [fd,zd,lambdaTor]=localFindTorMode(ev,C.Ts);
    modeStable=all(real(log(ev)/C.Ts)<1e-8);
    y0=s7a_discrete_average_core('output',z0,u0,pvec);
    for id=1:numel(o.Disturbances)
        dist=o.Disturbances{id};
        [t,uSeq]=localInputSequence(o.StopTime,C.Ts,o.StepTime,o.StepMagnitudePu,dist,Tm0,w0);
        % Simulink验证：S7A_DiscreteAvg_5MW是本轮唯一的离散平均副本。
        in=Simulink.SimulationInput('S7A_DiscreteAvg_5MW');
        in=in.setModelParameter('StopTime',num2str(o.StopTime,'%.15g'));
        in=in.setExternalInput(timeseries(uSeq,t));
        simOut=sim(in);
        [tSim,ySim]=localReadOutput(simOut);
        % 同源离散SSM：输入使用同一个采样序列，输出采用同一29通道定义。
        N=numel(t); zLin=z0; yLin=zeros(29,N);
        for k=1:N
            yLin(:,k)=y0+Cd*(zLin-z0)+Dd*uSeq(k,:).';
            zLin=z0+Ad*(zLin-z0)+Bd*uSeq(k,:).';
        end
        n=min(size(ySim,1),N); m=min(size(ySim,2),N);
        if numel(tSim)>=N, tUse=tSim(1:N); yUse=ySim(:,1:N); else, tUse=tSim(:); yUse=ySim(:,1:numel(tSim)); end
        n=min(size(yUse,1),29); m=min(size(yUse,2),N);
        yLinUse=yLin(1:n,1:m); yUse=yUse(1:n,1:m); tUse=tUse(1:m);
        idx=[7 8 6 3 1 2 9 10 11]; names={'Te','Tshaft','Udc','Ppcc','Pmsc','Pgsc','wt','wg','omegaRel'};
        for jj=1:numel(idx)
            j=idx(jj); if j>n, continue; end
            dnl=yUse(j,:)-y0(j); dssm=yLinUse(j,:)-y0(j);
            scale=max([1, rms(dnl), max(abs(dnl))]);
            row=row+1; rrow=struct(...
                'case',C.name,'disturbance',dist,'Ts_s',C.Ts,'delay_s',C.tau,...
                'fixed_residual_max',fixedResidual,'fixed_residual_norm',fixedNorm,...
                'all_poles_stable',modeStable,'tor_mode_Hz',fd,'tor_mode_zeta',zd,...
                'tor_lambda_real_sinv',real(lambdaTor),'signal',names{jj},...
                'nrmse',sqrt(mean((dnl-dssm).^2))/scale,'peak_error',max(abs(dnl-dssm))/scale,...
                'nl_peak',max(abs(dnl)),'ssm_peak',max(abs(dssm)),...
                'status','CONDITIONAL_REFERENCE_DISCRETE_AVERAGE');
            if isempty(results), results=rrow; else, results(end+1)=rrow; end %#ok<AGROW>
        end
        % 每个工况只保留一个Te和omegaRel用于综合图，不保存完整数组。
        fr=numel(figData)+1; figData(fr).case=C.name; figData(fr).dist=dist; figData(fr).t=tUse; 
        figData(fr).nlTe=yUse(7,:)-y0(7); figData(fr).ssmTe=yLinUse(7,:)-y0(7);
        figData(fr).nlW=yUse(11,:)-y0(11); figData(fr).ssmW=yLinUse(11,:)-y0(11);
    end
    crow=struct('case',C.name,'Ts_s',C.Ts,'delay_s',C.tau,...
        'description',C.description,'fixed_residual_max',fixedResidual,...
        'fixed_residual_norm',fixedNorm,'all_poles_stable',modeStable,...
        'tor_mode_Hz',fd,'tor_mode_zeta',zd,'tor_lambda_real_sinv',real(lambdaTor),...
        'state_count',31,'output_count',29,'status','CONDITIONAL_REFERENCE_DISCRETE_AVERAGE');
    if isempty(caseResults), caseResults=crow; else, caseResults(end+1)=crow; end %#ok<AGROW>
end

outCsv=fullfile(here,'S7_V2_NL_SSM_Validation.csv');
localWriteStructCsv(outCsv,results);
modeCsv=fullfile(here,'S7_V2_Discrete_Modes.csv'); localWriteStructCsv(modeCsv,caseResults);
figPath=''; if o.SaveFigures, figPath=localSaveFigure(figData,here); end
reportPath=fullfile(here,'S7_V2_FixedPoint_Report_CN.md');
localWriteReport(reportPath,matFile,outCsv,modeCsv,figPath,caseResults,results,o);

R=struct('status','CONDITIONAL_REFERENCE_DIGITAL_AVERAGE',...
    'model_path',fullfile(here,'S7A_DiscreteAvg_5MW.slx'),...
    'validation_csv',outCsv,'mode_csv',modeCsv,'report',reportPath,'figure',figPath,...
    'case_results',caseResults,'result_rows',numel(results));
fprintf('S7-2完成：离散平均模型V2摘要已写入 %s\n',reportPath);
end

function [Ad,Bd,Cd,Dd]=localDiscreteJacobian(z0,u0,p,Ts,tau)
n=numel(z0); m=numel(u0);
% ODE45的AbsTol为1e-8；输入扰动不能使用1e-6 Nm这类远低于
% 求解器容差的步长。这里按物理基值取中心差分步长，仍处于局部线性区。
h=1e-5*max(abs(z0),1);
hu=[1e-3*max(abs(p(39)),1e6); 1e-3*max(abs(p(37)),1e6); ...
    1e-3*max(abs(p(38)),1e6); 1e-3*max(abs(p(3)),1); 1e-3; ...
    1e-3*max(abs(p(2)),1)];
f0=s7a_discrete_average_core('step',z0,u0,p,Ts,tau);
y0=s7a_discrete_average_core('output',z0,u0,p);
Ad=zeros(n); Bd=zeros(n,m); Cd=zeros(29,n); Dd=zeros(29,m);
for k=1:n
    zp=z0; zm=z0; zp(k)=zp(k)+h(k); zm(k)=zm(k)-h(k);
    fp=s7a_discrete_average_core('step',zp,u0,p,Ts,tau); fm=s7a_discrete_average_core('step',zm,u0,p,Ts,tau);
    yp=s7a_discrete_average_core('output',zp,u0,p); ym=s7a_discrete_average_core('output',zm,u0,p);
    Ad(:,k)=(fp-fm)/(2*h(k)); Cd(:,k)=(yp-ym)/(2*h(k));
end
for k=1:m
    up=u0; um=u0; up(k)=up(k)+hu(k); um(k)=um(k)-hu(k);
    fp=s7a_discrete_average_core('step',z0,up,p,Ts,tau); fm=s7a_discrete_average_core('step',z0,um,p,Ts,tau);
    yp=s7a_discrete_average_core('output',z0,up,p); ym=s7a_discrete_average_core('output',z0,um,p);
    Bd(:,k)=(fp-fm)/(2*hu(k)); Dd(:,k)=(yp-ym)/(2*hu(k));
end
% 保留f0、y0用于调试断言，但不输出大数据。
assert(all(isfinite(f0)) && all(isfinite(y0)),'离散映射Jacobian含非有限值。');
end

function [f,z,lam]=localFindTorMode(ev,Ts)
lamAll=log(ev)/Ts; fAll=abs(imag(lamAll))/(2*pi); zAll=-real(lamAll)./max(abs(lamAll),eps);
sel=find(imag(lamAll)>1e-4 & fAll>0.5 & fAll<6);
if isempty(sel), [~,k]=min(abs(fAll-2.48)); else, [~,ii]=min(abs(fAll(sel)-2.48)); k=sel(ii); end
f=fAll(k); z=zAll(k); lam=lamAll(k);
end

function [t,u]=localInputSequence(stopTime,Ts,tStep,mag,dist,Tm0,w0)
N=round(stopTime/Ts)+1; t=(0:N-1)'*Ts; u=zeros(N,6);
ii=find(t>=tStep,1); if isempty(ii), ii=N; end
switch lower(dist)
    case 'mechanical', u(ii:end,1)=mag*Tm0;
    case 'grid_frequency', u(ii:end,4)=mag*w0;
    otherwise, error('Unknown disturbance: %s',dist);
end
end

function [t,y]=localReadOutput(simOut)
t=simOut.tout(:); ds=simOut.yout;
try
    el=ds.getElement(1); v=el.Values; y=v.Data.';
catch
    el=ds{1}; v=el.Values; y=v.Data.';
end
if isvector(y), y=y(:); end
end

function localWriteStructCsv(path,S)
if isempty(S), fid=fopen(path,'w'); fprintf(fid,'status\nEMPTY\n'); fclose(fid); return; end
fn=fieldnames(S); fid=fopen(path,'w'); assert(fid>0,'无法写入 %s',path); c=onCleanup(@()fclose(fid));
for k=1:numel(fn), if k>1, fprintf(fid,','); end, fprintf(fid,'%s',fn{k}); end, fprintf(fid,'\n');
for i=1:numel(S)
    for k=1:numel(fn)
        if k>1, fprintf(fid,','); end
        v=S(i).(fn{k}); if ischar(v)||isstring(v), s=char(v); s=strrep(s,'"','""'); fprintf(fid,'"%s"',s);
        elseif islogical(v), fprintf(fid,'%d',v); elseif isnumeric(v)&&isscalar(v), fprintf(fid,'%.15g',v); else, fprintf(fid,'"%s"',mat2str(v)); end
    end, fprintf(fid,'\n');
end
end

function path=localSaveFigure(F,here)
path=fullfile(here,'S7_V2_D1_D2_D3_NL_SSM_Comparison.png'); fig=figure('Visible','off','Color','w');
for k=1:numel(F)
    subplot(2,ceil(numel(F)/2),k); plot(F(k).t,F(k).nlTe,'b-','LineWidth',0.9); hold on; plot(F(k).t,F(k).ssmTe,'r--','LineWidth',0.9); grid on;
    title(sprintf('%s / %s: T_e',F(k).case,F(k).dist),'Interpreter','none'); xlabel('t (s)'); ylabel('\DeltaT_e (N m)'); legend('离散平均非线性','同源离散SSM','Location','best');
end
exportgraphics(fig,path,'Resolution',150); close(fig);
end

function localWriteReport(path,matFile,outCsv,modeCsv,figPath,C,R,o)
fid=fopen(path,'w'); assert(fid>0,'无法写入报告'); c=onCleanup(@()fclose(fid));
fprintf(fid,'# S7-2 离散平均模型 V2 固定点与非线性—离散SSM验证\n\n');
fprintf(fid,'生成时间：%s\n\n',datestr(now,31));
fprintf(fid,'## 结论等级\n\n');
fprintf(fid,'本轮结论为 **CONDITIONAL_REFERENCE_DIGITAL_AVERAGE**。模型 `S7A_DiscreteAvg_5MW.slx` 是从 M0 方程直接实现的离散平均参考模型，用于验证 S7-1 的离散化映射；它不是旧 C/S-Function 控制器的复刻，也不包含 PWM、开关、限幅、保护或数字采样器。\n\n');
fprintf(fid,'## 输入与范围\n\n');
fprintf(fid,'- M0工作点：`%s`\n- StopTime=%.6g s，扰动时刻=%.6g s，阶跃幅值=%.6g pu。\n- 工况：D1(Ts=Ts0, tau=Ts0)、D2(Ts=Ts0, tau=1.5Ts0)、D3(Ts=2Ts0, tau=Ts0)。\n- 每个工况分别施加机械转矩和电网频率小阶跃。\n\n',matFile,o.StopTime,o.StepTime,o.StepMagnitudePu);
fprintf(fid,'## V2验收结果\n\n');
fprintf(fid,'|工况|Ts(s)|tau(s)|固定点最大残差|归一化残差|轴系模态(Hz)|阻尼|全部极点|\n|---|---:|---:|---:|---:|---:|---:|---|\n');
for k=1:numel(C)
    fprintf(fid,'|%s|%.6g|%.6g|%.3e|%.3e|%.6f|%.6f|%s|\n',C(k).case,C(k).Ts_s,C(k).delay_s,C(k).fixed_residual_max,C(k).fixed_residual_norm,C(k).tor_mode_Hz,C(k).tor_mode_zeta,mat2str(C(k).all_poles_stable));
end
fprintf(fid,'\n');
if isempty(R), fprintf(fid,'没有生成通道结果。\n'); else
    fprintf(fid,'通道对照共 %d 行；具体 NRMSE、峰值和各扰动结果见 `%s`。\n\n',numel(R),outCsv);
end
fprintf(fid,'## 结果解释边界\n\n');
fprintf(fid,'1. 这里的“非线性”指离散平均方程的非线性时域迭代，和同一离散映射数值线性化得到的离散SSM具有同源性；因此它只能证明离散化映射内部的一致性，不能替代旧 EMT 或真实遗留数字控制器的独立验证。\n');
fprintf(fid,'2. 轴系模态由离散映射特征值提取；当前脚本未把短时域FFT误称为精确频率。\n');
fprintf(fid,'3. 固定点残差继承M0工作点的数值平衡误差；若要升级为严格 Gate V2，需要后续用真实控制器状态映射和独立平衡点重新验收。\n\n');
fprintf(fid,'## 产物\n\n- 离散模型：`%s`\n- 模态摘要：`%s`\n- 非线性—离散SSM摘要：`%s`\n',fullfile(fileparts(outCsv),'S7A_DiscreteAvg_5MW.slx'),modeCsv,outCsv);
if ~isempty(figPath), fprintf(fid,'- 综合对照图：`%s`\n',figPath); end
fprintf(fid,'\n');
end
