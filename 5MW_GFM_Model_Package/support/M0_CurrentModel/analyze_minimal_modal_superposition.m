function R = analyze_minimal_modal_superposition(models,outDir)
%ANALYZE_MINIMAL_MODAL_SUPERPOSITION
% 对正确的阶跃模态响应 y_k=R_k/lambda_k*(exp(lambda_k*t)-1) 进行幅相叠加。
% 不把冲激残差直接当作阶跃幅值；只保存摘要表，不保存原始时序。
if nargin<2 || isempty(outDir), outDir=fileparts(mfilename('fullpath')); end
[~,dominant,details,gate]=build_minimal_multimode_reconstruction(models,outDir);
rows=max(1,sum(arrayfun(@(z)max(1,numel(z.selected)),details))); rr=0;
T=table('Size',[rows 13], ...
    'VariableTypes',{'string','string','string','string','string','double','double','double','double','double','double','double','string'}, ...
    'VariableNames',{'Architecture','Disturbance','Output','ModeID','PhysicalClass','Frequency_Hz','Damping','ResidueMagnitude','ResiduePhase_deg','ContributionAtPeak','ContributionRatio','PeakTime_s','ConstructiveOrDestructive'});
for z=1:numel(details)
    D=details(z);
    if isempty(D.selected)
        rr=rr+1;
        T(rr,:)={string(D.architecture),string(D.disturbance),string(D.output),"NONE","NONE",NaN,NaN,0,NaN,0,0,NaN,"NO_EXCITATION"};
        continue
    end
    i0=find(D.t>=0.10,1); [~,j]=max(abs(D.full(i0:end))); ip=i0+j-1;
    tPeak=D.t(ip); tau=tPeak-0.10; total=D.full(ip);
    uAmp=localInputAmplitude(D.disturbance);
    for q=1:numel(D.selected)
        lam=D.lambda(q); rStep=D.residue(q); val=localStepContribution(lam,rStep,tau);
        rr=rr+1;
        T.Architecture(rr)=string(D.architecture); T.Disturbance(rr)=string(D.disturbance); T.Output(rr)=string(D.output);
        T.ModeID(rr)=string(sprintf('M%02d',D.selected(q))); T.PhysicalClass(rr)=string(D.physical_class{q});
        T.Frequency_Hz(rr)=abs(imag(lam))/(2*pi); T.Damping(rr)=-real(lam)/max(abs(lam),eps);
        T.ResidueMagnitude(rr)=abs(rStep)/max(uAmp,eps); T.ResiduePhase_deg(rr)=rad2deg(angle(rStep));
        T.ContributionAtPeak(rr)=val; T.ContributionRatio(rr)=abs(val)/max(abs(total),eps); T.PeakTime_s(rr)=tPeak;
        if sign(val)==sign(total), T.ConstructiveOrDestructive(rr)="CONSTRUCTIVE"; else, T.ConstructiveOrDestructive(rr)="DESTRUCTIVE"; end
    end
end
T=T(1:rr,:);
writetable(T,fullfile(outDir,'MinimalModal_Superposition_Summary.csv'));
% 兼容原有报告脚本：同步写入旧名称，但新文件是本阶段权威摘要。
writetable(T(:,{'Architecture','Disturbance','Output','ModeID','PhysicalClass','ResidueMagnitude','ResiduePhase_deg','ContributionAtPeak','ContributionRatio','ConstructiveOrDestructive'}), ...
    fullfile(outDir,'Modal_Superposition_At_FirstPeak.csv'));
R=struct('summary',T,'dominant',dominant,'details',details,'gate',gate);
end

function a=localInputAmplitude(name)
if contains(string(name),'angle','IgnoreCase',true), a=deg2rad(0.2); else, a=2*pi*0.05; end
end

function y=localStepContribution(lam,r,tau)
term=r/lam*(exp(lam*tau)-1);
if imag(lam)>1e-8, y=2*real(term); else, y=real(term); end
end
