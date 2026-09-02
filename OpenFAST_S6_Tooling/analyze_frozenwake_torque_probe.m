function S=analyze_frozenwake_torque_probe()
%ANALYZE_FROZENWAKE_TORQUE_PROBE OpenFAST-only frozen-wake torque probe.
% Compare a 250 N m HSS-brake step with a matched no-step run.  This is an
% independent temporary mechanical diagnostic only: it neither invokes
% Simulink nor validates the S6 hybrid state-space model.

here=fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(fileparts(here),'matlab-toolbox')));
[t0,rot0,gen0]=readRelativeSpeed(fullfile(here,'M3_FrozenWake_TorqueProbe_NoStep.outb'));
[t1,rot1,gen1]=readRelativeSpeed(fullfile(here,'M3_FrozenWake_TorqueProbe_Step250.outb'));
assert(isequal(size(t0),size(t1)) && max(abs(t0-t1))<1e-9, ...
    'The matched OpenFAST time bases are inconsistent.');

t=t0; gear=97; tStep=120; stepNm=250;
rel0=rot0-gen0/gear;
rel1=rot1-gen1/gear;
pre=t>=100 & t<tStep-1;
baselineOffset=mean(rel1(pre)-rel0(pre));
drel=(rel1-rel0)-baselineOffset;

win=t>=tStep+0.25 & t<=tStep+15;
tw=t(win); x=detrend(drel(win));
fs=1/median(diff(t)); N=numel(x);
f=fs*(0:floor(N/2))'/N;
A=abs(fft(x))/N; A=A(1:numel(f));
if numel(A)>2, A(2:end-1)=2*A(2:end-1); end
band=f>=0.2 & f<=10; fb=f(band); Ab=A(band);
[pk,loc]=findpeaks(Ab,fb,'SortStr','descend');
nPk=min(5,numel(pk)); pk=pk(1:nPk); loc=loc(1:nPk);
if nPk<2
    peakRatio=Inf;
else
    peakRatio=pk(1)/(pk(2)+eps);
end

% A free-decay estimate is accepted only if the differential spectrum is
% clearly dominated by one peak and its Hilbert envelope decreases overall.
% This prevents periodic aero/structural components from being labelled as
% torsional damping.
env=abs(hilbert(x));
early=mean(env(tw>=tStep+0.5 & tw<tStep+4));
late=mean(env(tw>=tStep+10 & tw<=tStep+15));
envelopeRatio=late/(early+eps);
isolated=(nPk>=1) && peakRatio>=1.5 && envelopeRatio<0.45;
if isolated
    fPeak=loc(1);
    fitWin=tw>=tStep+0.5 & tw<=tStep+10 & env>0.10*max(env);
    q=polyfit(tw(fitWin)-tStep,log(env(fitWin)),1);
    sigma=-q(1);
    zeta=sigma/sqrt(sigma^2+(2*pi*fPeak)^2);
    status="IDENTIFIABLE_FREE_DECAY";
else
    fPeak=NaN; sigma=NaN; zeta=NaN;
    status="NO_ISOLATED_DECAY";
end

S=table(stepNm,max(abs(rel1(pre)-rel0(pre))),max(abs(drel)), ...
    nPk,peakRatio,envelopeRatio,fPeak,sigma,zeta,status, ...
    'VariableNames',{'HSSBrakeStep_Nm','PreStepDifferenceMax_rpm', ...
    'PeakDifferentialRelativeSpeed_rpm','NumberOfReportedPeaks', ...
    'LargestToSecondPeakRatio','LateToEarlyEnvelopeRatio', ...
    'IdentifiedFrequency_Hz','IdentifiedDecayRate_per_s', ...
    'IdentifiedDampingRatio','IdentificationStatus'});
writetable(S,fullfile(here,'M3_FrozenWake_TorqueProbe_Summary.csv'));

fig=figure('Visible','off','Color','w','Position',[50 50 1280 720]);
tl=tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
nexttile;
plot(t,drel,'Color',[.15 .35 .80],'LineWidth',1); hold on; xline(tStep,'--r');
xlim([115 145]); grid on; xlabel('Time (s)'); ylabel('\Delta\omega_{rel,step-base} (rpm)');
title('Matched differential relative-speed response');
nexttile;
plot(fb,Ab,'k','LineWidth',1.2); hold on;
if nPk>0, scatter(loc,pk,30,'filled','MarkerFaceColor',[.85 .25 .2]); end
xlim([0 10]); grid on; xlabel('Frequency (Hz)'); ylabel('Amplitude (rpm)');
title(sprintf('Differential spectrum; largest/second = %.3f',peakRatio));
nexttile;
plot(tw,env,'Color',[.1 .1 .1],'LineWidth',1.2); grid on;
xlabel('Time (s)'); ylabel('Envelope (rpm)');
title(sprintf('Envelope late/early = %.3f; %s',envelopeRatio,status));
nexttile;
plot(t,rel0,'Color',[.45 .45 .45]); hold on; plot(t,rel1,'Color',[.2 .6 .2]); xline(tStep,'--r');
xlim([115 145]); grid on; xlabel('Time (s)'); ylabel('\omega_{rel} (rpm)');
title('No-step and step cases before differencing'); legend({'no-step','250 N m step','step instant'},'Location','best');
sgtitle(tl,'Temporary OpenFAST frozen-wake mechanical probe — not co-simulation');
exportgraphics(fig,fullfile(here,'M3_FrozenWake_TorqueProbe.png'),'Resolution',220); close(fig);

fid=fopen(fullfile(here,'M3_FrozenWake_TorqueProbe_Report_CN.md'),'w','n','UTF-8');
assert(fid>0); c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'# OpenFAST Frozen-Wake 独立柔性机械时域探针\n\n');
fprintf(fid,['本试验只运行 OpenFAST：8 m/s、`DBEMT_Mod=-1`。它比较一个无阶跃基准与 120 s 后施加 250 N m HSS 制动转矩的匹配试验；两者相减以消除共同的周期气动/结构分量。为使用简单 HSS 制动器，临时试验采用 `ModCoupling=1` 和重新标定的简单 VS 转矩律。**没有调用 Simulink，也没有与 M3 非线性模型或小信号模型联合仿真。**\n\n']);
fprintf(fid,'- 阶跃前差分最大值：%.6g rpm；\n',S.PreStepDifferenceMax_rpm);
fprintf(fid,'- 差分相对转速峰值：%.6g rpm；\n',S.PeakDifferentialRelativeSpeed_rpm);
fprintf(fid,'- 最大/次大谱峰比：%.6g；\n',S.LargestToSecondPeakRatio);
fprintf(fid,'- 包络后段/前段比：%.6g；\n',S.LateToEarlyEnvelopeRatio);
fprintf(fid,'- 识别状态：`%s`。\n\n',S.IdentificationStatus);
if status=="NO_ISOLATED_DECAY"
    fprintf(fid,['差分谱存在多个相近峰，且包络不满足单一指数自由衰减判据。因此本次探针**不报告轴系频率或阻尼**，也不得用于验证 OpenFAST 周期线性化/MBC 矩阵或 S6 混合模型。它只说明此小阶跃下 OpenFAST 柔性传动链的有限响应可被观察到。若要获得独立时域模态参数，需要另行设计可控的正弦转矩激励或可辨识的停机自由衰减试验。\n']);
else
    fprintf(fid,'本试验在预设判据下得到可辨识的局部自由衰减；仍只作为 OpenFAST 独立对象诊断，不能替代跨模型联合验证。\n');
end
end

function [t,rot,gen]=readRelativeSpeed(outb)
[X,names]=ReadFASTbinary(outb);
names=string(names(:));
idx=@(s)find(names==s,1);
iT=idx("Time"); iR=idx("RotSpeed"); iG=idx("GenSpeed");
assert(~isempty(iT) && ~isempty(iR) && ~isempty(iG), ...
    'Required channels Time/RotSpeed/GenSpeed are missing from %s.',outb);
t=X(:,iT); rot=X(:,iR); gen=X(:,iG);
end
