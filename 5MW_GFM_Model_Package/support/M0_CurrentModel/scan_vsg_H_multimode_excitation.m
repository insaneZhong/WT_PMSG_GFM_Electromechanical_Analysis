function T = scan_vsg_H_multimode_excitation(outDir)
%SCAN_VSG_H_MULTIMODE_EXCITATION 仅扫描 VSG-MWT 的 H；平衡点保持不变。
if nargin<1 || isempty(outDir), outDir=fileparts(mfilename('fullpath')); end
[models,base]=prepare_multimode_models('Modes',{'VSG'}); x=models{1}.x0; p0=base.parameter_vector; H0=p0(33); factors=[.5 .75 1 1.5 2];
T=table('Size',[numel(factors) 12], ...
 'VariableTypes',{'double','double','string','double','double','double','double','double','double','double','double','double'}, ...
 'VariableNames',{'H_s','H_Factor','Status','f_tor_Hz','zeta_tor','PoleReal','PoleImag','Rtor_frequency','Rsync_frequency','Rdc_frequency','PeakOmegaSh_frequency','Gamma_frequency_vs_H0'});
for k=1:numel(factors)
    p=p0; p(33)=H0*factors(k);
    try
        L=multimode_linearize_control(x,p,'VSG',struct); S=multimode_scan_metrics(L);
        T.H_s(k)=p(33); T.H_Factor(k)=factors(k); T.Status(k)="PASS"; T.f_tor_Hz(k)=S.f_tor; T.zeta_tor(k)=S.zeta_tor; T.PoleReal(k)=S.pole_real; T.PoleImag(k)=S.pole_imag; T.Rtor_frequency(k)=S.Rtor(2); T.Rsync_frequency(k)=S.Rsync(2); T.Rdc_frequency(k)=S.Rdc(2); T.PeakOmegaSh_frequency(k)=S.PeakOmegaSh_f;
    catch ME
        T.H_s(k)=p(33); T.H_Factor(k)=factors(k); T.Status(k)="FAIL"; T.f_tor_Hz(k)=NaN; T.zeta_tor(k)=NaN; T.PoleReal(k)=NaN; T.PoleImag(k)=NaN; T.Rtor_frequency(k)=NaN; T.Rsync_frequency(k)=NaN; T.Rdc_frequency(k)=NaN; T.PeakOmegaSh_frequency(k)=NaN; latest_failed_case=struct('case_name','H scan','H',p(33),'message',ME.message); save(fullfile(outDir,'latest_failed_case.mat'),'latest_failed_case');
    end
end
i0=find(abs(T.H_Factor-1)<eps,1); for k=1:height(T), T.Gamma_frequency_vs_H0(k)=T.Rtor_frequency(k)/max(T.Rtor_frequency(i0),1e-18); end
writetable(T,fullfile(outDir,'H_Multimode_Scan.csv'));
end
