function T = scan_dvc_bandwidth_multimode_excitation(outDir)
%SCAN_DVC_BANDWIDTH_MULTIMODE_EXCITATION 等比例缩放 MSC-DVC PI，其他参数不补偿。
if nargin<1 || isempty(outDir), outDir=fileparts(mfilename('fullpath')); end
[models,base]=prepare_multimode_models('Modes',{'VSG'}); x=models{1}.x0; p0=base.parameter_vector; factors=[.5 .75 1 1.25 1.5];
T=table('Size',[numel(factors) 13], ...
 'VariableTypes',{'double','string','double','double','double','double','double','double','double','double','double','double','string'}, ...
 'VariableNames',{'DVC_Scale','Status','f_tor_Hz','zeta_tor','PoleReal','PoleImag','Rdc_frequency','G_Udc_frequency','G_iqref_frequency','G_Te_frequency','Rtor_frequency','PeakOmegaSh_frequency','Interpretation'});
for k=1:numel(factors)
    p=p0; p(25)=p0(25)*factors(k); p(26)=p0(26)*factors(k);
    try
        L=multimode_linearize_control(x,p,'VSG',struct); S=multimode_scan_metrics(L);
        interp=iffLocal(S.Gte_f>1.2*multimode_scan_metrics(multimode_linearize_control(x,p0,'VSG',struct)).Gte_f,"amplification path","attenuation/passive path");
        T.DVC_Scale(k)=factors(k); T.Status(k)="PASS"; T.f_tor_Hz(k)=S.f_tor; T.zeta_tor(k)=S.zeta_tor; T.PoleReal(k)=S.pole_real; T.PoleImag(k)=S.pole_imag; T.Rdc_frequency(k)=S.Rdc(2); T.G_Udc_frequency(k)=S.Gudc_f; T.G_iqref_frequency(k)=S.Giqref_f; T.G_Te_frequency(k)=S.Gte_f; T.Rtor_frequency(k)=S.Rtor(2); T.PeakOmegaSh_frequency(k)=S.PeakOmegaSh_f; T.Interpretation(k)=interp;
    catch ME
        T.DVC_Scale(k)=factors(k); T.Status(k)="FAIL"; T.f_tor_Hz(k)=NaN; T.zeta_tor(k)=NaN; T.PoleReal(k)=NaN; T.PoleImag(k)=NaN; T.Rdc_frequency(k)=NaN; T.G_Udc_frequency(k)=NaN; T.G_iqref_frequency(k)=NaN; T.G_Te_frequency(k)=NaN; T.Rtor_frequency(k)=NaN; T.PeakOmegaSh_frequency(k)=NaN; T.Interpretation(k)="FAIL"; latest_failed_case=struct('case_name','DVC scan','factor',factors(k),'message',ME.message); save(fullfile(outDir,'latest_failed_case.mat'),'latest_failed_case');
    end
end
writetable(T,fullfile(outDir,'DVC_Multimode_Scan.csv'));
end

function v=iffLocal(c,a,b), if c, v=a; else, v=b; end, end
