function T = scan_scr_multimode_excitation(outDir)
%SCAN_SCR_MULTIMODE_EXCITATION SCR 扫描：只保存极点/残差/峰值摘要，不保存时序。
if nargin<1 || isempty(outDir), outDir=fileparts(mfilename('fullpath')); end
[baseModels,base]=prepare_multimode_models(); p0=base.parameter_vector; scr0=4; scrs=[2 3 4 6 8 10]; ix=[find(strcmp(base.models,'GFL'),1),find(strcmp(base.models,'VSG'),1)];
N=numel(scrs)*2; T=table('Size',[N 16], ...
 'VariableTypes',{'double','string','string','double','double','double','double','double','double','double','double','double','double','double','double','string'}, ...
 'VariableNames',{'SCR','Architecture','Status','EquilibriumResidual','f_tor_Hz','zeta_tor','Rtor_angle','Rtor_frequency','Rsync_angle','Rsync_frequency','Rdc_angle','Rdc_frequency','PeakOmegaSh_frequency','Gamma_angle_vs_GFL','Gamma_frequency_vs_GFL','Classification'});
rr=0;
for s=1:numel(scrs)
    p=p0; scale=scr0/scrs(s); p(9)=p0(9)*scale; p(10)=p0(10)*scale;
    for q=1:numel(ix)
        rr=rr+1; k=ix(q); mode=base.models{k}; label=base.labels{k}; flags=struct;
        try
            [xeq,meta]=solve_multimode_control_equilibrium(base.states(:,k),p,mode,flags);
            L=multimode_linearize_control(xeq,p,mode,flags); S=multimode_scan_metrics(L);
            T.SCR(rr)=scrs(s); T.Architecture(rr)=string(label); T.Status(rr)="PASS"; T.EquilibriumResidual(rr)=meta.residual_norm; T.f_tor_Hz(rr)=S.f_tor; T.zeta_tor(rr)=S.zeta_tor; T.Rtor_angle(rr)=S.Rtor(1); T.Rtor_frequency(rr)=S.Rtor(2); T.Rsync_angle(rr)=S.Rsync(1); T.Rsync_frequency(rr)=S.Rsync(2); T.Rdc_angle(rr)=S.Rdc(1); T.Rdc_frequency(rr)=S.Rdc(2); T.PeakOmegaSh_frequency(rr)=S.PeakOmegaSh_f;
        catch ME
            T.SCR(rr)=scrs(s); T.Architecture(rr)=string(label); T.Status(rr)="FAIL"; T.EquilibriumResidual(rr)=NaN; T.f_tor_Hz(rr)=NaN; T.zeta_tor(rr)=NaN; T.Rtor_angle(rr)=NaN; T.Rtor_frequency(rr)=NaN; T.Rsync_angle(rr)=NaN; T.Rsync_frequency(rr)=NaN; T.Rdc_angle(rr)=NaN; T.Rdc_frequency(rr)=NaN; T.PeakOmegaSh_frequency(rr)=NaN;
            latest_failed_case=struct('case_name','SCR scan','SCR',scrs(s),'architecture',label,'message',ME.message); save(fullfile(outDir,'latest_failed_case.mat'),'latest_failed_case');
        end
        T.Gamma_angle_vs_GFL(rr)=NaN; T.Gamma_frequency_vs_GFL(rr)=NaN; T.Classification(rr)="";
    end
end
for s=1:numel(scrs)
    ir=find(T.SCR==scrs(s) & contains(T.Architecture,"GFL"),1); im=find(T.SCR==scrs(s) & contains(T.Architecture,"MWT"),1);
    if T.Status(ir)=="PASS" && T.Status(im)=="PASS"
        T.Gamma_angle_vs_GFL(im)=T.Rtor_angle(im)/max(T.Rtor_angle(ir),1e-18); T.Gamma_frequency_vs_GFL(im)=T.Rtor_frequency(im)/max(T.Rtor_frequency(ir),1e-18);
        if abs(T.zeta_tor(im)-T.zeta_tor(ir))<0.002 && (T.Gamma_angle_vs_GFL(im)>1.5 || T.Gamma_frequency_vs_GFL(im)>1.5)
            T.Classification(im)="DISTURBANCE_PATH_RESHAPING_WITHOUT_POLE_SHIFT";
        else, T.Classification(im)="POLE_AND_OR_PATH_CHANGE"; end
    end
end
writetable(T,fullfile(outDir,'SCR_Multimode_Scan.csv'));
end
