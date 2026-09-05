function T = compare_gfm_gwt_vs_mwt_path(models,chainSummary,outDir)
%COMPARE_GFM_GWT_VS_MWT_PATH 比较两种GFM的网侧—DC-link—MSC—轴系路径。
if nargin<3 || isempty(outDir), outDir=fileparts(mfilename('fullpath')); end
signals={'P_GSC','Udc','iq_MSC_ref','iq_MSC','T_e','omega_sh','T_sh'};
distNames={'Grid angle','Grid frequency'}; rr=0; gwtProj=NaN(1,2); mwtProj=NaN(1,2);
T=table('Size',[numel(signals)*numel(distNames)+numel(distNames) 6], ...
    'VariableTypes',{'string','string','double','double','double','string'}, ...
    'VariableNames',{'Disturbance','Signal','GWT_Magnitude','MWT_Magnitude','MWT_to_GWT_Ratio','Interpretation'});
for d=1:numel(distNames)
    for s=1:numel(signals)
        gwt=chainSummary(chainSummary.Mode=="GFMGWT" & chainSummary.Disturbance==string(distNames{d}) & chainSummary.Signal==string(signals{s}) & chainSummary.FrequencyReference=="Common",:);
        mwt=chainSummary(chainSummary.Mode=="VSG" & chainSummary.Disturbance==string(distNames{d}) & chainSummary.Signal==string(signals{s}) & chainSummary.FrequencyReference=="Common",:);
        a=gwt.Magnitude(1); b=mwt.Magnitude(1); ratio=b/max(a,1e-18);
        if a<1e-12 && b>=1e-12, msg='GWT在该节点已截断；MWT保持非零传播';
        elseif ratio>1.5, msg='MWT相对GWT放大'; elseif ratio<0.67, msg='MWT相对GWT衰减'; else, msg='两条路径近似一致'; end
        rr=rr+1; T.Disturbance(rr)=string(distNames{d}); T.Signal(rr)=string(signals{s}); T.GWT_Magnitude(rr)=a; T.MWT_Magnitude(rr)=b; T.MWT_to_GWT_Ratio(rr)=ratio; T.Interpretation(rr)=string(msg);
    end
end
% 补充轴系模态左特征向量对外部输入的投影，用于区分“输入未注入”和“输出被抵消”。
for k=1:numel(models)
    if ~ismember(upper(models{k}.mode),{'GFMGWT','VSG'}), continue; end
    MD=multimode_modal_data(models{k}.A,models{k}.state_names); it=multimode_pick_torsion_mode(MD); w=MD.W(:,it);
    for d=1:2
        iu=d+2; a=abs(w'*models{k}.B(:,iu));
        if strcmpi(models{k}.mode,'GFMGWT'), gwtProj(d)=a; else, mwtProj(d)=a; end %#ok<AGROW>
    end
end
for d=1:2
    rr=rr+1; T.Disturbance(rr)=string(distNames{d}); T.Signal(rr)="w_tor^H B_d"; T.GWT_Magnitude(rr)=gwtProj(d); T.MWT_Magnitude(rr)=mwtProj(d); T.MWT_to_GWT_Ratio(rr)=mwtProj(d)/max(gwtProj(d),1e-18); T.Interpretation(rr)="轴系模态输入投影";
end
writetable(T,fullfile(outDir,'GWT_MWT_Path_Difference.csv'));
end
