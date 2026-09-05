function [T,firstDivergence] = analyze_grid_to_shaft_transfer_chain(models,outDir)
%ANALYZE_GRID_TO_SHAFT_TRANSFER_CHAIN
% 在各自轴系频率与共同 2.4942 Hz 下，计算网侧角度/频率扰动至逐级节点的传递函数。
if nargin<2 || isempty(outDir), outDir=fileparts(mfilename('fullpath')); end
signals={'P_GSC','Udc','iq_MSC_ref','iq_MSC','T_e','omega_sh','T_sh','P_f'};
distNames={'Grid angle','Grid frequency'}; inIdx=[3 4]; fRef=2.4942;
nRows=numel(models)*numel(inIdx)*2*numel(signals); rr=0;
T=table('Size',[nRows 10], ...
    'VariableTypes',{'string','string','string','string','string','double','double','double','double','string'}, ...
    'VariableNames',{'Architecture','Mode','Disturbance','Signal','FrequencyReference','Frequency_Hz','Magnitude','Phase_deg','GainRatio_vs_GFL','FirstDivergenceNode'});
for k=1:numel(models)
    L=models{k}; MD=multimode_modal_data(L.A,L.state_names); it=multimode_pick_torsion_mode(MD); fSelf=abs(imag(MD.lambda(it)))/(2*pi);
    sigIdx=zeros(size(signals)); for q=1:numel(signals), sigIdx(q)=find(strcmp(L.output_names,signals{q}),1); end
    for d=1:numel(inIdx)
        for fr=1:2
            f=iffLocal(fr==1,fSelf,fRef); refName=iffLocal(fr==1,'Self','Common');
            H=L.C*((1i*2*pi*f*eye(size(L.A))-L.A)\L.B)+L.D;
            for q=1:numel(signals)
                rr=rr+1; T.Architecture(rr)=string(L.label); T.Mode(rr)=string(L.mode);
                T.Disturbance(rr)=string(distNames{d}); T.Signal(rr)=string(signals{q}); T.FrequencyReference(rr)=string(refName);
                T.Frequency_Hz(rr)=f; T.Magnitude(rr)=abs(H(sigIdx(q),inIdx(d))); T.Phase_deg(rr)=rad2deg(angle(H(sigIdx(q),inIdx(d))));
                T.GainRatio_vs_GFL(rr)=NaN; T.FirstDivergenceNode(rr)="";
            end
        end
    end
end
for r=1:height(T)
    i0=find(T.Mode=="GFL" & T.Disturbance(r)==T.Disturbance & T.Signal(r)==T.Signal & T.FrequencyReference(r)==T.FrequencyReference,1);
    T.GainRatio_vs_GFL(r)=T.Magnitude(r)/max(T.Magnitude(i0),1e-18);
end
firstDivergence=table('Size',[numel(models)*numel(distNames) 3], ...
    'VariableTypes',{'string','string','string'},'VariableNames',{'Architecture','Disturbance','First_Divergence_Node'}); fr=0;
for k=1:numel(models)
    for d=1:numel(distNames)
        mode=string(models{k}.mode); ix=find(T.Mode==mode & T.Disturbance==string(distNames{d}) & T.FrequencyReference=="Common");
        node="NONE";
        if mode~="GFL"
            for q=1:numel(signals)
                iq=ix(T.Signal(ix)==string(signals{q})); ratio=T.GainRatio_vs_GFL(iq);
                if ratio>1.5 || ratio<0.67, node=string(signals{q}); break; end
            end
        end
        T.FirstDivergenceNode(ix)=node;
        fr=fr+1; firstDivergence.Architecture(fr)=string(models{k}.label); firstDivergence.Disturbance(fr)=string(distNames{d}); firstDivergence.First_Divergence_Node(fr)=node;
    end
end
writetable(T,fullfile(outDir,'GridToShaft_TransferChain_Summary.csv'));
end

function v=iffLocal(c,a,b), if c, v=a; else, v=b; end, end
