function T = analyze_frequency_vs_angle_excitation(models,outDir)
%ANALYZE_FREQUENCY_VS_ANGLE_EXCITATION 比较各主导模态的 w^H B_f 与 w^H B_theta。
if nargin<2 || isempty(outDir), outDir=fileparts(mfilename('fullpath')); end
maxRank=10; rr=0; T=table('Size',[numel(models)*maxRank 7], ...
 'VariableTypes',{'string','string','double','double','double','string','string'}, ...
 'VariableNames',{'Architecture','ModeID','FreqInputProjection','AngleInputProjection','Ratio','PhysicalClass','TopParticipatingStates'});
for a=1:numel(models)
    L=models{a}; MD=multimode_modal_data(L.A,L.state_names); cand=find((imag(MD.lambda)>1e-8) | (abs(imag(MD.lambda))<=1e-8 & abs(MD.lambda)>1e-6));
    score=zeros(numel(cand),1); bf=zeros(numel(cand),1); bt=zeros(numel(cand),1);
    for q=1:numel(cand), k=cand(q); bf(q)=abs(MD.W(:,k)'*L.B(:,4)); bt(q)=abs(MD.W(:,k)'*L.B(:,3)); score(q)=max(bf(q),bt(q)); end
    [~,ord]=sort(score,'descend');
    for q=1:min(maxRank,numel(ord))
        k=cand(ord(q)); rr=rr+1; T.Architecture(rr)=string(L.label); T.ModeID(rr)=string(sprintf('M%02d',k)); T.FreqInputProjection(rr)=bf(ord(q)); T.AngleInputProjection(rr)=bt(ord(q)); T.Ratio(rr)=bf(ord(q))/max(bt(ord(q)),1e-18); T.PhysicalClass(rr)=string(MD.physical_class{k}); T.TopParticipatingStates(rr)=string(MD.top_states{k});
    end
end
T=T(1:rr,:); writetable(T,fullfile(outDir,'Frequency_vs_Angle_Excitation_Explanation.csv'));
end
