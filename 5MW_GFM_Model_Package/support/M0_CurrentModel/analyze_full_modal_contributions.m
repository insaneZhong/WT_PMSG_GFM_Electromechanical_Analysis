function [T,modalCache] = analyze_full_modal_contributions(models,outDir)
%ANALYZE_FULL_MODAL_CONTRIBUTIONS
% 对 grid-angle/grid-frequency 至 omega_sh/T_sh 的全模态阶跃贡献排序。
if nargin<2 || isempty(outDir), outDir=fileparts(mfilename('fullpath')); end
distNames={'Grid angle','Grid frequency'}; inIdx=[3 4]; amps=[deg2rad(0.2),2*pi*0.05];
outNames={'omega_sh','T_sh'}; maxRank=10; nRows=numel(models)*numel(inIdx)*numel(outNames)*maxRank; rr=0;
T=table('Size',[nRows 13], ...
 'VariableTypes',{'string','string','string','double','string','double','double','double','double','double','double','string','double'}, ...
 'VariableNames',{'Architecture','Disturbance','Output','Rank','ModeID','PoleReal','PoleImag','Frequency_Hz','DampingRatio','ResidueMagnitude','ResiduePhase_deg','PhysicalClass','StepContributionMetric'});
T.TopParticipatingStates=strings(nRows,1); modalCache=cell(numel(models),1);
for a=1:numel(models)
    L=models{a}; MD=multimode_modal_data(L.A,L.state_names); modalCache{a}=MD;
    cand=localCandidates(MD.lambda);
    for d=1:numel(inIdx)
        for o=1:numel(outNames)
            iy=find(strcmp(L.output_names,outNames{o}),1); score=zeros(numel(cand),1); R=zeros(numel(cand),1);
            for q=1:numel(cand)
                k=cand(q); R(q)=L.C(iy,:)*MD.V(:,k)*(MD.W(:,k)'*L.B(:,inIdx(d)));
                score(q)=localPairFactor(MD.lambda(k))*abs(R(q)/MD.lambda(k))*amps(d);
            end
            [~,ord]=sort(score,'descend'); nt=min(maxRank,numel(ord));
            for q=1:nt
                k=cand(ord(q)); lam=MD.lambda(k); rr=rr+1;
                T.Architecture(rr)=string(L.label); T.Disturbance(rr)=string(distNames{d}); T.Output(rr)=string(outNames{o}); T.Rank(rr)=q;
                T.ModeID(rr)=string(sprintf('M%02d',k)); T.PoleReal(rr)=real(lam); T.PoleImag(rr)=imag(lam); T.Frequency_Hz(rr)=abs(imag(lam))/(2*pi);
                T.DampingRatio(rr)=-real(lam)/max(abs(lam),eps); T.ResidueMagnitude(rr)=abs(R(ord(q))); T.ResiduePhase_deg(rr)=rad2deg(angle(R(ord(q))));
                T.PhysicalClass(rr)=string(MD.physical_class{k}); T.TopParticipatingStates(rr)=string(MD.top_states{k}); T.StepContributionMetric(rr)=score(ord(q));
            end
        end
    end
end
T=T(1:rr,:); writetable(T,fullfile(outDir,'Full_Modal_Contribution_Ranking.csv'));
end

function cand=localCandidates(lam)
% 实极点单独保留；共轭对只保留正虚部代表元；剔除无动态贡献的零极点。
cand=find((imag(lam)>1e-8) | (abs(imag(lam))<=1e-8 & abs(lam)>1e-6));
end
function f=localPairFactor(lam), f=iffLocal(imag(lam)>1e-8,2,1); end
function v=iffLocal(c,a,b), if c, v=a; else, v=b; end, end
