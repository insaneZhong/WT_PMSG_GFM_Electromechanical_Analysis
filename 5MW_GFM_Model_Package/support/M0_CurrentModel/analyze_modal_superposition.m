function T = analyze_modal_superposition(details,outDir)
%ANALYZE_MODAL_SUPERPOSITION 在完整SSM第一主峰时刻比较最小模态集合的同相/反相贡献。
if nargin<2 || isempty(outDir), outDir=fileparts(mfilename('fullpath')); end
maxRows=sum(arrayfun(@(q)numel(q.selected),details)); rr=0;
T=table('Size',[max(maxRows,1) 10], ...
 'VariableTypes',{'string','string','string','string','string','double','double','double','double','string'}, ...
 'VariableNames',{'Architecture','Disturbance','Output','ModeID','PhysicalClass','ResidueMagnitude','ResiduePhase_deg','ContributionAtPeak','ContributionRatio','ConstructiveOrDestructive'});
for z=1:numel(details)
    D=details(z); if isempty(D.selected), continue; end
    ix=find(D.t>=0.10); [~,jp]=max(abs(D.full(ix))); ip=ix(jp); tau=D.t(ip)-0.10; total=D.full(ip);
    for q=1:numel(D.selected)
        lam=D.lambda(q); term=D.residue(q)/lam*(exp(lam*tau)-1); if imag(lam)>1e-8, val=2*real(term); else, val=real(term); end
        rr=rr+1; T.Architecture(rr)=string(D.architecture); T.Disturbance(rr)=string(D.disturbance); T.Output(rr)=string(D.output); T.ModeID(rr)=string(sprintf('M%02d',D.selected(q)));
        T.PhysicalClass(rr)=string(D.physical_class{q}); T.ResidueMagnitude(rr)=abs(D.residue(q)); T.ResiduePhase_deg(rr)=rad2deg(angle(D.residue(q))); T.ContributionAtPeak(rr)=val; T.ContributionRatio(rr)=abs(val)/max(abs(total),eps);
        T.ConstructiveOrDestructive(rr)=iffLocal(sign(val)==sign(total),"CONSTRUCTIVE","DESTRUCTIVE");
    end
end
T=T(1:rr,:); writetable(T,fullfile(outDir,'Modal_Superposition_At_FirstPeak.csv'));
end

function v=iffLocal(c,a,b), if c, v=a; else, v=b; end, end
