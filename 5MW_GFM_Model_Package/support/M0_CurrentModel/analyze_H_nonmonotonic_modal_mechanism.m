function R = analyze_H_nonmonotonic_modal_mechanism(outDir)
%ANALYZE_H_NONMONOTONIC_MODAL_MECHANISM
% 使用特征向量MAC与参与因子相似性跟踪H扫描中的同一物理模态。
% 仅分析VSG/MWT；不按频率最近原则重新匹配模态。
if nargin<1 || isempty(outDir), outDir=fileparts(mfilename('fullpath')); end
[models,base]=prepare_multimode_models(); ix=find(strcmpi(base.models,'VSG'),1); assert(~isempty(ix),'未找到VSG/MWT基准。');
p0=base.parameter_vector; xSeed=models{ix}.x0; Hs=[1 1.5 2 2.5 3 3.5 4 5];
trackNames={'TOR','SYNC','DC','GSC','SPEED'}; nTrack=numel(trackNames); rows=numel(Hs)*nTrack; rr=0;
T=table('Size',[rows 14], ...
    'VariableTypes',{'double','double','string','string','double','double','double','double','double','double','double','double','double','string'}, ...
    'VariableNames',{'H_s','H_Factor','TrackedMode','ModeID','Frequency_Hz','Damping','PoleReal','PoleImag','ResidueMagnitude','ResiduePhase_deg','InputProjection','OutputObservability','TrackingScore','PhysicalClass'});
prevV=cell(nTrack,1); prevPart=cell(nTrack,1); prevIdx=zeros(nTrack,1); xNow=xSeed;
for h=1:numel(Hs)
    p=p0; p(33)=Hs(h); [xNow,meta]=solve_multimode_control_equilibrium(xNow,p,'VSG',struct); assert(meta.pass,'H=%.3g 平衡点失败。',Hs(h));
    L=multimode_linearize_control(xNow,p,'VSG',struct); M=multimode_modal_data(L.A,L.state_names); cand=localCandidates(M.lambda);
    iout=find(strcmp(L.output_names,'omega_sh'),1); assert(~isempty(iout),'缺少omega_sh输出。');
    for q=1:nTrack
        if h==1, k=localInitialMode(M,trackNames{q},cand,prevIdx); score=1;
        else, [k,score]=localTrackMode(M,trackNames{q},cand,prevV{q},prevPart{q}); end
        prevIdx(q)=k; prevV{q}=M.V(:,k); prevPart{q}=M.participation(:,k);
        lam=M.lambda(k); proj=M.W(:,k)'*L.B(:,4); obs=L.C(iout,:)*M.V(:,k); residue=obs*proj;
        rr=rr+1; T.H_s(rr)=Hs(h); T.H_Factor(rr)=Hs(h)/p0(33); T.TrackedMode(rr)=string(trackNames{q}); T.ModeID(rr)=string(sprintf('M%02d',k));
        T.Frequency_Hz(rr)=abs(imag(lam))/(2*pi); T.Damping(rr)=-real(lam)/max(abs(lam),eps); T.PoleReal(rr)=real(lam); T.PoleImag(rr)=imag(lam);
        T.ResidueMagnitude(rr)=abs(residue); T.ResiduePhase_deg(rr)=rad2deg(angle(residue)); T.InputProjection(rr)=abs(proj); T.OutputObservability(rr)=abs(obs);
        T.TrackingScore(rr)=score; T.PhysicalClass(rr)=string(M.physical_class{k});
    end
end
writetable(T,fullfile(outDir,'H_Nonmonotonic_Modal_Mechanism.csv'));
tor=T(T.TrackedMode=="TOR",:); [~,ip]=max(tor.ResidueMagnitude); ref=tor(abs(tor.H_s-p0(33))==min(abs(tor.H_s-p0(33)),[],'all'),:); ref=ref(1,:);
poleChange=max(abs(tor.Damping-ref.Damping)/max(abs(ref.Damping),eps)); [~,ii]=max(tor.InputProjection); inputPeakH=tor.H_s(ii);
if poleChange<0.05 && abs(inputPeakH-tor.H_s(ip))<1e-10
    mech="INPUT_PROJECTION_DOMINATED";
elseif poleChange<0.05
    mech="OBSERVABILITY_OR_SUPERPOSITION_DOMINATED";
else
    mech="ELECTRICAL_POLE_EFFECT_OR_COMBINED";
end
fid=fopen(fullfile(outDir,'H_Nonmonotonic_Mechanism_CN.md'),'w'); assert(fid>0,'无法写入H机理报告。');
fprintf(fid,'# H非单调残差机理\n\n');
fprintf(fid,'仅使用VSG/MWT理想连续同源23状态模型。模态按MAC和参与因子连续跟踪，未按频率最近重选。\n\n');
fprintf(fid,'TOR残差峰值位于 H=%.3g s；TOR输入投影峰值位于 H=%.3g s；TOR阻尼最大相对变化为 %.3f%%。\n\n',tor.H_s(ip),inputPeakH,100*poleChange);
fprintf(fid,'当前分类：**%s**。这是一种该工作点下的扰动投影/观测结论，不等价于已证明模态混合或共振。\n',mech); fclose(fid);
R=struct('summary',T,'tor',tor,'mechanism',char(mech),'peak_H',tor.H_s(ip),'input_peak_H',inputPeakH,'pole_change',poleChange);
end

function cand=localCandidates(lam), cand=find(imag(lam)>=-1e-8 & abs(lam)>1e-7); end
function k=localInitialMode(M,name,cand,used)
P=M.participation; score=zeros(numel(cand),1);
for q=1:numel(cand)
    i=cand(q);
    switch name
        case 'TOR', score(q)=sum(P(1:3,i))*(abs(imag(M.lambda(i)))/(2*pi)>1);
        case 'SYNC', score(q)=sum(P(12:13,i));
        case 'DC', score(q)=sum(P([6 9],i));
        case 'GSC', score(q)=sum(P(14:23,i));
        case 'SPEED', score(q)=sum(P(2:3,i))/(1+abs(imag(M.lambda(i))));
    end
    if any(used==i), score(q)=-inf; end
end
[~,j]=max(score); k=cand(j);
end
function [k,best]=localTrackMode(M,name,cand,vPrev,pPrev)
score=-inf(numel(cand),1);
for q=1:numel(cand)
    i=cand(q); v=M.V(:,i); pp=M.participation(:,i);
    mac=abs(vPrev'*v)^2/max(real(vPrev'*vPrev)*real(v'*v),eps); ps=(pPrev'*pp)/max(norm(pPrev)*norm(pp),eps);
    classBonus=0.05*localClassAffinity(M,i,name); score(q)=0.70*mac+0.25*ps+classBonus;
end
[best,j]=max(real(score)); k=cand(j);
end
function a=localClassAffinity(M,i,name)
p=M.participation(:,i); switch name
    case 'TOR', a=sum(p(1:3)); case 'SYNC', a=sum(p(12:13)); case 'DC', a=sum(p([6 9])); case 'GSC', a=sum(p(14:23)); case 'SPEED', a=sum(p(2:3)); otherwise, a=0;
end
end
