function idx = multimode_pick_torsion_mode(M)
%MULTIMODE_PICK_TORSION_MODE 选择 1-5 Hz 内机械状态参与度最大的正频率模态。
cand=find(imag(M.lambda)>1e-8 & abs(imag(M.lambda))/(2*pi)>1 & abs(imag(M.lambda))/(2*pi)<5);
assert(~isempty(cand),'未找到 1-5 Hz 的轴系候选模态。');
score=sum(M.participation(1:3,cand),1); [~,j]=max(score); idx=cand(j);
end
