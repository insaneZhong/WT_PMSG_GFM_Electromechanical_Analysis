function M = multimode_modal_data(A,stateNames)
%MULTIMODE_MODAL_DATA 归一化左右特征向量、参与因子及物理分类。
if nargin<2, stateNames=multimode_state_names(); end
[V,D]=eig(A); lam=diag(D);
% inv(V) 的第 k 行就是满足 w_k^H v_j=delta_kj 的左特征向量行。
% 这比在 eig(A') 的共轭成对特征值之间做最近邻匹配更稳健。
Winv=V\eye(size(V));
n=numel(lam); Wm=zeros(size(V)); part=zeros(n,n);
for k=1:n
    v=V(:,k); w=conj(Winv(k,:)).';
    q=w'*v; assert(abs(q)>1e-12,'左右特征向量归一化失败。');
    w=w/conj(q); % 防御性归一化：使 w^H v=1。
    Wm(:,k)=w; part(:,k)=abs(v.*conj(w));
end
classes=stateClasses(numel(stateNames)); labels=cell(n,1); topStates=cell(n,1);
for k=1:n
    score=zeros(5,1);
    for c=1:5, score(c)=sum(part(classes==c,k)); end
    [~,ic]=max(score); labels{k}=className(ic);
    [~,ix]=sort(part(:,k),'descend'); take=ix(1:min(3,n)); topStates{k}=strjoin(stateNames(take),',');
end
M=struct('lambda',lam,'V',V,'W',Wm,'participation',part,'physical_class',{labels},'top_states',{topStates});
end

function c=stateClasses(n)
c=5*ones(n,1); c(1:3)=1; c([5 6 9])=2; c(10:13)=3; c(14:23)=4;
end
function s=className(i)
s={'TOR','DC','SYNC','GSC','OTHER'}; s=s{i};
end
