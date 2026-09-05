function R=analyze_modal_residue_decomposition(varargin)
%ANALYZE_MODAL_RESIDUE_DECOMPOSITION
% 计算 GFL/Droop-GFM/VSG-GFM 轴系模态的可观测性 O=Cv、模态可控性
% K=w^H B 以及两类轴系输出的归一化残差。只保存紧凑表格。
ip=inputParser; ip.addParameter('SaveSummary',true,@(x)islogical(x)&&isscalar(x)); ip.parse(varargin{:});
here=fileparts(mfilename('fullpath')); S=load(fullfile(here,'ThreeControl_Summary.mat'),'R'); base=S.R; assert(base.passed,'上一阶段 Gate A 未通过，停止残差分解。');
x=base.source_operating_point.x; p=base.source_operating_point.pvec; modes={'GFL','DROOP','VSG'}; labels={'GFL (ideal PLL)';'Droop-GFM';'VSG-GFM'};
dBase=[0.005*base.workpoint.Tsh_MNm(1)*1e6;0.005*p(1);deg2rad(0.2);2*pi*0.05]; names=base.disturbance_names;
C=[0 1 -1 zeros(1,20);p(21) p(22) -p(22) zeros(1,20)]; n=numel(x); nM=numel(modes);
% 四类扰动全部导出：机械转矩、气动功率、网侧相角、网侧频率。
% 这样长期汇总表与公平对比 MAT 中的四类扰动定义完全一致。
rows=numel(names)*nM; T=table('Size',[rows 13],'VariableTypes',{'string','string','double','double','double','double','double','double','double','double','double','double','double'}, ...
 'VariableNames',{'Control','Disturbance','f_tor_Hz','zeta_tor','MechanicalParticipation','O_omega_abs','O_Tsh_abs','K_mech_abs','K_aero_abs','K_angle_abs','K_freq_abs','R_omega_abs','R_Tsh_abs'});
M=cell(nM,1); row=0;
for k=1:nM
 [A,Bbar]=linearizeMode(x,p,modes{k},dBase); [lam,f,zeta,eta,v,wleft]=pickTorsionalMode(A); O=C*v; K=wleft'*Bbar; Res=[O(1)*K;O(2)*K];
 for j=1:numel(names)
  row=row+1; T.Control(row)=string(labels{k}); T.Disturbance(row)=string(names{j}); T.f_tor_Hz(row)=f; T.zeta_tor(row)=zeta; T.MechanicalParticipation(row)=eta; T.O_omega_abs(row)=abs(O(1)); T.O_Tsh_abs(row)=abs(O(2)); T.K_mech_abs(row)=abs(K(1)); T.K_aero_abs(row)=abs(K(2)); T.K_angle_abs(row)=abs(K(3)); T.K_freq_abs(row)=abs(K(4)); T.R_omega_abs(row)=abs(Res(1,j)); T.R_Tsh_abs(row)=abs(Res(2,j));
 end
 M{k}=struct('Control',labels{k},'A',A,'Bbar',Bbar,'lambda',lam,'f',f,'zeta',zeta,'eta',eta,'O',O,'K',K,'Residue',Res,'x',x,'p',p,'dBase',dBase);
end
R=struct('base',base,'table',T,'models',{M},'disturbance_names',{names},'dBase',dBase);
if ip.Results.SaveSummary, writetable(T,fullfile(here,'Modal_Residue_Decomposition_Summary.csv')); end
end

function [A,Bbar]=linearizeMode(x,p,mode,dBase)
n=numel(x); A=zeros(n); Bbar=zeros(n,4);
for j=1:n
 h=1e-6*max(abs(x(j)),1); xp=x; xm=x; xp(j)=xp(j)+h; xm(j)=xm(j)-h;
 A(:,j)=(source_aligned_rhs_control(xp,p,mode,zeros(4,1))-source_aligned_rhs_control(xm,p,mode,zeros(4,1)))/(2*h);
end
for j=1:4
 dp=zeros(4,1); dp(j)=dBase(j); Bbar(:,j)=(source_aligned_rhs_control(x,p,mode,dp)-source_aligned_rhs_control(x,p,mode,-dp))/2;
end
end

function [lam,f,zeta,eta,v,wleft]=pickTorsionalMode(A)
[V,D]=eig(A); ev=diag(D); [W,Dl]=eig(A'); el=diag(Dl); cand=find(imag(ev)>0 & abs(imag(ev))/(2*pi)>1 & abs(imag(ev))/(2*pi)<5); assert(~isempty(cand),'No torsional candidate.');
etaAll=zeros(numel(cand),1); leftCell=cell(numel(cand),1);
for k=1:numel(cand)
 i=cand(k); [~,j]=min(abs(el-conj(ev(i)))); v0=V(:,i); w0=W(:,j); a=w0'*v0; w0=w0/conj(a); leftCell{k}=w0; etaAll(k)=sum(abs(v0(1:3).*w0(1:3)))/max(sum(abs(v0.*w0)),eps);
end
[eta,ii]=max(etaAll); i=cand(ii); lam=ev(i); v=V(:,i); wleft=leftCell{ii}; f=abs(imag(lam))/(2*pi); zeta=-real(lam)/max(abs(lam),eps);
end
