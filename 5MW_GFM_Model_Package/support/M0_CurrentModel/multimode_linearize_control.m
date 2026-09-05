function L = multimode_linearize_control(x,p,mode,flags)
%MULTIMODE_LINEARIZE_CONTROL 统一的23状态数值线性化与内部节点输出线性化。
if nargin<4 || isempty(flags), flags=struct; end
n=numel(x); nd=4;
[y0,names,units]=source_aligned_internal_outputs_control(x,p,mode,zeros(nd,1),flags);
ny=numel(y0); A=zeros(n); B=zeros(n,nd); C=zeros(ny,n); D=zeros(ny,nd);
for j=1:n
    h=1e-6*max(abs(x(j)),1); e=zeros(n,1); e(j)=h;
    A(:,j)=(source_aligned_rhs_control(x+e,p,mode,zeros(nd,1),flags)-source_aligned_rhs_control(x-e,p,mode,zeros(nd,1),flags))/(2*h);
    C(:,j)=(source_aligned_internal_outputs_control(x+e,p,mode,zeros(nd,1),flags)-source_aligned_internal_outputs_control(x-e,p,mode,zeros(nd,1),flags))/(2*h);
end
hd=[1;1;1e-6;1e-4];
for j=1:nd
    h=hd(j); e=zeros(nd,1); e(j)=h;
    B(:,j)=(source_aligned_rhs_control(x,p,mode,e,flags)-source_aligned_rhs_control(x,p,mode,-e,flags))/(2*h);
    D(:,j)=(source_aligned_internal_outputs_control(x,p,mode,e,flags)-source_aligned_internal_outputs_control(x,p,mode,-e,flags))/(2*h);
end
L=struct('A',A,'B',B,'C',C,'D',D,'x0',x,'p',p,'mode',char(mode),'flags',flags,'y0',y0,'output_names',{names},'output_units',{units}, ...
    'state_names',{multimode_state_names()},'input_names',{{'DeltaTm','DeltaPaero','DeltaThetaGrid','DeltaOmegaGrid'}},'input_units',{{'N m','W','rad','rad/s'}});
end
