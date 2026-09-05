function S = mechanism_transition_point(p,mode,xSeed,flags,needSolve)
%MECHANISM_TRANSITION_POINT 单一参数点的极点、残差、复转矩和预测峰值摘要。
% 只返回标量摘要；不保存时序或完整工作区。
if nargin<4 || isempty(flags), flags=struct; end
if nargin<5, needSolve=true; end
if needSolve, [x,meta]=solve_multimode_control_equilibrium(xSeed,p,mode,flags); else, x=xSeed; meta=struct('pass',true,'residual_norm',0); end
assert(meta.pass,'工作点求解未通过。');
L=multimode_linearize_control(x,p,mode,flags); M=multimode_modal_data(L.A,L.state_names); it=multimode_pick_torsion_mode(M);
lam=M.lambda(it); f=abs(imag(lam))/(2*pi); iw=find(strcmp(L.output_names,'omega_sh'),1);
assert(~isempty(iw),'缺少omega_sh输出。');
rtor=L.C(iw,:)*M.V(:,it)*(M.W(:,it)'*L.B(:,4));
S=struct('x',x,'L',L,'M',M,'f_tor',f,'zeta_tor',-real(lam)/max(abs(lam),eps), ...
    'pole_real',real(lam),'pole_imag',imag(lam),'Rtor_frequency',abs(rtor), ...
    'Rdc_frequency',localClassResidue(L,M,iw,4,'DC'),'Rsync_frequency',localClassResidue(L,M,iw,4,'SYNC'), ...
    'Rgsc_frequency',localClassResidue(L,M,iw,4,'GSC'),'PeakOmegaSh_frequency',localStepPeak(L,iw), ...
    'EquilibriumResidual',meta.residual_norm);
[S.D_e,S.K_e]=localComplexTorque(L,p,f); S.G_Udc_to_iqref=localInternalGain(L,'iq_MSC_ref',9,f); S.G_Udc_to_Te=localInternalGain(L,'T_e',9,f);
end

function v=localClassResidue(L,M,iy,iu,name)
ix=find(strcmp(M.physical_class,name)); v=0;
for k=ix(:).'
    if imag(M.lambda(k))<-1e-8, continue, end
    r=L.C(iy,:)*M.V(:,k)*(M.W(:,k)'*L.B(:,iu)); v=max(v,abs(r));
end
end

function [de,ke]=localComplexTorque(L,p,f)
% 机械反馈开环：将omega_g作为外部速度输入，仅保留电气/控制状态4:23。
idx=4:23; cte=zeros(1,numel(idx)); cte(2)=p(18); w=2*pi*f;
g=cte*((1i*w*eye(numel(idx))-L.A(idx,idx))\L.A(idx,3)); de=real(g); ke=-w*imag(g);
end

function g=localInternalGain(L,name,inputState,f)
% 对局部Udc状态注入的开环频率灵敏度，仅表示通道幅值，不替代闭环传递函数。
iy=find(strcmp(L.output_names,name),1); w=2*pi*f; e=zeros(size(L.A,1),1); e(inputState)=1;
g=abs(L.C(iy,:)*((1i*w*eye(size(L.A))-L.A)\e));
end

function y=localStepPeak(L,iy)
d=zeros(4,1); d(4)=2*pi*.05; [~,~,Y]=multimode_simulate_linear_step(L,d,.10,10,3001); y=max(abs(Y(:,iy)));
end
