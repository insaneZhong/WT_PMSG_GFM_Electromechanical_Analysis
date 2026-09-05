function S = multimode_scan_metrics(L)
%MULTIMODE_SCAN_METRICS 统一提取轴系极点、控制类别残差及关键传递函数摘要。
MD=multimode_modal_data(L.A,L.state_names); it=multimode_pick_torsion_mode(MD); lam=MD.lambda(it); iw=find(strcmp(L.output_names,'omega_sh'),1);
S=struct('f_tor',abs(imag(lam))/(2*pi),'zeta_tor',-real(lam)/abs(lam),'pole_real',real(lam),'pole_imag',imag(lam));
for d=1:2
    iu=d+2; R=L.C(iw,:)*MD.V(:,it)*(MD.W(:,it)'*L.B(:,iu));
    S.Rtor(d)=abs(R); S.Rtor_step(d)=abs(R/lam);
    S.Rsync(d)=localClassMax(L,MD,iw,iu,'SYNC'); S.Rdc(d)=localClassMax(L,MD,iw,iu,'DC');
end
f=S.f_tor; H=L.C*((1i*2*pi*f*eye(size(L.A))-L.A)\L.B)+L.D;
S.Gudc_f=abs(H(find(strcmp(L.output_names,'Udc'),1),4));
S.Giqref_f=abs(H(find(strcmp(L.output_names,'iq_MSC_ref'),1),4));
S.Gte_f=abs(H(find(strcmp(L.output_names,'T_e'),1),4));
d=zeros(4,1); d(4)=2*pi*.05; [~,~,Y]=multimode_simulate_linear_step(L,d,.10,10,5001); S.PeakOmegaSh_f=max(abs(Y(:,iw)));
end

function v=localClassMax(L,MD,iy,iu,name)
ix=find(strcmp(MD.physical_class,name)); v=0;
for k=ix(:).'
    if imag(MD.lambda(k))< -1e-8, continue; end
    R=L.C(iy,:)*MD.V(:,k)*(MD.W(:,k)'*L.B(:,iu)); v=max(v,abs(R));
end
end
