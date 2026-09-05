function dx=source_aligned_rhs(x,p)
%SOURCE_ALIGNED_RHS Continuous 23-state RHS sharing the retained physical
% RC-LCL measurement plane and currentmodel_continuous_controller_io laws.
Sb=p(1); Vdc0=p(2); w0=p(3); Vg=p(4); Rf=p(5); Lf=p(6); Cf=p(7); Rd=p(8); Rg=p(9); Lg=p(10); Cdc=p(11); wm0=p(12); Rs=p(13); Ld=p(14); Lq=p(15); psi=p(16); np=p(17); Kt=p(18); Jt=p(19); Jg=p(20); Ksh=p(21); Dsh=p(22); Dt=p(23); Dg=p(24); Kpdc=p(25); Kidc=p(26); Kpmi=p(27); Kimi=p(28); Kpgi=p(29); Kigi=p(30); Kpgv=p(31); Kigv=p(32); H=p(33); mp=p(34); wpf=p(35); kq=p(36); Pref=p(37); Qref=p(38); Tm0=p(39); E0=p(40); sP=p(41); ffIg=p(42); ffVpcc=p(43);
theta=x(1); wt=x(2); wg=x(3); imd=x(4); imq=x(5); xiDc=x(6); xiMid=x(7); xiMiq=x(8); Udc=x(9); Pf=x(10); Qf=x(11); wv=x(12); delta=x(13); xiVd=x(14); xiVq=x(15); xiId=x(16); xiIq=x(17); ifd=x(18); ifq=x(19); vcd=x(20); vcq=x(21); igd=x(22); igq=x(23);
we=np*wg; Tgen=Kt*imq; Tsh=Ksh*theta+Dsh*(wt-wg); wcoi=(Jt*wt+Jg*wg)/(Jt+Jg);
eDc=Vdc0-Udc; imdRef=0; imqRef=Kpdc*eDc+xiDc; eMid=-imd; eMiq=imqRef-imq;
vmd=Kpmi*eMid+xiMid+Rs*imdRef-we*Lq*imqRef; vmq=Kpmi*eMiq+xiMiq+Rs*imqRef+we*(Ld*imdRef+psi);
Pmsc=1.5*(vmd*imd+vmq*imq);
% vnode is the measured PCC voltage and drives both filter inductors;
% vcap is the capacitor voltage behind the series damping resistor.
% SPS 等效网络矩阵核对：PCC 电压为 vcap+0.1*if-0.1001*ig，
% 不能用对称 Rd*(if-ig) 替代，否则工作点功率滤波残差约为 1e-3 pu。
icapd=ifd-igd; icapq=ifq-igq; vnodeD=vcd+Rd*ifd-(Rd+1e-4)*igd; vnodeQ=vcq+Rd*ifq-(Rd+1e-4)*igq;
Pmeas=1.5*(vnodeD*igd+vnodeQ*igq); Qmeas=1.5*(vnodeQ*igd-vnodeD*igq);
c=cos(delta); s=sin(delta); vpd=c*vnodeD+s*vnodeQ; vpq=-s*vnodeD+c*vnodeQ; ifld=c*ifd+s*ifq; iflq=-s*ifd+c*ifq; igld=c*igd+s*igq; iglq=-s*igd+c*igq;
Vref=E0+kq*(Qref-Qf); evd=Vref-vpd; evq=-vpq; ifdRef=Kpgv*evd+xiVd-Cf*wv*vpq+ffIg*igld; ifqRef=Kpgv*evq+xiVq+Cf*wv*vpd+ffIg*iglq; eid=ifdRef-ifld; eiq=ifqRef-iflq;
ucd=Kpgi*eid+xiId-wv*Lf*iflq+ffVpcc*(vpd+Rf*ifld); ucq=Kpgi*eiq+xiIq+wv*Lf*ifld+ffVpcc*(vpq+Rf*iflq); uinvD=c*ucd-s*ucq; uinvQ=s*ucd+c*ucq; Pgsc=1.5*(uinvD*ifd+uinvQ*ifq);
dx=zeros(23,1); dx(1)=wt-wg; dx(2)=(Tm0*wm0/wt-Tsh-Dt*(wcoi-wm0))/Jt; dx(3)=(Tsh-Tgen-Dg*(wcoi-wm0))/Jg;
% Retained PMSM1 passive dq equations, verified directly at its internal
% voltage/current ports. Keep these signs identical to the nonlinear copy.
dx(4)=(vmd-Rs*imd+we*Lq*imq)/Ld;
dx(5)=(vmq-Rs*imq-we*(Ld*imd+psi))/Lq;
dx(6)=Kidc*eDc; dx(7)=Kimi*eMid; dx(8)=Kimi*eMiq; dx(9)=(Pmsc-Pgsc)/(Cdc*Udc); dx(10)=wpf*(Pmeas-Pf); dx(11)=wpf*(Qmeas-Qf); dx(12)=w0/(2*H*Sb)*(sP*(Pref-Pf)-(wv-w0)/mp); dx(13)=wv-w0; dx(14)=Kigv*evd; dx(15)=Kigv*evq; dx(16)=Kigi*eid; dx(17)=Kigi*eiq; Rf_eff=Rf+1e-4; dx(18)=(uinvD-vnodeD-Rf_eff*ifd+w0*Lf*ifq)/Lf; dx(19)=(uinvQ-vnodeQ-Rf_eff*ifq-w0*Lf*ifd)/Lf; dx(20)=icapd/Cf+w0*vcq; dx(21)=icapq/Cf-w0*vcd; dx(22)=(vnodeD-Vg-Rg*igd+w0*Lg*igq)/Lg; dx(23)=(vnodeQ-Rg*igq-w0*Lg*igd)/Lg;
end
