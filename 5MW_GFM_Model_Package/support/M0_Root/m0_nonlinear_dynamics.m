function [dx,y] = m0_nonlinear_dynamics(x,u,p)
%M0_NONLINEAR_DYNAMICS 23状态M0理想连续非线性模型。
%  输入u=[dTm,dPref,dQref,dOmegaGrid,dVgrid_pu,dVdcRef]'.
%  所有VSC交流电压命令直接施加；无PWM、延迟、限幅或状态机。
%#codegen

% 参数展开（顺序由m0_pack_parameters唯一规定）
Sb=p(1); VdcRef0=p(2); w0=p(3); Vg0=p(4);
Rf=p(5); Lf=p(6); Cf=p(7); Rd=p(8); Rg=p(9); Lg=p(10);
Cdc=p(11); wm0=p(12); Rs=p(13); Ld=p(14); Lq=p(15);
psi=p(16); np=p(17); Kt=p(18); Jt=p(19); Jg=p(20);
Ksh=p(21); Dsh=p(22); Dt=p(23); Dg=p(24);
Kpdc=p(25); Kidc=p(26); Kpmi=p(27); Kimi=p(28);
Kpgi=p(29); Kigi=p(30); Kpgv=p(31); Kigv=p(32);
H=p(33); mp=p(34); wpf=p(35); kq=p(36);
Pref0=p(37); Qref0=p(38); Tm0=p(39); E0=p(40); sP=p(41);
ffIg=p(42); ffVpcc=p(43);

% 状态展开
theta=x(1); wt=x(2); wg=x(3); imd=x(4); imq=x(5);
xiDc=x(6); xiMid=x(7); xiMiq=x(8); Udc=x(9);
Pf=x(10); Qf=x(11); wv=x(12); delta=x(13);
xiVd=x(14); xiVq=x(15); xiId=x(16); xiIq=x(17);
ifd=x(18); ifq=x(19); vcfd=x(20); vcfq=x(21);
igd=x(22); igq=x(23);

dTm=u(1); dPref=u(2); dQref=u(3); dwGrid=u(4);
dVgrid=u(5); dVdcRef=u(6);
% M0冻结机械输入功率而不是恒转矩。这样既移除MPPT/Pitch反馈，
% 又不会人为给公共转速模态增加恒转矩负阻尼；dTm仍用于小扰动试验。
Tm=Tm0*wm0/wt+dTm; Pref=Pref0+dPref; Qref=Qref0+dQref;
wgrid=w0+dwGrid; Vgrid=Vg0*(1+dVgrid); VdcRef=VdcRef0+dVdcRef;

%% 两质量轴系
omegaRel=wt-wg;
omegaCoi=(Jt*wt+Jg*wg)/(Jt+Jg);
Tshaft=Ksh*theta+Dsh*omegaRel;
Tgen=Kt*imq;

%% MSC Type-A DVC和连续电流环
eDc=VdcRef-Udc;
imqRef=Kpdc*eDc+xiDc;
imdRef=0;
eMid=imdRef-imd; eMiq=imqRef-imq;
we=np*wg;
vmd=Kpmi*eMid+xiMid+Rs*imdRef+we*Lq*imqRef;
vmq=Kpmi*eMiq+xiMiq+Rs*imqRef-we*(Ld*imdRef+psi);
Pmsc=-1.5*(vmd*imd+vmq*imq);  % 正值表示MSC向DC-link送入功率

%% GSC坐标变换、PCC功率和连续控制
icapd=ifd-igd; icapq=ifq-igq;
vpccd=vcfd+Rd*icapd; vpccq=vcfq+Rd*icapq;
Ppcc=1.5*(vpccd*igd+vpccq*igq);
Qpcc=1.5*(vpccq*igd-vpccd*igq);

c=cos(delta); s=sin(delta);
% 网格dq -> VSG局部dq
vpd=c*vpccd+s*vpccq;
vpq=-s*vpccd+c*vpccq;
ifld=c*ifd+s*ifq;
iflq=-s*ifd+c*ifq;
igld=c*igd+s*igq;
iglq=-s*igd+c*igq;

Vref=E0+kq*(Qref-Qf);
evd=Vref-vpd; evq=-vpq;
ifdRef=Kpgv*evd+xiVd-Cf*wv*vpq+ffIg*igld;
ifqRef=Kpgv*evq+xiVq+Cf*wv*vpd+ffIg*iglq;
eid=ifdRef-ifld; eiq=ifqRef-iflq;
% 与现有GFM代码一致：无PCC电压前馈，仅保留交叉解耦。
ucd=Kpgi*eid+xiId-wv*Lf*iflq+ffVpcc*(vpd+Rf*ifld);
ucq=Kpgi*eiq+xiIq+wv*Lf*ifld+ffVpcc*(vpq+Rf*iflq);
% VSG局部dq -> 网格dq；理想VSC直接施加命令
uinvd=c*ucd-s*ucq;
uinvq=s*ucd+c*ucq;
Pgsc=1.5*(uinvd*ifd+uinvq*ifq); % 正值表示GSC从DC-link取出功率

vgridd=Vgrid; vgridq=0;
Pgrid=1.5*(vgridd*igd+vgridq*igq);

%% 连续状态方程
dx=zeros(23,1);
dx(1)=omegaRel;
% 两个阻尼转矩均只读取同一个COI转速；因Dt/Jt=Dg/Jg，二者在
% d(omega_t-omega_g)/dt中严格相消，只锚定公共转速而不修改轴系模态。
dx(2)=(Tm-Tshaft-Dt*(omegaCoi-wm0))/Jt;
dx(3)=(Tshaft-Tgen-Dg*(omegaCoi-wm0))/Jg;
dx(4)=(vmd-Rs*imd-we*Lq*imq)/Ld;
dx(5)=(vmq-Rs*imq+we*(Ld*imd+psi))/Lq;
dx(6)=Kidc*eDc;
dx(7)=Kimi*eMid;
dx(8)=Kimi*eMiq;
dx(9)=(Pmsc-Pgsc)/(Cdc*Udc);
dx(10)=wpf*(Ppcc-Pf);
dx(11)=wpf*(Qpcc-Qf);
dx(12)=w0/(2*H*Sb)*(sP*(Pref-Pf)-(wv-wgrid)/mp);
dx(13)=wv-wgrid;
dx(14)=Kigv*evd;
dx(15)=Kigv*evq;
dx(16)=Kigi*eid;
dx(17)=Kigi*eiq;
dx(18)=(uinvd-vpccd-Rf*ifd+wgrid*Lf*ifq)/Lf;
dx(19)=(uinvq-vpccq-Rf*ifq-wgrid*Lf*ifd)/Lf;
dx(20)=(ifd-igd)/Cf+wgrid*vcfq;
dx(21)=(ifq-igq)/Cf-wgrid*vcfd;
dx(22)=(vpccd-vgridd-Rg*igd+wgrid*Lg*igq)/Lg;
dx(23)=(vpccq-vgridq-Rg*igq-wgrid*Lg*igd)/Lg;

%% 只读诊断：三个功率面和严格能量关系
dEfilter=1.5*Lf*(ifd*dx(18)+ifq*dx(19))+ ...
    1.5*Cf*(vcfd*dx(20)+vcfq*dx(21));
PfilterLoss=1.5*Rf*(ifd^2+ifq^2)+ ...
    1.5*Rd*(icapd^2+icapq^2);
filterResidual=Pgsc-Ppcc-dEfilter-PfilterLoss;
dcResidual=Cdc*Udc*dx(9)-(Pmsc-Pgsc);

y=zeros(29,1);
y(1)=Pmsc; y(2)=Pgsc; y(3)=Ppcc; y(4)=Pgrid;
y(5)=Qpcc; y(6)=Udc; y(7)=Tgen; y(8)=Tshaft;
y(9)=wt; y(10)=wg; y(11)=omegaRel; y(12)=wv; y(13)=delta;
y(14)=hypot(vpccd,vpccq); y(15)=hypot(ifd,ifq);
y(16)=hypot(igd,igq); y(17)=dcResidual; y(18)=filterResidual;
y(19)=dx(9); y(20)=Tm*wt; y(21)=Tgen*wg;
y(22)=hypot(uinvd,uinvq); y(23)=imdRef; y(24)=imqRef;
y(25)=Pf; y(26)=Qf; y(27)=Vref; y(28)=dEfilter;
y(29)=PfilterLoss;
end
