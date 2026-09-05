function varargout=s7a_discrete_average_core(action,varargin)
%S7A_DISCRETE_AVERAGE_CORE  M0方程推导的离散平均参考实现核心。
%
% action='step'     : z(k+1)=Phi(z(k),u(k))
% action='output'   : 在采样时刻计算29个M0诊断输出
% action='commands' : 由状态和输入计算四个VSC交流电压命令
%
% z=[x(23); command(k); command(k-1)]，物理plant在一个采样周期内
% 连续积分，11个软件/控制状态按Forward-Euler更新。该文件服务于
% S7A参考数字平均模型，不复刻旧C/S-Function，也不包含PWM/限幅。
switch lower(char(action))
    case 'step'
        varargout{1}=stepMap(varargin{:});
    case 'output'
        varargout{1}=sampleOutput(varargin{:});
    case 'commands'
        varargout{1}=referenceCommands(varargin{:});
    otherwise
        error('Unknown S7A core action: %s',char(action));
end
end

function zn=stepMap(z,u,p,Ts,tau)
x=z(1:23); oldCmd=z(24:27); oldCmd2=z(28:31);
q=algebraic(x,u,p); newCmd=[q.vmd;q.vmq;q.uinvd;q.uinvq];
tau=max(0,double(tau)); Ts=double(Ts);
if tau<=Ts
    x1=integratePlant(x,oldCmd,u,p,tau);
    x2=integratePlant(x1,newCmd,u,p,Ts-tau);
else
    nDelay=floor(tau/Ts); remDelay=tau-nDelay*Ts;
    if nDelay==1
        x1=integratePlant(x,oldCmd2,u,p,min(remDelay,Ts));
        x2=integratePlant(x1,oldCmd,u,p,Ts-min(remDelay,Ts));
    else
        error('S7A reference implementation supports tau<=1.5Ts only.');
    end
end
% 11个软件/控制状态：采样时刻更新，采用Forward-Euler候选实现。
x2(6)=x(6)+Ts*q.dxDc;
x2(7)=x(7)+Ts*q.dxMid;
x2(8)=x(8)+Ts*q.dxMiq;
x2(10)=x(10)+Ts*q.dxPf;
x2(11)=x(11)+Ts*q.dxQf;
x2(12)=x(12)+Ts*q.dxWv;
x2(13)=x(13)+Ts*q.dxDelta;
x2(14)=x(14)+Ts*q.dxVd;
x2(15)=x(15)+Ts*q.dxVq;
x2(16)=x(16)+Ts*q.dxId;
x2(17)=x(17)+Ts*q.dxIq;
zn=[x2;newCmd;oldCmd];
end

function cmd=referenceCommands(x,u,p)
q=algebraic(x,u,p); cmd=[q.vmd;q.vmq;q.uinvd;q.uinvq];
end

function y=sampleOutput(z,u,p)
x=z(1:23); q=algebraic(x,u,p); cmd=[q.vmd;q.vmq;q.uinvd;q.uinvq];
P=unpack(p);
theta=x(1); wt=x(2); wg=x(3); imd=x(4); imq=x(5); Udc=x(9);
ifd=x(18); ifq=x(19); vcfd=x(20); vcfq=x(21); igd=x(22); igq=x(23);
Tm=P.Tm0*P.wm0/max(wt,eps)+u(1); wgrid=P.w0+u(4); Vgrid=P.Vg0*(1+u(5));
omegaRel=wt-wg; Tshaft=P.Ksh*theta+P.Dsh*omegaRel; Tgen=P.Kt*imq;
vpccd=vcfd+P.Rd*(ifd-igd); vpccq=vcfq+P.Rd*(ifq-igq);
Pmsc=q.Pmsc; Pgsc=q.Pgsc; Ppcc=q.Ppcc; Qpcc=q.Qpcc;
dxTmp=plantRhs(x,cmd,u,p); dEfilter=1.5*P.Lf*(ifd*dxTmp(18)+ifq*dxTmp(19))+ ...
    1.5*P.Cf*(vcfd*dxTmp(20)+vcfq*dxTmp(21));
PfilterLoss=1.5*P.Rf*(ifd^2+ifq^2)+1.5*P.Rd*((ifd-igd)^2+(ifq-igq)^2);
dcResidual=P.Cdc*Udc*dxTmp(9)-(Pmsc-Pgsc);
filterResidual=Pgsc-Ppcc-dEfilter-PfilterLoss;
y=zeros(29,1);
y(1)=Pmsc; y(2)=Pgsc; y(3)=Ppcc; y(4)=1.5*(Vgrid*igd); y(5)=Qpcc; y(6)=Udc;
y(7)=Tgen; y(8)=Tshaft; y(9)=wt; y(10)=wg; y(11)=omegaRel;
y(12)=x(12); y(13)=x(13); y(14)=hypot(vpccd,vpccq); y(15)=hypot(ifd,ifq);
y(16)=hypot(igd,igq); y(17)=dcResidual; y(18)=filterResidual; y(19)=dxTmp(9);
y(20)=Tm*wt; y(21)=Tgen*wg; y(22)=hypot(cmd(3),cmd(4));
y(23)=q.imdRef; y(24)=q.imqRef; y(25)=x(10); y(26)=x(11); y(27)=q.Vref;
y(28)=dEfilter; y(29)=PfilterLoss;
end

function q=algebraic(x,u,p)
P=unpack(p);
theta=x(1); wt=x(2); wg=x(3); imd=x(4); imq=x(5); xiDc=x(6);
xiMid=x(7); xiMiq=x(8); Udc=x(9); Pf=x(10); Qf=x(11);
wv=x(12); delta=x(13); xiVd=x(14); xiVq=x(15); xiId=x(16); xiIq=x(17);
ifd=x(18); ifq=x(19); vcfd=x(20); vcfq=x(21); igd=x(22); igq=x(23);
dPref=u(2); dQref=u(3); dwGrid=u(4); dVgrid=u(5); dVdcRef=u(6);
Pref=P.Pref0+dPref; Qref=P.Qref0+dQref; wgrid=P.w0+dwGrid; Vgrid=P.Vg0*(1+dVgrid); %#ok<NASGU>
VdcRef=P.VdcRef0+dVdcRef;
omegaRel=wt-wg; %#ok<NASGU>
eDc=VdcRef-Udc; imqRef=P.Kpdc*eDc+xiDc; imdRef=0;
eMid=imdRef-imd; eMiq=imqRef-imq; we=P.np*wg;
vmd=P.Kpmi*eMid+xiMid+P.Rs*imdRef+we*P.Lq*imqRef;
vmq=P.Kpmi*eMiq+xiMiq+P.Rs*imqRef-we*(P.Ld*imdRef+P.psi);
Pmsc=-1.5*(vmd*imd+vmq*imq);
icapd=ifd-igd; icapq=ifq-igq; vpccd=vcfd+P.Rd*icapd; vpccq=vcfq+P.Rd*icapq;
Ppcc=1.5*(vpccd*igd+vpccq*igq); Qpcc=1.5*(vpccq*igd-vpccd*igq);
c=cos(delta); s=sin(delta);
vpd=c*vpccd+s*vpccq; vpq=-s*vpccd+c*vpccq;
ifld=c*ifd+s*ifq; iflq=-s*ifd+c*ifq; igld=c*igd+s*igq; iglq=-s*igd+c*igq;
Vref=P.E0+P.kq*(Qref-Qf); evd=Vref-vpd; evq=-vpq;
ifdRef=P.Kpgv*evd+xiVd-P.Cf*wv*vpq+P.ffIg*igld;
ifqRef=P.Kpgv*evq+xiVq+P.Cf*wv*vpd+P.ffIg*iglq;
eid=ifdRef-ifld; eiq=ifqRef-iflq;
ucd=P.Kpgi*eid+xiId-wv*P.Lf*iflq+P.ffVpcc*(vpd+P.Rf*ifld);
ucq=P.Kpgi*eiq+xiIq+wv*P.Lf*ifld+P.ffVpcc*(vpq+P.Rf*iflq);
uinvd=c*ucd-s*ucq; uinvq=s*ucd+c*ucq; Pgsc=1.5*(uinvd*ifd+uinvq*ifq);
q=struct('vmd',vmd,'vmq',vmq,'uinvd',uinvd,'uinvq',uinvq,'Pmsc',Pmsc,'Pgsc',Pgsc, ...
    'Ppcc',Ppcc,'Qpcc',Qpcc,'imdRef',imdRef,'imqRef',imqRef,'Vref',Vref, ...
    'dxDc',P.Kidc*eDc,'dxMid',P.Kimi*eMid,'dxMiq',P.Kimi*eMiq, ...
    'dxPf',P.wpf*(Ppcc-Pf),'dxQf',P.wpf*(Qpcc-Qf), ...
    'dxWv',P.w0/(2*P.H*P.Sb)*(P.sP*(Pref-Pf)-(wv-wgrid)/P.mp), ...
    'dxDelta',wv-wgrid,'dxVd',P.Kigv*evd,'dxVq',P.Kigv*evq, ...
    'dxId',P.Kigi*eid,'dxIq',P.Kigi*eiq);
end

function dz=plantRhs(x,cmd,u,p)
P=unpack(p);
theta=x(1); wt=x(2); wg=x(3); imd=x(4); imq=x(5); Udc=x(9);
ifd=x(18); ifq=x(19); vcfd=x(20); vcfq=x(21); igd=x(22); igq=x(23);
dTm=u(1); dVgrid=u(5); dwGrid=u(4);
Tm=P.Tm0*P.wm0/max(wt,eps)+dTm; wgrid=P.w0+dwGrid; Vgrid=P.Vg0*(1+dVgrid);
omegaRel=wt-wg; omegaCoi=(P.Jt*wt+P.Jg*wg)/(P.Jt+P.Jg);
Tshaft=P.Ksh*theta+P.Dsh*omegaRel; Tgen=P.Kt*imq; we=P.np*wg;
vpccd=vcfd+P.Rd*(ifd-igd); vpccq=vcfq+P.Rd*(ifq-igq);
vmd=cmd(1); vmq=cmd(2); uinvd=cmd(3); uinvq=cmd(4);
dz=zeros(23,1); dz(1)=omegaRel;
dz(2)=(Tm-Tshaft-P.Dt*(omegaCoi-P.wm0))/P.Jt;
dz(3)=(Tshaft-Tgen-P.Dg*(omegaCoi-P.wm0))/P.Jg;
dz(4)=(vmd-P.Rs*imd-we*P.Lq*imq)/P.Ld;
dz(5)=(vmq-P.Rs*imq+we*(P.Ld*imd+P.psi))/P.Lq;
Pmsc=-1.5*(vmd*imd+vmq*imq); Pgsc=1.5*(uinvd*ifd+uinvq*ifq);
dz(9)=(Pmsc-Pgsc)/(P.Cdc*max(Udc,eps));
dz(18)=(uinvd-vpccd-P.Rf*ifd+wgrid*P.Lf*ifq)/P.Lf;
dz(19)=(uinvq-vpccq-P.Rf*ifq-wgrid*P.Lf*ifd)/P.Lf;
dz(20)=(ifd-igd)/P.Cf+wgrid*vcfq;
dz(21)=(ifq-igq)/P.Cf-wgrid*vcfd;
dz(22)=(vpccd-Vgrid-P.Rg*igd+wgrid*P.Lg*igq)/P.Lg;
dz(23)=(vpccq-P.Rg*igq-wgrid*P.Lg*igd)/P.Lg;
end

function xn=integratePlant(x,cmd,u,p,dt)
if dt<=0, xn=x; return; end
phys=[1:5 9 18:23]; xp=x(phys);
opts=odeset('RelTol',1e-7,'AbsTol',1e-8,'MaxStep',max(dt/4,1e-8));
[~,xx]=ode45(@(t,z)plantRhsEmbedded(t,z,x,phys,cmd,u,p),[0 dt],xp,opts); %#ok<ASGLU>
xn=x; xn(phys)=xx(end,:).';
end

function dzp=plantRhsEmbedded(~,xp,xfull,phys,cmd,u,p)
x=xfull; x(phys)=xp; dz=plantRhs(x,cmd,u,p); dzp=dz(phys);
end

function P=unpack(p)
p=p(:); P=struct();
P.Sb=p(1); P.VdcRef0=p(2); P.w0=p(3); P.Vg0=p(4); P.Rf=p(5); P.Lf=p(6); P.Cf=p(7); P.Rd=p(8); P.Rg=p(9); P.Lg=p(10);
P.Cdc=p(11); P.wm0=p(12); P.Rs=p(13); P.Ld=p(14); P.Lq=p(15); P.psi=p(16); P.np=p(17); P.Kt=p(18); P.Jt=p(19); P.Jg=p(20);
P.Ksh=p(21); P.Dsh=p(22); P.Dt=p(23); P.Dg=p(24); P.Kpdc=p(25); P.Kidc=p(26); P.Kpmi=p(27); P.Kimi=p(28);
P.Kpgi=p(29); P.Kigi=p(30); P.Kpgv=p(31); P.Kigv=p(32); P.H=p(33); P.mp=p(34); P.wpf=p(35); P.kq=p(36);
P.Pref0=p(37); P.Qref0=p(38); P.Tm0=p(39); P.E0=p(40); P.sP=p(41); P.ffIg=p(42); P.ffVpcc=p(43);
end
