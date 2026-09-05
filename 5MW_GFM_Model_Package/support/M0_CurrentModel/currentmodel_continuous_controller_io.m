function z = currentmodel_continuous_controller_io(w)
%CURRENTMODEL_CONTINUOUS_CONTROLLER_IO Explicit continuous controller core.
%   The input is [u(20); x_ctrl(11); pvec(43)].  The output is
%   [dx_ctrl(11); y_legacy(41)].  It contains no persistent state, no
%   sample-and-hold, PWM, delay, limiter, anti-windup, PLL, pre-sync, or
%   active-damping branch.  All dynamic controller states are supplied by
%   visible Simulink Continuous Integrator blocks in MOTOR_CONTROL1.

u = w(1:20);
x = w(21:31);
p = w(32:74);

% Parameter convention is exactly m0_pack_parameters / M0 source order.
Sb=p(1); Vdc0=p(2); w0=p(3); Rf=p(5); Lf=p(6); Cf=p(7);
Rs=p(13); Ld=p(14); Lq=p(15); psi=p(16); np=p(17);
Kpdc=p(25); Kidc=p(26); Kpmi=p(27); Kimi=p(28);
Kpgi=p(29); Kigi=p(30); Kpgv=p(31); Kigv=p(32);
H=p(33); mp=p(34); wpf=p(35); kq=p(36); Pref0=p(37);
Qref0=p(38); E0=p(40); sP=p(41); ffIg=p(42); ffVpcc=p(43);

% Controller states: DVC, MSC PI(d/q), P/Q LPF, VSG(w/delta),
% GSC voltage PI(d/q), GSC current PI(d/q).
xiDc=x(1); xiMid=x(2); xiMiq=x(3); Pf=x(4); Qf=x(5);
wv=x(6); delta=x(7); xiVd=x(8); xiVq=x(9); xiId=x(10); xiIq=x(11);

% Raw S-function input order retained from the original current model.
ia=u(1); ib=u(2); ic=u(3); Udc=u(4);
% 原模型第 5 路为机械角速度；原 C 代码在 dq 解耦项中显式乘
% p->par.Polar。连续模型必须保留这一 SI 单位转换，不能把该输入
% 误当作电角速度。
we=np*u(5); thetaMech=u(6);
% The retained root current-measurement blocks are oriented from the
% PMSM1 publishes mechanical rotor speed and position on this retained
% measurement bus.  Convert them exactly once to the electrical dq frame.
we=np*u(5); thetae=np*u(6);
% The retained root current-measurement blocks are oriented from the
% converter-side inductor through the PCC toward the grid.  Their outputs
% therefore already use the same positive direction as the dq LCL model.
% Do not infer this sign from the former non-periodic t=0 state: the exact
% SPS periodic solution and the branch topology both require a direct map.
ifa=u(7); ifb=u(8); ifc=u(9);
uab=u(10); ubc=u(11); uca=u(12);
iga=u(13); igb=u(14); igc=u(15);
systemTime=u(16);
PrefIn=u(18); VdcRefIn=u(20);

% The idealized copy owns an explicit absolute active-power command input:
% Pref0 plus a default-disabled Step block.  It replaces the retained MPPT
% start-up command so the same small disturbance can drive this nonlinear
% model and the source-aligned small-signal model.
if abs(PrefIn)>1, Pref=PrefIn; else, Pref=Pref0; end
if VdcRefIn > 1, VdcRef=VdcRefIn; else, VdcRef=Vdc0; end
Qref=Qref0;

% MSC abc/dq: same Clarke/Park orientation as motorcontrol_legacy_ad_base.
ca=cos(thetae); sa=sin(thetae);
imalpha=ia;
imbeta=(ia+2*ib)/sqrt(3);
imd=imalpha*ca+imbeta*sa;
imq=imbeta*ca-imalpha*sa;
eDc=VdcRef-Udc;
imdRef=0;
imqRef=Kpdc*eDc+xiDc;
eMid=imdRef-imd;
eMiq=imqRef-imq;
% The retained ideal VSC exposes the generator-side terminal-voltage
% orientation, whereas the PMSG internal dq equations use the passive
% stator-voltage orientation.  The source-aligned port audit therefore
% keeps this command mapping explicit.  It is not interchangeable with
% the raw Ud_fwd/Uq_fwd signs in motorcontrol_legacy_ad_base.c unless the
% current and voltage interface transformations are converted together.
% Match the retained PMSM1 dq plant exactly:
% did=(vd-Rs*id+we*Lq*iq)/Ld and
% diq=(vq-Rs*iq-we*(Ld*id+psi))/Lq.
% These signs are taken from PMSM1/elemodel3, not from the prior M0
% generator-coordinate convention.
vmd=Kpmi*eMid+xiMid+Rs*imdRef-we*Lq*imqRef;
vmq=Kpmi*eMiq+xiMiq+Rs*imqRef+we*(Ld*imdRef+psi);
vmsAlpha=vmd*ca-vmq*sa;
vmsBeta=vmd*sa+vmq*ca;

% PCC voltage reconstructed from measured line-line values, then Clarke.
vpa=(uab-uca)/3;
vpb=(ubc-uab)/3;
vpc=(uca-ubc)/3;
vpAlpha=(2/3)*(vpa-0.5*vpb-0.5*vpc);
vpBeta=(sqrt(3)/3)*(vpb-vpc);
ifAlpha=(2/3)*(ifa-0.5*ifb-0.5*ifc);
ifBeta=(sqrt(3)/3)*(ifb-ifc);
igAlpha=(2/3)*(iga-0.5*igb-0.5*igc);
igBeta=(sqrt(3)/3)*(igb-igc);
Ppcc=1.5*(vpAlpha*igAlpha+vpBeta*igBeta);
Qpcc=1.5*(vpBeta*igAlpha-vpAlpha*igBeta);

% In the continuous ideal model the physical relative VSG angle is used.
% The retained source is an ideal fixed 50 Hz grid whose phase at t=0 is
% -pi/2.  Using atan2(v_PCC) here would silently add an instantaneous PLL:
% at 5 MW the LCL/line voltage drop makes its angle differ from the grid
% reference used by the M0 state equations.  The explicit analytic grid
% phase keeps the nonlinear and small-signal angle definitions identical.
thetaGrid=w0*systemTime-pi/2;
thetaVsg=thetaGrid+delta;
cg=cos(thetaVsg); sg=sin(thetaVsg);
vpd=vpAlpha*cg+vpBeta*sg;
vpq=vpBeta*cg-vpAlpha*sg;
ifd=ifAlpha*cg+ifBeta*sg;
ifq=ifBeta*cg-ifAlpha*sg;
igd=igAlpha*cg+igBeta*sg;
igq=igBeta*cg-igAlpha*sg;

Vref=E0+kq*(Qref-Qf);
evd=Vref-vpd;
evq=-vpq;
ifdRef=Kpgv*evd+xiVd-Cf*wv*vpq+ffIg*igd;
ifqRef=Kpgv*evq+xiVq+Cf*wv*vpd+ffIg*igq;
eid=ifdRef-ifd;
eiq=ifqRef-ifq;
ucd=Kpgi*eid+xiId-wv*Lf*ifq+ffVpcc*(vpd+Rf*ifd);
ucq=Kpgi*eiq+xiIq+wv*Lf*ifd+ffVpcc*(vpq+Rf*ifq);
vgsAlpha=ucd*cg-ucq*sg;
vgsBeta=ucd*sg+ucq*cg;

dx=zeros(11,1);
dx(1)=Kidc*eDc;
dx(2)=Kimi*eMid;
dx(3)=Kimi*eMiq;
dx(4)=wpf*(Ppcc-Pf);
dx(5)=wpf*(Qpcc-Qf);
dx(6)=w0/(2*H*Sb)*(sP*(Pref-Pf)-(wv-w0)/mp);
dx(7)=wv-w0;
dx(8)=Kigv*evd;
dx(9)=Kigv*evq;
dx(10)=Kigi*eid;
dx(11)=Kigi*eiq;

% Preserve the legacy 41-element controller output contract.  Gate outputs
% are deliberately zero because both bridges are replaced by ideal VSCs.
y=zeros(41,1);
y(13)=Pref; y(14)=Ppcc; y(15)=wv; y(16)=thetaVsg;
y(17)=ucd; y(18)=ucq; y(19)=Qpcc; y(20)=Vref;
y(21)=Vref; y(22)=vpd; y(23)=ifdRef; y(24)=ifd;
y(25)=0; y(26)=vpq; y(27)=ifqRef; y(28)=thetaGrid;
y(29)=1; y(30)=ifq; y(31)=imqRef; y(32)=Kpdc*eDc+xiDc;
y(33)=imq; y(34)=vmd; y(35)=vmq; y(36)=hypot(vmd,vmq);
y(37)=1.5*y(36)/max(Udc,1);
y(38)=vmsAlpha; y(39)=vmsBeta; y(40)=vgsAlpha; y(41)=vgsBeta;
z=[dx;y];
end
