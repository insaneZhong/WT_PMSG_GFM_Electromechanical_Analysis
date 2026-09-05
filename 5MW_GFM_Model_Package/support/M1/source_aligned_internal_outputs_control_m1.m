function [y,names,units]=source_aligned_internal_outputs_control_m1(x,p,mode,d,flags)
%SOURCE_ALIGNED_INTERNAL_OUTPUTS_CONTROL_M1 M1同源内部测量节点。
if nargin<4||isempty(d), d=zeros(4,1); end
if nargin<5||isempty(flags), flags=struct; end
mode=upper(char(string(mode))); d=d(:);
Vdc0=p(2); w0=p(3); Rf=p(5); Lf=p(6); Cf=p(7); Rd=p(8); Rs=p(13); Ld=p(14); Lq=p(15); psi=p(16); np=p(17); Kt=p(18); Ksh=p(21); Dsh=p(22); Kpdc=p(25); Kpmi=p(27); Kpgi=p(29); Kpgv=p(31); mp=p(34); kq=p(36); Pref=p(37); Qref=p(38); E0=p(40); ffIg=p(42); ffVpcc=p(43); %#ok<NASGU>
wg=x(3); imd=x(4); imq=x(5); xiDc=x(6); Udc=x(9); Pf=x(10); Qf=x(11); wv=x(12); delta=x(13); xiVd=x(14); xiVq=x(15); xiId=x(16); xiIq=x(17); ifd=x(18); ifq=x(19); vcd=x(20); vcq=x(21); igd=x(22); igq=x(23);
cutUdcToDvc=getFlag(flags,'cutUdcToDvc',false); cutDvcToTe=getFlag(flags,'cutDvcToTe',false);
thetaEff=delta+d(3); c=cos(thetaEff); s=sin(thetaEff);
switch mode
    case 'VSG', wctrl=wv;
    case 'DROOP', wctrl=w0+mp*(Pref-Pf);
    case 'GFL', wctrl=w0+d(4);
    case {'GFMGWT','ALPHADC'}, wctrl=wv;
    otherwise, error('Unsupported control mode: %s',mode);
end
eDcCtrl=Vdc0-(cutUdcToDvc*Vdc0+(~cutUdcToDvc)*Udc);
if strcmp(mode,'GFMGWT')
    assert(isfield(flags,'imqRef0'),'GFMGWT requires imqRef0.'); imqRef=flags.imqRef0;
    if getFlag(flags,'mpptLocal',false)
        Kmppt=getFlagValue(flags,'Kmppt_iq_per_radps',2*flags.imqRef0/p(12));
        imqRef=imqRef+Kmppt*(wg-p(12));
    end
elseif strcmp(mode,'ALPHADC')
    alphaDc=getFlagValue(flags,'alphaDc',NaN);
    assert(isfinite(alphaDc)&&alphaDc>=0&&alphaDc<=1,'ALPHADC requires alphaDc in [0,1].');
    assert(isfield(flags,'imqRef0')&&isfield(flags,'xiDcBias'),'ALPHADC requires imqRef0 and xiDcBias.');
    imqRef=flags.imqRef0+alphaDc*(Kpdc*eDcCtrl+xiDc-flags.xiDcBias);
else
    imqRef=Kpdc*eDcCtrl+xiDc;
end
if cutDvcToTe&&isfield(flags,'imqRef0'), imqRef=flags.imqRef0; end
we=np*wg; imdRef=0; eMid=-imd; eMiq=imqRef-imq;
vmdCmd=Kpmi*eMid+x(7)+Rs*imdRef-we*Lq*imqRef;
vmqCmd=Kpmi*eMiq+x(8)+Rs*imqRef+we*(Ld*imdRef+psi);
vScale=vscDcScale(Udc,Vdc0,flags); vmd=vScale*vmdCmd; vmq=vScale*vmqCmd;
Pmsc=1.5*(vmd*imd+vmq*imq);
vnodeD=vcd+Rd*ifd-(Rd+1e-4)*igd; vnodeQ=vcq+Rd*ifq-(Rd+1e-4)*igq;
Pmeas=1.5*(vnodeD*igd+vnodeQ*igq); Qmeas=1.5*(vnodeQ*igd-vnodeD*igq);
vpd=c*vnodeD+s*vnodeQ; vpq=-s*vnodeD+c*vnodeQ;
ifld=c*ifd+s*ifq; iflq=-s*ifd+c*ifq; igld=c*igd+s*igq; iglq=-s*igd+c*igq;
Vref=E0+kq*(Qref-Qf); evd=Vref-vpd; evq=-vpq;
ifdRef=Kpgv*evd+xiVd-Cf*wctrl*vpq+ffIg*igld;
ifqRef=Kpgv*evq+xiVq+Cf*wctrl*vpd+ffIg*iglq;
eid=ifdRef-ifld; eiq=ifqRef-iflq;
ucdCmd=Kpgi*eid+xiId-wctrl*Lf*iflq+ffVpcc*(vpd+Rf*ifld);
ucqCmd=Kpgi*eiq+xiIq+wctrl*Lf*ifld+ffVpcc*(vpq+Rf*iflq);
uinvDCmd=c*ucdCmd-s*ucqCmd; uinvQCmd=s*ucdCmd+c*ucqCmd;
uinvD=vScale*uinvDCmd; uinvQ=vScale*uinvQCmd;
Pgsc=1.5*(uinvD*ifd+uinvQ*ifq);
Tsh=Ksh*x(1)+Dsh*(x(2)-wg);
y=[Pgsc;Udc;imqRef;imq;Kt*imq;x(2)-wg;Tsh;Pf;Pmsc;Pmeas;wg;delta;wv];
names={'P_GSC','Udc','iq_MSC_ref','iq_MSC','T_e','omega_sh','T_sh','P_f','P_MSC','P_PCC','omega_g','delta','omega_v'};
units={'W','V','A','A','N m','rad/s','N m','W','W','W','rad/s','rad','rad/s'};
end
function s=vscDcScale(Udc,Vdc0,flags)
kind=upper(char(string(getFlagText(flags,'vscDcMode','REALTIME_FF'))));
switch kind, case 'FIXED_NORM',s=Udc/Vdc0; case {'REALTIME_FF','M0'},s=1; otherwise,error('Unknown flags.vscDcMode: %s',kind); end
end
function v=getFlag(s,n,d),if isfield(s,n)&&~isempty(s.(n)),v=logical(s.(n));else,v=d;end,end
function v=getFlagValue(s,n,d),if isfield(s,n)&&~isempty(s.(n)),v=double(s.(n));else,v=d;end,end
function v=getFlagText(s,n,d),if isfield(s,n)&&~isempty(s.(n)),v=s.(n);else,v=d;end,end
