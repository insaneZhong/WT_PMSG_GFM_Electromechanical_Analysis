function dx=source_aligned_rhs_control(x,p,mode,d,flags)
%SOURCE_ALIGNED_RHS_CONTROL 公共23状态方程的控制架构版本。
% mode: 'GFL'（理想PLL/电网角频率跟踪）、'DROOP'、'VSG'、'GFMGWT'、'ALPHADC'。
% d=[DeltaTm; DeltaPm; DeltaThetaGrid; DeltaOmegaGrid]，SI单位。
% GFMGWT 表示 GSC-DVC + MSC-MPPT/定转矩；其余三种模式保持原定义。
if nargin<4 || isempty(d), d=zeros(4,1); end
if nargin<5 || isempty(flags), flags=struct; end
cutGfmGrid=getFlag(flags,'cutGfmGrid',false); cutPgscToDc=getFlag(flags,'cutPgscToDc',false); cutUdcToDvc=getFlag(flags,'cutUdcToDvc',false); cutDvcToTe=getFlag(flags,'cutDvcToTe',false);
mode=upper(char(string(mode))); d=d(:); assert(numel(d)==4,'d must have 4 elements.');

Sb=p(1); Vdc0=p(2); w0=p(3); Vg=p(4); Rf=p(5); Lf=p(6); Cf=p(7); Rd=p(8); Rg=p(9); Lg=p(10); Cdc=p(11); wm0=p(12); Rs=p(13); Ld=p(14); Lq=p(15); psi=p(16); np=p(17); Kt=p(18); Jt=p(19); Jg=p(20); Ksh=p(21); Dsh=p(22); Dt=p(23); Dg=p(24); Kpdc=p(25); Kidc=p(26); Kpmi=p(27); Kimi=p(28); Kpgi=p(29); Kigi=p(30); Kpgv=p(31); Kigv=p(32); H=p(33); mp=p(34); wpf=p(35); kq=p(36); Pref=p(37); Qref=p(38); Tm0=p(39); E0=p(40); sP=p(41); ffIg=p(42); ffVpcc=p(43);
KpGscDvc=getFlagValue(flags,'KpGscDvc',5e3); KiGscDvc=getFlagValue(flags,'KiGscDvc',5e2);
theta=x(1); wt=x(2); wg=x(3); imd=x(4); imq=x(5); xiDc=x(6); xiMid=x(7); xiMiq=x(8); Udc=x(9); Pf=x(10); Qf=x(11); wv=x(12); delta=x(13); xiVd=x(14); xiVq=x(15); xiId=x(16); xiIq=x(17); ifd=x(18); ifq=x(19); vcd=x(20); vcq=x(21); igd=x(22); igq=x(23);

wgrid=w0+d(4); wgridCtrl=w0+(~cutGfmGrid)*d(4);
switch mode
    case 'VSG'
        wctrl=wv;
    case 'DROOP'
        % 代数P-f下垂；wv状态保留为公共状态占位，不参与外环动态。
        wctrl=w0+mp*(Pref-Pf);
    case 'GFL'
        % 理想PLL代理：角频率直接跟随电网，delta保持相对相角。
        wctrl=wgridCtrl;
    case {'GFMGWT','ALPHADC'}
        % GFM-GWT：MSC采用MPPT/定转矩，GSC-VSG的有功参考由DC-link外环给出。
        % 该实现对应文献中的GSC-DVC分配：DVC改变GSC有功指令，而不是MSC iq*。
        wctrl=wv;
    otherwise
        error('Unsupported control mode: %s',mode);
end
deltaEff=delta+d(3);

we=np*wg; Tgen=Kt*imq; Tsh=Ksh*theta+Dsh*(wt-wg); wcoi=(Jt*wt+Jg*wg)/(Jt+Jg);
eDc=Vdc0-Udc; eDcCtrl=Vdc0-(cutUdcToDvc*Vdc0+(~cutUdcToDvc)*Udc); imdRef=0;
if strcmp(mode,'GFMGWT')
    assert(isfield(flags,'imqRef0') && isfinite(flags.imqRef0), ...
        'GFMGWT requires flags.imqRef0 as the fixed MSC MPPT/torque current reference.');
    imqRef=flags.imqRef0;
elseif strcmp(mode,'ALPHADC')
    % 连续DC-link调节责任分配：alpha=0严格对应GWT，alpha=1严格对应MWT。
    % xiDc采用MSC电流积分量坐标；GSC积分量通过Ki比值作纯状态尺度变换，
    % 因而不会插值plant，也不会改变共同平衡点。
    alphaDc=getFlagValue(flags,'alphaDc',NaN);
    assert(isfinite(alphaDc) && alphaDc>=0 && alphaDc<=1,'ALPHADC requires alphaDc in [0,1].');
    assert(isfield(flags,'imqRef0') && isfield(flags,'xiDcBias'),'ALPHADC requires imqRef0 and xiDcBias.');
    zDc=xiDc-flags.xiDcBias;
    imqRef=flags.imqRef0+alphaDc*(Kpdc*eDcCtrl+zDc);
else
    imqRef=Kpdc*eDcCtrl+xiDc;
end
if cutDvcToTe && isfield(flags,'imqRef0'), imqRef=flags.imqRef0; end
eMid=-imd; eMiq=imqRef-imq;
vmd=Kpmi*eMid+xiMid+Rs*imdRef-we*Lq*imqRef; vmq=Kpmi*eMiq+xiMiq+Rs*imqRef+we*(Ld*imdRef+psi);
Pmsc=1.5*(vmd*imd+vmq*imq);

icapd=ifd-igd; icapq=ifq-igq; vnodeD=vcd+Rd*ifd-(Rd+1e-4)*igd; vnodeQ=vcq+Rd*ifq-(Rd+1e-4)*igq;
Pmeas=1.5*(vnodeD*igd+vnodeQ*igq); Qmeas=1.5*(vnodeQ*igd-vnodeD*igq);
c=cos(deltaEff); s=sin(deltaEff); vpd=c*vnodeD+s*vnodeQ; vpq=-s*vnodeD+c*vnodeQ; ifld=c*ifd+s*ifq; iflq=-s*ifd+c*ifq; igld=c*igd+s*igq; iglq=-s*igd+c*igq;
Vref=E0+kq*(Qref-Qf); evd=Vref-vpd; evq=-vpq; ifdRef=Kpgv*evd+xiVd-Cf*wctrl*vpq+ffIg*igld; ifqRef=Kpgv*evq+xiVq+Cf*wctrl*vpd+ffIg*iglq; eid=ifdRef-ifld; eiq=ifqRef-iflq;
ucd=Kpgi*eid+xiId-wctrl*Lf*iflq+ffVpcc*(vpd+Rf*ifld); ucq=Kpgi*eiq+xiIq+wctrl*Lf*ifld+ffVpcc*(vpq+Rf*iflq); uinvD=c*ucd-s*ucq; uinvQ=s*ucd+c*ucq; Pgsc=1.5*(uinvD*ifd+uinvQ*ifq);

dx=zeros(23,1); TmIn=Tm0*wm0/max(wt,1e-9)+d(1)+d(2)/max(wt,1e-9);
dx(1)=wt-wg; dx(2)=(TmIn-Tsh-Dt*(wcoi-wm0))/Jt; dx(3)=(Tsh-Tgen-Dg*(wcoi-wm0))/Jg;
dx(4)=(vmd-Rs*imd+we*Lq*imq)/Ld;
dx(5)=(vmq-Rs*imq-we*(Ld*imd+psi))/Lq;
if strcmp(mode,'GFMGWT')
    % GSC-DVC积分状态；MSC-DVC在该架构中旁路。
    dx(6)=KiGscDvc*eDc;
elseif strcmp(mode,'ALPHADC')
    dx(6)=(~cutUdcToDvc)*Kidc*eDc;
else
    dx(6)=(~cutUdcToDvc)*Kidc*eDc;
end
dx(7)=Kimi*eMid; dx(8)=Kimi*eMiq;
PgscDc=Pgsc; if cutPgscToDc && isfield(flags,'Pgsc0'), PgscDc=flags.Pgsc0; end
dx(9)=(Pmsc-PgscDc)/(Cdc*Udc); dx(10)=wpf*(Pmeas-Pf); dx(11)=wpf*(Qmeas-Qf);
switch mode
    case 'VSG'
        dx(12)=w0/(2*H*Sb)*(sP*(Pref-Pf)-(wv-w0)/mp);
        dx(13)=wctrl-wgridCtrl;
    case 'DROOP'
        dx(12)=0;
        dx(13)=wctrl-wgridCtrl;
    case 'GFL'
        dx(12)=0;
        dx(13)=0;
    case 'GFMGWT'
        Pctrl=Pref-KpGscDvc*eDc-xiDc;
        mpUse=getFlagValue(flags,'mpGwt',mp);
        dx(12)=w0/(2*H*Sb)*(sP*(Pctrl-Pf)-(wv-w0)/mpUse);
        dx(13)=wctrl-wgridCtrl;
    case 'ALPHADC'
        alphaDc=getFlagValue(flags,'alphaDc',NaN);
        KpG=getFlagValue(flags,'KpGscDvc',5e3); KiG=getFlagValue(flags,'KiGscDvc',5e2);
        zDc=xiDc-getFlagValue(flags,'xiDcBias',xiDc);
        % (KiG/Kidc)*zDc 与GWT的GSC-DVC积分状态严格相似变换。
        Pctrl=Pref-(1-alphaDc)*(KpG*eDc+(KiG/Kidc)*zDc);
        mpUse=getFlagValue(flags,'mpGwt',mp);
        dx(12)=w0/(2*H*Sb)*(sP*(Pctrl-Pf)-(wv-w0)/mpUse);
        dx(13)=wctrl-wgridCtrl;
 end
Rf_eff=Rf+1e-4; dx(14)=Kigv*evd; dx(15)=Kigv*evq; dx(16)=Kigi*eid; dx(17)=Kigi*eiq; dx(18)=(uinvD-vnodeD-Rf_eff*ifd+w0*Lf*ifq)/Lf; dx(19)=(uinvQ-vnodeQ-Rf_eff*ifq-w0*Lf*ifd)/Lf; dx(20)=icapd/Cf+w0*vcq; dx(21)=icapq/Cf-w0*vcd; dx(22)=(vnodeD-Vg-Rg*igd+w0*Lg*igq)/Lg; dx(23)=(vnodeQ-Rg*igq-w0*Lg*igd)/Lg;
end

function v=getFlag(s,name,default)
if isfield(s,name) && ~isempty(s.(name)), v=logical(s.(name)); else, v=default; end
end

function v=getFlagValue(s,name,default)
if isfield(s,name) && ~isempty(s.(name)), v=double(s.(name)); else, v=default; end
end
