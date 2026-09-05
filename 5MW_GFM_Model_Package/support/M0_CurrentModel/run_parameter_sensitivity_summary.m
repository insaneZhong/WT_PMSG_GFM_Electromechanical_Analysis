function R=run_parameter_sensitivity_summary(varargin)
%RUN_PARAMETER_SENSITIVITY_SUMMARY
% 第一轮一维扫描：H、DVC带宽、GSC电压环带宽、SCR。
% 每个参数点均在公共物理方程上重新求平衡；仅保存小信号汇总表。
ip=inputParser;
ip.addParameter('SaveSummary',true,@(x)islogical(x)&&isscalar(x));
ip.parse(varargin{:});
here=fileparts(mfilename('fullpath')); addpath(here,fileparts(here));
M=analyze_modal_residue_decomposition('SaveSummary',false);
assert(M.base.passed,'Gate A 未通过，停止参数扫描。');
base=M.models{3}; x0=base.x; p0=base.p; mode='VSG';
% 参数名、数值和单位均显式记录，避免把不同尺度混入一个扫描。
spec={ ...
    'H_s',[1.5 3 6]; ...
    'DVC_scale',[0.5 1 2]; ...
    'GSC_voltage_bw_scale',[0.5 1 2]; ...
    'SCR',[2 4 8]; ...
    'Power_pu',[0.6 0.8 1.0]};
rows=0; for k=1:size(spec,1), rows=rows+numel(spec{k,2}); end
T=table('Size',[rows 16], ...
    'VariableTypes',{'string','double','double','double','double','double','double','double','double','double','double','double','double','double','double','string'}, ...
    'VariableNames',{'Parameter','Value','SCR','P_MW','f_tor_Hz','zeta_GFL','zeta_Droop','zeta_VSG','Gamma_angle_Droop','Gamma_angle_VSG','Gamma_freq_Droop','Gamma_freq_VSG','Rangle_GFL','Rangle_Droop','Rangle_VSG','Status'});
row=0; out=cell(rows,1);
for is=1:size(spec,1)
    pname=spec{is,1}; vals=spec{is,2};
    for iv=1:numel(vals)
        val=vals(iv); row=row+1;
        try
            [x,p,solveInfo]=makeCase(x0,p0,pname,val);
            Q=caseMetrics(x,p,mode); Qgfl=caseMetrics(x,p,'GFL'); Qdr=caseMetrics(x,p,'DROOP');
            gaD=abs(Qdr.Rangle)/max(abs(Qgfl.Rangle),eps); gaV=abs(Q.Rangle)/max(abs(Qgfl.Rangle),eps);
            gfD=abs(Qdr.Rfreq)/max(abs(Qgfl.Rfreq),eps); gfV=abs(Q.Rfreq)/max(abs(Qgfl.Rfreq),eps);
            T.Parameter(row)=string(pname); T.Value(row)=val; T.SCR(row)=p(9)*0+4; T.P_MW(row)=p(37)/1e6;
            T.f_tor_Hz(row)=Q.f; T.zeta_GFL(row)=Qgfl.zeta; T.zeta_Droop(row)=Qdr.zeta; T.zeta_VSG(row)=Q.zeta;
            T.Gamma_angle_Droop(row)=gaD; T.Gamma_angle_VSG(row)=gaV; T.Gamma_freq_Droop(row)=gfD; T.Gamma_freq_VSG(row)=gfV;
            T.Rangle_GFL(row)=abs(Qgfl.Rangle); T.Rangle_Droop(row)=abs(Qdr.Rangle); T.Rangle_VSG(row)=abs(Q.Rangle); T.Status(row)="PASS";
            if strcmpi(pname,'SCR'), T.SCR(row)=val; end
            out{row}=struct('x',x,'p',p,'solve',solveInfo,'VSG',Q,'GFL',Qgfl,'DROOP',Qdr);
        catch ME
            T.Parameter(row)=string(pname); T.Value(row)=val; T.SCR(row)=NaN; T.P_MW(row)=NaN; T.f_tor_Hz(row)=NaN; T.zeta_GFL(row)=NaN; T.zeta_Droop(row)=NaN; T.zeta_VSG(row)=NaN; T.Gamma_angle_Droop(row)=NaN; T.Gamma_angle_VSG(row)=NaN; T.Gamma_freq_Droop(row)=NaN; T.Gamma_freq_VSG(row)=NaN; T.Rangle_GFL(row)=NaN; T.Rangle_Droop(row)=NaN; T.Rangle_VSG(row)=NaN; T.Status(row)="FAIL: "+string(ME.message);
        end
    end
end
R=struct('table',T,'cases',{out},'base',base);
if ip.Results.SaveSummary, writetable(T,fullfile(here,'Parameter_Sensitivity_Summary.csv')); end
end

function [x,p,info]=makeCase(x0,p0,name,val)
p=p0; xInit=x0;
switch lower(name)
    case 'h_s'
        p(33)=val;
    case 'dvc_scale'
        p(25)=p0(25)*val; p(26)=p0(26)*val;
    case 'gsc_voltage_bw_scale'
        p(31)=p0(31)*val; p(32)=p0(32)*val^2;
    case 'scr'
        zbase=(690^2)/5e6; w0=2*pi*50; lbase=zbase/w0; p(9)=0.02*zbase*(4/val); p(10)=0.25*lbase*(4/val);
    case 'power_pu'
        p(37)=p0(37)*val; p(39)=p0(39)*val;
    otherwise, error('Unknown scan parameter %s',name);
end
needsSolve=strcmpi(name,'SCR') || strcmpi(name,'Power_pu');
if ~needsSolve
    x=xInit; info=struct('exitflag',1,'residual',norm(source_aligned_rhs_control(x,p,'VSG',zeros(4,1)),inf));
    return;
end
sx=[1;1;1;1e4;1e4;1e4;1e4;1e4;1e3;5e6;5e6;1;1;1e4;1e4;1e4;1e4;1e6;1e6;1e6;1e6;1e6;1e6];
sr=[1;1;1;1e4;1e4;1e4;1e4;1e4;1e6;5e8;5e8;1;1;1e4;1e4;1e4;1e4;1e6;1e6;1e6;1e6;1e6;1e6];
opts=optimoptions('fsolve','Display','off','FunctionTolerance',1e-10,'StepTolerance',1e-10,'OptimalityTolerance',1e-10,'MaxIterations',500);
[z,fval,exitflag]=fsolve(@(z)source_aligned_rhs_control(sx.*z,p,'VSG',zeros(4,1))./sr,xInit./sx,opts); x=sx.*z;
info=struct('exitflag',exitflag,'residual',norm(fval,inf)); assert(exitflag>0 && info.residual<1e-8,'SCR equilibrium solve failed: flag=%g residual=%g',exitflag,info.residual);
end

function Q=caseMetrics(x,p,mode)
dBase=[0.005*p(1);0.005*p(1);deg2rad(0.2);2*pi*0.05]; [A,Bbar]=linearizeLocal(x,p,mode); [lam,f,zeta,v,wleft]=pickLocal(A); C=[0 1 -1 zeros(1,20);p(21) p(22) -p(22) zeros(1,20)]; O=C*v; Res=O*(wleft'*Bbar); [Cp,Dp]=pathLocal(x,p,mode,dBase); w=2*pi*f; Ha=Cp*((1i*w*eye(size(A))-A)\Bbar)+Dp; Q=struct('A',A,'B',Bbar,'lambda',lam,'f',f,'zeta',zeta,'Rangle',Res(1,3),'Rfreq',Res(1,4),'TeAngle',Ha(4,3),'omegaAngle',Ha(5,3));
end

function [A,Bbar]=linearizeLocal(x,p,mode)
n=numel(x); A=zeros(n); Bbar=zeros(n,4);
for j=1:n
    h=1e-6*max(abs(x(j)),1); xp=x; xm=x; xp(j)=xp(j)+h; xm(j)=xm(j)-h; A(:,j)=(source_aligned_rhs_control(xp,p,mode,zeros(4,1))-source_aligned_rhs_control(xm,p,mode,zeros(4,1)))/(2*h);
end
for j=1:4
    dp=zeros(4,1); dp(j)=1e-6; Bbar(:,j)=(source_aligned_rhs_control(x,p,mode,dp)-source_aligned_rhs_control(x,p,mode,-dp))/(2e-6);
end
end

function [Cp,Dp]=pathLocal(x,p,mode,dBase)
y0=pathLocalOutputs(x,p,mode,zeros(4,1)); ny=numel(y0); n=numel(x); Cp=zeros(ny,n); Dp=zeros(ny,4);
for j=1:n
    h=1e-6*max(abs(x(j)),1); xp=x; xm=x; xp(j)=xp(j)+h; xm(j)=xm(j)-h; Cp(:,j)=(pathLocalOutputs(xp,p,mode,zeros(4,1))-pathLocalOutputs(xm,p,mode,zeros(4,1)))/(2*h);
end
for j=1:4
    dp=zeros(4,1); dp(j)=dBase(j); Dp(:,j)=(pathLocalOutputs(x,p,mode,dp)-pathLocalOutputs(x,p,mode,-dp))/2;
end
end

function y=pathLocalOutputs(x,p,mode,d)
Vdc0=p(2); w0=p(3); Lf=p(6); Cf=p(7); Rd=p(8); Rs=p(13); Ld=p(14); Lq=p(15); psi=p(16); np=p(17); Kt=p(18); Kpdc=p(25); Kpmi=p(27); Kpgi=p(29); Kpgv=p(31); mp=p(34); kq=p(36); Pref=p(37); Qref=p(38); E0=p(40); ffIg=p(42); ffVpcc=p(43);
imd=x(4); imq=x(5); xiDc=x(6); Udc=x(9); Pf=x(10); Qf=x(11); wv=x(12); delta=x(13); xiMid=x(7); xiMiq=x(8); xiVd=x(14); xiVq=x(15); xiId=x(16); xiIq=x(17); ifd=x(18); ifq=x(19); vcd=x(20); vcq=x(21); igd=x(22); igq=x(23); wg=x(3); wc=w0+d(4); if strcmpi(mode,'VSG'), wc=wv; elseif strcmpi(mode,'DROOP'), wc=w0+mp*(Pref-Pf); end
we=np*wg; eDc=Vdc0-Udc; imqRef=Kpdc*eDc+xiDc; eMid=-imd; eMiq=imqRef-imq; imdRef=0; %#ok<NASGU>
vnodeD=vcd+Rd*ifd-(Rd+1e-4)*igd; vnodeQ=vcq+Rd*ifq-(Rd+1e-4)*igq; c=cos(delta); s=sin(delta); vpd=c*vnodeD+s*vnodeQ; vpq=-s*vnodeD+c*vnodeQ; ifld=c*ifd+s*ifq; iflq=-s*ifd+c*ifq; igld=c*igd+s*igq; iglq=-s*igd+c*igq; Vref=E0+kq*(Qref-Qf); evd=Vref-vpd; evq=-vpq; ifdRef=Kpgv*evd+xiVd-Cf*wc*vpq+ffIg*igld; ifqRef=Kpgv*evq+xiVq+Cf*wc*vpd+ffIg*iglq; eid=ifdRef-ifld; eiq=ifqRef-iflq; ucd=Kpgi*eid+xiId-wc*Lf*iflq+ffVpcc*(vpd+Rs*ifld); ucq=Kpgi*eiq+xiIq+wc*Lf*ifld+ffVpcc*(vpq+Rs*iflq); uinvD=c*ucd-s*ucq; uinvQ=s*ucd+c*ucq; Pgsc=1.5*(uinvD*ifd+uinvQ*ifq); y=[Pgsc;Udc;imqRef;Kt*imq;x(2)-x(3)];
end

function [lam,f,zeta,v,wleft]=pickLocal(A)
[V,D]=eig(A); ev=diag(D); [W,Dl]=eig(A'); el=diag(Dl); cand=find(imag(ev)>0 & abs(imag(ev))/(2*pi)>1 & abs(imag(ev))/(2*pi)<5); assert(~isempty(cand),'Torsional candidate not found.'); eta=zeros(numel(cand),1); lc=cell(numel(cand),1);
for k=1:numel(cand), i=cand(k); [~,j]=min(abs(el-conj(ev(i)))); v0=V(:,i); w0=W(:,j); a=w0'*v0; w0=w0/conj(a); lc{k}=w0; eta(k)=sum(abs(v0(1:3).*w0(1:3)))/max(sum(abs(v0.*w0)),eps); end
[~,ii]=max(eta); i=cand(ii); lam=ev(i); v=V(:,i); wleft=lc{ii}; f=abs(imag(lam))/(2*pi); zeta=-real(lam)/max(abs(lam),eps);
end
