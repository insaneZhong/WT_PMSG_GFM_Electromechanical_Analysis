function R=analyze_grid_to_shaft_path(M,varargin)
%ANALYZE_GRID_TO_SHAFT_PATH 网侧扰动到轴系的逐级传递增益。
if nargin<1 || isempty(M), M=analyze_modal_residue_decomposition('SaveSummary',false); end
ip=inputParser; ip.addParameter('SaveSummary',true,@(x)islogical(x)&&isscalar(x)); ip.parse(varargin{:});
here=fileparts(mfilename('fullpath')); modes={'GFL','DROOP','VSG'}; labels={'GFL (ideal PLL)';'Droop-GFM';'VSG-GFM'}; names={'Mechanical torque';'Aerodynamic power';'Grid angle';'Grid frequency'}; vars={'P_GSC','Udc','iq_MSC_ref','Te','omega_sh'};
rows=numel(modes)*numel(names)*numel(vars); T=table('Size',[rows 8],'VariableTypes',{'string','string','string','double','double','double','double','double'},'VariableNames',{'Control','Disturbance','Variable','f_tor_Hz','Gain','Phase_deg','Gamma_vs_GFL','Gain_normalized'}); row=0; allData=cell(numel(modes),1);
for k=1:numel(modes)
 Q=M.models{k}; x=Q.x; p=Q.p; A=Q.A; B=Q.Bbar; f=Q.f; w=2*pi*f; [Cp,Dp]=pathJacobian(x,p,modes{k},M.dBase); H=zeros(numel(vars),numel(names));
 for j=1:numel(names), H(:,j)=Cp*((1i*w*eye(size(A))-A)\B(:,j))+Dp(:,j); end
 allData{k}=H;
 for j=1:numel(names)
  for q=1:numel(vars)
   row=row+1; T.Control(row)=labels{k}; T.Disturbance(row)=names{j}; T.Variable(row)=vars{q}; T.f_tor_Hz(row)=f; T.Gain(row)=abs(H(q,j)); T.Phase_deg(row)=angle(H(q,j))*180/pi; T.Gain_normalized(row)=abs(H(q,j)); if k==1, T.Gamma_vs_GFL(row)=1; else, T.Gamma_vs_GFL(row)=abs(H(q,j))/max(abs(allData{1}(q,j)),eps); end
  end
 end
end
R=struct('table',T,'transfer',allData,'models',{M.models},'variables',{vars},'disturbances',{names});
if ip.Results.SaveSummary, writetable(T,fullfile(here,'Disturbance_Path_Summary.csv')); end
end

function [C,D]=pathJacobian(x,p,mode,dBase)
y0=pathOutputs(x,p,mode,zeros(4,1)); ny=numel(y0); n=numel(x); C=zeros(ny,n); D=zeros(ny,4);
for j=1:n
 h=1e-6*max(abs(x(j)),1); xp=x; xm=x; xp(j)=xp(j)+h; xm(j)=xm(j)-h; C(:,j)=(pathOutputs(xp,p,mode,zeros(4,1))-pathOutputs(xm,p,mode,zeros(4,1)))/(2*h);
end
for j=1:4
 dp=zeros(4,1); dp(j)=dBase(j); D(:,j)=(pathOutputs(x,p,mode,dp)-pathOutputs(x,p,mode,-dp))/2;
end
end

function y=pathOutputs(x,p,mode,d)
Vdc0=p(2); w0=p(3); Rf=p(5); Lf=p(6); Cf=p(7); Rd=p(8); Rs=p(13); Ld=p(14); Lq=p(15); psi=p(16); np=p(17); Kt=p(18); Kpdc=p(25); Kpgi=p(29); Kpgv=p(31); mp=p(34); kq=p(36); Pref=p(37); Qref=p(38); E0=p(40); ffIg=p(42); ffVpcc=p(43);
 imd=x(4); imq=x(5); xiDc=x(6); Udc=x(9); Pf=x(10); Qf=x(11); wv=x(12); delta=x(13); xiVd=x(14); xiVq=x(15); xiId=x(16); xiIq=x(17); ifd=x(18); ifq=x(19); vcd=x(20); vcq=x(21); igd=x(22); igq=x(23); wg=x(3);
 switch upper(mode), case 'VSG', wc=wv; case 'DROOP', wc=w0+mp*(Pref-Pf); case 'GFL', wc=w0+d(4); otherwise, error('Unsupported mode.'); end
 we=np*wg; eDc=Vdc0-Udc; imdRef=0; imqRef=Kpdc*eDc+xiDc; eMid=-imd; eMiq=imqRef-imq; vmd=Kpgi*eMid+xiVd+Rs*imdRef-we*Lq*imqRef; vmq=Kpgi*eMiq+xiVq+Rs*imqRef+we*(Ld*imdRef+psi); %#ok<NASGU>
 vnodeD=vcd+Rd*ifd-(Rd+1e-4)*igd; vnodeQ=vcq+Rd*ifq-(Rd+1e-4)*igq; c=cos(delta+d(3)); s=sin(delta+d(3)); vpd=c*vnodeD+s*vnodeQ; vpq=-s*vnodeD+c*vnodeQ; ifld=c*ifd+s*ifq; iflq=-s*ifd+c*ifq; igld=c*igd+s*igq; iglq=-s*igd+c*igq; Vref=E0+kq*(Qref-Qf); evd=Vref-vpd; evq=-vpq; ifdRef=Kpgv*evd+xiVd-Cf*wc*vpq+ffIg*igld; ifqRef=Kpgv*evq+xiVq+Cf*wc*vpd+iglq; eid=ifdRef-ifld; eiq=ifqRef-iflq; ucd=Kpgi*eid+xiId-wc*Lf*iflq+ffVpcc*(vpd+Rf*ifld); ucq=Kpgi*eiq+xiIq+wc*Lf*ifld+ffVpcc*(vpq+Rf*iflq); uinvD=c*ucd-s*ucq; uinvQ=s*ucd+c*ucq; Pgsc=1.5*(uinvD*ifd+uinvQ*ifq); y=[Pgsc;Udc;imqRef;Kt*imq;x(2)-x(3)];
end
