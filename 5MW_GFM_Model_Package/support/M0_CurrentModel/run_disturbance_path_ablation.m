function R=run_disturbance_path_ablation(varargin)
%RUN_DISTURBANCE_PATH_ABLATION
% 在同一严格工作点下，对 GFM、DC-link、DVC 和 MSC 内环反馈做局部旁路。
% 不重新求平衡点；旁路均采用平衡点常值保持，避免把初值变化误判为通道贡献。
% 仅保存一张紧凑 CSV，不保存时域数据。
ip=inputParser;
ip.addParameter('SaveSummary',true,@(x)islogical(x)&&isscalar(x));
ip.parse(varargin{:});
here=fileparts(mfilename('fullpath'));
M=analyze_modal_residue_decomposition('SaveSummary',false);
assert(M.base.passed,'Gate A 未通过，停止通道消融。');
modes={'GFL','DROOP','VSG'};
labels={'GFL (ideal PLL)';'Droop-GFM';'VSG-GFM'};
distNames={'Grid angle';'Grid frequency'};
distIdx=[3 4];
cases={'C0 Full';'C1 No GFM grid sync';'C2 No GSC-to-DC';'C3 No DC-to-DVC';'C4 No DVC-to-Te'};
caseCodes={'full','cutGfmGrid','cutPgscToDc','cutUdcToDvc','cutDvcToTe'};
rows=numel(modes)*numel(cases)*numel(distNames);
T=table('Size',[rows 10], ...
    'VariableTypes',{'string','string','string','double','double','double','double','double','double','double'}, ...
    'VariableNames',{'Control','Case','Disturbance','f_tor_Hz','zeta_tor','R_omega_abs','R_Tsh_abs','eta_omega','eta_Tsh','Te_path_gain'});
row=0; records=cell(numel(modes),numel(cases));
for im=1:numel(modes)
    mode=modes{im}; baseModel=M.models{im}; x=baseModel.x; p=baseModel.p; f=baseModel.f;
    imq0=p(25)*(p(2)-x(9))+x(6);
    pg0=pathOutputsAblation(x,p,mode,zeros(4,1),struct);
    pg0=pg0(1);
    % 统一使用当前控制模式的轴系特征值和左/右特征向量。
    for ic=1:numel(cases)
        flags=struct;
        switch caseCodes{ic}
            case 'cutGfmGrid', flags.cutGfmGrid=true;
            case 'cutPgscToDc', flags.cutPgscToDc=true; flags.Pgsc0=pg0;
            case 'cutUdcToDvc', flags.cutUdcToDvc=true;
            case 'cutDvcToTe', flags.cutDvcToTe=true; flags.imqRef0=imq0;
        end
        [A,Bbar]=linearizeFlags(x,p,mode,flags);
        [lam,fm,zeta,v,wleft]=pickMode(A);
        C=[0 1 -1 zeros(1,20);p(21) p(22) -p(22) zeros(1,20)];
        O=C*v; K=wleft'*Bbar; Res=O*K;
        [Cp,Dp]=pathJacobianFlags(x,p,mode,flags,M.dBase);
        Hresp=zeros(5,4); ww=2*pi*fm;
        for jd=1:4
            Hresp(:,jd)=Cp*((1i*ww*eye(size(A))-A)\Bbar(:,jd))+Dp(:,jd);
        end
        TeGain=abs(Hresp(4,3));
        records{im,ic}=struct('A',A,'B',Bbar,'lambda',lam,'f',fm,'zeta',zeta,'Residue',Res,'Path',Hresp);
        for jd=1:numel(distIdx)
            j=distIdx(jd); row=row+1;
            T.Control(row)=string(labels{im}); T.Case(row)=string(cases{ic}); T.Disturbance(row)=string(distNames{jd});
            T.f_tor_Hz(row)=fm; T.zeta_tor(row)=zeta; T.R_omega_abs(row)=abs(Res(1,j)); T.R_Tsh_abs(row)=abs(Res(2,j));
            % C0 为该控制模式自身的完整闭环基准，不把 GFL 与 GFM 直接混作消融基线。
            if ic==1
                T.eta_omega(row)=0; T.eta_Tsh(row)=0;
            else
                baseRes=records{im,1}.Residue;
                T.eta_omega(row)=1-abs(Res(1,j))/max(abs(baseRes(1,j)),eps);
                T.eta_Tsh(row)=1-abs(Res(2,j))/max(abs(baseRes(2,j)),eps);
            end
            T.Te_path_gain(row)=TeGain;
        end
    end
end
R=struct('table',T,'records',{records},'models',{M.models},'disturbances',{distNames},'cases',{cases});
if ip.Results.SaveSummary
    writetable(T,fullfile(here,'Disturbance_Path_Ablation_Summary.csv'));
end
end

function [A,Bbar]=linearizeFlags(x,p,mode,flags)
n=numel(x); A=zeros(n); Bbar=zeros(n,4);
for j=1:n
    h=1e-6*max(abs(x(j)),1); xp=x; xm=x; xp(j)=xp(j)+h; xm(j)=xm(j)-h;
    A(:,j)=(source_aligned_rhs_control(xp,p,mode,zeros(4,1),flags)-source_aligned_rhs_control(xm,p,mode,zeros(4,1),flags))/(2*h);
end
for j=1:4
    dp=zeros(4,1); dp(j)=1e-6;
    Bbar(:,j)=(source_aligned_rhs_control(x,p,mode,dp,flags)-source_aligned_rhs_control(x,p,mode,-dp,flags))/(2e-6);
end
end

function [Cp,Dp]=pathJacobianFlags(x,p,mode,flags,dBase)
y0=pathOutputsAblation(x,p,mode,zeros(4,1),flags); ny=numel(y0); n=numel(x); Cp=zeros(ny,n); Dp=zeros(ny,4);
for j=1:n
    h=1e-6*max(abs(x(j)),1); xp=x; xm=x; xp(j)=xp(j)+h; xm(j)=xm(j)-h;
    Cp(:,j)=(pathOutputsAblation(xp,p,mode,zeros(4,1),flags)-pathOutputsAblation(xm,p,mode,zeros(4,1),flags))/(2*h);
end
for j=1:4
    dp=zeros(4,1); dp(j)=dBase(j); Dp(:,j)=(pathOutputsAblation(x,p,mode,dp,flags)-pathOutputsAblation(x,p,mode,-dp,flags))/2;
end
end

function y=pathOutputsAblation(x,p,mode,d,flags)
% 与 source_aligned_rhs_control 使用相同的连续控制方程，仅输出逐级信号。
if nargin<5 || isempty(flags), flags=struct; end
cutGfmGrid=getFlag(flags,'cutGfmGrid',false); cutUdcToDvc=getFlag(flags,'cutUdcToDvc',false); cutDvcToTe=getFlag(flags,'cutDvcToTe',false);
Vdc0=p(2); w0=p(3); Rf=p(5); Lf=p(6); Cf=p(7); Rd=p(8); Rs=p(13); Ld=p(14); Lq=p(15); psi=p(16); np=p(17); Kt=p(18); Kpdc=p(25); Kpmi=p(27); Kpgi=p(29); Kpgv=p(31); mp=p(34); kq=p(36); Pref=p(37); Qref=p(38); E0=p(40); ffIg=p(42); ffVpcc=p(43);
imd=x(4); imq=x(5); xiDc=x(6); Udc=x(9); Pf=x(10); Qf=x(11); wv=x(12); delta=x(13); xiMid=x(7); xiMiq=x(8); xiVd=x(14); xiVq=x(15); xiId=x(16); xiIq=x(17); ifd=x(18); ifq=x(19); vcd=x(20); vcq=x(21); igd=x(22); igq=x(23); wg=x(3);
wgrid=w0+d(4); wgridCtrl=w0+(~cutGfmGrid)*d(4);
switch upper(mode), case 'VSG', wc=wv; case 'DROOP', wc=w0+mp*(Pref-Pf); case 'GFL', wc=wgridCtrl; otherwise, error('Unsupported mode.'); end
we=np*wg; eDc=Vdc0-Udc; eDcCtrl=Vdc0-(cutUdcToDvc*Vdc0+(~cutUdcToDvc)*Udc); imqRef=Kpdc*eDcCtrl+xiDc;
if getFlag(flags,'cutDvcToTe',false) && isfield(flags,'imqRef0'), imqRef=flags.imqRef0; end
eMid=-imd; eMiq=imqRef-imq; imdRef=0; vmd=Kpmi*eMid+xiMid+Rs*imdRef-we*Lq*imqRef; vmq=Kpmi*eMiq+xiMiq+Rs*imqRef+we*(Ld*imdRef+psi); %#ok<NASGU>
vnodeD=vcd+Rd*ifd-(Rd+1e-4)*igd; vnodeQ=vcq+Rd*ifq-(Rd+1e-4)*igq; c=cos(delta+d(3)); s=sin(delta+d(3)); vpd=c*vnodeD+s*vnodeQ; vpq=-s*vnodeD+c*vnodeQ; ifld=c*ifd+s*ifq; iflq=-s*ifd+c*ifq; igld=c*igd+s*igq; iglq=-s*igd+c*igq; Vref=E0+kq*(Qref-Qf); evd=Vref-vpd; evq=-vpq; ifdRef=Kpgv*evd+xiVd-Cf*wc*vpq+ffIg*igld; ifqRef=Kpgv*evq+xiVq+Cf*wc*vpd+ffIg*iglq; eid=ifdRef-ifld; eiq=ifqRef-iflq; ucd=Kpgi*eid+xiId-wc*Lf*iflq+ffVpcc*(vpd+Rf*ifld); ucq=Kpgi*eiq+xiIq+wc*Lf*ifld+ffVpcc*(vpq+Rf*iflq); uinvD=c*ucd-s*ucq; uinvQ=s*ucd+c*ucq; Pgsc=1.5*(uinvD*ifd+uinvQ*ifq);
y=[Pgsc;Udc;imqRef;Kt*imq;x(2)-x(3)];
end

function [lam,f,zeta,v,wleft]=pickMode(A)
[V,D]=eig(A); ev=diag(D); [W,Dl]=eig(A'); el=diag(Dl); cand=find(imag(ev)>0 & abs(imag(ev))/(2*pi)>1 & abs(imag(ev))/(2*pi)<5); assert(~isempty(cand),'No torsional candidate.');
etaAll=zeros(numel(cand),1); leftCell=cell(numel(cand),1);
for k=1:numel(cand)
    i=cand(k); [~,j]=min(abs(el-conj(ev(i)))); v0=V(:,i); w0=W(:,j); a=w0'*v0; w0=w0/conj(a); leftCell{k}=w0; etaAll(k)=sum(abs(v0(1:3).*w0(1:3)))/max(sum(abs(v0.*w0)),eps);
end
[~,ii]=max(etaAll); i=cand(ii); lam=ev(i); v=V(:,i); wleft=leftCell{ii}; f=abs(imag(lam))/(2*pi); zeta=-real(lam)/max(abs(lam),eps);
end

function v=getFlag(s,name,default)
if isfield(s,name) && ~isempty(s.(name)), v=logical(s.(name)); else, v=default; end
end
