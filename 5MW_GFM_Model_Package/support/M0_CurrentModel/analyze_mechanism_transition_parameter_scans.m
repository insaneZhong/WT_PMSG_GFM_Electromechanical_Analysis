function R = analyze_mechanism_transition_parameter_scans(Hdetail,outDir)
%ANALYZE_MECHANISM_TRANSITION_PARAMETER_SCANS
% 统一生成SCR、H、DVC的一维机制迁移数据。每点只重新求平衡/SSM，不保存时序。
if nargin<2 || isempty(outDir), outDir=fileparts(mfilename('fullpath')); end
[models,base]=prepare_multimode_models(); iv=find(strcmpi(base.models,'VSG'),1); ig=find(strcmpi(base.models,'GFL'),1);
p0=base.parameter_vector; xV=models{iv}.x0; xG=models{ig}.x0; scr0=4;
% 扩展SCR点，便于观察路径到联合塑形的分区；H使用已通过连续跟踪的结果。
scrs=[2 2.5 3 4 5 6 8 10]; dvcs=[.5 .75 1 1.25 1.5 1.75 2];
Tsc=localSCR(scrs,p0,xV,xG,scr0); Tdvc=localDVC(dvcs,p0,xV); Th=localH(Hdetail,p0);
writetable(Tsc,fullfile(outDir,'Mechanism_SCR_Transition.csv')); writetable(Tdvc,fullfile(outDir,'Mechanism_DVC_Transition.csv')); writetable(Th,fullfile(outDir,'Mechanism_H_Transition.csv'));
all=[localTag(Tsc,"SCR"); localTag(Tdvc,"DVC"); localTag(Th,"H")]; writetable(all,fullfile(outDir,'Mechanism_Closure_Summary.csv'));
R=struct('SCR',Tsc,'DVC',Tdvc,'H',Th,'summary',all,'reference',struct('SCR',scr0,'H',p0(33),'DVCScale',1));
end

function T=localSCR(scrs,p0,xV,xG,scr0)
T=localTable(numel(scrs)); r=0;
for s=scrs
    r=r+1; p=p0; p(9)=p0(9)*scr0/s; p(10)=p0(10)*scr0/s;
    try
        SV=mechanism_transition_point(p,'VSG',xV,struct,true); SG=mechanism_transition_point(p,'GFL',xG,struct,true);
        T(r,:)=localRow(s,"PASS",SV,SG);
    catch ME, T(r,:)=localFail(s,ME.message); end
end
end

function T=localDVC(scales,p0,xV)
Sref=mechanism_transition_point(p0,'VSG',xV,struct,false); T=localTable(numel(scales)); r=0;
for a=scales
    r=r+1; p=p0; p(25)=p0(25)*a; p(26)=p0(26)*a;
    try, S=mechanism_transition_point(p,'VSG',xV,struct,true); T(r,:)=localRow(a,"PASS",S,Sref); catch ME, T(r,:)=localFail(a,ME.message); end
end
end

function T=localH(Hdetail,p0)
hs=unique(Hdetail.H_s).'; SrefRow=Hdetail(Hdetail.TrackedMode=="TOR" & abs(Hdetail.H_s-p0(33))<1e-10,:); SrefRow=SrefRow(1,:); T=localTable(numel(hs)); r=0;
for h=hs
    r=r+1; tor=Hdetail(Hdetail.TrackedMode=="TOR" & abs(Hdetail.H_s-h)<1e-10,:); dc=Hdetail(Hdetail.TrackedMode=="DC" & abs(Hdetail.H_s-h)<1e-10,:); sy=Hdetail(Hdetail.TrackedMode=="SYNC" & abs(Hdetail.H_s-h)<1e-10,:); gs=Hdetail(Hdetail.TrackedMode=="GSC" & abs(Hdetail.H_s-h)<1e-10,:);
    ipole=abs(tor.Damping-SrefRow.Damping)/max(abs(SrefRow.Damping),eps); ipath=abs(log10(max(tor.ResidueMagnitude,eps)/max(SrefRow.ResidueMagnitude,eps)));
    T.Parameter(r)=h; T.Status(r)="PASS"; T.f_tor_Hz(r)=tor.Frequency_Hz; T.zeta_tor(r)=tor.Damping; T.PoleReal(r)=tor.PoleReal; T.Rtor_frequency(r)=tor.ResidueMagnitude; T.Rdc_frequency(r)=dc.ResidueMagnitude; T.Rsync_frequency(r)=sy.ResidueMagnitude; T.Rgsc_frequency(r)=gs.ResidueMagnitude; T.PeakOmegaSh_frequency(r)=NaN; T.D_e(r)=NaN; T.K_e(r)=NaN; T.G_Udc_iqref(r)=NaN; T.G_Udc_Te(r)=NaN; T.I_pole(r)=ipole; T.I_path(r)=ipath; T.Gamma_path(r)=tor.ResidueMagnitude/max(SrefRow.ResidueMagnitude,eps); T.Reference(r)="VSG baseline H=3 s"; T.Classification(r)=localClass(ipole,ipath);
end
end

function T=localTable(n)
T=table('Size',[n 19],'VariableTypes',{'double','string','double','double','double','double','double','double','double','double','double','double','double','double','double','double','double','string','string'}, ...
 'VariableNames',{'Parameter','Status','f_tor_Hz','zeta_tor','PoleReal','Rtor_frequency','Rdc_frequency','Rsync_frequency','Rgsc_frequency','PeakOmegaSh_frequency','D_e','K_e','G_Udc_iqref','G_Udc_Te','I_pole','I_path','Gamma_path','Reference','Classification'});
end

function row=localRow(p,status,S,Ref)
ip=abs(S.zeta_tor-Ref.zeta_tor)/max(abs(Ref.zeta_tor),eps); gamma=S.Rtor_frequency/max(Ref.Rtor_frequency,eps); row={p,status,S.f_tor,S.zeta_tor,S.pole_real,S.Rtor_frequency,S.Rdc_frequency,S.Rsync_frequency,S.Rgsc_frequency,S.PeakOmegaSh_frequency,S.D_e,S.K_e,S.G_Udc_to_iqref,S.G_Udc_to_Te,ip,abs(log10(max(gamma,eps))),gamma,"matched reference",localClass(ip,abs(log10(max(gamma,eps))))};
end
function row=localFail(p,msg), row={p,"FAIL",NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,string(msg),"FAIL"}; end
function c=localClass(ip,ia)
ep=.05; ed=log10(1.20);
if ip<ep && ia>ed, c="PATH_SHAPING_DOMINATED"; elseif ip>=ep && ia>ed, c="JOINT_POLE_PATH_SHAPING"; else, c="WEAK_CONTROL_EFFECT"; end
end
function T=localTag(T,tag), T.ParameterFamily=repmat(string(tag),height(T),1); T=movevars(T,'ParameterFamily','Before','Parameter'); end
