function R = build_mechanism_region_map(outDir)
%BUILD_MECHANISM_REGION_MAP
% 以匹配SCR的GFL作为结构参考，形成SCR-H与SCR-DVC的一维/二维机制区域摘要。
% 分类阈值用于呈现连续指标，不被当作物理稳定边界。
if nargin<1 || isempty(outDir), outDir=fileparts(mfilename('fullpath')); end
[models,base]=prepare_multimode_models(); p0=base.parameter_vector; iv=find(strcmpi(base.models,'VSG'),1); ig=find(strcmpi(base.models,'GFL'),1);
xV=models{iv}.x0; xG=models{ig}.x0; scrs=[2 3 4 6 8 10]; Hs=[1.5 3 6]; dvcs=[.5 1 1.5 2]; scr0=4;
gfl=cell(numel(scrs),1); vsgBase=cell(numel(scrs),1);
for s=1:numel(scrs)
    p=p0; p(9)=p0(9)*scr0/scrs(s); p(10)=p0(10)*scr0/scrs(s); gfl{s}=mechanism_transition_point(p,'GFL',xG,struct,true); vsgBase{s}=mechanism_transition_point(p,'VSG',xV,struct,true);
end
A=localMap('SCR_H',scrs,Hs,p0,xV,gfl,'H',"matched GFL at same SCR");
% DVC参数机制以同SCR、DVC=1的VSG基准为参考；这样“弱效应”是有物理含义的基准点，而非人为补点。
B=localMap('SCR_DVC',scrs,dvcs,p0,xV,vsgBase,'DVC',"VSG at matched SCR, DVC=1"); T=[A;B];
writetable(T,fullfile(outDir,'Mechanism_Region_Map_Summary.csv'));
classes=unique(T.MechanismClass,'stable'); counts=table(classes,zeros(numel(classes),1),'VariableNames',{'MechanismClass','Count'});
for k=1:numel(classes), counts.Count(k)=sum(T.MechanismClass==classes(k)); end
writetable(counts,fullfile(outDir,'Mechanism_Region_Class_Count.csv'));
R=struct('summary',T,'scr_h',A,'scr_dvc',B,'class_counts',counts,'thresholds',struct('I_pole',.05,'I_path',log10(1.2)));
end

function T=localMap(family,scrs,vals,p0,xV,refs,kind,referenceLabel)
N=numel(scrs)*numel(vals); T=table('Size',[N 16], ...
 'VariableTypes',{'string','double','double','double','string','double','double','double','double','double','double','double','double','string','string','string'}, ...
 'VariableNames',{'Map','SCR','ControlValue','ControlValueFactor','Status','f_tor_Hz','zeta_tor','I_pole','Rtor_frequency','I_path','Gamma_path','PeakOmegaSh_frequency','Gamma_response','DominantModalSet','MechanismClass','Reference'}); r=0; scr0=4;
for s=1:numel(scrs)
    for v=1:numel(vals)
        r=r+1; p=p0; p(9)=p0(9)*scr0/scrs(s); p(10)=p0(10)*scr0/scrs(s);
        if strcmp(kind,'H'), p(33)=vals(v); factor=vals(v)/p0(33); else, p(25)=p0(25)*vals(v); p(26)=p0(26)*vals(v); factor=vals(v); end
        try
            S=mechanism_transition_point(p,'VSG',xV,struct,true); G=refs{s}; ip=abs(S.zeta_tor-G.zeta_tor)/max(abs(G.zeta_tor),eps); gamma=S.Rtor_frequency/max(G.Rtor_frequency,eps); gp=S.PeakOmegaSh_frequency/max(G.PeakOmegaSh_frequency,eps);
            T.Map(r)=family; T.SCR(r)=scrs(s); T.ControlValue(r)=vals(v); T.ControlValueFactor(r)=factor; T.Status(r)="PASS"; T.f_tor_Hz(r)=S.f_tor; T.zeta_tor(r)=S.zeta_tor; T.I_pole(r)=ip; T.Rtor_frequency(r)=S.Rtor_frequency; T.I_path(r)=abs(log10(max(gamma,eps))); T.Gamma_path(r)=gamma; T.PeakOmegaSh_frequency(r)=S.PeakOmegaSh_frequency; T.Gamma_response(r)=gp; T.DominantModalSet(r)=localDominant(S); T.MechanismClass(r)=localClass(ip,abs(log10(max(gamma,eps)))); T.Reference(r)=referenceLabel;
        catch ME
            T.Map(r)=family; T.SCR(r)=scrs(s); T.ControlValue(r)=vals(v); T.ControlValueFactor(r)=factor; T.Status(r)="FAIL"; T.f_tor_Hz(r)=NaN; T.zeta_tor(r)=NaN; T.I_pole(r)=NaN; T.Rtor_frequency(r)=NaN; T.I_path(r)=NaN; T.Gamma_path(r)=NaN; T.PeakOmegaSh_frequency(r)=NaN; T.Gamma_response(r)=NaN; T.DominantModalSet(r)=string(ME.message); T.MechanismClass(r)="FAIL"; T.Reference(r)="";
        end
    end
end
end

function s=localDominant(S)
[~,i]=max([S.Rtor_frequency,S.Rdc_frequency,S.Rsync_frequency,S.Rgsc_frequency]); s=["TOR","DC","SYNC","GSC"]; s=s(i); end
function c=localClass(ip,ia), if ip<.05 && ia>log10(1.2), c="PATH_SHAPING_DOMINATED"; elseif ip>=.05 && ia>log10(1.2), c="JOINT_POLE_PATH_SHAPING"; else, c="WEAK_CONTROL_EFFECT"; end, end
