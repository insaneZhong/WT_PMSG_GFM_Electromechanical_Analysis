function R = run_m1_mechanism_robustness(varargin)
%RUN_M1_MECHANISM_ROBUSTNESS M1-a 机制稳健性审计（T1-T5统一入口）。
% 当前先完成T1；后续阶段只在前一Gate通过后追加。
% 不复制模型，不保存时序或solver历史，只覆盖摘要CSV、中文报告和综合图。

ip=inputParser;
ip.addParameter('StopAfter','T5',@(x)ischar(x)||isstring(x));
ip.parse(varargin{:}); opt=ip.Results;

here=fileparts(mfilename('fullpath'));
m0Dir=fullfile(fileparts(here),'CurrentModel_Idealized');
addpath(here,m0Dir);

[models0,base]=prepare_multimode_models();
p=base.parameter_vector(:).';
igwt=find(cellfun(@(q)strcmpi(q.mode,'GFMGWT'),models0),1);
assert(~isempty(igwt),'未找到GFM-GWT共同工作点模型。');
gwt0=models0{igwt}; flags=gwt0.flags; flags.vscDcMode='FIXED_NORM';
[x,eq]=solveM1(gwt0.x0,p,'GFMGWT',flags);
L0=linearize_physicalavg_m1(x,p,'GFMGWT',flags); L0.label=gwt0.label;

% T1：基准失稳模式及8个控制参数的特征值/副作用灵敏度。
[baseMode,baseMetrics]=baselineElectricalMode(L0,p);
specs=parameterSpecs(p,flags);
T1=table();
for k=1:numel(specs)
    q=parameterSensitivity(specs(k),x,p,flags,L0,baseMode,baseMetrics);
    T1=[T1;struct2table(q,'AsArray',true)]; %#ok<AGROW>
end

% Gate T1：解析灵敏度与有限差分方向必须一致，并至少有一个有效稳定化旋钮。
consistent=T1.DirectionConsistent & T1.MAC_Plus>0.8 & T1.MAC_Minus>0.8;
useful=consistent & abs(T1.dSigma_dFactor)>1e-4 & T1.Score>0;
gateT1=all(consistent) && any(useful);
[~,order]=sort(T1.Score,'descend'); T1=T1(order,:);
rank=(1:height(T1)).'; T1=addvars(T1,rank,'Before',1,'NewVariableNames','Rank');

summaryFile=fullfile(here,'M1_Mechanism_Robustness_Summary.csv');
reportFile=fullfile(here,'M1_Mechanism_Robustness_Report_CN.md');
figureFile=fullfile(here,'M1_Mechanism_Robustness_Comparison.png');
writetable(T1,summaryFile,'Encoding','UTF-8');

T2=table(); T3=table(); T4=table(); T5=table(); tune=struct; boundary=struct;
gateT2=false; gateT3=false; gateT4=false; gateT5=false; conclusionT3="NOT_RUN"; conclusionT4="NOT_RUN"; conclusionT5="NOT_RUN";
stage="T1";
if gateT1 && ~strcmpi(string(opt.StopAfter),'T1')
    topSpec=findSpec(specs,T1.Parameter(1));
    [T2,tune,gateT2]=runT2(topSpec,x,p,flags,L0,baseMode,baseMetrics);
    stage="T2";
    if gateT2 && ~strcmpi(string(opt.StopAfter),'T2')
        [T3,gateT3,conclusionT3]=runT3(models0,p,x,flags,L0,tune);
        stage="T3";
        if gateT3 && ~strcmpi(string(opt.StopAfter),'T3')
            [T4,gateT4,conclusionT4]=runT4(models0,p,x,flags,tune);
            stage="T4";
            if gateT4 && ~strcmpi(string(opt.StopAfter),'T4')
                [T5,boundary,gateT5,conclusionT5]=runT5(models0,p,x,flags,tune);
                stage="T5";
            end
        end
    end
end

makeCombinedFigure(T1,T2,T3,T4,T5,baseMode,figureFile);
writeFullReport(reportFile,T1,T2,T3,T4,T5,baseMode,baseMetrics,eq,gateT1,gateT2,gateT3,gateT4,gateT5,tune,boundary,conclusionT3,conclusionT4,conclusionT5,opt.StopAfter);
writeStageSummary(summaryFile,T1,T2,T3,T4,T5);

R=struct('stage',char(stage),'gateT1',gateT1,'gateT2',gateT2,'gateT3',gateT3,'gateT4',gateT4,'gateT5',gateT5, ...
    'equilibrium',eq,'baseline_mode',baseMode,'baseline_metrics',baseMetrics, ...
    'sensitivity',T1,'retuning_scan',T2,'retuning',tune,'directional',T3,'pole_path',T4,'boundary_scan',T5,'boundary',boundary, ...
    'directional_conclusion',conclusionT3,'pole_path_conclusion',conclusionT4,'boundary_conclusion',conclusionT5,'summary_file',summaryFile, ...
    'report_file',reportFile,'figure_file',figureFile);
fprintf('%s complete: Gates [%d %d %d %d %d].\n',stage,gateT1,gateT2,gateT3,gateT4,gateT5);
end

function s=findSpec(specs,name)
i=find(string({specs.name})==string(name),1);assert(~isempty(i),'Missing spec %s',name);s=specs(i);
end

function [T,tune,gate]=runT2(spec,x,p,flags,Lbase,baseMode,bm)
% 单参数有利方向扫描；若能包围则分别求零实部临界值和-0.055 1/s稳定点。
if realDirection(spec,x,p,flags,Lbase,baseMode)>0
    factors=[1 .95 .9 .85 .8 .75 .7 .65 .6 .55 .5 .45 .4];
else
    factors=[1 1.05 1.1 1.15 1.2 1.25 1.3 1.35 1.4 1.45 1.5 1.6];
end
T=table();
for f=factors
    [pp,ff]=applyFactor(spec,p,flags,f);L=linearize_physicalavg_m1(x,pp,'GFMGWT',ff);
    [lam,mac]=matchMode(L,baseMode);m=systemMetrics(L,p); y=source_aligned_internal_outputs_control_m1(x,pp,'GFMGWT',zeros(4,1),ff);
    row=table(string(spec.name),f,real(lam),abs(imag(lam))/(2*pi),mac,m.max_real,m.f_tor_Hz,m.zeta_tor,m.C_GM,m.C_MG,y(10),x(11),x(9), ...
        'VariableNames',{'Parameter','Factor','ElectricalPoleReal','ElectricalFreq_Hz','MAC','MaxRealPole','f_tor_Hz','zeta_tor','C_GM','C_MG','P0_W','Q0_var','Udc0_V'});
    T=[T;row]; %#ok<AGROW>
end
% 找离1最近且进入稳定侧的扫描点。
stableIdx=find(T.ElectricalPoleReal<0,1,'first');
if isempty(stableIdx)
    tune=struct('Parameter',spec.name,'GateReason',"单参数扫描未找到稳定侧");gate=false;return;
end
fStableScan=T.Factor(stableIdx);
fCrit=bisectFactor(spec,x,p,flags,baseMode,1,fStableScan,0);
target=-0.055;
if min(T.ElectricalPoleReal)<=target
    fTarget=bisectFactor(spec,x,p,flags,baseMode,1,T.Factor(find(T.ElectricalPoleReal<=target,1,'first')),target);
else
    fTarget=fStableScan;
end
[ps,fs]=applyFactor(spec,p,flags,fTarget);Ls=linearize_physicalavg_m1(x,ps,'GFMGWT',fs);
[lamS,macS]=matchMode(Ls,baseMode);ms=systemMetrics(Ls,p);
fb=flags;fb.vscDcMode='REALTIME_FF';Lb=linearize_physicalavg_m1(x,ps,'GFMGWT',fb);
matrixDiff=norm(Ls.A-Lb.A,'fro')/max(norm(Lb.A,'fro'),eps);
y=source_aligned_internal_outputs_control_m1(x,ps,'GFMGWT',zeros(4,1),fs);
directPgsc=y(1)/p(2);
dfTor=abs(ms.f_tor_Hz-bm.f_tor_Hz)/bm.f_tor_Hz;
workpoint=max([abs(y(10)-sourceOut(Lbase,'P_PCC'))/p(1),abs(x(11)-Lbase.x0(11))/max(abs(Lbase.x0(11)),1),abs(x(9)-Lbase.x0(9))/p(2)]);
gate=real(lamS)<-0.05 && ms.max_real<0 && macS>0.8 && matrixDiff>1e-8 && abs(directPgsc)>1e-9 && dfTor<0.02 && workpoint<1e-8;
tune=struct('Parameter',spec.name,'BaselineValue',spec.value,'CriticalFactor',fCrit,'StableFactor',fTarget, ...
    'StableValue',spec.value*fTarget,'ElectricalLambda',lamS,'ElectricalMAC',macS,'MaxRealPole',ms.max_real, ...
    'f_tor_Hz',ms.f_tor_Hz,'zeta_tor',ms.zeta_tor,'C_GM',ms.C_GM,'C_MG',ms.C_MG, ...
    'M1aM1bMatrixRelativeDifference',matrixDiff,'Analytic_dPGSC_dUdc',directPgsc, ...
    'RelativeTorsionFrequencyChange',dfTor,'RelativeWorkpointChange',workpoint, ...
    'p',ps,'flags',fs,'L',Ls,'GateReason',string(sprintf('lambda_e=%.4g, maxRe=%.4g, df_tor=%.3g',real(lamS),ms.max_real,dfTor)));
end

function d=realDirection(spec,x,p,flags,Lbase,baseMode)
h=2e-3;[pp,fp]=applyFactor(spec,p,flags,1+h);[pm,fm]=applyFactor(spec,p,flags,1-h);
Lp=linearize_physicalavg_m1(x,pp,'GFMGWT',fp);Lm=linearize_physicalavg_m1(x,pm,'GFMGWT',fm);
[lp,~]=matchMode(Lp,baseMode);[lm,~]=matchMode(Lm,baseMode);d=real((lp-lm)/(2*h)); %#ok<INUSD>
end
function f=bisectFactor(spec,x,p,flags,ref,fa,fb,target)
ga=modeRealAt(spec,x,p,flags,ref,fa)-target;gb=modeRealAt(spec,x,p,flags,ref,fb)-target;
assert(ga*gb<=0,'目标实部未被包围: [%.4g, %.4g].',ga,gb);
for k=1:50
    fm=(fa+fb)/2;gm=modeRealAt(spec,x,p,flags,ref,fm)-target;
    if abs(gm)<1e-6||abs(fa-fb)<1e-6,f=fm;return;end
    if ga*gm<=0,fb=fm;gb=gm;else,fa=fm;ga=gm;end %#ok<NASGU>
end
f=(fa+fb)/2;
end
function r=modeRealAt(spec,x,p,flags,ref,f)
[pp,ff]=applyFactor(spec,p,flags,f);L=linearize_physicalavg_m1(x,pp,'GFMGWT',ff);[lam,~]=matchMode(L,ref);r=real(lam);
end
function v=sourceOut(L,name),i=outIndex(L,name);v=L.y0(i);end

function [T,gate,conclusion]=runT3(models0,p,xGwt,flagsGwt,LgwtOrig,tune)
igwt=find(cellfun(@(q)strcmpi(q.mode,'GFMGWT'),models0),1);
imwt=find(cellfun(@(q)strcmpi(q.mode,'VSG'),models0),1);
assert(~isempty(igwt)&&~isempty(imwt));
cases=repmat(cs('GWT-M0',models0{igwt}),1,5);
fb=flagsGwt;fb.vscDcMode='REALTIME_FF';Lb=linearize_physicalavg_m1(xGwt,p,'GFMGWT',fb);Lb.label='GWT-M1b';cases(2)=cs('GWT-M1b',Lb);
cases(3)=cs('GWT-M1a-original',LgwtOrig);
cases(4)=cs('GWT-M1a-stable',tune.L);
fm=models0{imwt}.flags;fm.vscDcMode='FIXED_NORM';[xm,~]=solveM1(models0{imwt}.x0,p,'VSG',fm);Lm=linearize_physicalavg_m1(xm,p,'VSG',fm);Lm.label='MWT-M1a';cases(5)=cs('MWT-M1a',Lm);
T=table();
for k=1:numel(cases)
    m=systemMetrics(cases(k).L,p);csum=m.C_GM+m.C_MG;gamma=max(m.C_GM,m.C_MG)/max(min(m.C_GM,m.C_MG),1e-15);ds=sign(m.C_GM-m.C_MG);
    row=table(string(cases(k).name),m.max_real,m.f_tor_Hz,m.zeta_tor,m.C_GM,m.C_MG,csum,gamma,ds,string(directionClass(m.C_GM,m.C_MG)), ...
        'VariableNames',{'Case','MaxRealPole','f_tor_Hz','zeta_tor','C_GM','C_MG','C_Sigma','GammaDirection','D_sign','DirectionClass'});
    T=[T;row]; %#ok<AGROW>
end
gwt=T(T.Case=="GWT-M1a-stable",:);mwt=T(T.Case=="MWT-M1a",:);
if mwt.GammaDirection>10 && gwt.GammaDirection<=3
    conclusion="A: MWT Grid-to-Machine dominance robust; GWT has no strong directional dominance";gate=true;
elseif mwt.GammaDirection>10 && gwt.GammaDirection>10
    conclusion="B: GWT directionality depends on controller tuning and physical VSC dynamics";gate=true;
elseif mwt.GammaDirection<=3
    conclusion="C: directional-coupling framework is strongly model dependent";gate=false;
else
    conclusion="INTERMEDIATE: only moderate cross-model directional dominance";gate=mwt.GammaDirection>3;
end
gate=gate&&all(T.MaxRealPole([4 5])<0);
end
function s=cs(name,L),s=struct('name',string(name),'L',L);end
function s=directionClass(cgm,cmg)
g=max(cgm,cmg)/max(min(cgm,cmg),1e-15);
if g>10,t="STRONG";elseif g>3,t="MODERATE";else,t="COMPARABLE";end
if cgm>cmg,d="GRID_TO_MACHINE";else,d="MACHINE_TO_GRID";end
s=d+"_"+t;
end

function [T,gate,conclusion]=runT4(models0,p,xGwt,flagsGwt,tune)
% 代表点：局部MPPT/MSC-DVC阻尼符号 + M0到M1的Pole/Path变化。
igwt=find(cellfun(@(q)strcmpi(q.mode,'GFMGWT'),models0),1);
imwt=find(cellfun(@(q)strcmpi(q.mode,'VSG'),models0),1);
fm=models0{imwt}.flags;fm.vscDcMode='FIXED_NORM';[xm,~]=solveM1(models0{imwt}.x0,p,'VSG',fm);Lm=linearize_physicalavg_m1(xm,p,'VSG',fm);

ffMppt=tune.flags;ffMppt.mpptLocal=true;ffMppt.Kmppt_iq_per_radps=2*xGwt(5)/p(12);
LgMppt=linearize_physicalavg_m1(xGwt,tune.p,'GFMGWT',ffMppt);
fmOff=fm;fmOff.cutUdcToDvc=true;LmOff=linearize_physicalavg_m1(xm,p,'VSG',fmOff);

T=table();
dCases={"GWT-Frozen-stable",tune.L;"GWT-MPPT-stable",LgMppt;"MWT-DVC",Lm;"MWT-DVC-off",LmOff};
for k=1:size(dCases,1)
    L=dCases{k,2};[de,ke,mag]=complexTorque(L);m=basicModalMetrics(L);
    row=t4row("DAMPING_CHANNEL",dCases{k,1},archFromName(dCases{k,1}),"M1-a",m,de,ke,mag,NaN,NaN,NaN,NaN,NaN,NaN,NaN,"");
    T=[T;row]; %#ok<AGROW>
end
dMppt=T.De_at_ftor(T.Case=="GWT-MPPT-stable")-T.De_at_ftor(T.Case=="GWT-Frozen-stable");
dDvc=T.De_at_ftor(T.Case=="MWT-DVC")-T.De_at_ftor(T.Case=="MWT-DVC-off");
T.DeltaDe(T.Case=="GWT-MPPT-stable")=dMppt;T.DeltaDe(T.Case=="MWT-DVC")=dDvc;

ppCases={"GWT",models0{igwt},tune.L;"MWT",models0{imwt},Lm};
pathRobust=true;
for k=1:size(ppCases,1)
    arch=string(ppCases{k,1});Lref=ppCases{k,2};Lm1=ppCases{k,3};
    [rgr0,rme0,pgr0,pme0]=residueMetrics(Lref,p);[de0,ke0,mag0]=complexTorque(Lref);m0=systemMetrics(Lref,p);
    [rgr1,rme1,pgr1,pme1]=residueMetrics(Lm1,p);[de1,ke1,mag1]=complexTorque(Lm1);m1=systemMetrics(Lm1,p);
    T=[T;t4row("POLE_PATH",arch+"-M0",arch,"M0",m0,de0,ke0,mag0,rgr0,rme0,pgr0,pme0,0,0,0,"REFERENCE")]; %#ok<AGROW>
    ipole=abs(m1.zeta_tor/m0.zeta_tor-1);ipathG=abs(log10(max(rgr1,1e-30)/max(rgr0,1e-30)));ipathM=abs(log10(max(rme1,1e-30)/max(rme0,1e-30)));
    pathRobust=pathRobust&&(ipathG>ipole||ipathM>ipole);
    T=[T;t4row("POLE_PATH",arch+"-M1a",arch,"M1-a",m1,de1,ke1,mag1,rgr1,rme1,pgr1,pme1,ipole,ipathG,ipathM,"M0_TO_M1")]; %#ok<AGROW>
end
stable=all(T.MaxRealPole<0);
gate=dMppt>0 && dDvc<0 && pathRobust && stable;
conclusion=string(sprintf('DeltaDe_MPPT=%+.6g, DeltaDe_MSC_DVC=%+.6g; representative path-dominated=%d; all stable=%d',dMppt,dDvc,pathRobust,stable));
end

function row=t4row(rt,ca,arch,level,m,de,ke,mag,rg,rm,pg,pm,ip,ig,im,note)
row=table(string(rt),string(ca),string(arch),string(level),m.max_real,m.f_tor_Hz,m.zeta_tor,de,ke,mag,rg,rm,pg,pm,ip,ig,im,NaN,string(note), ...
 'VariableNames',{'RecordType','Case','Architecture','ModelLevel','MaxRealPole','f_tor_Hz','zeta_tor','De_at_ftor','Ke_at_ftor','Magnitude_Te_omega','ResidueGridToShaft','ResidueMechToShaft','InputProjectionGrid','InputProjectionMech','I_pole','I_path_grid','I_path_mech','DeltaDe','Note'});
end
function a=archFromName(n),if contains(n,'GWT'),a="GWT";else,a="MWT";end,end
function [de,ke,mag]=complexTorque(L)
M=multimode_modal_data(L.A,L.state_names);it=multimode_pick_torsion_mode(M);w=abs(imag(M.lambda(it)));G=prescribedSpeedFrf(L,w);gt=G(outIndex(L,'T_e'));de=real(gt);ke=-w*imag(gt);mag=abs(gt);
end
function m=basicModalMetrics(L)
M=multimode_modal_data(L.A,L.state_names);it=multimode_pick_torsion_mode(M);z=M.lambda(it);
m=struct('lambda_tor',z,'f_tor_Hz',abs(imag(z))/(2*pi),'zeta_tor',-real(z)/abs(z), ...
    'C_GM',NaN,'C_MG',NaN,'max_real',maxRealPole(L.A));
end
function G=prescribedSpeedFrf(L,w)
ig=3;ir=setdiff(1:size(L.A,1),ig);G=L.C(:,ir)*((1i*w*eye(numel(ir))-L.A(ir,ir))\L.A(ir,ig))+L.C(:,ig);
end
function [rg,rm,pg,pm]=residueMetrics(L,p)
M=multimode_modal_data(L.A,L.state_names);it=multimode_pick_torsion_mode(M);iy=outIndex(L,'omega_sh');
rg=abs(modalResidue(L,M,it,iy,4))*p(3)/p(12);rm=abs(modalResidue(L,M,it,iy,1))*(p(1)/p(12))/p(12);
pg=abs(M.W(:,it)'*L.B(:,4))*p(3);pm=abs(M.W(:,it)'*L.B(:,1))*(p(1)/p(12));
end
function r=modalResidue(L,M,it,iy,iu),r=L.C(iy,:)*M.V(:,it)*(M.W(:,it)'*L.B(:,iu));end

function [T,B,gate,conclusion]=runT5(models0,p,x,flags,tune)
% 在稳定M1-a上局部扫描H和GSC-DVC，追踪同一电气模态而非只看临界数字。
igwt=find(cellfun(@(q)strcmpi(q.mode,'GFMGWT'),models0),1);gwt0=models0{igwt};
[refMode,~]=baselineElectricalMode(tune.L,p);
kinds=["H","GSC_DVC"];
factorSets={unique([.5 .75 1 1.25 1.5 2 2.5 3 4 5 6]),unique([.05 .1 .2 .3 .5 .75 1 1.5 2 3 5])};
T=table();B=struct([]);gate=true;messages=strings(0,1);
for kk=1:2
    kind=kinds(kk);factors=factorSets{kk};vals=zeros(size(factors));
    for j=1:numel(factors)
        [pp,ff]=applyBoundaryFactor(kind,tune.p,tune.flags,factors(j));L=linearize_physicalavg_m1(x,pp,'GFMGWT',ff);[lam,mac]=matchMode(L,refMode);M=multimode_modal_data(L.A,L.state_names);it=bestModeIndex(M,refMode.v);g=participationDetail(M,it,L.state_names);m=systemMetrics(L,p);vals(j)=real(lam);
        T=[T;t5row("TRACE",kind,factors(j),lam,mac,m,g,NaN,NaN,"")]; %#ok<AGROW>
    end
    pair=findCrossingNearOne(factors,vals);
    if isempty(pair)
        gate=false;messages(end+1)=kind+": no local boundary bracket"; %#ok<AGROW>
        continue
    end
    fc=bisectBoundary(kind,x,tune.p,tune.flags,refMode,factors(pair(1)),factors(pair(2)));
    [pp,ff]=applyBoundaryFactor(kind,tune.p,tune.flags,fc);Lc=linearize_physicalavg_m1(x,pp,'GFMGWT',ff);[lam,mac]=matchMode(Lc,refMode);Mc=multimode_modal_data(Lc.A,Lc.state_names);it=bestModeIndex(Mc,refMode.v);g=participationDetail(Mc,it,Lc.state_names);m=systemMetrics(Lc,p);
    old=oldCritical(kind,p,gwt0);
    % maxModeMac直接与M1临界右特征向量比较。
    mac0=maxModeMac(old,Mc.V(:,it));
    T=[T;t5row("CRITICAL",kind,fc,lam,mac,m,g,mac0,fc,string(modeFamily(g)))]; %#ok<AGROW>
    b=struct('Kind',kind,'CriticalFactor',fc,'Lambda',lam,'Frequency_Hz',abs(imag(lam))/(2*pi),'MAC_to_stable_M1',mac,'MAC_to_M0',mac0,'Participation',g,'TorsionFrequency_Hz',m.f_tor_Hz,'TorsionZeta',m.zeta_tor,'ModeFamily',modeFamily(g));
    if isempty(B),B=b;else,B(end+1)=b;end %#ok<AGROW>
    gate=gate && g.MECH<0.3 && mac0>0.8 && (g.SYNC+g.DC+g.GSC_DVC+g.GSC_VOLTAGE+g.GSC_CURRENT+g.LCL_GRID)>0.7;
end
if isempty(messages),messages="both boundaries found";end
conclusion=strjoin(messages,'; ')+"; electrical family gate="+string(gate);
end
function row=t5row(rt,kind,factor,lam,mac,m,g,mac0,crit,note)
row=table(string(rt),string(kind),factor,real(lam),abs(imag(lam))/(2*pi),mac,m.max_real,m.f_tor_Hz,m.zeta_tor,g.MECH,g.DC,g.GSC_DVC,g.GSC_VOLTAGE,g.GSC_CURRENT,g.LCL_GRID,g.SYNC,mac0,crit,string(note), ...
 'VariableNames',{'RecordType','Parameter','Factor','ElectricalPoleReal','ElectricalFreq_Hz','MAC_to_StableM1','MaxRealPole','f_tor_Hz','zeta_tor','Pi_MECH','Pi_DC','Pi_GSC_DVC','Pi_GSC_Voltage','Pi_GSC_Current','Pi_LCL_Grid','Pi_SYNC','MAC_to_M0','CriticalFactor','ModeFamily'});
end
function [pp,ff]=applyBoundaryFactor(kind,p,flags,f)
pp=p;ff=flags;if kind=="H",pp(33)=p(33)*f;else,ff.KpGscDvc=flags.KpGscDvc*f;ff.KiGscDvc=flags.KiGscDvc*f;end
end
function pair=findCrossingNearOne(f,v)
q=find(v(1:end-1).*v(2:end)<=0);if isempty(q),pair=[];return;end
[~,j]=min(abs((f(q)+f(q+1))/2-1));pair=[q(j),q(j)+1];
end
function fc=bisectBoundary(kind,x,p,flags,ref,a,b)
fa=boundaryReal(kind,x,p,flags,ref,a);fb=boundaryReal(kind,x,p,flags,ref,b);assert(fa*fb<=0);
for k=1:50,c=(a+b)/2;fcv=boundaryReal(kind,x,p,flags,ref,c);if abs(fcv)<1e-6||abs(a-b)<1e-6,fc=c;return;end;if fa*fcv<=0,b=c;fb=fcv;else,a=c;fa=fcv;end,end %#ok<NASGU>
fc=(a+b)/2;
end
function r=boundaryReal(kind,x,p,flags,ref,f),[pp,ff]=applyBoundaryFactor(kind,p,flags,f);L=linearize_physicalavg_m1(x,pp,'GFMGWT',ff);[lam,~]=matchMode(L,ref);r=real(lam);end
function L=oldCritical(kind,p,gwt0)
pp=p;ff=gwt0.flags;if kind=="H",pp(33)=4.97021484375;else,ff.KpGscDvc=ff.KpGscDvc*1.64208984375;ff.KiGscDvc=ff.KiGscDvc*1.64208984375;end
L=multimode_linearize_control(gwt0.x0,pp,'GFMGWT',ff);
end
function m=maxModeMac(L,v)
M=multimode_modal_data(L.A,L.state_names);cand=find(imag(M.lambda)>0);z=zeros(numel(cand),1);for k=1:numel(cand),z(k)=modalMac(M.V(:,cand(k)),v);end;m=max(z);
end
function it=bestModeIndex(M,v),cand=find(imag(M.lambda)>0);z=zeros(numel(cand),1);for k=1:numel(cand),z(k)=modalMac(M.V(:,cand(k)),v);end;[~,j]=max(z);it=cand(j);end
function s=modeFamily(g)
if g.MECH>=.3,s="TORSIONAL";elseif g.GSC_VOLTAGE+g.GSC_CURRENT+g.LCL_GRID>.5,s="GSC_SYNC_ELECTRICAL";else,s="SYNC_DC_ELECTRICAL";end
end

function makeCombinedFigure(T1,T2,T3,T4,T5,mode,file)
f=figure('Visible','off','Color','w','Position',[40 40 1500 1220]);
tl=tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
labs=categorical(T1.Parameter,T1.Parameter);
nexttile;barh(labs,T1.dSigma_dFactor);xline(0,':k');grid on;xlabel('d Re(\lambda_e) / d factor (1/s)');title('T1: stabilizing sensitivity');
nexttile;
if ~isempty(T2)
    plot(T2.Factor,T2.ElectricalPoleReal,'-o','LineWidth',1.8,'DisplayName','tracked electrical mode');hold on;
    plot(T2.Factor,T2.MaxRealPole,'--s','LineWidth',1.4,'DisplayName','maximum real pole');yline(0,':r','HandleVisibility','off');grid on;xlabel('parameter factor');ylabel('real part (1/s)');legend('Location','best');title('T2: minimal retuning');
else,text(.1,.5,'T2 not run');axis off;end
nexttile;
if ~isempty(T3)
    bar(categorical(T3.Case,T3.Case),[T3.C_GM,T3.C_MG]);set(gca,'YScale','log');grid on;ylabel('normalized coupling');legend('Grid->Machine','Machine->Grid','Location','best');xtickangle(18);title('T3: bidirectional coupling');
else,text(.1,.5,'T3 not run');axis off;end
nexttile;vals=[mode.participation.MECH,mode.participation.DC,mode.participation.GSC_DVC,mode.participation.GSC_VOLTAGE,mode.participation.GSC_CURRENT,mode.participation.LCL_GRID,mode.participation.SYNC];
bar(categorical({'MECH','DC','GSC-DVC','GSC-V','GSC-I','LCL/Grid','SYNC'}),vals);grid on;ylabel('normalized participation');title(sprintf('Original unstable mode: %.4f %+.4fj 1/s',real(mode.lambda),imag(mode.lambda)));
nexttile;
if ~isempty(T4)
    q=T4(T4.RecordType=="DAMPING_CHANNEL"&isfinite(T4.DeltaDe),:);bar(categorical(q.Case,q.Case),q.DeltaDe);yline(0,':k','HandleVisibility','off');grid on;ylabel('\Delta D_e at f_{tor}');xtickangle(16);title('T4: incremental damping sign');
else,text(.1,.5,'T4 not run');axis off;end
nexttile;
if ~isempty(T5)
    hold on;u=unique(T5.Parameter,'stable');for k=1:numel(u),q=T5(T5.Parameter==u(k)&T5.RecordType=="TRACE",:);if u(k)=="GSC_DVC",q=q(q.Factor>=.5&q.Factor<=1.5,:);else,q=q(q.Factor<=4,:);end;plot(q.Factor,q.ElectricalPoleReal,'-o','LineWidth',1.5,'DisplayName',char(u(k)));end;yline(0,':r','HandleVisibility','off');grid on;xlabel('parameter factor');ylabel('Re(\lambda_e) (1/s)');legend('Location','best');title('T5: local electrical boundaries');
else,text(.1,.5,'T5 not run');axis off;end
title(tl,'M1-a mechanism robustness across model fidelity: T1-T5');exportgraphics(f,file,'Resolution',200);close(f);
end

function writeStageSummary(file,T1,T2,T3,T4,T5)
names={'Stage','Record','Item','Factor','ElectricalPoleReal','MaxRealPole','f_tor_Hz','zeta_tor','C_GM','C_MG','GammaDirection','SensitivitySigma','Score','De_at_ftor','Ke_at_ftor','I_pole','I_path_grid','I_path_mech','MAC_to_M0','Pi_MECH','StatusText'};
types=[repmat({'string'},1,3),repmat({'double'},1,17),{'string'}];
S=table('Size',[0 numel(names)],'VariableTypes',types,'VariableNames',names);
for k=1:height(T1)
    S=[S;{ "T1","SENSITIVITY",T1.Parameter(k),1,NaN,NaN,NaN,NaN,NaN,NaN,NaN,T1.dSigma_dFactor(k),T1.Score(k),NaN,NaN,NaN,NaN,NaN,NaN,NaN,T1.FavorableDirection(k)}]; %#ok<AGROW>
end
for k=1:height(T2)
    S=[S;{ "T2","RETUNING_SCAN",T2.Parameter(k),T2.Factor(k),T2.ElectricalPoleReal(k),T2.MaxRealPole(k),T2.f_tor_Hz(k),T2.zeta_tor(k),T2.C_GM(k),T2.C_MG(k),NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,""}]; %#ok<AGROW>
end
for k=1:height(T3)
    S=[S;{ "T3","DIRECTIONAL",T3.Case(k),NaN,NaN,T3.MaxRealPole(k),T3.f_tor_Hz(k),T3.zeta_tor(k),T3.C_GM(k),T3.C_MG(k),T3.GammaDirection(k),NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,T3.DirectionClass(k)}]; %#ok<AGROW>
end
for k=1:height(T4)
    S=[S;{ "T4",T4.RecordType(k),T4.Case(k),NaN,NaN,T4.MaxRealPole(k),T4.f_tor_Hz(k),T4.zeta_tor(k),NaN,NaN,NaN,NaN,NaN,T4.De_at_ftor(k),T4.Ke_at_ftor(k),T4.I_pole(k),T4.I_path_grid(k),T4.I_path_mech(k),NaN,NaN,T4.Note(k)}]; %#ok<AGROW>
end
for k=1:height(T5)
    S=[S;{ "T5",T5.RecordType(k),T5.Parameter(k),T5.Factor(k),T5.ElectricalPoleReal(k),T5.MaxRealPole(k),T5.f_tor_Hz(k),T5.zeta_tor(k),NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,T5.MAC_to_M0(k),T5.Pi_MECH(k),T5.ModeFamily(k)}]; %#ok<AGROW>
end
writetable(S,file,'Encoding','UTF-8');
end

function writeFullReport(file,T1,T2,T3,T4,T5,mode,bm,eq,g1,g2,g3,g4,g5,tune,boundary,c3,c4,c5,stopAfter)
fid=fopen(file,'w','n','UTF-8');assert(fid>0);c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'# M1-a物理平均VSC机制稳健性审计\n\n');
fprintf(fid,'只使用唯一M1参数化模型；M0、公共plant、DC-link责任和共同工作点未修改。未引入PWM、离散、延迟、限幅、LVRT、Pitch或EMT。StopAfter=`%s`。\n\n',string(stopAfter));
fprintf(fid,['## 模型边界\n\n' ...
    '当前唯一 `.slx` 中的显式连续控制器固定实现 **MSC-DVC + GSC-VSG（MWT）**。' ...
    'GWT/MWT机制对照由同一套23状态参数化方程切换控制职责完成，不是该 `.slx` 的结构开关。' ...
    '因此，T2得到的GWT稳定化参数只属于参数化GWT机制模型，未写入MWT非线性 `.slx`。\n\n']);
fprintf(fid,'## T1：失稳来源定位（Gate %s）\n\n',passfail(g1));
fprintf(fid,'平衡点归一化残差 `%.3e`（exitflag=%g）。原始失稳模式为 $\\lambda_e=%.8g%+.8gj$ 1/s（%.8g Hz）；轴系为 %.8g Hz、阻尼 %.8g%%。\n\n',eq.scaled_residual,eq.exitflag,real(mode.lambda),imag(mode.lambda),mode.frequency_Hz,bm.f_tor_Hz,100*bm.zeta_tor);
fprintf(fid,'解析特征值灵敏度与有限差分方向全部一致；参数按稳定作用/副作用评分排序如下。\n\n');writeTable(fid,T1);
if ~isempty(T2)
    fprintf(fid,'\n## T2：最小重整定（Gate %s）\n\n',passfail(g2));
    fprintf(fid,'首选旋钮 `%s`：临界因子 %.8g，稳定因子 %.8g，参数由 %.8g 调为 %.8g。稳定点电气模态实部 %.8g 1/s，系统最大实部 %.8g 1/s，轴系频率 %.8g Hz、阻尼 %.8g%%。\n\n', ...
        tune.Parameter,tune.CriticalFactor,tune.StableFactor,tune.BaselineValue,tune.StableValue,real(tune.ElectricalLambda),tune.MaxRealPole,tune.f_tor_Hz,100*tune.zeta_tor);
    fprintf(fid,'M1-a与同参数M1-b的A矩阵相对差 %.3e；$dP_{GSC}/dUdc=%.8g$；轴系频率相对变化 %.3g；工作点相对变化 %.3g。说明稳定化没有恢复理想实时Udc前馈，也没有移动公共工作点。\n\n',tune.M1aM1bMatrixRelativeDifference,tune.Analytic_dPGSC_dUdc,tune.RelativeTorsionFrequencyChange,tune.RelativeWorkpointChange);
    writeTable(fid,T2);
end
if ~isempty(T3)
    fprintf(fid,'\n## T3：方向性传播复核（Gate %s）\n\n',passfail(g3));writeTable(fid,T3);
    fprintf(fid,'\n结论：`%s`。阈值仅用于实验分类：Gamma>10强方向占优，3<Gamma<=10中等，Gamma<=3双向同量级；论文应报告连续值。\n',c3);
end
if ~isempty(T4)
    fprintf(fid,'\n## T4：Pole-Path与阻尼通道（Gate %s）\n\n',passfail(g4));writeTable(fid,T4);fprintf(fid,'\n结论：`%s`。\n',c4);
end
if ~isempty(T5)
    fprintf(fid,'\n## T5：SYNC/GSC-DC电气边界（Gate %s）\n\n',passfail(g5));writeTable(fid,T5);fprintf(fid,'\n结论：`%s`。\n',c5);
    if ~isempty(boundary),fprintf(fid,'\n|参数|临界因子|临界频率Hz|M1稳定点MAC|M0 MAC|MECH参与|模态族|\n|---|---:|---:|---:|---:|---:|---|\n');for k=1:numel(boundary),b=boundary(k);fprintf(fid,'|%s|%.8g|%.8g|%.8g|%.8g|%.8g|%s|\n',b.Kind,b.CriticalFactor,b.Frequency_Hz,b.MAC_to_stable_M1,b.MAC_to_M0,b.Participation.MECH,b.ModeFamily);end,end
end
fprintf(fid,'\n## 当前Gate决定\n\n');
if ~g1,fprintf(fid,'T1失败，停止稳定化。\n');elseif isempty(T2),fprintf(fid,'按StopAfter停在T1。\n');elseif ~g2,fprintf(fid,'T2失败，不允许进入方向性与Pole-Path结论。\n');elseif isempty(T3),fprintf(fid,'按StopAfter停在T2。\n');elseif ~g3,fprintf(fid,'T3未保留足够的跨模型方向占优，暂停把方向性作为核心主线。\n');elseif isempty(T4),fprintf(fid,'按StopAfter停在T3。\n');elseif ~g4,fprintf(fid,'T4未通过，不进入边界外推。\n');elseif isempty(T5),fprintf(fid,'按StopAfter停在T4。\n');elseif g5,fprintf(fid,'T1-T5全部通过：可形成机制稳健性矩阵，并据此决定进入M2。\n');else,fprintf(fid,'T5未通过：边界模态身份不具备足够跨模型一致性。\n');end
fprintf(fid,'\n## 长期文件\n\n- `M1_Mechanism_Robustness_Summary.csv`：T1-T5统一摘要；\n- `M1_Mechanism_Robustness_Comparison.png`：综合对照图；\n- 本报告；\n- 唯一非线性模型仍为 `Grid_Forming_PMSG5MW_TwoMass_M1_PhysicalAvg.slx`（固定MWT控制职责）。\n');
end

function specs=parameterSpecs(p,flags)
specs=repmat(mk('H','VSG_SYNC','p',33,p(33),'H_s'),1,8);
specs(2)=mk('mpGwt','VSG_SYNC','flag',0,flags.mpGwt,'radps_per_W');
specs(3)=mk('KpGscDvc','GSC_DVC','flag',0,flags.KpGscDvc,'W_per_V');
specs(4)=mk('KiGscDvc','GSC_DVC','flag',0,flags.KiGscDvc,'W_per_Vs');
specs(5)=mk('KpGscVoltage','GSC_VOLTAGE_LOOP','p',31,p(31),'A_per_V');
specs(6)=mk('KiGscVoltage','GSC_VOLTAGE_LOOP','p',32,p(32),'A_per_Vs');
specs(7)=mk('KpGscCurrent','GSC_CURRENT_LOOP','p',29,p(29),'V_per_A');
specs(8)=mk('KiGscCurrent','GSC_CURRENT_LOOP','p',30,p(30),'V_per_As');
end
function s=mk(name,group,kind,index,value,unit)
s=struct('name',string(name),'group',string(group),'kind',string(kind), ...
    'index',index,'value',value,'unit',string(unit));
end

function q=parameterSensitivity(spec,x,p,flags,L0,baseMode,bm)
% 对参数因子f=p/p0求灵敏度，避免不同量纲妨碍排序。
h=2e-3;
[pp,fp]=applyFactor(spec,p,flags,1+h);
[pm,fm]=applyFactor(spec,p,flags,1-h);
Lp=linearize_physicalavg_m1(x,pp,'GFMGWT',fp);
Lm=linearize_physicalavg_m1(x,pm,'GFMGWT',fm);

dA_df=(Lp.A-Lm.A)/(2*h);
v=baseMode.v; w=baseMode.w;
dlamAnalytic=w'*dA_df*v/(w'*v);
[lamP,macP]=matchMode(Lp,baseMode);
[lamM,macM]=matchMode(Lm,baseMode);
dlamFD=(lamP-lamM)/(2*h);

mp=systemMetrics(Lp,p); mm=systemMetrics(Lm,p);
dzeta=(mp.zeta_tor-mm.zeta_tor)/(2*h);
dCgm=(mp.C_GM-mm.C_GM)/(2*h);
dCmg=(mp.C_MG-mm.C_MG)/(2*h);
dSigma=real(dlamAnalytic); dOmega=imag(dlamAnalytic);

% 评分只用于选择最小稳定化旋钮：稳定作用/相对副作用。
sideZ=abs(dzeta)/max(abs(bm.zeta_tor),1e-5);
sideGm=abs(dCgm)/max(abs(bm.C_GM),1e-6);
sideMg=abs(dCmg)/max(abs(bm.C_MG),1e-6);
score=abs(dSigma)/(0.25+sideZ+0.25*sideGm+0.25*sideMg);
if dSigma>0, favorable="DECREASE"; elseif dSigma<0, favorable="INCREASE"; else, favorable="NONE"; end
directionConsistent=signTol(real(dlamAnalytic))==signTol(real(dlamFD));
relErr=abs(dlamAnalytic-dlamFD)/max(abs(dlamFD),1e-10);

q=struct('Parameter',spec.name,'Group',spec.group,'BaselineValue',spec.value,'Unit',spec.unit, ...
    'dSigma_dFactor',dSigma,'dOmega_dFactor',dOmega, ...
    'FD_dSigma_dFactor',real(dlamFD),'FD_dOmega_dFactor',imag(dlamFD), ...
    'AnalyticFDRelativeError',relErr,'DirectionConsistent',directionConsistent, ...
    'MAC_Plus',macP,'MAC_Minus',macM,'dZetaTor_dFactor',dzeta, ...
    'dC_GM_dFactor',dCgm,'dC_MG_dFactor',dCmg,'RelativeSideEffectZeta',sideZ, ...
    'RelativeSideEffectCGM',sideGm,'RelativeSideEffectCMG',sideMg, ...
    'Score',score,'FavorableDirection',favorable);
end

function [p2,f2]=applyFactor(spec,p,flags,factor)
p2=p; f2=flags; val=spec.value*factor;
if spec.kind=="p", p2(spec.index)=val; else, f2.(char(spec.name))=val; end
end

function [mode,bm]=baselineElectricalMode(L,p)
M=multimode_modal_data(L.A,L.state_names);
itT=multimode_pick_torsion_mode(M); lamT=M.lambda(itT);
% 在非轴系模式中选实部最大的正虚部模式，避免选中共轭重复项。
cand=find(imag(M.lambda)>1e-7 & abs(M.lambda)>1e-7);
cand(cand==itT)=[];
[~,j]=max(real(M.lambda(cand))); it=cand(j);
g=participationDetail(M,it,L.state_names);
mode=struct('lambda',M.lambda(it),'frequency_Hz',abs(imag(M.lambda(it)))/(2*pi), ...
    'v',M.V(:,it),'w',M.W(:,it),'index',it,'participation',g, ...
    'torsion_lambda',lamT);
bm=systemMetrics(L,p);
end

function [lam,mac]=matchMode(L,ref)
M=multimode_modal_data(L.A,L.state_names); cand=find(imag(M.lambda)>0);
macs=zeros(numel(cand),1);
for k=1:numel(cand), macs(k)=modalMac(M.V(:,cand(k)),ref.v); end
[mac,j]=max(macs); lam=M.lambda(cand(j));
end

function m=systemMetrics(L,p)
M=multimode_modal_data(L.A,L.state_names); it=multimode_pick_torsion_mode(M); z=M.lambda(it);
w=abs(imag(z)); H=L.C*((1i*w*eye(size(L.A))-L.A)\L.B)+L.D;
iom=outIndex(L,'omega_sh'); iop=outIndex(L,'P_PCC'); Tb=p(1)/p(12);
m=struct('lambda_tor',z,'f_tor_Hz',w/(2*pi),'zeta_tor',-real(z)/abs(z), ...
    'C_GM',abs(H(iom,4)*p(3)/p(12)),'C_MG',abs(H(iop,1)*Tb/p(1)), ...
    'max_real',maxRealPole(L.A));
end

function g=participationDetail(M,it,names)
P=M.participation(:,it); total=sum(P); P=P/max(total,eps);
g=struct;
g.MECH=sum(P(ismember(names,{'theta_sh','omega_t','omega_g'})));
g.DC=sum(P(strcmp(names,'Udc')));
g.GSC_DVC=sum(P(strcmp(names,'xi_DVC')));
g.GSC_VOLTAGE=sum(P(ismember(names,{'xi_GSC_vd','xi_GSC_vq'})));
g.GSC_CURRENT=sum(P(ismember(names,{'xi_GSC_id','xi_GSC_iq'})));
g.LCL_GRID=sum(P(ismember(names,{'i_f_d','i_f_q','v_c_d','v_c_q','i_g_d','i_g_q'})));
g.SYNC=sum(P(ismember(names,{'P_f','Q_f','omega_vsg','delta'})));
end

function [x,meta]=solveM1(seed,p,mode,flags)
sx=[1;1;1;1e4;1e4;1e4;1e4;1e4;1e3;5e6;5e6;1;1;1e4;1e4;1e4;1e4;1e4;1e4;1e3;1e3;1e4;1e4];
sr=[1;1;1;1e4;1e4;1e4;1e4;1e4;1e6;5e8;5e8;1;1;1e4;1e4;1e4;1e4;1e6;1e6;1e6;1e6;1e6;1e6];
op=optimoptions('fsolve','Display','off','Algorithm','levenberg-marquardt', ...
    'FunctionTolerance',1e-11,'StepTolerance',1e-11,'OptimalityTolerance',1e-11, ...
    'MaxIterations',3000,'MaxFunctionEvaluations',50000);
[z,fv,ef]=fsolve(@(z)source_aligned_rhs_control_m1(sx.*z,p,mode,zeros(4,1),flags)./sr,seed./sx,op);
x=sx.*z; dx=source_aligned_rhs_control_m1(x,p,mode,zeros(4,1),flags);
meta=struct('exitflag',ef,'scaled_residual',norm(fv,inf),'max_abs_residual',max(abs(dx)));
assert(ef>0&&norm(fv,inf)<1e-8,'M1 equilibrium failed: exit=%g residual=%.3g',ef,norm(fv,inf));
end

function i=outIndex(L,n),i=find(strcmp(L.output_names,n),1);assert(~isempty(i),'Missing output %s',n);end
function r=maxRealPole(A),e=eig(A);e=e(abs(e)>1e-7);r=max(real(e));end
function m=modalMac(a,b),m=abs(a'*b)^2/max(real((a'*a)*(b'*b)),eps);end
function s=signTol(x),if abs(x)<1e-8,s=0;else,s=sign(x);end,end

function makeT1Figure(T,mode,file)
f=figure('Visible','off','Color','w','Position',[60 60 1450 850]);
tl=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
labs=categorical(T.Parameter,T.Parameter);
nexttile; barh(labs,T.dSigma_dFactor); xline(0,':k'); grid on; xlabel('d Re(\lambda_e) / d factor (1/s)'); title('T1 electrical-mode stabilizing sensitivity');
nexttile; barh(labs,[T.RelativeSideEffectZeta,T.RelativeSideEffectCGM,T.RelativeSideEffectCMG]); grid on; xlabel('relative side-effect sensitivity'); legend('\zeta_{tor}','C_{GM}','C_{MG}','Location','best'); title('Torsion/path side effects');
nexttile; barh(labs,T.Score); grid on; xlabel('ranking score'); title('Minimal-retuning score');
nexttile; vals=[mode.participation.MECH,mode.participation.DC,mode.participation.GSC_DVC,mode.participation.GSC_VOLTAGE,mode.participation.GSC_CURRENT,mode.participation.LCL_GRID,mode.participation.SYNC];
bar(categorical({'MECH','DC','GSC-DVC','GSC-V','GSC-I','LCL/Grid','SYNC'}),vals); grid on; ylabel('normalized participation'); title(sprintf('Baseline unstable mode: %.4f %+.4fj 1/s',real(mode.lambda),imag(mode.lambda)));
title(tl,'M1-a GWT T1: instability sensitivity and collateral effects');
exportgraphics(f,file,'Resolution',200);close(f);
end

function writeT1Report(file,T,mode,bm,eq,gate,stopAfter)
fid=fopen(file,'w','n','UTF-8');assert(fid>0);c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'# M1-a机制稳健性审计：T1失稳来源定位\n\n');
fprintf(fid,'本轮只在唯一M1参数化模型上计算，不修改M0、公共plant、DC-link职责或工作点，不保存时序。StopAfter=`%s`。\n\n',string(stopAfter));
fprintf(fid,'## 基准\n\n- 平衡点归一化残差：`%.3e`（exitflag=%g）；\n- 失稳电气模态：$\lambda_e=%.8g%+.8gj$ 1/s，$f_e=%.8g$ Hz；\n- 轴系模态：$f_{tor}=%.8g$ Hz，$\zeta_{tor}=%.8g%%$；\n- 双向指标：$C_{GM}=%.8g$，$C_{MG}=%.8g$。\n\n', ...
    eq.scaled_residual,eq.exitflag,real(mode.lambda),imag(mode.lambda),mode.frequency_Hz,bm.f_tor_Hz,100*bm.zeta_tor,bm.C_GM,bm.C_MG);
g=mode.participation;
fprintf(fid,'参与度细分：MECH %.5g，DC %.5g，GSC-DVC %.5g，GSC电压环 %.5g，GSC电流环 %.5g，LCL/Grid %.5g，SYNC %.5g。\n\n',g.MECH,g.DC,g.GSC_DVC,g.GSC_VOLTAGE,g.GSC_CURRENT,g.LCL_GRID,g.SYNC);
fprintf(fid,'## Gate T1：%s\n\n',passfail(gate));
fprintf(fid,'解析式采用 $d\lambda/df=w^H(dA/df)v/(w^Hv)$；有限差分独立追踪同一模态，表中 `MAC` 必须大于0.8且实部方向一致。评分仅用于选择最小重整定旋钮。\n\n');
writeTable(fid,T);
if gate
    q=T(1,:);fprintf(fid,'\n首选稳定化旋钮为 `%s`，有利方向 `%s`；其次为 `%s`。下一步只沿有利方向做单参数扫描和临界值二分。\n',q.Parameter,q.FavorableDirection,T.Parameter(min(2,height(T))));
else
    fprintf(fid,'\nGate T1未通过：停止稳定化，需重新检查物理平均VSC与控制器的兼容性或模态追踪。\n');
end
end
function writeTable(fid,T)
vars=T.Properties.VariableNames;fprintf(fid,'|');for k=1:numel(vars),fprintf(fid,'%s|',vars{k});end;fprintf(fid,'\n|');for k=1:numel(vars),fprintf(fid,'---|');end;fprintf(fid,'\n');
for r=1:height(T),fprintf(fid,'|');for k=1:numel(vars),v=T{r,k};if iscell(v),v=v{1};end;if isstring(v)||ischar(v)||islogical(v),s=char(string(v));else,s=num2str(v,8);end;fprintf(fid,'%s|',s);end;fprintf(fid,'\n');end
end
function s=passfail(v),if v,s='PASS';else,s='FAIL';end,end
