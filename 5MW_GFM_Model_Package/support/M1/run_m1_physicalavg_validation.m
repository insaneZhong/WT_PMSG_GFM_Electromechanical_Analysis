function R=run_m1_physicalavg_validation()
%RUN_M1_PHYSICALAVG_VALIDATION M0/M1物理平均VSC的三阶段最小证据链。
% 只保存一个摘要CSV、一份中文报告和一张综合图；不保存时序或求解历史。
here=fileparts(mfilename('fullpath'));
m0Dir=fullfile(fileparts(here),'CurrentModel_Idealized');
liuDir=fullfile(fileparts(here),'Liu_Negative_Damping_Audit');
addpath(here,m0Dir,liuDir);
modelFile=fullfile(here,'Grid_Forming_PMSG5MW_TwoMass_M1_PhysicalAvg.slx');
if ~isfile(modelFile), build_m1_physical_average_model('RunShortSimulation',true); end
[models0,base]=prepare_multimode_models(); p=base.parameter_vector(:).';

cases=struct([]);
for k=1:numel(models0)
    cases(k).mode=models0{k}.mode; cases(k).label=models0{k}.label;
    cases(k).flags=models0{k}.flags; cases(k).xSeed=models0{k}.x0; cases(k).L0=models0{k};
end

% ---------- Gate M1：共同平衡点、Udc结构通道与方向性 ----------
variants={'M0','M1-a Fixed normalization','M1-b Realtime feedforward'};
dcModes={'M0','FIXED_NORM','REALTIME_FF'};
P=table(); modelCache=cell(numel(cases),numel(variants));
for a=1:numel(cases)
    for v=1:numel(variants)
        flags=cases(a).flags; flags.vscDcMode=dcModes{v};
        if v==1
            x=cases(a).xSeed; L=cases(a).L0;
            residual=scaledResidualM0(x,p,cases(a).mode,cases(a).flags);
            eqExit=1;
        else
            [x,eq]=solveM1(cases(a).xSeed,p,cases(a).mode,flags); residual=eq.scaled_residual; eqExit=eq.exitflag;
            L=linearize_physicalavg_m1(x,p,cases(a).mode,flags); L.label=cases(a).label;
        end
        modelCache{a,v}=L;
        mm=modalMetrics(L); w=2*pi*mm.f;
        Gu=prescribedStateFrf(L,9,w); iPg=outIndex(L,'P_GSC'); iPp=outIndex(L,'P_PCC'); iIq=outIndex(L,'iq_MSC_ref');
        Ib=p(1)/(1.5*p(40));
        nPg=abs(Gu(iPg))*p(2)/p(1); nPp=abs(Gu(iPp))*p(2)/p(1); nIq=abs(Gu(iIq))*p(2)/Ib;
        Hio=frfMatrix(L,w); iom=outIndex(L,'omega_sh'); iop=outIndex(L,'P_PCC'); Tb=p(1)/p(12);
        cgm=abs(Hio(iom,4)*p(3)/p(12)); cmg=abs(Hio(iop,1)*Tb/p(1));
        dirIndex=(cgm-cmg)/(cgm+cmg+eps); gamma=max(cgm,cmg)/max(min(cgm,cmg),1e-15);
        dIq=analyticIqUdc(cases(a).mode,cases(a).flags,p);
        dPg=analyticPgscUdc(x,p,cases(a).mode,flags,dcModes{v});
        row=table("PATH",string(variants{v}),string(cases(a).label),string(cases(a).mode),mm.f,mm.zeta,real(mm.lambda),maxStableReal(L),residual,eqExit, ...
            real(Gu(iPg)),imag(Gu(iPg)),nPg,real(Gu(iPp)),imag(Gu(iPp)),nPp,real(Gu(iIq)),imag(Gu(iIq)),nIq,dIq,dPg, ...
            cgm,cmg,dirIndex,gamma,string(directionLabel(cgm,cmg)),string(classifyNormalized(nPp,dPg)), ...
            NaN,NaN,NaN,NaN,"","",NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,"", ...
            'VariableNames',summaryNames());
        P=[P;row]; %#ok<AGROW>
    end
end

% M1-b必须严格回归M0；M1-a必须存在物理Udc->VSC直接偏导。
regErr=max(cellfun(@(x,y)norm(x.A-y.A,'fro')/max(norm(x.A,'fro'),eps),modelCache(:,1),modelCache(:,3)));
eqPass=all(P.MaxNormalizedResidual(P.RecordType=="PATH")<1e-8);
stablePass=all(P.MaxRealPole(P.RecordType=="PATH")<1e-7);
actuationPass=all(P.Analytic_dPGSC_dUdc(P.ModelVariant=="M1-a Fixed normalization")~=0);
% M1-b理论上与M0相同；数值Jacobian与独立平衡点求解允许1e-8相对误差。
regPass=regErr<1e-8;

% 对MWT与GWT分别判断严格零、方向占优或同量级；GFL只作参考。
gateCases=table();
for key=["GFM-MWT","GFM-GWT"]
    ix=P.ModelVariant=="M1-a Fixed normalization" & contains(P.Architecture,key);
    if ~any(ix), continue; end
    q=P(ix,:); rev=min(q.C_GridToMachine,q.C_MachineToGrid);
    if rev<1e-10, outcome="A: reverse path remains near zero";
    elseif q.GammaDirection>=10, outcome="B: reverse path opens but directional dominance remains";
    elseif q.GammaDirection<3, outcome="C: forward and reverse paths are comparable";
    else, outcome="B-weak: moderate directional dominance"; end
    gateCases=[gateCases;table(string(key),q.C_GridToMachine,q.C_MachineToGrid,q.GammaDirection,string(outcome), ...
        'VariableNames',{'Architecture','C_GM','C_MG','GammaDirection','Outcome'})]; %#ok<AGROW>
end
stopForArtifact=any(startsWith(gateCases.Outcome,"C:"));
gateM1=eqPass&&actuationPass&&regPass;
stopFurther=stopForArtifact||~stablePass;

% 即使Gate条件性停止，也记录M1-a新出现的不稳定模态归属。
for a=1:numel(cases)
    L=modelCache{a,2}; M=multimode_modal_data(L.A,L.state_names); e=M.lambda; e(abs(e)<1e-7)=-Inf; [~,it]=max(real(e));
    if real(M.lambda(it))>0
        g=participationGroups(M,it,L.state_names,L.flags); row=blankSummaryRow();
        row.RecordType="BASELINE_UNSTABLE_MODE"; row.ModelVariant="M1-a Fixed normalization"; row.Architecture=string(cases(a).label); row.ControlMode=string(cases(a).mode);
        row.MaxRealPole=real(M.lambda(it)); row.ElectricalPoleReal=real(M.lambda(it)); row.ElectricalFreq_Hz=abs(imag(M.lambda(it)))/(2*pi);
        row.Pi_MECH=g.MECH; row.Pi_DC=g.DC; row.Pi_GSC_DVC=g.GSC_DVC; row.Pi_GSC=g.GSC; row.Pi_SYNC=g.SYNC;
        if g.MECH<.3&&g.GSC>.5,row.MechanismClass="GSC-SYNC electrical mode";elseif g.MECH<.3&&(g.SYNC+g.DC+g.GSC_DVC)>.5,row.MechanismClass="SYNC-DC electrical mode";else,row.MechanismClass="other electrical mode";end
        P=[P;row]; %#ok<AGROW>
    end
end

% ---------- Gate M2：代表架构Pole/Path/Residue跨模型复核 ----------
if gateM1 && ~stopFurther
    for a=1:numel(cases)
        for v=[1 2]
            L=modelCache{a,v}; mm=modalMetrics(L); w=2*pi*mm.f; Gs=prescribedStateFrf(L,3,w); gt=Gs(outIndex(L,'T_e'));
            M=multimode_modal_data(L.A,L.state_names); it=multimode_pick_torsion_mode(M);
            rGrid=modalResidue(L,M,it,outIndex(L,'omega_sh'),4)*p(3)/p(12);
            rMech=modalResidue(L,M,it,outIndex(L,'omega_sh'),1)*(p(1)/p(12))/p(12);
            modalSet=dominantModalSet(L,4,5);
            pathRow=P(P.RecordType=="PATH" & P.ModelVariant==string(variants{v}) & P.Architecture==string(cases(a).label),:);
            row=blankSummaryRow(); row.RecordType="ROBUSTNESS"; row.ModelVariant=string(variants{v}); row.Architecture=string(cases(a).label); row.ControlMode=string(cases(a).mode);
            row.f_tor_Hz=mm.f; row.zeta_tor=mm.zeta; row.TorsionPoleReal=real(mm.lambda); row.MaxRealPole=maxStableReal(L); row.De_at_ftor=real(gt); row.Ke_at_ftor=-w*imag(gt);
            row.ResidueGridToShaft=abs(rGrid); row.ResidueMechToShaft=abs(rMech); row.DominantModalSet=string(modalSet);
            row.C_GridToMachine=pathRow.C_GridToMachine; row.C_MachineToGrid=pathRow.C_MachineToGrid; row.GammaDirection=pathRow.GammaDirection; row.Direction=pathRow.Direction;
            P=[P;row]; %#ok<AGROW>
        end
    end
    robust=mechanismRobustness(P);
else
    robust=table("Stage M2",false,false,false,false,"Gate M1 conditionally stopped: M1-a is unstable or a reverse path became comparable", ...
        'VariableNames',{'Mechanism','Robust','PoleRobust','PathOrderRobust','ModalSetRobust','Reason'});
end

% ---------- Gate M3：只在旧临界值附近追踪非轴系SYNC-DC模态 ----------
boundary=table(); gateM3=false;
if gateM1 && ~stopFurther
    g=find(contains(string({cases.label}),"GFM-GWT"),1); assert(~isempty(g),'Missing GFM-GWT case.');
    flags=cases(g).flags; flags.vscDcMode='FIXED_NORM'; flags.mpptLocal=true; flags.Kmppt_iq_per_radps=2*cases(g).xSeed(5)/p(12);
    x=modelCache{g,2}.x0;
    [BH,critH]=localBoundary('H',x,p,'GFMGWT',flags,4.97021484375);
    [BD,critD]=localBoundary('DVC',x,p,'GFMGWT',flags,1.64208984375);
    boundary=[BH;BD];
    refH=criticalModeAt('H',cases(g).xSeed,p,'GFMGWT',rmfieldSafe(flags,'vscDcMode'),4.97021484375,false);
    refD=criticalModeAt('DVC',cases(g).xSeed,p,'GFMGWT',rmfieldSafe(flags,'vscDcMode'),1.64208984375,false);
    macH=modalMac(critH.vec,refH.vec); macD=modalMac(critD.vec,refD.vec);
    critRows={critH,critD}; macs=[macH,macD];
    for k=1:2
        c=critRows{k}; row=blankSummaryRow(); row.RecordType="BOUNDARY_CRITICAL"; row.ModelVariant="M1-a Fixed normalization"; row.Architecture="GFM-GWT-MPPT"; row.ControlMode="GFMGWT";
        row.BoundaryParameter=string(c.kind); row.BoundaryValue=c.value; row.ElectricalPoleReal=real(c.lambda); row.ElectricalFreq_Hz=abs(imag(c.lambda))/(2*pi); row.f_tor_Hz=c.ftor; row.zeta_tor=c.zetaTor;
        row.Pi_MECH=c.part.MECH; row.Pi_DC=c.part.DC; row.Pi_GSC_DVC=c.part.GSC_DVC; row.Pi_GSC=c.part.GSC; row.Pi_SYNC=c.part.SYNC; row.MAC_to_M0=macs(k); row.MechanismClass=string(boundaryClass(c));
        P=[P;row]; %#ok<AGROW>
    end
    for r=1:height(boundary)
        row=blankSummaryRow(); row.RecordType="BOUNDARY_TRACE"; row.ModelVariant="M1-a Fixed normalization"; row.Architecture="GFM-GWT-MPPT"; row.ControlMode="GFMGWT";
        row.BoundaryParameter=boundary.Parameter(r); row.BoundaryValue=boundary.Value(r); row.ElectricalPoleReal=boundary.ElectricalPoleReal(r); row.ElectricalFreq_Hz=boundary.ElectricalFreq_Hz(r); row.f_tor_Hz=boundary.f_tor_Hz(r); row.zeta_tor=boundary.zeta_tor(r);
        P=[P;row]; %#ok<AGROW>
    end
    gateM3=critH.pass&&critD.pass&&critH.zetaTor>0&&critD.zetaTor>0&&critH.part.MECH<0.3&&critD.part.MECH<0.3;
end

writetable(P,fullfile(here,'M1_PhysicalAvg_Summary.csv'),'Encoding','UTF-8');
makeFigure(P,boundary,fullfile(here,'M0_M1_PhysicalAvg_Comparison.png'));
R=struct('summary',P,'gate_cases',gateCases,'robustness',robust,'boundary',boundary,'gates', ...
    struct('M1_equilibrium',eqPass,'M1_stable',stablePass,'M1b_regression',regPass,'M1_physical_actuation',actuationPass,'M1_overall',gateM1,'stop_for_artifact',stopForArtifact,'stop_further',stopFurther,'M3_mechanism',gateM3), ...
    'm1b_matrix_relative_error',regErr,'model_file',modelFile);
writeReport(R,fullfile(here,'M1_PhysicalAvg_Report_CN.md'));
fprintf('M1 validation complete. Gate M1=%d, stopForArtifact=%d, Gate M3=%d\n',gateM1,stopForArtifact,gateM3);
end

function names=summaryNames()
names={'RecordType','ModelVariant','Architecture','ControlMode','f_tor_Hz','zeta_tor','TorsionPoleReal','MaxRealPole','MaxNormalizedResidual','EquilibriumExitflag', ...
 'G_PGSC_Udc_Re','G_PGSC_Udc_Im','G_PGSC_Udc_Norm','G_PPCC_Udc_Re','G_PPCC_Udc_Im','G_PPCC_Udc_Norm','G_iqref_Udc_Re','G_iqref_Udc_Im','G_iqref_Udc_Norm','Analytic_diqref_dUdc','Analytic_dPGSC_dUdc', ...
 'C_GridToMachine','C_MachineToGrid','DirectionIndex','GammaDirection','Direction','UdcPccClassification','De_at_ftor','Ke_at_ftor','ResidueGridToShaft','ResidueMechToShaft','DominantModalSet','BoundaryParameter','BoundaryValue','ElectricalPoleReal','ElectricalFreq_Hz','Pi_MECH','Pi_DC','Pi_GSC_DVC','Pi_GSC','Pi_SYNC','MAC_to_M0','MechanismClass'};
end
function r=blankSummaryRow()
t=cell(1,numel(summaryNames())); n=summaryNames();
for k=1:numel(n), if ismember(n{k},{'RecordType','ModelVariant','Architecture','ControlMode','Direction','UdcPccClassification','DominantModalSet','BoundaryParameter','MechanismClass'}),t{k}="";else,t{k}=NaN;end,end
r=cell2table(t,'VariableNames',n);
end
function [x,meta]=solveM1(seed,p,mode,flags)
sx=[1;1;1;1e4;1e4;1e4;1e4;1e4;1e3;5e6;5e6;1;1;1e4;1e4;1e4;1e4;1e4;1e4;1e3;1e3;1e4;1e4];
sr=[1;1;1;1e4;1e4;1e4;1e4;1e4;1e6;5e8;5e8;1;1;1e4;1e4;1e4;1e4;1e6;1e6;1e6;1e6;1e6;1e6];
op=optimoptions('fsolve','Display','off','Algorithm','levenberg-marquardt','FunctionTolerance',1e-11,'StepTolerance',1e-11,'OptimalityTolerance',1e-11,'MaxIterations',3000,'MaxFunctionEvaluations',50000);
if strcmpi(mode,'GFL')
    active=[1:11 14:23]; z0=seed(active)./sx(active); [z,fv,ef]=fsolve(@red,z0,op); x=seed; x(active)=sx(active).*z;
else
    [z,fv,ef]=fsolve(@(z)source_aligned_rhs_control_m1(sx.*z,p,mode,zeros(4,1),flags)./sr,seed./sx,op); x=sx.*z;
end
dx=source_aligned_rhs_control_m1(x,p,mode,zeros(4,1),flags); meta=struct('exitflag',ef,'residual',max(abs(dx)),'scaled_residual',norm(fv,inf));
assert(ef>0&&norm(fv,inf)<1e-8,'M1 equilibrium failed: %s, exit=%g, scaled residual=%.3g',mode,ef,norm(fv,inf));
    function rr=red(z),xx=seed;xx(active)=sx(active).*z;dd=source_aligned_rhs_control_m1(xx,p,mode,zeros(4,1),flags);rr=dd(active)./sr(active);end
end
function r=scaledResidualM0(x,p,mode,flags)
sr=[1;1;1;1e4;1e4;1e4;1e4;1e4;1e6;5e8;5e8;1;1;1e4;1e4;1e4;1e4;1e6;1e6;1e6;1e6;1e6;1e6];
dx=source_aligned_rhs_control(x,p,mode,zeros(4,1),flags);
if strcmpi(mode,'GFL'),active=[1:11 14:23];r=norm(dx(active)./sr(active),inf);else,r=norm(dx./sr,inf);end
end
function G=prescribedStateFrf(L,idx,w), ir=setdiff(1:size(L.A,1),idx); G=L.C(:,ir)*((1i*w*eye(numel(ir))-L.A(ir,ir))\L.A(ir,idx))+L.C(:,idx); end
function H=frfMatrix(L,w),H=L.C*((1i*w*eye(size(L.A))-L.A)\L.B)+L.D;end
function i=outIndex(L,n),i=find(strcmp(L.output_names,n),1);assert(~isempty(i),'Missing output %s',n);end
function m=modalMetrics(L),M=multimode_modal_data(L.A,L.state_names);it=multimode_pick_torsion_mode(M);z=M.lambda(it);m=struct('lambda',z,'f',abs(imag(z))/(2*pi),'zeta',-real(z)/abs(z));end
function x=maxStableReal(L),e=eig(L.A);e=e(abs(e)>1e-7);x=max(real(e));end
function d=analyticIqUdc(mode,flags,p),if strcmpi(mode,'GFMGWT')||(isfield(flags,'cutUdcToDvc')&&flags.cutUdcToDvc),d=0;else,d=-p(25);end,end
function d=analyticPgscUdc(x,p,mode,flags,kind),if strcmpi(kind,'FIXED_NORM'),flags.vscDcMode='REALTIME_FF';[y,~,~]=source_aligned_internal_outputs_control_m1(x,p,mode,zeros(4,1),flags);d=y(1)/p(2);else,d=0;end,end
function s=classifyNormalized(g,direct),if abs(direct)<1e-12&&g<1e-10,s='STRUCTURAL_ZERO';elseif g<1e-3,s='WEAK_COUPLING';elseif abs(direct)>0&&g<1e-2,s='OPERATING_POINT_CANCELLATION';else,s='NONZERO';end,end
function s=directionLabel(cgm,cmg),di=(cgm-cmg)/(cgm+cmg+eps);if max(cgm,cmg)<1e-10,s='BOTH_WEAK';elseif di>0.2,s='GRID_TO_MACHINE_DOMINANT';elseif di<-.2,s='MACHINE_TO_GRID_DOMINANT';else,s='BALANCED_BIDIRECTIONAL';end,end
function r=modalResidue(L,M,it,iy,iu),r=L.C(iy,:)*M.V(:,it)*(M.W(:,it)'*L.B(:,iu));end
function s=dominantModalSet(L,iu,nKeep)
M=multimode_modal_data(L.A,L.state_names); iy=outIndex(L,'omega_sh'); cand=find(imag(M.lambda)>=-1e-9 & abs(M.lambda)>1e-7); z=struct('mag',{},'name',{},'f',{});
for k=1:numel(cand),it=cand(k);z(k).mag=abs(modalResidue(L,M,it,iy,iu));z(k).name=classifyMode(M,it);z(k).f=abs(imag(M.lambda(it)))/(2*pi);end
[~,ix]=sort([z.mag],'descend');ix=ix(1:min(nKeep,numel(ix)));parts=cell(1,numel(ix));for k=1:numel(ix),parts{k}=sprintf('%s@%.3gHz',z(ix(k)).name,z(ix(k)).f);end;s=strjoin(parts,'+');
end
function s=classifyMode(M,it),P=M.participation(:,it);sc=[sum(P(1:3)),sum(P([6 9])),sum(P(10:13)),sum(P(14:23)),sum(P(2:3)),sum(P(4:8))];nm={'TOR','DC','SYNC','GSC','SPEED','MSC'};[~,q]=max(sc);s=nm{q};end
function T=mechanismRobustness(P)
T=table(); arch=unique(P.Architecture(P.RecordType=="ROBUSTNESS"),'stable');
for k=1:numel(arch),a=arch(k);m=P(P.RecordType=="ROBUSTNESS"&P.Architecture==a,:);m0=m(m.ModelVariant=="M0",:);m1=m(m.ModelVariant=="M1-a Fixed normalization",:);df=abs(m1.f_tor_Hz-m0.f_tor_Hz)/m0.f_tor_Hz;pole=df<0.02;path=strcmp(m1.Direction,m0.Direction)||m1.GammaDirection>=10;rs=contains(m1.DominantModalSet,"TOR")&&contains(m1.DominantModalSet,"SYNC");T=[T;table(a,pole&&path,pole,path,rs,sprintf('df=%.3g, gamma=%.3g',df,m1.GammaDirection),'VariableNames',{'Mechanism','Robust','PoleRobust','PathOrderRobust','ModalSetRobust','Reason'})];end
end
function [T,c]=localBoundary(kind,x,p,mode,flags,oldCrit)
lo=.8*oldCrit;hi=1.2*oldCrit;[flo,~]=marginAt(kind,lo,x,p,mode,flags);[fhi,~]=marginAt(kind,hi,x,p,mode,flags);
if flo*fhi>=0,lo=.6*oldCrit;hi=1.5*oldCrit;[flo,~]=marginAt(kind,lo,x,p,mode,flags);[fhi,~]=marginAt(kind,hi,x,p,mode,flags);end
assert(flo<0&&fhi>0,'M1 %s local boundary not bracketed: [%.3g,%.3g].',kind,flo,fhi);
vals=linspace(lo,hi,5);T=table();for v=vals,[f,m]=marginAt(kind,v,x,p,mode,flags);T=[T;boundaryRow(kind,v,f,m)];end %#ok<AGROW>
for k=1:28,mid=(lo+hi)/2;[fm,mm]=marginAt(kind,mid,x,p,mode,flags);if fm>0,hi=mid;else,lo=mid;end;if abs(fm)<1e-5||abs(hi-lo)<1e-5,break;end,end
cv=(lo+hi)/2;[fr,c]=marginAt(kind,cv,x,p,mode,flags);c.kind=kind;c.value=cv;c.pass=abs(fr)<2e-3;c.lambda=c.lambda;c.vec=c.vec;T=[T;boundaryRow(kind,cv,fr,c)];
end
function row=boundaryRow(kind,v,f,m),row=table(string(kind),v,f,abs(imag(m.lambda))/(2*pi),m.ftor,m.zetaTor,'VariableNames',{'Parameter','Value','ElectricalPoleReal','ElectricalFreq_Hz','f_tor_Hz','zeta_tor'});end
function [f,m]=marginAt(kind,val,x,p,mode,flags)
pp=p;ff=flags;if strcmpi(kind,'H'),pp(33)=val;else,ff.KpGscDvc=5e3*val;ff.KiGscDvc=5e2*val;end
L=linearize_physicalavg_m1(x,pp,mode,ff);M=multimode_modal_data(L.A,L.state_names);itT=multimode_pick_torsion_mode(M);it=pickElectrical(M,itT);m.lambda=M.lambda(it);m.vec=M.V(:,it);m.part=participationGroups(M,it,L.state_names,ff);m.ftor=abs(imag(M.lambda(itT)))/(2*pi);m.zetaTor=-real(M.lambda(itT))/abs(M.lambda(itT));f=real(m.lambda);
end
function c=criticalModeAt(kind,x,p,mode,flags,val,useM1),if strcmpi(kind,'H'),p(33)=val;else,flags.KpGscDvc=5e3*val;flags.KiGscDvc=5e2*val;end;if useM1,L=linearize_physicalavg_m1(x,p,mode,flags);else,L=multimode_linearize_control(x,p,mode,flags);end;M=multimode_modal_data(L.A,L.state_names);itT=multimode_pick_torsion_mode(M);it=pickElectrical(M,itT);c=struct('vec',M.V(:,it),'lambda',M.lambda(it));end
function it=pickElectrical(M,itT),e=M.lambda;ban=false(size(e));ban(itT)=true;[~,ic]=min(abs(e-conj(e(itT))));ban(ic)=true;score=real(e);score(ban|abs(e)<1e-7|abs(imag(e))/(2*pi)<.5)=-Inf;if all(~isfinite(score)),score=real(e);score(ban|abs(e)<1e-7)=-Inf;end;[~,it]=max(score);end
function g=participationGroups(M,it,names,flags),P=M.participation(:,it);g.MECH=sum(P(ismember(names,{'theta_sh','omega_t','omega_g'})));g.DC=sum(P(ismember(names,{'Udc'})));g.GSC_DVC=sum(P(ismember(names,{'xi_DVC'})));g.GSC=sum(P(ismember(names,{'xi_GSC_vd','xi_GSC_vq','xi_GSC_id','xi_GSC_iq','i_f_d','i_f_q','v_c_d','v_c_q','i_g_d','i_g_q'})));g.SYNC=sum(P(ismember(names,{'P_f','Q_f','omega_vsg','delta'})));z=g.MECH+g.DC+g.GSC_DVC+g.GSC+g.SYNC;fn=fieldnames(g);for k=1:numel(fn),g.(fn{k})=g.(fn{k})/max(z,eps);end %#ok<INUSD>
end
function m=modalMac(a,b),m=abs(a'*b)^2/max((a'*a)*(b'*b),eps);end
function s=boundaryClass(c),if c.part.MECH<.3&&(c.part.SYNC+c.part.DC+c.part.GSC_DVC)>.5,s='SYNC-DC electrical mode';else,s='mechanism changed';end,end
function s=rmfieldSafe(s,n),if isfield(s,n),s=rmfield(s,n);end,end
function makeFigure(P,B,file)
f=figure('Visible','off','Color','w','Position',[80 80 1450 900]);tiledlayout(2,2,'TileSpacing','compact');
nexttile;ix=P.RecordType=="PATH"&contains(P.Architecture,"GFM-MWT");vv=max([P.G_PPCC_Udc_Norm(ix),P.G_PGSC_Udc_Norm(ix)],1e-10);bar(categorical({'M0','M1-a','M1-b'}),vv);set(gca,'YScale','log');ylim([1e-10 1e2]);grid on;ylabel('normalized magnitude');legend('|P_{PCC}/Udc|','|P_{GSC}/Udc|');title('MWT: DC-link reverse path');
nexttile;ix=P.RecordType=="PATH"&P.ModelVariant~="M1-b Realtime feedforward";labs=strings(sum(ix),1);qq=find(ix);for k=1:numel(qq),labs(k)=shortArch(P.Architecture(qq(k)))+"/"+iffText(P.ModelVariant(qq(k))=="M0","M0","M1-a");end;vals=max([P.C_GridToMachine(ix),P.C_MachineToGrid(ix)],1e-10);bar(categorical(labs,labs),vals);set(gca,'YScale','log');ylim([1e-10 1e3]);grid on;ylabel('normalized coupling');legend('Grid->Machine','Machine->Grid');title('Directional coupling');xtickangle(20);
nexttile;ix=P.RecordType=="PATH"&(P.ModelVariant=="M0"|P.ModelVariant=="M1-a Fixed normalization");hold on;aa=unique(P.Architecture(ix),'stable');mk={'o','s','^'};for k=1:numel(aa),q=ix&P.Architecture==aa(k);plot(P.TorsionPoleReal(q),P.f_tor_Hz(q),[mk{k} '-'],'LineWidth',1.3,'MarkerSize',7,'DisplayName',char(shortArch(aa(k))));end;xline(0,':k','HandleVisibility','off');grid on;xlabel('Re(lambda_{tor})');ylabel('f_{tor} (Hz)');legend('Location','best');title('Torsional poles: M0 -> M1-a');
nexttile;if ~isempty(B),for n=unique(B.Parameter,'stable')',q=B.Parameter==n;plot(B.ElectricalPoleReal(q),B.ElectricalFreq_Hz(q),'-o','LineWidth',1.4,'DisplayName',char(n));hold on;end;xline(0,':k','HandleVisibility','off');grid on;xlabel('Re(lambda_e)');ylabel('f_e (Hz)');legend('Location','best');title('M1-a local electrical boundary');else,u=P(P.RecordType=="BASELINE_UNSTABLE_MODE",:);if ~isempty(u),vals=[u.Pi_MECH,u.Pi_DC,u.Pi_GSC_DVC,u.Pi_GSC,u.Pi_SYNC];bar(1,vals,'stacked');ylim([0 1]);grid on;xticks(1);xticklabels(shortArch(u.Architecture));ylabel('participation ratio');legend('MECH','DC','GSC-DVC','GSC','SYNC','Location','eastoutside');title(sprintf('M1-a unstable mode: %.3f%+.3fj 1/s',u.ElectricalPoleReal(1),2*pi*u.ElectricalFreq_Hz(1)));else,text(.1,.5,'Stage M3 conditionally stopped');axis off;end;end
exportgraphics(f,file,'Resolution',200);close(f);
end
function s=shortArch(a),a=string(a);if contains(a,'GFM-MWT'),s="MWT";elseif contains(a,'GFM-GWT'),s="GWT";else,s="GFL";end,end
function s=iffText(c,a,b),if c,s=string(a);else,s=string(b);end,end
function writeReport(R,file)
P=R.summary;fid=fopen(file,'w','n','UTF-8');c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'# M1物理平均VSC跨模型验证报告\n\n');fprintf(fid,'M0保持不变；M1只有一个可切换模型。M1-a使用固定直流基准归一化，M1-b使用实时Udc前馈。未加入PWM、采样、延迟、限幅、LVRT或Pitch。\n\n');
fprintf(fid,'## Gate M1\n\n- 平衡点：%s\n- 全部代表架构稳定：%s\n- M1-b回归M0：%s（A矩阵最大相对误差 %.3e）\n- M1-a物理直接通道：%s\n\n',pass(R.gates.M1_equilibrium),pass(R.gates.M1_stable),pass(R.gates.M1b_regression),R.m1b_matrix_relative_error,pass(R.gates.M1_physical_actuation));
fprintf(fid,'|架构|C_GM|C_MG|Gamma_dir|结论|\n|---|---:|---:|---:|---|\n');for k=1:height(R.gate_cases),q=R.gate_cases(k,:);fprintf(fid,'|%s|%.5g|%.5g|%.5g|%s|\n',q.Architecture,q.C_GM,q.C_MG,q.GammaDirection,q.Outcome);end
fprintf(fid,'\n若M1-a把反向通道打开但Gamma_dir仍远大于1，原结论应改写为“方向占优”，不再宣称严格结构零。若两向同量级，本报告已停止后续强化。\n\n');
if R.gates.stop_further
    u=P(P.RecordType=="BASELINE_UNSTABLE_MODE",:); [~,j]=max(u.MaxRealPole); q=u(j,:);
    fprintf(fid,'**条件性停止：** M1-a至少一个架构不稳定或两向耦合已经同量级。最不稳定代表点为 `%s`，最大实部 %.6g 1/s，频率 %.6g Hz，归属 `%s`；MECH参与 %.4g，GSC参与 %.4g，SYNC/DC/GSC-DVC参与合计 %.4g。因此未在该未整定基准上继续计算M2/M3并声称跨模型稳健。\n\n',q.Architecture,q.MaxRealPole,q.ElectricalFreq_Hz,q.MechanismClass,q.Pi_MECH,q.Pi_GSC,q.Pi_SYNC+q.Pi_DC+q.Pi_GSC_DVC);
end
fprintf(fid,'## Gate M2：Pole–Path稳健性\n\n');if ~isempty(R.robustness),fprintf(fid,'|对象|Robust|Pole|Path|Modal set|说明|\n|---|---|---|---|---|---|\n');for k=1:height(R.robustness),q=R.robustness(k,:);fprintf(fid,'|%s|%s|%s|%s|%s|%s|\n',q.Mechanism,pass(q.Robust),pass(q.PoleRobust),pass(q.PathOrderRobust),pass(q.ModalSetRobust),q.Reason);end,end
fprintf(fid,'\n## Gate M3：SYNC–DC边界\n\n');if R.gates.M3_mechanism,ix=P.RecordType=="BOUNDARY_CRITICAL";for k=find(ix)',q=P(k,:);fprintf(fid,'- %s临界约 %.6g；临界电气模态 %.5g Hz；轴系阻尼 %.4g%%；MECH参与 %.4g；SYNC/DC/GSC-DVC参与合计 %.4g；与M0 MAC %.5g。\n',q.BoundaryParameter,q.BoundaryValue,q.ElectricalFreq_Hz,100*q.zeta_tor,q.Pi_MECH,q.Pi_SYNC+q.Pi_DC+q.Pi_GSC_DVC,q.MAC_to_M0);end;fprintf(fid,'\n结论：临界数值允许变化，但首先失稳的仍是非轴系SYNC–DC电气模态。\n');else,fprintf(fid,'未通过或因Gate M1情况C而停止。\n');end
fprintf(fid,'\n## 文件\n\n- `M1_PhysicalAvg_Summary.csv`：全部摘要；\n- `M0_M1_PhysicalAvg_Comparison.png`：唯一综合图；\n- `Grid_Forming_PMSG5MW_TwoMass_M1_PhysicalAvg.slx`：唯一M1模型。\n');
end
function s=pass(x),if x,s='PASS';else,s='FAIL';end,end
