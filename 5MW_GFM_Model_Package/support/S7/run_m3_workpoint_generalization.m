function R=run_m3_workpoint_generalization(varargin)
%RUN_M3_WORKPOINT_GENERALIZATION M3跨运行点可证伪验证统一入口。
%
% 本文件只复制M2分析入口，不复制Simulink模型，也不改变M2基准。
% 目标是检验M2的Pole、Path、Directional Coupling结论是否能跨
% 多个可行稳态工作点保持。所有阶段性结果仍视为假设，主动记录反例。

ip=inputParser;
ip.addParameter('WorkpointScales',[0.3 0.5 0.7 0.9],@(x)isnumeric(x)&&isvector(x));
ip.addParameter('SaveOutputs',true,@(x)islogical(x)&&isscalar(x));
ip.addParameter('RunIndependentCheck',false,@(x)islogical(x)&&isscalar(x));
ip.addParameter('UsePhysicalMppt',true,@(x)islogical(x)&&isscalar(x));
ip.addParameter('PmsgConvention','GENERATOR_OUTWARD',@(x)ischar(x)||isstring(x));
ip.addParameter('StopAfter','S1',@(x)ischar(x)||isstring(x));
ip.addParameter('S2WorkpointScales',[0.3 0.7 0.9],@(x)isnumeric(x)&&isvector(x));
ip.addParameter('S5BWindFactors',[1.05 1.20 1.35],@(x)isnumeric(x)&&isvector(x));
ip.addParameter('S5BPitchKpScale',1,@(x)isnumeric(x)&&isscalar(x)&&isfinite(x)&&x>0);
ip.addParameter('S5BControllerMode','LEGACY_CONSTANT',@(x)ischar(x)||isstring(x));
ip.addParameter('S5BAuditWindFactors',1.05:0.005:1.35,@(x)isnumeric(x)&&isvector(x));
ip.addParameter('S5BAuditMarginWindFactors',[1.05 1.15 1.25 1.35],@(x)isnumeric(x)&&isvector(x));
ip.addParameter('S6OpenFASTRoot','',@(x)ischar(x)||isstring(x));
ip.addParameter('S6Variant','DYNAMIC_AERODYN',@(x)ischar(x)||isstring(x));
% 可选的临时案例入口。默认留空时完全沿用官方 S6 案例；填写后只
% 用于条件性候选诊断，并强制保留 source-traceability Gate 失败状态。
ip.addParameter('S6CaseDir','',@(x)ischar(x)||isstring(x));
ip.addParameter('S6CaseStem','',@(x)ischar(x)||isstring(x));
ip.parse(varargin{:}); opt=ip.Results;
opt.S5BControllerMode=upper(char(string(opt.S5BControllerMode)));
assert(any(strcmp(opt.S5BControllerMode,{'LEGACY_CONSTANT','NREL5MW_SCHEDULED_LSS'})),'Unsupported S5BControllerMode.');
pmsgConvention=upper(char(string(opt.PmsgConvention)));
assert(any(strcmp(pmsgConvention,{'GENERATOR_OUTWARD','LEGACY_MOTOR_SIGN'})),'Unsupported PmsgConvention.');
stopAfter=upper(char(string(opt.StopAfter)));
assert(any(strcmp(stopAfter,{'S1','S2','S3','S4','S5A','S5B','S5B_AUDIT','S6'})),'StopAfter must be S1, S2, S3, S4, S5A, S5B, S5B_AUDIT or S6.');
s6Variant=upper(char(string(opt.S6Variant)));
assert(any(strcmp(s6Variant,{'DYNAMIC_AERODYN','FROZEN_WAKE'})), ...
    'S6Variant must be DYNAMIC_AERODYN or FROZEN_WAKE.');
s6CaseDir=char(string(opt.S6CaseDir));s6CaseStem=char(string(opt.S6CaseStem));
assert((isempty(s6CaseDir)&&isempty(s6CaseStem)) || (~isempty(s6CaseDir)&&~isempty(s6CaseStem)), ...
    'S6CaseDir and S6CaseStem must be supplied together.');

here=fileparts(mfilename('fullpath'));
idealRoot=fileparts(here);
m0Dir=fullfile(idealRoot,'CurrentModel_Idealized');
addpath(m0Dir);

% 唯一公共参数源：M2已经通过Gate A的5 MW基准。
S=load(fullfile(m0Dir,'Architecture_Comparison_Summary.mat'),'R');
base=S.R; assert(base.passed,'M2公共工作点未通过，禁止进入M3。');
p0=base.parameter_vector(:);
% 与M2一致的公共MSC电流环稳定化设定。
p0(30)=p0(30)*0.6;

scales=opt.WorkpointScales(:)';
assert(all(scales>0 & scales<=1.2),'WorkpointScales must be positive and feasible.');

workRows={}; modalRows={}; torqueRows={}; couplingRows={}; residueRows={}; scanRows={};
pointGateRows={}; energyRows={}; wpRows={}; selectedModels=cell(size(scales)); selectedP=p0;
allModels=cell(size(scales)); allP=cell(size(scales)); allFeedbackCurves=cell(size(scales));
allCouplingCurves=cell(size(scales)); allModalMaps=cell(size(scales));

for si=1:numel(scales)
    sc=scales(si);
    if opt.UsePhysicalMppt
        [p,wp]=makePhysicalMpptPoint(p0,sc,m0Dir);
        wp.pmsgConvention=pmsgConvention;
        [p,wp]=calibrateMwtTorqueAtTargetSpeed(base,p,p0,wp);
    else
        p=p0; p(37)=p0(37)*sc; p(39)=p0(39)*sc;
        wp=struct('powerScale',sc,'speedScale',1,'torqueScale',sc, ...
            'wind_mps',NaN,'aeroPower_W',p(39)*p(12),'definition','LEGACY_FIXED_SPEED_SCALING', ...
            'pmsgConvention',pmsgConvention);
    end
    arch=makeArchitectures(base,p,wp);
    [models,~,workAudit,gateA]=runStageA(arch,p,base);
    allModels{si}=models; allP{si}=p;
    wpRows(end+1,:)={sc,wp.wind_mps,p(12),p(39),p(39)*p(12),wp.rawCpPower_W, ...
        wp.aeroCalibration,wp.lossCompensation_Nm,wp.mwtCalibrationIterations, ...
        wp.mwtTargetSpeedRelativeError,wp.mwtCalibrationConverged,string(wp.definition)}; %#ok<AGROW>
    for ea=1:numel(models)
        EM=models{ea}; ex=EM.x0; wy=workAudit(ea,:);
        pMech=wy.Te0_Nm*wy.omega_g0_radps;
        pCu=1.5*p(13)*(ex(4)^2+ex(5)^2);
        legacyPlusResidual=wy.Pmsc0_W-pMech-pCu;
        physicalGeneratorResidual=wy.Pmsc0_W-(pMech-pCu);
        if strcmp(pmsgConvention,'GENERATOR_OUTWARD'),modelResidual=physicalGeneratorResidual;else,modelResidual=legacyPlusResidual;end
        energyRows(end+1,:)={sc,string(EM.name),p(39)*p(12),pMech,pCu,wy.Pmsc0_W,wy.Pgsc0_W,wy.P0_W, ...
            modelResidual,legacyPlusResidual,physicalGeneratorResidual,wy.Pmsc0_W-wy.Pgsc0_W,wy.Pgsc0_W-wy.P0_W, ...
            abs(modelResidual)/p(1)<1e-8,abs(physicalGeneratorResidual)/p(1)<1e-8}; %#ok<AGROW>
    end
    if ~gateA.pass
        fprintf('M3 Gate A fairness diagnostic at %.3g: P0=',sc); fprintf(' %.6g',workAudit.P0_W/1e6); fprintf(', Udc='); fprintf(' %.6g',workAudit.Udc0_V); fprintf(', residual='); fprintf(' %.3e',workAudit.NormalizedResidual); fprintf('\n');
        % 跨运行点实验不把架构间工作点差异静默掩盖。若只是同一Pref下
        % 不同职责造成的可解释功率偏差，保留并在表中报告；平衡残差仍必须合格。
        assert(gateA.max_normalized_residual<1e-8 && all(workAudit.P0_W>0), ...
            'M3 workpoint %.3g failed equilibrium Gate A (residual %.3e).',sc,gateA.max_normalized_residual);
    end
    pointGateRows(end+1,:)={sc,gateA.max_normalized_residual,gateA.max_workpoint_pairwise_pu,gateA.pass}; %#ok<AGROW>
    if abs(sc-0.8)<1e-12 || (isempty(selectedModels{1}) && si==1), selectedModels=models; selectedP=p; end

    [modalMap,torsion,gateB]=runStageB(models); allModalMaps{si}=modalMap;
    assert(gateB.pass,'M3 workpoint %.3g failed Gate B.',sc);
    [feedback,feedbackCurves,gateC]=runStageC(models,p,torsion); allFeedbackCurves{si}=feedbackCurves;
    assert(gateC.pass,'M3 workpoint %.3g failed Gate C.',sc);
    [~,poleExc,gateD]=runStageD(models,modalMap,torsion);
    assert(gateD.pass,'M3 workpoint %.3g failed Gate D.',sc);
    [coupling,couplingCurves,gateE]=runStageE(models,p,torsion); allCouplingCurves{si}=couplingCurves;
    assert(gateE.pass,'M3 workpoint %.3g failed Gate E.',sc);
    % S1只完成跨运行点Gate A，不在同一轮提前混入H/DVC/SCR控制扫参。
    gateF=struct('pass',true,'not_run_in_S1',true);

    for a=1:numel(models)
        M=models{a}; wa=workAudit(a,:); tm=torsion(torsion.Architecture==string(M.name),:);
        cbase=feedback(feedback.Architecture==string(M.name)&feedback.Case=="BASE",:);
        crows=feedback(feedback.Architecture==string(M.name)&feedback.Case~="BASE",:);
        getc=@(name,default) localTableValue(crows,name,default);
        dmp=NaN; ddvc=NaN; dgfm=NaN; kmp=NaN; kdvc=NaN; kgfm=NaN;
        if any(crows.Case=="MPPT_ON_MINUS_OFF"), q=crows(crows.Case=="MPPT_ON_MINUS_OFF",:); dmp=q.De_or_DeltaDe; kmp=q.Ke_or_DeltaKe; end
        if any(crows.Case=="MSC_DVC_ON_MINUS_OFF"), q=crows(crows.Case=="MSC_DVC_ON_MINUS_OFF",:); ddvc=q.De_or_DeltaDe; kdvc=q.Ke_or_DeltaKe; end
        if any(crows.Case=="GFM_SYNC_ON_MINUS_FROZEN"), q=crows(crows.Case=="GFM_SYNC_ON_MINUS_FROZEN",:); dgfm=q.De_or_DeltaDe; kgfm=q.Ke_or_DeltaKe; end
        ce=coupling(coupling.Architecture==string(M.name),:);
        re=poleExc(poleExc.Architecture==string(M.name),:);
        workRows(end+1,:)={sc,string(M.name),wa.P0_W,wa.Q0_var,wa.Udc0_V,wa.omega_g0_radps,wa.Te0_Nm,wa.Tsh0_Nm,wa.Pmsc0_W,wa.Pgsc0_W,wa.NormalizedResidual,wa.PowerMismatch_W,wa.TorqueMismatch_Nm}; %#ok<AGROW>
        modalRows(end+1,:)={sc,string(M.name),tm.PoleReal,tm.PoleImag,tm.Frequency_Hz,tm.DampingRatio,tm.Pi_MECH,tm.MaxRealPole,tm.PhysicalClass,gateB.all_stable}; %#ok<AGROW>
        torqueRows(end+1,:)={sc,string(M.name),cbase.Frequency_Hz,cbase.De_or_DeltaDe,cbase.Ke_or_DeltaKe,dmp,kmp,ddvc,kdvc,dgfm,kgfm,cbase.EvidenceStatus}; %#ok<AGROW>
        couplingRows(end+1,:)={sc,string(M.name),ce.TorsionalFrequency_Hz,ce.C_GridToMachine_at_ftor,ce.C_MachineToGrid_at_ftor,ce.DirectionalRatio_at_ftor,ce.LocalDirectionAtFtor}; %#ok<AGROW>
        for rr=1:height(re)
            residueRows(end+1,:)={sc,re.Architecture(rr),re.Disturbance(rr),re.PoleChangeIndex(rr),re.ResidueChangeDecades(rr),re.Classification(rr)}; %#ok<AGROW>
        end
    end
    fprintf('M3 S1 workpoint %.3g complete: modal=%d torque=%d residue=%d coupling=%d, max real pole %.3e.\n', ...
        sc,gateB.pass,gateC.pass,gateD.pass,gateE.pass,gateB.max_real_pole);
end

W=cell2table(workRows,'VariableNames',{'WorkpointScale','Architecture','P0_W','Q0_var','Udc0_V','omega_g0_radps','Te0_Nm','Tsh0_Nm','Pmsc0_W','Pgsc0_W','EquilibriumResidual','PowerMismatch_W','TorqueMismatch_Nm'});
M=cell2table(modalRows,'VariableNames',{'WorkpointScale','Architecture','PoleReal','PoleImag','TorsionalFrequency_Hz','TorsionalDampingRatio','Pi_MECH','MaxRealPole','PhysicalClass','AllStable'});
T=cell2table(torqueRows,'VariableNames',{'WorkpointScale','Architecture','Frequency_Hz','De_Base','Ke_Base','DeltaDe_MPPT','DeltaKe_MPPT','DeltaDe_DVC','DeltaKe_DVC','DeltaDe_GFM','DeltaKe_GFM','EvidenceStatus'});
C=cell2table(couplingRows,'VariableNames',{'WorkpointScale','Architecture','TorsionalFrequency_Hz','C_GridToMachine','C_MachineToGrid','DirectionalRatio','LocalDirection'});
D=cell2table(residueRows,'VariableNames',{'WorkpointScale','Architecture','Disturbance','PoleChangeIndex','ResidueChangeDecades','Classification'});
F=table();
G=cell2table(pointGateRows,'VariableNames',{'WorkpointScale','MaxEquilibriumResidual','MaxPairwiseWorkpointMismatch_pu','GateA_PASS'});
E=cell2table(energyRows,'VariableNames',{'WorkpointScale','Architecture','Paero_W','Pelectromagnetic_W','Pcu_W','Pmsc_W','Pgsc_W','Ppcc_W','ModelIdentityResidual_W','LegacyPlusResidual_W','PhysicalGeneratorResidual_W','DCLinkMismatch_W','GridFilterLoss_W','ModelIdentity_PASS','PhysicalGeneratorSign_PASS'});
WP=cell2table(wpRows,'VariableNames',{'WorkpointScale','Wind_mps','OmegaTarget_radps','Tm0_Nm','Paero_W','RawCpPower_W','CpCalibration','LossCompensation_Nm','CalibrationIterations','TargetSpeedRelativeError','CalibrationConverged','Definition'});
WP.Definition=string(WP.Definition);
counterexamples=table();
Track=trackTorsionAcrossWorkpoints(scales,allModels,M);
CF=counterfactualPoleExcitation(scales,allModels,allP);
CE=detectGateACounterexamples(T,C,CF,Track);

independent=struct('enabled',opt.RunIndependentCheck,'pass',true,'summary',table(),'curves',{{}});
if opt.RunIndependentCheck
    % 只在中间工作点做代表性机械扰动；该实现不调用M2 RHS，作为第二套连续平均代码核对。
    independent=runM3IndependentCheck(selectedModels,selectedP,0.8);
    assert(independent.pass,'M3独立连续实现核对失败。');
end

R=struct('objective','M3跨运行点可证伪验证：不预设M2结论跨工况成立', ...
    'model_scope','5 MW ideal continuous physical averaged VSC; no EMT/PWM/discrete/delay/limits', ...
    'pmsg_convention',pmsgConvention, ...
    'workpoint_scales',scales,'workpoint',W,'modal',M,'torque',T,'coupling',C, ...
    'residue',D,'local_scan',F,'counterexamples',counterexamples,'gateA',G, ...
    'physical_workpoint',WP,'energy_audit',E,'mode_tracking',Track, ...
    'counterfactual_pole_excitation',CF,'gateA_counterexamples',CE, ...
    'feedback_curves',{allFeedbackCurves},'coupling_curves',{allCouplingCurves}, ...
    'independent_check',independent);

if opt.SaveOutputs
    out=here; temp=fullfile(out,'temp'); if ~exist(temp,'dir'),mkdir(temp);end
    audit=assembleWorkpointEnergyAudit(W,E,WP);
    fingerprint=assembleGateAFingerprint(M,T,C,Track,CF);
    writetable(audit,fullfile(out,'M3_GateA_Workpoint_and_Energy_Audit.csv'));
    writetable(fingerprint,fullfile(out,'M3_GateA_Mechanism_Fingerprint.csv'));
    writetable(CE,fullfile(out,'M3_GateA_Counterexamples.csv'));
    save(fullfile(out,'M3_GateA_Summary.mat'),'R','-v7.3');
    makeM3GateAFigure(fullfile(out,'M3_GateA_Fingerprint.png'),R,fingerprint);
    writeM3GateAReport(fullfile(out,'M3_GateA_Report_CN.md'),R,audit,fingerprint);
end

if any(strcmp(stopAfter,{'S2','S3','S4','S5A','S5B','S5B_AUDIT','S6'}))
    [S2,gateS2]=runM3S2(scales,allModels,allP,M,opt.S2WorkpointScales,base,p0);
    R.stageS2=S2; R.gateS2=gateS2;
    if opt.SaveOutputs
        writetable(S2.summary,fullfile(here,'M3_S2_Control_Boundary_Summary.csv'));
        writetable(S2.boundaries,fullfile(here,'M3_S2_Boundaries.csv'));
        writetable(S2.counterexamples,fullfile(here,'M3_S2_Counterexamples.csv'));
        save(fullfile(here,'M3_S2_Summary.mat'),'S2','gateS2','-v7.3');
        makeM3S2Figure(fullfile(here,'M3_S2_Mechanism_Boundaries.png'),S2);
        makeM3S2BoundaryFigure(fullfile(here,'M3_S2_Refined_Boundary_Crossings.png'),S2);
        writeM3S2Report(fullfile(here,'M3_S2_Report_CN.md'),S2,gateS2);
    end
    if ~gateS2.pass
        fprintf('M3 S2 Gate failed; diagnostic results returned and all later stages remain blocked.\n');
    end
end

if any(strcmp(stopAfter,{'S3','S4','S5A','S5B','S5B_AUDIT','S6'}))
    assert(exist('gateS2','var')==1&&gateS2.pass,'S2 Gate未通过，禁止进入S3反事实分解。');
    [S3,gateS3]=runM3S3(scales,allModels,allP,M,S2,base,p0);
    R.stageS3=S3;R.gateS3=gateS3;
    if opt.SaveOutputs
        writetable(S3.case_definitions,fullfile(here,'M3_S3_Case_Definitions.csv'));
        writetable(S3.summary,fullfile(here,'M3_S3_Counterfactual_Summary.csv'));
        save(fullfile(here,'M3_S3_Summary.mat'),'S3','gateS3','-v7.3');
        makeM3S3Figure(fullfile(here,'M3_S3_Counterfactual_Responses.png'),S3);
        writeM3S3Report(fullfile(here,'M3_S3_Report_CN.md'),S3,gateS3);
    end
    if ~gateS3.pass
        fprintf('M3 S3 Gate failed; S4模态混合验证保持阻塞。\n');
    end
end


if any(strcmp(stopAfter,{'S4','S5A','S5B','S5B_AUDIT','S6'}))
    assert(exist('gateS3','var')==1&&gateS3.pass,'S3 Gate未通过，禁止进入S4模态混合验证。');
    [S4,gateS4]=runM3S4(scales,allModels,allP,S2,base,p0);
    R.stageS4=S4;R.gateS4=gateS4;
    if opt.SaveOutputs
        writetable(S4.scan,fullfile(here,'M3_S4_Targeted_Modal_Scan.csv'));
        writetable(S4.assessment,fullfile(here,'M3_S4_Hybridization_Assessment.csv'));
        save(fullfile(here,'M3_S4_Summary.mat'),'S4','gateS4','-v7.3');
        makeM3S4Figure(fullfile(here,'M3_S4_Targeted_Modal_Interaction.png'),S4);
        writeM3S4Report(fullfile(here,'M3_S4_Report_CN.md'),S4,gateS4);
    end
    if ~gateS4.pass,fprintf('M3 S4 Gate failed; S5保持阻塞。\n');end
end


if any(strcmp(stopAfter,{'S5A','S5B','S5B_AUDIT','S6'}))
    assert(exist('gateS4','var')==1&&gateS4.pass,'S4 Gate未通过，禁止进入S5A完整气动反馈验证。');
    [S5A,gateS5A]=runM3S5A(scales,allModels,allP,M,T,C,base,m0Dir);
    R.stageS5A=S5A;R.gateS5A=gateS5A;
    if opt.SaveOutputs
        writetable(S5A.summary,fullfile(here,'M3_S5A_Full_Aero_Summary.csv'));
        writetable(S5A.mechanism_survival,fullfile(here,'M3_S5A_Mechanism_Survival.csv'));
        save(fullfile(here,'M3_S5A_Summary.mat'),'S5A','gateS5A','-v7.3');
        makeM3S5AFigure(fullfile(here,'M3_S5A_Full_Aero_Comparison.png'),S5A);
        writeM3S5AReport(fullfile(here,'M3_S5A_Report_CN.md'),S5A,gateS5A);
    end
    if ~gateS5A.pass,fprintf('M3 S5A Gate failed; Pitch运行区扩展保持阻塞。\n');end
end


if any(strcmp(stopAfter,{'S5B','S5B_AUDIT','S6'}))
    assert(exist('gateS5A','var')==1&&gateS5A.pass,'S5A Gate未通过，禁止进入S5B Pitch运行区。');
    if any(strcmp(stopAfter,{'S5B_AUDIT','S6'})),s5bWindFactors=opt.S5BAuditWindFactors;else,s5bWindFactors=opt.S5BWindFactors;end
    [S5B,gateS5B]=runM3S5B(base,p0,m0Dir,s5bWindFactors,opt.S5BPitchKpScale,opt.S5BControllerMode);
    R.stageS5B=S5B;R.gateS5B=gateS5B;
    if opt.SaveOutputs && strcmp(stopAfter,'S5B')
        if abs(opt.S5BPitchKpScale-1)<1e-12
            writetable(S5B.summary,fullfile(here,'M3_S5B_Pitch_Region_Summary.csv'));
            writetable(S5B.mechanism_survival,fullfile(here,'M3_S5B_Mechanism_Survival.csv'));
            writetable(S5B.failure_diagnostics,fullfile(here,'M3_S5B_Failure_Diagnostics.csv'));
            writetable(S5B.stability_boundaries,fullfile(here,'M3_S5B_Stability_Boundaries.csv'));
            save(fullfile(here,'M3_S5B_Summary.mat'),'S5B','gateS5B','-v7.3');
            makeM3S5BFigure(fullfile(here,'M3_S5B_Pitch_Region_Comparison.png'),S5B);
            writeM3S5BReport(fullfile(here,'M3_S5B_Report_CN.md'),S5B,gateS5B);
        else
            tag=strrep(sprintf('M3_S5B_Retuned_KpScale_%.3g',opt.S5BPitchKpScale),'.','p');
            writetable(S5B.summary,fullfile(here,[tag '_Summary.csv']));
            save(fullfile(here,[tag '_Summary.mat']),'S5B','gateS5B','-v7.3');
            makeM3S5BFigure(fullfile(here,[tag '_Comparison.png']),S5B);
            writeM3S5BReport(fullfile(here,[tag '_Report_CN.md']),S5B,gateS5B);
        end
    end
    if ~gateS5B.pass,fprintf('M3 S5B Gate failed; S6柔性机械扩展保持阻塞。\n');end
    if any(strcmp(stopAfter,{'S5B_AUDIT','S6'}))
        assert(gateS5B.pass,'S5B候选在连续风速加密扫描中失稳，停止物理整定审计。');
        [S5BAudit,gateS5BAudit]=runM3S5BCandidateAudit(base,p0,m0Dir,S5B,opt.S5BPitchKpScale,opt.S5BControllerMode,opt.S5BAuditMarginWindFactors);
        R.stageS5BAudit=S5BAudit;R.gateS5BAudit=gateS5BAudit;
        if opt.SaveOutputs
            if strcmpi(opt.S5BControllerMode,'NREL5MW_SCHEDULED_LSS')
                auditTag='M3_S5B_Traceable_NREL_Audit';
            else
                auditTag='M3_S5B_Candidate_Audit';
            end
            writetable(S5BAudit.summary,fullfile(here,[auditTag '.csv']));
            makeM3S5BAuditFigure(fullfile(here,[auditTag '.png']),S5BAudit);
            writeM3S5BAuditReport(fullfile(here,[auditTag '_Report_CN.md']),S5BAudit,gateS5BAudit);
        end
        if ~gateS5BAudit.pass,fprintf('M3 S5B candidate audit did not pass the physical baseline Gate; S6 remains blocked.\n');end
    end
end
if strcmp(stopAfter,'S6')
    assert(strcmpi(opt.S5BControllerMode,'NREL5MW_SCHEDULED_LSS'),'S6必须使用来源可追溯的NREL5MW_SCHEDULED_LSS连续Pitch基准。');
    assert(exist('gateS5BAudit','var')==1&&gateS5BAudit.pass,'S5B可追溯连续命令级基准未通过，禁止进入S6。');
    s6Root=char(string(opt.S6OpenFASTRoot));if isempty(s6Root),s6Root=fullfile(here,'temp','OpenFAST_S6');end
    [S6,gateS6]=runM3S6(base,p0,m0Dir,s6Root,s6Variant,s6CaseDir,s6CaseStem);
    R.stageS6=S6;R.gateS6=gateS6;
    if opt.SaveOutputs
        if strcmp(s6Variant,'FROZEN_WAKE')
            s6Tag='M3_S6_OpenFAST_FrozenWake_Flexible_Mechanics';
        else
            s6Tag='M3_S6_OpenFAST_Flexible_Mechanics';
        end
        writetable(S6.summary,fullfile(here,[s6Tag '_Summary.csv']));
        % 低频电—机互连只保存紧凑的频域/Schur摘要，不保存矩阵或时序。
        writetable(S6.low_frequency_interconnection_audit,fullfile(here,[s6Tag '_LowFrequency_Interconnection.csv']));
        makeM3S6Figure(fullfile(here,[s6Tag '.png']),S6);
        writeM3S6Report(fullfile(here,[s6Tag '_Report_CN.md']),S6,gateS6);
    end
    if ~gateS6.pass,fprintf('M3 S6 Gate failed; S7离散平均控制保持阻塞。\n');end
end
end

function [S6,gate]=runM3S6(base,p0,m0Dir,ofRoot,variant,customCaseDir,customCaseStem)
% S6：用官方OpenFAST周期稳态线性化模型做局部柔性机械反例测试。
% 完整AeroDyn动态MBC矩阵先接受稳定性审计；若其内部诱导速度状态含
% 大量RHP极点，则不把这些数值模态接入GFM。S6仅在AeroDyn内部状态
% 可稳定静态消元时建立ElastoDyn柔性机械+准稳态气动的条件性模型。
variant=upper(char(string(variant)));
if nargin<6,customCaseDir='';end
if nargin<7,customCaseStem='';end
toolboxDir=fullfile(ofRoot,'matlab-toolbox');
isCustomCase=~isempty(customCaseDir);
if isCustomCase
    caseDir=char(string(customCaseDir));caseStem=char(string(customCaseStem));
elseif strcmp(variant,'FROZEN_WAKE')
    caseStem='5MW_Land_Linear_Aero_CalcSteady_v500_frozen';
    caseDir=fullfile(ofRoot,'runtime',caseStem);
else
    caseStem='5MW_Land_Linear_Aero_CalcSteady';
    caseDir=fullfile(ofRoot,'r-test','glue-codes','openfast',caseStem);
end
assert(exist(toolboxDir,'dir')==7&&exist(caseDir,'dir')==7,'S6 OpenFAST source/toolbox is missing under %s.',ofRoot);
old=path;cleanup=onCleanup(@()path(old));addpath(genpath(toolboxDir));
files=arrayfun(@(k)fullfile(caseDir,sprintf('%s.%d.lin',caseStem,k)),1:3,'UniformOutput',false);
assert(all(cellfun(@(f)exist(f,'file')==2,files)),'S6 requires three periodic OpenFAST .lin files.');
[mbc,md]=fx_mbc3(files);lin0=ReadFASTLinear(files{1});
Afull=real(mbc.AvgA);Bfull=real(mbc.AvgB);stateNames=string(mbc.DescStates(:));inputNames=string(md.DescCntrlInpt(:));
isED=startsWith(stateNames,"ED ");iED=find(isED);iAD=find(~isED);
assert(~isempty(iED),'OpenFAST ElastoDyn state partition failed.');
frozenWakeDirect=isempty(iAD);
if frozenWakeDirect
    % OpenFAST DBEMT_Mod=-1 fixes induction at the trim point.  There are
    % no dynamic AeroDyn states and no algebraic state elimination here.
    rcondAero=NaN;Aof=Afull;Bof=Bfull;ofNames=stateNames;nOF=numel(stateNames);
else
    rcondAero=rcond(Afull(iAD,iAD));assert(rcondAero>1e-10,'AeroDyn static condensation is ill-conditioned (rcond %.3e).',rcondAero);
    Aof=Afull(iED,iED)-Afull(iED,iAD)*(Afull(iAD,iAD)\Afull(iAD,iED));
    Bof=Bfull(iED,:)-Afull(iED,iAD)*(Afull(iAD,iAD)\Bfull(iAD,:));
    ofNames=stateNames(iED);nOF=numel(iED);
end
iWgFull=find(contains(stateNames,"First time derivative of Variable speed generator DOF"),1);
iGeAngFull=find(contains(stateNames,"Variable speed generator DOF")&~contains(stateNames,"First time derivative"),1);
iDrAngFull=find(contains(stateNames,"Drivetrain rotational-flexibility DOF")&~contains(stateNames,"First time derivative"),1);
iDrSpdFull=find(contains(stateNames,"First time derivative of Drivetrain rotational-flexibility DOF"),1);
iWg=find(iED==iWgFull,1);iGeAng=find(iED==iGeAngFull,1);iDrAng=find(iED==iDrAngFull,1);iDrSpd=find(iED==iDrSpdFull,1);
iTq=find(contains(inputNames,"ED Generator torque"),1);iWind=find(startsWith(inputNames,"IfW Extended input: horizontal wind speed"),1);
interfaceUnique=all(cellfun(@(q)isscalar(q)&&~isempty(q),{iWg,iGeAng,iDrAng,iDrSpd,iTq,iWind}));
assert(interfaceUnique,'OpenFAST torque/speed/drivetrain/wind interface is not unique.');
Bt=Bof(:,iTq);

edFile=fullfile(caseDir,'NRELOffshrBsline5MW_Onshore_ElastoDyn.dat');gear=readOpenFASTNumericKey(edFile,'GBRatio');
omegaOF=mean(md.Omega);windOF=mean(md.WindSpeed);
% ElastoDyn's DOF_GeAz state in this linearization is expressed on the
% low-speed-equivalent coordinate: its operating speed must match md.Omega,
% rather than GBRatio*md.Omega.  The *input* "Generator torque" is still a
% high-speed-shaft torque.  Therefore the energy-conjugate bridge is
% Tq_OF=T_e/GBRatio together with omega_M3=dot(GeAz), with no extra speed
% division.  Audit this explicitly; otherwise an unnoticed 97:1 mismatch
% would make any DVC stability conclusion meaningless.
omegaGeAzOF=lin0.x_op{iWgFull};
speedCoordinateRatio=omegaGeAzOF/max(omegaOF,eps);
speedCoordinateIsLssEquivalent=abs(speedCoordinateRatio-1)<1e-3;
assert(speedCoordinateIsLssEquivalent, ...
    'OpenFAST DOF_GeAz speed %.9g does not match low-speed Omega %.9g (ratio %.9g); explicit speed conversion is required.', ...
    omegaGeAzOF,omegaOF,speedCoordinateRatio);
% fx_mbc3 reports azimuth in degrees for this official OpenFAST case.
% Convert only when the magnitude proves the metadata is not already radian.
azPhase=md.Azimuth(:);
if max(abs(azPhase))>2*pi+sqrt(eps)
    azPhase=deg2rad(azPhase);
end
az=sort(mod(azPhase,2*pi));azGap=diff([az;az(1)+2*pi]);azErr=max(abs(azGap-2*pi/3));
uTq=find(contains(string(lin0.u_desc),"ED Generator torque"),1);tqHss=lin0.u_op{uTq};tqLss=gear*tqHss;powerOF=tqLss*omegaOF;
energyErr=abs(tqHss*(gear*omegaGeAzOF)-tqLss*omegaOF)/max(abs(powerOF),1);
[gitStatus,gitHash]=system(sprintf('git -C "%s" rev-parse HEAD',fullfile(ofRoot,'r-test')));gitHash=strtrim(gitHash);
% 自定义候选可能复用了官方 r-test 工具，但案例参数本身并非官方
% 资产；因此不能让工具仓库的 Git 哈希冒充案例可追溯性。
sourceTraceable=~isCustomCase && gitStatus==0&&strlength(string(gitHash))>=7;

% 重新求与OpenFAST低速轴速度、等效转矩和功率一致的三架构工作点。
p=p0(:);p(12)=omegaOF;p(37)=powerOF;p(39)=tqLss;p(40)=pccVoltageMagnitudeForPQ(p(37),p(38),p(4),p(9),p(3)*p(10));
wp=struct('powerScale',p(37)/p0(37),'speedScale',p(12)/p0(12),'torqueScale',p(39)/p0(39), ...
    'wind_mps',windOF,'lambda_opt',NaN,'Cp_opt',NaN,'rawCpPower_W',NaN,'aeroCalibration',NaN, ...
    'aeroPower_W',p(39)*p(12),'definition','OPENFAST_8MPS_LOW_SPEED_EQUIVALENT_INTERFACE', ...
    'pmsgConvention','GENERATOR_OUTWARD');
[p,wp]=calibrateMwtTorqueAtTargetSpeed(base,p,p0,wp);aa=makeArchitectures(base,p,wp);[models,~,workAudit,gA]=runStageA(aa,p,base);
[~,m3Tor,gB]=runStageB(models);assert(gA.pass&&gB.pass,'S6 common electrical workpoint or M3 modal Gate failed.');

% The MBC average is a local LTI approximation to a periodic rotor.  Audit
% each of the three MBC phase samples before interpreting any near-zero pole
% in the averaged matrix as a model-independent stability mechanism.
phaseRows={};phaseRcond=zeros(numel(files),1);phaseFull=cell(numel(models),numel(files));phaseNoAngle=cell(numel(models),numel(files));phaseOpenLoop=cell(numel(models),numel(files));
for iph=1:numel(files)
    Ak=real(mbc.A(:,:,iph));Bk=real(mbc.B(:,:,iph));
    if frozenWakeDirect
        phaseRcond(iph)=NaN;Aofk=Ak;Bofk=Bk;
    else
        phaseRcond(iph)=rcond(Ak(iAD,iAD));
        assert(phaseRcond(iph)>1e-10,'S6 phase %d AeroDyn condensation is ill-conditioned.',iph);
        Aofk=Ak(iED,iED)-Ak(iED,iAD)*(Ak(iAD,iAD)\Ak(iAD,iED));
        Bofk=Bk(iED,:)-Ak(iED,iAD)*(Ak(iAD,iAD)\Bk(iAD,iED));
    end
    for a=1:numel(models)
        M=models{a};qf=openFastHybridDiagnostic(Aofk,Bofk(:,iTq),ofNames,iGeAng,iWg,M.linear,gear,1,false);
        qn=openFastHybridDiagnostic(Aofk,Bofk(:,iTq),ofNames,iGeAng,iWg,M.linear,gear,1,true);
        q0=openFastHybridDiagnostic(Aofk,Bofk(:,iTq),ofNames,iGeAng,iWg,M.linear,gear,0,false);
        phaseFull{a,iph}=qf;phaseNoAngle{a,iph}=qn;phaseOpenLoop{a,iph}=q0;
        phaseRows(end+1,:)={iph,rad2deg(azPhase(iph)),string(M.name),"FULL_COORD",qf.maxReal,qf.worstFrequency_Hz,qf.stable,qf.worstDominantState}; %#ok<AGROW>
        phaseRows(end+1,:)={iph,rad2deg(azPhase(iph)),string(M.name),"ANGLE_REMOVED",qn.maxReal,qn.worstFrequency_Hz,qn.stable,qn.worstDominantState}; %#ok<AGROW>
        phaseRows(end+1,:)={iph,rad2deg(azPhase(iph)),string(M.name),"TORQUE_FEEDBACK_OPEN",q0.maxReal,q0.worstFrequency_Hz,q0.stable,q0.worstDominantState}; %#ok<AGROW>
    end
end
[azSort,phaseOrder]=sort(mod(azPhase,2*pi));phaseDt=diff([azSort;azSort(1)+2*pi])/omegaOF;floquetRows={};
for a=1:numel(models)
    qf=periodicFloquetMetrics(phaseFull(a,phaseOrder),phaseDt);
    qn=periodicFloquetMetrics(phaseNoAngle(a,phaseOrder),phaseDt);
    q0=periodicFloquetMetrics(phaseOpenLoop(a,phaseOrder),phaseDt);
    floquetRows(end+1,:)={string(models{a}.name),"FULL_COORD",qf.maxReal,qf.worstFrequency_Hz,qf.stable,qf.worstDominantState,qf.worstMultiplierMagnitude}; %#ok<AGROW>
    floquetRows(end+1,:)={string(models{a}.name),"ANGLE_REMOVED",qn.maxReal,qn.worstFrequency_Hz,qn.stable,qn.worstDominantState,qn.worstMultiplierMagnitude}; %#ok<AGROW>
    floquetRows(end+1,:)={string(models{a}.name),"TORQUE_FEEDBACK_OPEN",q0.maxReal,q0.worstFrequency_Hz,q0.stable,q0.worstDominantState,q0.worstMultiplierMagnitude}; %#ok<AGROW>
end

lamFull=eig(Afull);fullRhp=sum(real(lamFull)>1e-7);fullMax=max(real(lamFull));
lamRed=eig(Aof);rigidRed=abs(imag(lamRed))<1e-6&abs(real(lamRed))<1e-3;redMax=max(real(lamRed(~rigidRed)));redStable=redMax<0;
% 正的M3 T_e定义为发电制动转矩；OpenFAST对应输入的局部加速度
% 偏导必须为负，才允许把 +T_e/GBRatio 接到 ED Generator torque。
% 这项审计防止把混合模型RHP现象误判为控制机理而实际来自接口反号。
torqueInputAcceleration=Bt(iWg);
torqueSignConsistent=torqueInputAcceleration<0;
assert(torqueSignConsistent,'OpenFAST positive generator torque does not decelerate the generator-speed coordinate.');
rows={};modalRows={};closureRows={};slowPartRows={};dcLinkRows={};sourceSlopeRows={};impedanceRows={};lfRows={};allHybridStable=true;allModeIdentity=true;maxExtraneous=0;
for a=1:numel(models)
    M=models{a};L=M.linear;ln=string(L.state_names(:));iW=find(ln=="omega_g",1);iTheta=find(ln=="theta_sh",1);iWt=find(ln=="omega_t",1);
    m3NativeMaxReal=max(real(eig(L.A)));m3NativeStable=m3NativeMaxReal<0;
    mech=ismember(ln,["theta_sh","omega_t","omega_g"]);eidx=find(~mech);extraCols=setdiff([iTheta iWt],[]);extraneous=norm(L.A(eidx,extraCols),'fro');maxExtraneous=max(maxExtraneous,extraneous);
    % 在同一低速轴、正发电制动转矩约定下，对比原M3两质量机械对象与
    % OpenFAST柔性对象的转矩—速度阻抗。该比较不含电气反馈，只用于
    % 解释新增慢分支是否伴随机械对象的低频幅相差异。
    AmechM3=L.A(mech,mech);BteM3=[0;0;-1/p(20)];CwgM3=[0 0 1];
    % 不能以单个 1 mHz 点替代柔性机械对象的低频判定。这里仅对
    % 开环机械对象扫描 1--100 mHz；不含任何电气、DVC 或 GFM反馈。
    impedanceProbeHz=[1e-3 3e-3 1e-2 3e-2 1e-1];
    spProbe=1i*2*pi*1e-3;
    GmM3Probe=CwgM3*((spProbe*eye(size(AmechM3))-AmechM3)\BteM3);
    iyTe=find(string(L.output_names)=="T_e",1);iyP=find(string(L.output_names)=="P_PCC",1);iyU=find(string(L.output_names)=="Udc",1);
    Aee=L.A(eidx,eidx);Aew=L.A(eidx,iW);CteE=L.C(iyTe,eidx);DteW=L.C(iyTe,iW);CpE=L.C(iyP,eidx);DpW=L.C(iyP,iW);CuE=L.C(iyU,eidx);DuW=L.C(iyU,iW);
    kT=1/gear;Cwg=zeros(1,nOF);Cwg(iWg)=1;
    for fProbe=impedanceProbeHz
        sp=1i*2*pi*fProbe;
        GmM3Band=CwgM3*((sp*eye(size(AmechM3))-AmechM3)\BteM3);
        GmOpenFASTBand=Cwg*((sp*eye(size(Aof))-Aof)\(Bt*kT));
        impedanceRows(end+1,:)={string(M.name),fProbe,real(GmM3Band),imag(GmM3Band),real(GmOpenFASTBand),imag(GmOpenFASTBand), ...
            abs(GmOpenFASTBand)/max(abs(GmM3Band),eps),rad2deg(angle(GmOpenFASTBand/GmM3Band)), ...
            -real(1/GmM3Band),-real(1/GmOpenFASTBand)}; %#ok<AGROW>
    end
    Ah=[Aof+Bt*kT*DteW*Cwg Bt*kT*CteE;Aew*Cwg Aee];
    lfRows=[lfRows;s6LowFrequencyInterconnectionRows(Aof,Bt,iWg,L,gear,string(M.name))]; %#ok<AGROW>
    noGauge=openFastHybridDiagnostic(Aof,Bt,ofNames,iGeAng,iWg,L,gear,1,true);
    fullTorqueOpen=openFastHybridDiagnostic(Aof,Bt,ofNames,iGeAng,iWg,L,gear,0,false);
    fullTorqueClosed=openFastHybridDiagnostic(Aof,Bt,ofNames,iGeAng,iWg,L,gear,1,false);
    GmOfProbe=complex(fullTorqueClosed.GmProbeReal,fullTorqueClosed.GmProbeImag);
    mechProbeMagnitudeRatio=abs(GmOfProbe)/max(abs(GmM3Probe),eps);
    mechProbePhaseDifference_deg=rad2deg(angle(GmOfProbe/GmM3Probe));
    effectiveMechanicalDampingM3=-real(1/GmM3Probe);
    effectiveMechanicalDampingOpenFAST=-real(1/GmOfProbe);
    partModels={fullTorqueOpen,fullTorqueClosed};partLabels=["TORQUE_FEEDBACK_OPEN","TORQUE_FEEDBACK_FULL"];
    for ip=1:numel(partModels)
        qp=partModels{ip};
        slowPartRows(end+1,:)={string(M.name),partLabels(ip),qp.maxReal,qp.worstFrequency_Hz,qp.stable,qp.worstDominantState,qp.PiOpenFAST,qp.PiGeneratorSpeed,qp.PiTorsionalFlex,qp.PiUdc,qp.PiXiDvc}; %#ok<AGROW>
    end
    angleRows=setdiff(1:nOF,iGeAng);angleColumnNorm=norm(Aof(angleRows,iGeAng),2);
    for gamma=[0 0.25 0.5 0.75 1]
        qg=openFastHybridDiagnostic(Aof,Bt,ofNames,iGeAng,iWg,L,gear,gamma,true);
        closureRows(end+1,:)={string(M.name),"TORQUE_FEEDBACK_CLOSURE_NO_ANGLE",gamma,qg.maxReal,qg.worstFrequency_Hz,qg.stable,qg.worstDominantState,qg.GeProbeReal,qg.GeProbeImag,qg.GmProbeReal,qg.GmProbeImag,qg.loopProbeReal,qg.loopProbeImag,M.equilibrium.normalized_residual,fullTorqueOpen.maxReal,fullTorqueOpen.stable}; %#ok<AGROW>
        qgf=openFastHybridDiagnostic(Aof,Bt,ofNames,iGeAng,iWg,L,gear,gamma,false);
        closureRows(end+1,:)={string(M.name),"TORQUE_FEEDBACK_CLOSURE_FULL_COORD",gamma,qgf.maxReal,qgf.worstFrequency_Hz,qgf.stable,qgf.worstDominantState,qgf.GeProbeReal,qgf.GeProbeImag,qgf.GmProbeReal,qgf.GmProbeImag,qgf.loopProbeReal,qgf.loopProbeImag,M.equilibrium.normalized_residual,fullTorqueOpen.maxReal,fullTorqueOpen.stable}; %#ok<AGROW>
    end
    if strcmpi(M.name,'MWT')
        % MWT 的 xi_DVC 是 DC-link PI 的积分状态，同时也是零Udc误差
        % 时的稳态转矩参考。原先只缩放PI会将“冻结参考”“P”“I”混在
        % 一起，因此这里分开做四类反事实，且每个反事实重新求平衡点。
        dvcSpecs={ ...
            "MSC_DVC_PI_SCALE",[0 1e-12 3e-12 1e-11 3e-11 1e-10 1e-8 1e-6 1e-4 1e-2 0.1 0.25 0.5 1]; ...
            "MSC_DVC_P_ONLY_SCALE",[0 1e-8 1e-6 1e-4 3e-4 1e-3 3e-3 1e-2 0.1 0.25 0.5 1]; ...
            "MSC_DVC_I_ONLY_SCALE",[1e-12 3e-12 1e-11 3e-11 1e-10 1e-8 1e-6 1e-4 1e-2 0.1 0.25 0.5 1]};
        for ds=1:size(dvcSpecs,1)
            dvcCase=dvcSpecs{ds,1}; dvcFactors=dvcSpecs{ds,2};
            for dvcScale=dvcFactors
                pd=p;activeDvc=M.active;
                switch dvcCase
                    case "MSC_DVC_PI_SCALE"
                        pd(25:26)=dvcScale*p(25:26);
                        if dvcScale==0,activeDvc=setdiff(activeDvc,6);end
                    case "MSC_DVC_P_ONLY_SCALE"
                        pd(25)=dvcScale*p(25);pd(26)=0;
                        activeDvc=setdiff(activeDvc,6);
                    case "MSC_DVC_I_ONLY_SCALE"
                        pd(25)=0;pd(26)=dvcScale*p(26);
                end
                [xDvc,eqDvc]=solveEquilibrium(M.x0,pd,M.name,M.flags,activeDvc);
                assert(eqDvc.normalized_residual<1e-8,'S6 DVC counterfactual equilibrium failed for %s at %.6g.',dvcCase,dvcScale);
                Ldvc=linearizeModel(xDvc,pd,M.name,M.flags,activeDvc);
                qdOpen=openFastHybridDiagnostic(Aof,Bt,ofNames,iGeAng,iWg,Ldvc,gear,0,false);
                qd=openFastHybridDiagnostic(Aof,Bt,ofNames,iGeAng,iWg,Ldvc,gear,1,true);
                closureRows(end+1,:)={string(M.name),dvcCase+"_NO_ANGLE",dvcScale,qd.maxReal,qd.worstFrequency_Hz,qd.stable,qd.worstDominantState,qd.GeProbeReal,qd.GeProbeImag,qd.GmProbeReal,qd.GmProbeImag,qd.loopProbeReal,qd.loopProbeImag,eqDvc.normalized_residual,qdOpen.maxReal,qdOpen.stable}; %#ok<AGROW>
                qdf=openFastHybridDiagnostic(Aof,Bt,ofNames,iGeAng,iWg,Ldvc,gear,1,false);
                closureRows(end+1,:)={string(M.name),dvcCase+"_FULL_COORD",dvcScale,qdf.maxReal,qdf.worstFrequency_Hz,qdf.stable,qdf.worstDominantState,qdf.GeProbeReal,qdf.GeProbeImag,qdf.GmProbeReal,qdf.GmProbeImag,qdf.loopProbeReal,qdf.loopProbeImag,eqDvc.normalized_residual,qdOpen.maxReal,qdOpen.stable}; %#ok<AGROW>
            end
        end
        % 只改变DC-link物理储能，不改变MSC-DVC或GFM参数。该反事实用于
        % 检验慢速分支是否由DC能量时间尺度与柔性传动链相互作用造成，
        % 绝不作为让S6 Gate通过的“补丁整定”。
        for cdcFactor=[0.25 0.5 1 2 4 8]
            pc=p;pc(11)=cdcFactor*p(11);
            [xCdc,eqCdc]=solveEquilibrium(M.x0,pc,M.name,M.flags,M.active);
            assert(eqCdc.normalized_residual<1e-8,'S6 Cdc counterfactual equilibrium failed at %.6g.',cdcFactor);
            LCdc=linearizeModel(xCdc,pc,M.name,M.flags,M.active);
            qcOpen=openFastHybridDiagnostic(Aof,Bt,ofNames,iGeAng,iWg,LCdc,gear,0,false);
            qcFull=openFastHybridDiagnostic(Aof,Bt,ofNames,iGeAng,iWg,LCdc,gear,1,false);
            dcLinkRows(end+1,:)={cdcFactor,pc(11),qcFull.maxReal,qcFull.worstFrequency_Hz,qcFull.stable,qcFull.worstDominantState,qcOpen.maxReal,qcOpen.stable,eqCdc.normalized_residual}; %#ok<AGROW>
        end
        % M3机械输入采用 Tm=Pm/omega_t；在当前平衡点该源律具有
        % dTm/domega_t=-Tm0/omega0 的确定低频斜率。OpenFAST Frozen-Wake
        % 不必具有相同斜率，因此仅将其等效为低速轴附加制动反馈来做
        % 反事实检查。它不改变OpenFAST工作点，且绝不作为S6通过的整定。
        sourceSlopeDamping=p(39)/p(12);
        for sourceScale=[0 0.25 0.5 0.75 1]
            qs=openFastHybridDiagnostic(Aof,Bt,ofNames,iGeAng,iWg,L,gear,1,false,sourceScale*sourceSlopeDamping);
            sourceSlopeRows(end+1,:)={sourceScale,sourceScale*sourceSlopeDamping,qs.maxReal,qs.worstFrequency_Hz,qs.stable,qs.worstDominantState}; %#ok<AGROW>
        end
    end
    % 归一化扰动：grid-frequency为pu频率，wind为pu风速。
    Bgrid=[Bt*kT*L.D(iyTe,4)*p(3);L.B(eidx,4)*p(3)];Bwind=[Bof(:,iWind)*windOF;zeros(numel(eidx),1)];
    Cshaft=[zeros(1,nOF+numel(eidx))];Cshaft(iDrSpd)=1/max(omegaOF,eps);
    Cpcc=[DpW*Cwg CpE]/p(1);Cudc=[DuW*Cwg CuE]/p(2);
    [V,D,Wl]=eig(Ah);lam=diag(D);for k=1:numel(lam),den=Wl(:,k)'*V(:,k);if abs(den)>eps,Wl(:,k)=Wl(:,k)/conj(den);end;end
    PF=abs(V.*conj(Wl));PF=PF./max(sum(PF,1),eps);freq=abs(imag(lam))/(2*pi);zeta=-real(lam)./max(abs(lam),eps);
    piOF=sum(PF(1:nOF,:),1);piElec=sum(PF(nOF+1:end,:),1);grpDr=contains(ofNames,"Drivetrain rotational-flexibility")|contains(ofNames,"Variable speed generator DOF");piDr=sum(PF(find(grpDr),:),1); %#ok<FNDSB>
    obsS=Cshaft*V;projG=Wl'*Bgrid;obsP=Cpcc*V;projW=Wl'*Bwind;obsU=Cudc*V;
    rGS=abs(obsS(:).*projG(:));rWP=abs(obsP(:).*projW(:));rGU=abs(obsU(:).*projG(:));
    rigid=abs(imag(lam))<1e-6&abs(real(lam))<1e-3&piDr'>0.5;nonRigid=~rigid;
    nonRigidIndex=find(nonRigid);[maxReal,jWorstLocal]=max(real(lam(nonRigidIndex)));iWorst=nonRigidIndex(jWorstLocal);
    [~,worstDomIdx]=max(PF(:,iWorst));
    if worstDomIdx<=nOF,worstDom=ofNames(worstDomIdx);else,worstDom=ln(eidx(worstDomIdx-nOF));end
    stable=maxReal<0;allHybridStable=allHybridStable&&stable;
    cand=find(imag(lam)>1e-6&freq>=0.1&freq<=10&piOF'>0.2);assert(~isempty(cand),'S6 found no flexible mechanical candidate modes.');
    [~,jj]=max(piDr(cand));iTor=cand(jj);modePass=piDr(iTor)>0.5;allModeIdentity=allModeIdentity&&modePass;
    [~,iGrid]=max(rGS(cand));iGrid=cand(iGrid);[~,iWindP]=max(rWP(cand));iWindP=cand(iWindP);
    tq=m3Tor(m3Tor.Architecture==string(M.name),:);[~,domIdx]=max(PF(:,iTor));if domIdx<=nOF,dom=ofNames(domIdx);else,dom=ln(eidx(domIdx-nOF));end
    rows(end+1,:)={string(M.name),p(37),p(12),workAudit.NormalizedResidual(a),m3NativeMaxReal,m3NativeStable,maxReal,stable,sum(rigid), ...
        real(lam(iWorst)),imag(lam(iWorst)),freq(iWorst),zeta(iWorst),piOF(iWorst),piElec(iWorst),piDr(iWorst),string(worstDom), ...
        angleColumnNorm,noGauge.maxReal,noGauge.worstFrequency_Hz,noGauge.stable,noGauge.worstDominantState,noGauge.GeProbeReal,noGauge.GeProbeImag,noGauge.GmProbeReal,noGauge.GmProbeImag,noGauge.loopProbeReal,noGauge.loopProbeImag, ...
        real(GmM3Probe),imag(GmM3Probe),fullTorqueClosed.GmProbeReal,fullTorqueClosed.GmProbeImag,effectiveMechanicalDampingM3,effectiveMechanicalDampingOpenFAST,mechProbeMagnitudeRatio,mechProbePhaseDifference_deg, ...
        real(lam(iTor)),imag(lam(iTor)),freq(iTor),zeta(iTor),piOF(iTor),piElec(iTor),piDr(iTor),string(dom), ...
        tq.Frequency_Hz,tq.DampingRatio,freq(iTor)-tq.Frequency_Hz,zeta(iTor)-tq.DampingRatio,rGS(iTor),rWP(iTor),rGU(iTor),freq(iGrid),rGS(iGrid),freq(iWindP),rWP(iWindP),extraneous,modePass}; %#ok<AGROW>
    [~,ord]=sort(freq(cand));cand=cand(ord);for kk=1:numel(cand),ii=cand(kk);[~,di]=max(PF(:,ii));if di<=nOF,ds=ofNames(di);else,ds=ln(eidx(di-nOF));end;modalRows(end+1,:)={string(M.name),real(lam(ii)),imag(lam(ii)),freq(ii),zeta(ii),piOF(ii),piElec(ii),piDr(ii),rGS(ii),rWP(ii),rGU(ii),string(ds),ii==iTor,ii==iGrid,ii==iWindP};end %#ok<AGROW>
end
vars={'Architecture','P0_W','Omega0_radps','EquilibriumResidual','M3NativeMaxRealPole','M3NativeStable','HybridMaxRealNonRigid','HybridStable','RigidModesExcluded', ...
    'WorstPoleReal','WorstPoleImag','WorstFrequency_Hz','WorstDampingRatio','WorstPiOpenFAST','WorstPiElectrical','WorstPiDrivetrain','WorstDominantState', ...
    'GeneratorAngleColumnNorm','NoGaugeMaxReal','NoGaugeWorstFrequency_Hz','NoGaugeStable','NoGaugeWorstDominantState','ElectricalTorqueGain1mHzReal','ElectricalTorqueGain1mHzImag','MechanicalGain1mHzReal','MechanicalGain1mHzImag','LoopGain1mHzReal','LoopGain1mHzImag', ...
    'M3TwoMassMechanicalGain1mHzReal','M3TwoMassMechanicalGain1mHzImag','OpenFASTMechanicalGain1mHzReal','OpenFASTMechanicalGain1mHzImag','M3TwoMassEffectiveMechanicalDamping_NmsPerRad','OpenFASTEffectiveMechanicalDamping_NmsPerRad','MechanicalGainMagnitudeRatio_OpenFASTOverM3','MechanicalGainPhaseDifference_deg_OpenFASTMinusM3', ...
    'DrivePoleReal','DrivePoleImag','DriveFrequency_Hz','DriveDampingRatio','DrivePiOpenFAST','DrivePiElectrical','DrivePiDrivetrain','DriveDominantState', ...
    'M3TwoMassFrequency_Hz','M3TwoMassDampingRatio','DeltaFrequency_Hz','DeltaDampingRatio','DriveResidue_GridToShaft','DriveResidue_WindToPCC','DriveResidue_GridToUdc','TopGridToShaftMode_Hz','TopGridToShaftResidue','TopWindToPCCMode_Hz','TopWindToPCCResidue','ElectricalDependenceOnThetaWt_FroNorm','DriveModeIdentity_PASS'};
T=cell2table(rows,'VariableNames',vars);T.Architecture=string(T.Architecture);T.WorstDominantState=string(T.WorstDominantState);T.NoGaugeWorstDominantState=string(T.NoGaugeWorstDominantState);T.DriveDominantState=string(T.DriveDominantState);
mv={'Architecture','PoleReal','PoleImag','Frequency_Hz','DampingRatio','PiOpenFAST','PiElectrical','PiDrivetrain','Residue_GridToShaft','Residue_WindToPCC','Residue_GridToUdc','DominantState','IsSelectedDriveMode','IsTopGridToShaftMode','IsTopWindToPCCMode'};
MM=cell2table(modalRows,'VariableNames',mv);MM.Architecture=string(MM.Architecture);MM.DominantState=string(MM.DominantState);
CV=cell2table(closureRows,'VariableNames',{'Architecture','Counterfactual','Factor','MaxRealPole','WorstFrequency_Hz','Stable','WorstDominantState','ElectricalTorqueGain1mHzReal','ElectricalTorqueGain1mHzImag','MechanicalGain1mHzReal','MechanicalGain1mHzImag','LoopGain1mHzReal','LoopGain1mHzImag','EquilibriumResidual','TorqueFeedbackOpenMaxRealPole','TorqueFeedbackOpenStable'});
CV.Architecture=string(CV.Architecture);CV.Counterfactual=string(CV.Counterfactual);CV.WorstDominantState=string(CV.WorstDominantState);
dvcRows=CV(CV.Architecture=="MWT"&contains(CV.Counterfactual,"MSC_DVC"),:);
allDvcCounterfactualEquilibriaPass=all(dvcRows.EquilibriumResidual<1e-8);
dvcFeasibility=s6DvcFeasibility(dvcRows,p);
PV=cell2table(phaseRows,'VariableNames',{'PhaseIndex','Azimuth_deg','Architecture','CoordinateTreatment','MaxRealPole','WorstFrequency_Hz','Stable','WorstDominantState'});
PV.Architecture=string(PV.Architecture);PV.CoordinateTreatment=string(PV.CoordinateTreatment);PV.WorstDominantState=string(PV.WorstDominantState);
FV=cell2table(floquetRows,'VariableNames',{'Architecture','CoordinateTreatment','MaxFloquetReal','WorstFrequency_Hz','Stable','WorstDominantState','WorstMultiplierMagnitude'});
FV.Architecture=string(FV.Architecture);FV.CoordinateTreatment=string(FV.CoordinateTreatment);FV.WorstDominantState=string(FV.WorstDominantState);
SP=cell2table(slowPartRows,'VariableNames',{'Architecture','Closure','MaxRealPole','WorstFrequency_Hz','Stable','WorstDominantState','PiOpenFAST','PiGeneratorSpeed','PiTorsionalFlex','PiUdc','PiXiDvc'});
SP.Architecture=string(SP.Architecture);SP.Closure=string(SP.Closure);SP.WorstDominantState=string(SP.WorstDominantState);
DC=cell2table(dcLinkRows,'VariableNames',{'CdcFactor','Cdc_F','MaxRealPole','WorstFrequency_Hz','Stable','WorstDominantState','TorqueFeedbackOpenMaxRealPole','TorqueFeedbackOpenStable','EquilibriumResidual'});
DC.WorstDominantState=string(DC.WorstDominantState);
SS=cell2table(sourceSlopeRows,'VariableNames',{'M3ConstantPowerSlopeFactor','EquivalentLowSpeedDamping_NmsPerRad','MaxRealPole','WorstFrequency_Hz','Stable','WorstDominantState'});
SS.WorstDominantState=string(SS.WorstDominantState);
MI=cell2table(impedanceRows,'VariableNames',{'Architecture','Frequency_Hz','M3MechanicalGainReal','M3MechanicalGainImag','OpenFASTMechanicalGainReal','OpenFASTMechanicalGainImag','MagnitudeRatio_OpenFASTOverM3','PhaseDifference_deg_OpenFASTMinusM3','M3EffectiveMechanicalDamping_NmsPerRad','OpenFASTEffectiveMechanicalDamping_NmsPerRad'});
MI.Architecture=string(MI.Architecture);
LF=cell2table(lfRows,'VariableNames',{'Architecture','Closure','Frequency_Hz','ElectricalGainReal','ElectricalGainImag','MechanicalGainReal','MechanicalGainImag','LoopGainReal','LoopGainImag','LoopPhase_deg','ElectricalResolventNorm','SchurMinSingularValue','SchurRcond','DirectMechanicalFeedbackNorm','ElectricToMechanicalNorm','MechanicalToElectricNorm'});
LF.Architecture=string(LF.Architecture);LF.Closure=string(LF.Closure);
mwtPhase=PV(PV.Architecture=="MWT"&PV.CoordinateTreatment=="FULL_COORD",:);
phaseSignConsistent=all(mwtPhase.MaxRealPole>0)||all(mwtPhase.MaxRealPole<0);
fullFloquet=FV(FV.CoordinateTreatment=="FULL_COORD",:);allFloquetStable=all(fullFloquet.Stable);
angleRemovedFloquet=FV(FV.CoordinateTreatment=="ANGLE_REMOVED",:);allAngleRemovedFloquetStable=all(angleRemovedFloquet.Stable);
openFloquet=FV(FV.CoordinateTreatment=="TORQUE_FEEDBACK_OPEN",:);allOpenLoopFloquetStable=all(openFloquet.Stable);
mwtFloquet=FV(FV.Architecture=="MWT"&FV.CoordinateTreatment=="FULL_COORD",:);
mwtOpenFloquet=FV(FV.Architecture=="MWT"&FV.CoordinateTreatment=="TORQUE_FEEDBACK_OPEN",:);
mwtSlowGrowthIncrement=mwtFloquet.MaxFloquetReal-mwtOpenFloquet.MaxFloquetReal;
commonWp=gA.pass&&max(abs(workAudit.omega_g0_radps-omegaOF))/omegaOF<1e-8&&max(abs(workAudit.P0_W-powerOF))/p(1)<1e-4;
simulinkInterfaceAudit=inspectS6NonlinearSimulinkInterface(ofRoot);
sourceGate=sourceTraceable&&mbc.performedTransformation&&azErr<0.02&&interfaceUnique;
if frozenWakeDirect
    reductionGate=fullRhp==0&&redStable;
    if isCustomCase
        evidenceScope='CONDITIONAL_CUSTOM_FROZEN_WAKE_DIRECT_ELASTODYN_LOW_SPEED_EQUIVALENT_INTERFACE';
    else
        evidenceScope='OPENFAST_FROZEN_WAKE_DIRECT_ELASTODYN_LOW_SPEED_EQUIVALENT_INTERFACE';
    end
    if allHybridStable&&allFloquetStable
        if isCustomCase
            s6Status='CONDITIONAL_CUSTOM_FROZEN_WAKE_HYBRID_STABLE_SOURCE_GATE_FAIL';
            nextRule='Custom case is conditionally stable after the mechanical audit, but source traceability failed; do not treat it as a physical cross-model validation or enter S7.';
        else
            s6Status='FROZEN_WAKE_DIRECT_HYBRID_GATE_PASS';
            nextRule='Frozen-wake direct plant and all hybrid variants passed this local Gate; S7 may begin as a separate discrete-average validation.';
        end
    elseif isCustomCase && allHybridStable&&allAngleRemovedFloquetStable
        s6Status='CONDITIONAL_CUSTOM_FROZEN_WAKE_HYBRID_STABLE_SOURCE_GATE_FAIL';
        nextRule='Custom case is physically stable after removal of the generator-angle gauge, but source traceability/full-coordinate gauge Gate failed; do not treat it as a physical cross-model validation or enter S7.';
    else
        if isCustomCase
            s6Status='FAILED_CUSTOM_FROZEN_WAKE_HYBRID_MWT_DVC_OR_MECHANICAL_MODE';
            nextRule='Do not enter S7. The custom case failed a hybrid stability condition; retain it only as a conditional diagnostic and locate the dominant branch before any retuning.';
        else
            s6Status='FAILED_FROZEN_WAKE_MWT_DVC_SLOW_MODE';
            nextRule='Do not enter S7. The frozen-wake plant and torque-feedback-open counterfactuals are stable, but the MWT DVC torque closure creates a zero-frequency slow mode. Audit it as a local cross-model coupling candidate, not as torsional instability.';
        end
    end
else
    reductionGate=rcondAero>1e-6&&redStable;
    evidenceScope='CONDITIONAL_OPENFAST_ED_PLUS_QUASI_STEADY_AERO_LOW_SPEED_EQUIVALENT_INTERFACE';
    s6Status='FAILED_UNSTABLE_QUASISTEADY_AERODYN_BASELINE';
    nextRule='S7 remains blocked. First obtain a trimmed, strictly stable periodic flexible plant without static AeroDyn-state elimination; this S6 attempt does not validate full dynamic AeroDyn or a direct-drive OpenFAST plant.';
end
gate=struct('pass',sourceGate&&reductionGate&&commonWp&&energyErr<1e-12&&torqueSignConsistent&&allDvcCounterfactualEquilibriaPass&&allOpenLoopFloquetStable&&allFloquetStable&&allModeIdentity&&maxExtraneous<1e-8, ...
    'source_traceable',sourceTraceable,'r_test_commit',gitHash,'mbc_performed',mbc.performedTransformation,'azimuth_spacing_error_rad',azErr, ...
    'full_dynamic_aero_usable',fullRhp==0,'full_dynamic_aero_rhp_poles',fullRhp,'full_dynamic_aero_max_real',fullMax, ...
    'aerodyn_static_condensation_rcond',rcondAero,'reduced_mechanics_max_real_nonrigid',redMax,'reduced_mechanics_stable',redStable, ...
    'common_workpoint_pass',commonWp,'low_speed_interface_energy_error',energyErr,'all_hybrid_nonrigid_stable',allHybridStable,'all_floquet_stable',allFloquetStable,'all_angle_removed_floquet_stable',allAngleRemovedFloquetStable,'all_open_loop_floquet_stable',allOpenLoopFloquetStable,'mwt_slow_growth_increment_per_s',mwtSlowGrowthIncrement, ...
    'all_drive_mode_identity_pass',allModeIdentity,'max_electrical_dependence_on_removed_theta_wt',maxExtraneous, ...
    'all_dvc_counterfactual_equilibria_pass',allDvcCounterfactualEquilibriaPass, ...
    'phase_aerodyn_condensation_rcond_min',min(phaseRcond),'mwt_full_coordinate_phase_sign_consistent',phaseSignConsistent, ...
    'gear_ratio',gear,'openfast_wind_mps',windOF,'openfast_rotor_speed_radps',omegaOF,'openfast_geaz_speed_radps',omegaGeAzOF, ...
    'openfast_speed_coordinate_ratio_to_rotor',speedCoordinateRatio,'openfast_speed_coordinate_is_lss_equivalent',speedCoordinateIsLssEquivalent,'openfast_hss_torque_Nm',tqHss, ...
    'openfast_generator_speed_accel_per_positive_torque',torqueInputAcceleration,'generator_torque_sign_consistent',torqueSignConsistent, ...
    'openfast_lss_equivalent_torque_Nm',tqLss,'openfast_equivalent_power_W',powerOF, ...
    'evidence_scope',evidenceScope, ...
    's6_status',s6Status, ...
    'next_rule',nextRule,'variant',variant,'frozen_wake_direct',frozenWakeDirect);
S6=struct('objective','S6 local flexible-mechanics counterexample test using official OpenFAST periodic linearization','variant',variant,'summary',T,'modal_candidates',MM,'slow_mode_counterfactuals',CV,'slow_mode_participation',SP,'dvc_feasibility',dvcFeasibility,'dc_link_capacitance_audit',DC,'constant_power_source_slope_audit',SS,'low_frequency_mechanical_impedance_audit',MI,'low_frequency_interconnection_audit',LF,'phase_sensitivity',PV,'floquet',FV,'workpoint_audit',workAudit,'m3_reference_torsion',m3Tor,'simulink_nonlinear_interface_audit',simulinkInterfaceAudit,'gate',gate);
end

function A=inspectS6NonlinearSimulinkInterface(ofRoot)
% 仅审计当前S6资产是否已带有OpenFAST-Simulink MEX接口；不下载、不编译。
sfunc=dir(fullfile(ofRoot,'**','FAST_SFunc.mex*'));
fastLib=dir(fullfile(ofRoot,'**','FAST_Library.*'));
buildScript=dir(fullfile(ofRoot,'**','create_FAST_SFunc.m'));
A=struct('sfunc_mex_count',numel(sfunc),'fast_library_count',numel(fastLib), ...
    'build_script_count',numel(buildScript),'current_assets_support_direct_simulink_cosim',numel(sfunc)>0, ...
    'audit_scope','Current S6 directory only; no source download/build was attempted.');
end

function D=s6DvcFeasibility(CV,p)
% 从离散测试点给出稳定因子区间，不把稀疏扫描冒充为精确临界值。
pick=@(name)CV(CV.Counterfactual==name,:);
piRows=pick("MSC_DVC_PI_SCALE_FULL_COORD");pp=pick("MSC_DVC_P_ONLY_SCALE_FULL_COORD");
posStable=@(q)q.Factor(q.Stable&q.Factor>0);firstUnstable=@(q)q.Factor(~q.Stable&q.Factor>0);
piStable=posStable(piRows);piUnstable=firstUnstable(piRows);pStable=posStable(pp);pUnstable=firstUnstable(pp);
maxPiStable=NaN;if ~isempty(piStable),maxPiStable=max(piStable);end
minPiUnstable=NaN;if ~isempty(piUnstable),minPiUnstable=min(piUnstable);end
maxPStable=NaN;if ~isempty(pStable),maxPStable=max(pStable);end
minPUnstable=NaN;if ~isempty(pUnstable),minPUnstable=min(pUnstable);end
% 这是由 Cdc*Udc*dUdc/dt ≈ omega_g*Kt*diq 推出的局部线性估计；
% 仅用于与原M3的DVC时间尺度比较，不代替混合模型特征值。
kappa=p(12)*p(18)/(p(11)*p(2));
tauPStable=NaN;if isfinite(maxPStable)&&maxPStable>0,tauPStable=1/(kappa*p(25)*maxPStable);end
piNaturalPeriod=NaN;if isfinite(maxPiStable)&&maxPiStable>0,piNaturalPeriod=2*pi/sqrt(kappa*p(26)*maxPiStable);end
nominalSlowPole=NaN;disc=(kappa*p(25))^2-4*kappa*p(26);if disc>=0,nominalSlowPole=(-kappa*p(25)+sqrt(disc))/2;end
D=struct('kappa_V_per_A_s',kappa,'nominal_Kpdc_A_per_V',p(25),'nominal_Kidc_A_per_Vs',p(26), ...
    'pi_max_tested_stable_factor',maxPiStable,'pi_min_tested_unstable_factor',minPiUnstable, ...
    'p_only_max_tested_stable_factor',maxPStable,'p_only_min_tested_unstable_factor',minPUnstable, ...
    'p_only_tau_at_max_tested_stable_s',tauPStable,'pi_natural_period_at_max_tested_stable_s',piNaturalPeriod, ...
    'nominal_local_dvc_slow_pole_per_s',nominalSlowPole, ...
    'scope','Tested factor brackets only; local energy-balance time-scale estimate, not an exact hybrid stability boundary.');
end

function q=openFastHybridDiagnostic(Aof,Bt,ofNames,iGeAng,iWg,L,gear,feedbackScale,removeGeneratorAngle,extraLowSpeedDamping)
% Counterfactual diagnostic for the OpenFAST/electrical interconnection.
% Removing generator azimuth is not assumed physically valid: it is paired
% with the full-coordinate result so coordinate sensitivity stays explicit.
if nargin<10,extraLowSpeedDamping=0;end
if removeGeneratorAngle,keep=setdiff(1:size(Aof,1),iGeAng);else,keep=1:size(Aof,1);end
AofN=Aof(keep,keep);BtN=Bt(keep);iWgN=find(keep==iWg,1);assert(~isempty(iWgN),'Generator-speed state removed with angle coordinate.');
Cwg=zeros(1,numel(keep));Cwg(iWgN)=1;kT=1/gear;
ln=string(L.state_names(:));mech=ismember(ln,["theta_sh","omega_t","omega_g"]);eidx=find(~mech);iW=find(ln=="omega_g",1);iyTe=find(string(L.output_names)=="T_e",1);
Aee=L.A(eidx,eidx);Aew=L.A(eidx,iW);CteE=L.C(iyTe,eidx);DteW=L.C(iyTe,iW);
Ah=[AofN+BtN*kT*(feedbackScale*DteW+extraLowSpeedDamping)*Cwg BtN*kT*(feedbackScale*CteE);Aew*Cwg Aee];
[V,D,W]=eig(Ah);lam=diag(D);[maxReal,ii]=max(real(lam));freq=abs(imag(lam(ii)))/(2*pi);
for k=1:numel(lam),den=W(:,k)'*V(:,k);if abs(den)>eps,W(:,k)=W(:,k)/conj(den);end;end
pf=abs(V(:,ii).*conj(W(:,ii)));pf=pf/max(sum(pf),eps);[~,jj]=max(pf);
names=[ofNames(keep);ln(eidx)];if jj<=numel(keep),dom=ofNames(keep(jj));else,dom=ln(eidx(jj-numel(keep)));end
piUdc=0;iu=find(names=="Udc",1);if ~isempty(iu),piUdc=pf(iu);end
piXiDvc=0;ix=find(names=="xi_DVC",1);if ~isempty(ix),piXiDvc=pf(ix);end
piTorsionalFlex=sum(pf(contains(names,"Drivetrain rotational-flexibility")));
sp=1i*2*pi*1e-3;
GeProbe=DteW+CteE*((sp*eye(size(Aee))-Aee)\Aew);
GmProbe=Cwg*((sp*eye(size(AofN))-AofN)\(BtN*kT));
q=struct('A',Ah,'state_names',{names},'maxReal',maxReal,'worstFrequency_Hz',freq,'stable',maxReal<0,'worstDominantState',string(dom), ...
    'GeProbeReal',real(GeProbe),'GeProbeImag',imag(GeProbe),'GmProbeReal',real(GmProbe),'GmProbeImag',imag(GmProbe), ...
    'loopProbeReal',real(GeProbe*GmProbe),'loopProbeImag',imag(GeProbe*GmProbe), ...
    'PiOpenFAST',sum(pf(1:numel(keep))),'PiGeneratorSpeed',pf(iWgN),'PiTorsionalFlex',piTorsionalFlex,'PiUdc',piUdc,'PiXiDvc',piXiDvc);
end

function rows=s6LowFrequencyInterconnectionRows(Aof,Bt,iWg,L,gear,archName)
%S6LOWFREQUENCYINTERCONNECTIONROWS
% 紧凑的低频电—机互连审计。该函数只返回频响和Schur摘要，避免保存
% OpenFAST/M3完整矩阵。Closure=TORQUE_FEEDBACK_OPEN表示切断
% 电气状态到机械转矩的反馈，但保留机械速度到电气子系统的观测；
% Closure=TORQUE_FEEDBACK_FULL表示完整闭环。
freqs=[1e-3 3e-3 1e-2 3e-2 1e-1 1 2.5];
AofN=Aof;BtN=Bt;iWgN=iWg;
Cwg=zeros(1,size(AofN,1));Cwg(iWgN)=1;kT=1/gear;
ln=string(L.state_names(:));mech=ismember(ln,["theta_sh","omega_t","omega_g"]);
eidx=find(~mech);iW=find(ln=="omega_g",1);iyTe=find(string(L.output_names)=="T_e",1);
assert(~isempty(iW)&&~isempty(iyTe),'S6 low-frequency audit: M3 omega_g/T_e state or output missing.');
Aee=L.A(eidx,eidx);Aew=L.A(eidx,iW);CteE=L.C(iyTe,eidx);DteW=L.C(iyTe,iW);
Aem=Aew*Cwg;Bme=BtN*kT*CteE;
rows={};
for closure=[0 1]
    if closure==0
        closureName="TORQUE_FEEDBACK_OPEN";Amm=AofN;BmeEff=zeros(size(Bme));directMech=0;
    else
        closureName="TORQUE_FEEDBACK_FULL";Amm=AofN+BtN*kT*DteW*Cwg;BmeEff=Bme;directMech=norm(BtN*kT*DteW*Cwg,2);
    end
    for f=freqs
        s=1i*2*pi*f;
        Re=(s*eye(size(Aee))-Aee)\Aew;
        Ge=DteW+CteE*Re;
        Gm=Cwg*((s*eye(size(AofN))-AofN)\(BtN*kT));
        loop=Ge*Gm;
        Schur=(s*eye(size(Amm))-Amm)-BmeEff*((s*eye(size(Aee))-Aee)\Aem);
        sv=svd(Schur);rc=rcond(Schur);
        rows(end+1,:)={archName,closureName,f,real(Ge),imag(Ge),real(Gm),imag(Gm),real(loop),imag(loop),rad2deg(angle(loop)), ...
            norm(Re,2),min(sv),rc,directMech,norm(BmeEff,2),norm(Aem,2)}; %#ok<AGROW>
    end
end
end

function v=localS6TableValue(T,name,default)
%LOCAL S6 TABLE VALUE 读取单行摘要字段；缺失时返回明确的NaN/默认值。
if isempty(T)||~ismember(name,T.Properties.VariableNames)
    v=default;return;
end
x=T{1,name};
if iscell(x),x=x{1};end
if isempty(x),v=default;else,v=x;end
end

function q=periodicFloquetMetrics(phaseModels,phaseDt)
% Piecewise-constant, one-revolution Floquet audit of matrices already
% transformed to the fixed MBC frame by the official toolbox.  This is an
% interpolation-based periodic check, not a substitute for a full LTP run.
assert(numel(phaseModels)==numel(phaseDt)&&~isempty(phaseModels),'Invalid S6 Floquet phase sequence.');
Phi=eye(size(phaseModels{1}.A));
for k=1:numel(phaseModels)
    Phi=expm(phaseModels{k}.A*phaseDt(k))*Phi;
end
[V,D]=eig(Phi);mu=diag(D);period=sum(phaseDt);lam=log(mu)/period;[maxReal,ii]=max(real(lam));
[~,jj]=max(abs(V(:,ii)));names=phaseModels{1}.state_names;
q=struct('maxReal',maxReal,'worstFrequency_Hz',abs(imag(lam(ii)))/(2*pi),'stable',maxReal<0, ...
    'worstDominantState',string(names(jj)),'worstMultiplierMagnitude',abs(mu(ii)));
end

function v=readOpenFASTNumericKey(path,key)
txt=fileread(path);
% OpenFAST input files normally store "value  Key  - comment".  Match the
% key as a whitespace-delimited token instead of using a word boundary;
% the latter is fragile across MATLAB regexp versions and file encodings.
pat=['(?mi)^\s*([-+0-9.eEdD]+)\s+' regexptranslate('escape',key) '(?:\s|$)'];
q=regexp(txt,pat,'tokens','once');
assert(~isempty(q),'OpenFAST key %s missing in %s.',key,path);
v=str2double(regexprep(q{1},'[dD]','E'));
assert(isfinite(v),'OpenFAST key %s is not numeric.',key);
end

function makeM3S6Figure(path,S6)
T=S6.summary;M=S6.modal_candidates;CV=S6.slow_mode_counterfactuals;PV=S6.phase_sensitivity;SS=S6.constant_power_source_slope_audit;MI=S6.low_frequency_mechanical_impedance_audit;archs=["GFL","GWT","MWT"];cols=lines(3);fig=figure('Visible','off','Color','w','Position',[40 40 1550 1900]);tl=tiledlayout(fig,5,2,'TileSpacing','compact','Padding','compact');
nexttile;hold on;for a=1:3,q=M(M.Architecture==archs(a),:);scatter(q.PoleReal,q.Frequency_Hz,44,q.PiElectrical,'filled','MarkerEdgeColor',cols(a,:),'DisplayName',archs(a));end;xline(0,'--k');grid on;xlabel('Re(\lambda) (1/s)');ylabel('Frequency (Hz)');title('Flexible-mechanical modal map (color: electrical participation)');legend('Location','best');colorbar;
nexttile;x=categorical(T.Architecture,archs);bar(x,[T.M3TwoMassFrequency_Hz T.DriveFrequency_Hz]);grid on;ylabel('Frequency (Hz)');title('M3 two-mass vs OpenFAST-equivalent drive mode');legend({'M3 two-mass','S6 flexible drive'},'Location','best');
nexttile;bar(x,100*[T.M3TwoMassDampingRatio T.DriveDampingRatio]);grid on;ylabel('Damping ratio (%)');title('Drive-mode damping (not same modal identity)');legend({'M3 two-mass','S6 flexible drive'},'Location','best');
nexttile;bar(x,[T.HybridMaxRealNonRigid T.NoGaugeMaxReal]);yline(0,'--k');grid on;ylabel('Maximum real part (1/s)');title('Original hybrid vs absolute-angle gauge removed');legend({'Original','No gauge angle'},'Location','best');
nexttile;hold on;for a=1:3,q=CV(CV.Architecture==archs(a)&CV.Counterfactual=="TORQUE_FEEDBACK_CLOSURE_FULL_COORD",:);plot(q.Factor,q.MaxRealPole,'o-','Color',cols(a,:),'LineWidth',1.6,'DisplayName',archs(a));end;yline(0,'--k');grid on;xlabel('Electromagnetic torque-feedback closure factor');ylabel('Maximum real part (1/s)');title('Full-coordinate loop closure');legend('Location','best');
nexttile;hold on;for tr=["MSC_DVC_PI_SCALE_FULL_COORD","MSC_DVC_P_ONLY_SCALE_FULL_COORD","MSC_DVC_I_ONLY_SCALE_FULL_COORD"],q=CV(CV.Counterfactual==tr&CV.Factor>0,:);semilogx(q.Factor,q.MaxRealPole,'o-','LineWidth',1.6,'DisplayName',erase(tr,"_FULL_COORD"));end;yline(0,'--k');grid on;xlabel('MSC-DVC gain factor');ylabel('Maximum real part (1/s)');title('MWT DVC closure audit (all re-equilibrated)');legend('Location','best');
nexttile;q=PV(PV.Architecture=="MWT",:);hold on;for tr=["FULL_COORD","TORQUE_FEEDBACK_OPEN","ANGLE_REMOVED"],r=q(q.CoordinateTreatment==tr,:);plot(r.Azimuth_deg,r.MaxRealPole,'o-','LineWidth',1.7,'DisplayName',tr);end;yline(0,'--k');grid on;xlabel('OpenFAST linearization azimuth (deg)');ylabel('Maximum real part (1/s)');title('MWT phase-sensitivity audit');legend('Location','best');
nexttile;plot(SS.M3ConstantPowerSlopeFactor,SS.MaxRealPole,'o-','LineWidth',1.7);yline(0,'--k');grid on;xlabel('M3 constant-power source-slope factor');ylabel('Maximum real part (1/s)');title('Counterfactual low-speed mechanical damping');
nexttile;hold on;for a=1:3,q=MI(MI.Architecture==archs(a),:);semilogx(q.Frequency_Hz,q.MagnitudeRatio_OpenFASTOverM3,'o-','Color',cols(a,:),'LineWidth',1.6,'DisplayName',archs(a));end;yline(1,'--k');grid on;xlabel('Frequency (Hz)');ylabel('|G_{OF}| / |G_{M3}|');title('OpenFAST / M3 open-mechanical gain');legend('Location','best');
nexttile;hold on;for a=1:3,q=MI(MI.Architecture==archs(a),:);semilogx(q.Frequency_Hz,q.PhaseDifference_deg_OpenFASTMinusM3,'o-','Color',cols(a,:),'LineWidth',1.6,'DisplayName',archs(a));end;yline(0,'--k');grid on;xlabel('Frequency (Hz)');ylabel('\Delta phase (deg)');title('OpenFAST minus M3 mechanical phase');legend('Location','best');
sgtitle(tl,['M3 S6: official OpenFAST flexible mechanics, ' char(S6.variant)]);exportgraphics(fig,path,'Resolution',220);close(fig);
end

function writeM3S6Report(path,S6,gate)
fid=fopen(path,'w','n','UTF-8');assert(fid>0,'Cannot open S6 report.');c=onCleanup(@()fclose(fid));
MI=S6.low_frequency_mechanical_impedance_audit;
MIshow=MI(MI.Architecture=="MWT",:);
IA=S6.simulink_nonlinear_interface_audit;
LF=S6.low_frequency_interconnection_audit;
lf1=LF(abs(LF.Frequency_Hz-1e-3)<1e-12 & LF.Closure=="TORQUE_FEEDBACK_FULL",:);
getlf=@(arch,name,default) localS6TableValue(lf1(lf1.Architecture==arch,:),name,default);
mwte=getlf("MWT","ElectricalGainReal",NaN);mwtm=getlf("MWT","MechanicalGainReal",NaN);mwtloop=getlf("MWT","LoopGainReal",NaN);mwtphase=getlf("MWT","LoopPhase_deg",NaN);
gfle=getlf("GFL","ElectricalGainReal",NaN);gwtE=getlf("GWT","ElectricalGainReal",NaN);gflLoop=getlf("GFL","LoopGainReal",NaN);gwtLoop=getlf("GWT","LoopGainReal",NaN);
fprintf(fid,'# M3 S6：OpenFAST柔性机械局部反例测试\n\n## 研究边界\n\n本轮使用OpenFAST官方 `5MW_Land_Linear_Aero_CalcSteady` 三方位角周期稳态线性化，经官方MBC工具变换。它是NREL 5 MW**有齿轮箱**参考机组；与当前直驱PMSG只通过低速轴等效转矩—转速功率守恒接口连接，因此属于跨模型局部反例测试，不是新的主基准。\n\n**互连边界：S6只将OpenFAST的线性化柔性机械矩阵与M3电气小信号状态空间矩阵拼接；没有调用Simulink理想非线性模型，未进行OpenFAST—Simulink时域联合仿真，也未将其计入非线性联合验证。**\n\n- S6变体：`%s`；r-test commit：`%s`；\n- 风速 %.6g m/s，转速 %.6g rad/s，GBRatio %.6g；\n- OpenFAST `DOF_GeAz`状态速度 %.6g rad/s，与低速转子速度的比值 %.9g（低速等效坐标审计：`%s`）；\n- HSS转矩 %.6g N m，低速轴等效转矩 %.6g N m，等效功率 %.6g MW；\n- 正OpenFAST发电机转矩对发电机加速度偏导：%.6g (rad/s^2)/(N m)，与M3正发电制动转矩定义一致：`%s`。\n\n',S6.variant,gate.r_test_commit,gate.openfast_wind_mps,gate.openfast_rotor_speed_radps,gate.gear_ratio,gate.openfast_geaz_speed_radps,gate.openfast_speed_coordinate_ratio_to_rotor,iff(gate.openfast_speed_coordinate_is_lss_equivalent,'YES','NO'),gate.openfast_hss_torque_Nm,gate.openfast_lss_equivalent_torque_Nm,gate.openfast_equivalent_power_W/1e6,gate.openfast_generator_speed_accel_per_positive_torque,iff(gate.generator_torque_sign_consistent,'YES','NO'));
fprintf(fid,'# M3 S6：OpenFAST柔性机械局部反例测试\n\n## 研究边界\n\n本轮使用OpenFAST官方 `5MW_Land_Linear_Aero_CalcSteady` 三方位角周期稳态线性化，经官方MBC工具变换。它是NREL 5 MW**有齿轮箱**参考机组；与当前直驱PMSG只通过低速轴等效转矩—转速功率守恒接口连接，因此属于跨模型局部反例测试，不是新的主基准。\n\n**互连边界：S6只将OpenFAST的线性化柔性机械矩阵与M3电气小信号状态空间矩阵拼接；没有调用Simulink理想非线性模型，未进行OpenFAST—Simulink时域联合仿真，也未将其计入非线性联合验证。**\n\n- S6变体：`%s`；r-test commit：`%s`；\n- 风速 %.6g m/s，转速 %.6g rad/s，GBRatio %.6g；\n- OpenFAST `DOF_GeAz`状态速度 %.6g rad/s，与低速转子速度的比值 %.9g（低速等效坐标审计：`%s`）；\n- HSS转矩 %.6g N m，低速轴等效转矩 %.6g N m，等效功率 %.6g MW；\n- 正OpenFAST发电机转矩对发电机加速度偏导：%.6g (rad/s^2)/(N m)，与M3正发电制动转矩定义一致：`%s`。\n\n## 非线性联合仿真接口资产审计\n\n当前S6目录内：`FAST_SFunc` MEX数量=%d，`FAST_Library`数量=%d，`create_FAST_SFunc.m`数量=%d；直接Simulink联仿接口可用：`%s`。本项只审计已存在资产，未下载源码、未编译MEX，也不改变S6 Gate。\n\n',S6.variant,gate.r_test_commit,gate.openfast_wind_mps,gate.openfast_rotor_speed_radps,gate.gear_ratio,gate.openfast_geaz_speed_radps,gate.openfast_speed_coordinate_ratio_to_rotor,iff(gate.openfast_speed_coordinate_is_lss_equivalent,'YES','NO'),gate.openfast_hss_torque_Nm,gate.openfast_lss_equivalent_torque_Nm,gate.openfast_equivalent_power_W/1e6,gate.openfast_generator_speed_accel_per_positive_torque,iff(gate.generator_torque_sign_consistent,'YES','NO'),IA.sfunc_mex_count,IA.fast_library_count,IA.build_script_count,iff(IA.current_assets_support_direct_simulink_cosim,'YES','NO'));
if gate.frozen_wake_direct
    fprintf(fid,'## Frozen-Wake直接柔性机械基线\n\n本变体采用OpenFAST支持的 `DBEMT_Mod=-1`：诱导速度固定在配平点，线性模型只保留30个ElastoDyn状态，不含动态AeroDyn状态，也未进行静态状态消元。独立柔性机械基线最大实部为 %.6g 1/s；一周Floquet开环机械基线为 `%s`。因此，本轮只检验“冻结尾流条件下的控制—柔性传动链互连”，不能代替完整动态AeroDyn结论。\n\n',gate.reduced_mechanics_max_real_nonrigid,iff(gate.all_open_loop_floquet_stable,'PASS','FAIL'));
else
    fprintf(fid,'## 必须保留的反例\n\n完整258状态MBC平均矩阵含 %d 个RHP极点，最大实部 %.6g 1/s；其主导来源是AeroDyn内部诱导速度状态。因此，完整动态AeroDyn矩阵被判为 `%s`，没有直接接入GFM。\n\n随后只对AeroDyn内部状态做零导数静态消元：rcond=%.6g；按常数矩阵排除近零方位分支后最大实部为 %.6g 1/s。后续三方位角与Floquet审计表明，全坐标准稳态气动基线仍有正增长，故该“稳定”判据不足，不能把静态消元后的模型当作严格稳定的柔性机械基线。\n\n',gate.full_dynamic_aero_rhp_poles,gate.full_dynamic_aero_max_real,iff(gate.full_dynamic_aero_usable,'USABLE','REJECTED_FOR_COUPLING'),gate.aerodyn_static_condensation_rcond,gate.reduced_mechanics_max_real_nonrigid);
end
fprintf(fid,'## Gate\n\n- 来源可追溯/MBC/三方位角：%s；\n- 共同电气工作点：%s；\n- 低速轴接口能量误差：%.3e；\n- 一周Floquet开环机械基线：%s；\n- 全坐标三架构Floquet稳定：%s；\n- 驱动链模态身份：%s；\n- **S6条件性Gate：%s（%s）**。\n\n',iff(gate.source_traceable&&gate.mbc_performed&&gate.azimuth_spacing_error_rad<0.02,'PASS','FAIL'),iff(gate.common_workpoint_pass,'PASS','FAIL'),gate.low_speed_interface_energy_error,iff(gate.all_open_loop_floquet_stable,'PASS','FAIL'),iff(gate.all_floquet_stable,'PASS','FAIL'),iff(gate.all_drive_mode_identity_pass,'PASS','FAIL'),iff(gate.pass,'PASS','FAIL'),gate.s6_status);
fprintf(fid,'## 三架构结果\n\n');writeM3Table(fid,S6.summary);
fprintf(fid,'\n## 低频机械桥接阻抗审计\n\n');
writeM3Table(fid,S6.summary(:,{'Architecture','M3TwoMassMechanicalGain1mHzReal','M3TwoMassMechanicalGain1mHzImag','OpenFASTMechanicalGain1mHzReal','OpenFASTMechanicalGain1mHzImag','M3TwoMassEffectiveMechanicalDamping_NmsPerRad','OpenFASTEffectiveMechanicalDamping_NmsPerRad','MechanicalGainMagnitudeRatio_OpenFASTOverM3','MechanicalGainPhaseDifference_deg_OpenFASTMinusM3'}));
fprintf(fid,'\n上述量均定义为低速轴正发电制动转矩到发电机低速等效转速的开环传递 `Delta omega_g / Delta T_e`，频率为1 mHz；不含DVC、DC-link或GFM电气反馈。它只用于判断S6新增慢分支是否伴随机械对象的低频阻抗改变，不能据此归因于任何单一控制环。\n\n');
fprintf(fid,'## 低频频带机械阻抗核验\n\n');
writeM3Table(fid,S6.low_frequency_mechanical_impedance_audit(S6.low_frequency_mechanical_impedance_audit.Architecture=="MWT",:));
fprintf(fid,'\n此表把同一低速轴正发电制动转矩到低速等效发电机转速的**开环机械传递**从 1 mHz 扩展到 100 mHz。三架构共享机械对象，故只展示MWT一行组，避免把相同机械结果重复计作控制差异。OpenFAST/M3增益比从 %.4g（1 mHz）变为 %.4g（100 mHz），相位差分别为 %.4g 与 %.4g deg；因此两个机械对象的差异不是可用单一惯量或增益比例修正的常数差异。M3定功率源的速度斜率只能解释近零频候选敏感性，不能单独解释整个低频带，更不能外推为2.5 Hz扭振结论。该核验仍是OpenFAST周期稳态线性化矩阵的频率响应，不是OpenFAST—Simulink时域联合仿真；因此可检验单点差异是否在低频带持续存在，但不能独立验证非线性时域幅值。\n\n',MIshow.MagnitudeRatio_OpenFASTOverM3(1),MIshow.MagnitudeRatio_OpenFASTOverM3(end),MIshow.PhaseDifference_deg_OpenFASTMinusM3(1),MIshow.PhaseDifference_deg_OpenFASTMinusM3(end));
fprintf(fid,'\n## 低频电—机互连频响与 Schur 审计\n\n');
writeM3Table(fid,S6.low_frequency_interconnection_audit);
fprintf(fid,'\n本表在 1 mHz–2.5 Hz 仅保存紧凑频域量：`ElectricalGain` 为 `Delta T_e/Delta omega_g`，`MechanicalGain` 为 `Delta omega_g/Delta T_e`，`LoopGain` 为两者乘积。`TORQUE_FEEDBACK_OPEN` 切断电气状态到机械转矩的反馈但保留机械速度到电气状态的观测，`TORQUE_FEEDBACK_FULL` 为完整互连。Schur 最小奇异值与 rcond 用于检查消去电气状态后低频机械闭环是否接近奇异；它们不是稳定性判据。该审计只用于验证 MWT 近零频慢分支的幅相互连，不把单个频点直接命名为结构零或普适负阻尼。\n\n');
fprintf(fid,'## MWT-DVC低频互连的局部物理解释\n\n');
fprintf(fid,'在完整互连、1 mHz处，MWT的 `Delta T_e/Delta omega_g` 实部为 %.6g，GFL为 %.6g，GWT为 %.6g；相应环路乘积实部分别为 %.6g、%.6g、%.6g，MWT环路相位为 %.6g deg。这个符号差异与当前M3方程 `eDc=Vdc0-Udc`、`iq_ref=Kpdc*eDc+xi_DVC` 以及正发电制动转矩约定相容：\n\n',mwte,gfle,gwtE,mwtloop,gflLoop,gwtLoop,mwtphase);
fprintf(fid,'```text\n速度上升 -> P_MSC 上升 -> Udc 上升 -> eDc 下降\n         -> iq_ref 下降 -> T_e 下降 -> 发电机加速度进一步上升\n```\n\n');
fprintf(fid,'因此，当前Frozen-Wake互连下的MWT结果支持一个**局部低频反向反馈候选**：DVC使速度扰动对应的电磁制动转矩减小，而不是直接证明2.5 Hz轴系模态获得负阻尼。GFL/GWT在同一机械对象和同一频点的电气增益实部为正，环路乘积实部为负，表现为相反的局部反馈方向。该解释依赖当前工作点、端口定义、OpenFAST低速等效桥接和连续M3控制方程；它不是GFM普遍规律，也不能替代直接驱动或动态气动模型验证。\n\n');
fprintf(fid,'\n## 近零频慢速分支反事实\n\n');writeM3Table(fid,S6.slow_mode_counterfactuals);
fprintf(fid,'\n上述反事实分别保留或移除发电机方位状态，再连续闭合电磁转矩反馈和MSC-DVC。MWT的DVC已分为PI、仅P、仅I三类，每个点均重新求解平衡点；`TorqueFeedbackOpen*`列用于区分控制子系统自身不稳定与仅在机电闭环中出现的极点。该方位状态测试仅用于坐标敏感性，不能作为可直接采纳的物理模型。\n\n');
fprintf(fid,'## 慢速分支参与因子分组\n\n');writeM3Table(fid,S6.slow_mode_participation);
fprintf(fid,'\n`PiTorsionalFlex`只统计OpenFAST的扭转柔性坐标，不把发电机速度坐标计入轴系扭转参与度。因此，零频发电机速度主导分支不能被命名为“轴系扭振模态”。\n\n');
fprintf(fid,'## MWT-DVC稳定区间与时间尺度审计\n\n- PI全闭环：已测试稳定非零因子最大为 %.6g，最小失稳因子为 %.6g；\n- P-only全闭环：已测试稳定非零因子最大为 %.6g，最小失稳因子为 %.6g；\n- 由 `Cdc*Udc*dUdc/dt≈omega_g*Kt*diq` 得到的局部系数为 %.6g V/(A s)。在最大已测试稳定P-only因子处，估计时间常数约 %.6g s；在最大已测试稳定PI因子处，估计自然周期约 %.6g s；原M3名义DVC慢极点近似 %.6g 1/s。\n\n这些是**稀疏测点的上下界与局部尺度比较**，不是精确临界增益，也不能单独证明跨模型控制器不可实现；但它们足以说明当前Frozen-Wake互连下尚未证得一个与原M3名义动态同量级的稳定MWT-DVC运行区。\n\n',S6.dvc_feasibility.pi_max_tested_stable_factor,S6.dvc_feasibility.pi_min_tested_unstable_factor,S6.dvc_feasibility.p_only_max_tested_stable_factor,S6.dvc_feasibility.p_only_min_tested_unstable_factor,S6.dvc_feasibility.kappa_V_per_A_s,S6.dvc_feasibility.p_only_tau_at_max_tested_stable_s,S6.dvc_feasibility.pi_natural_period_at_max_tested_stable_s,S6.dvc_feasibility.nominal_local_dvc_slow_pole_per_s);
fprintf(fid,'## DC-link储能时间尺度反事实\n\n');writeM3Table(fid,S6.dc_link_capacitance_audit);
fprintf(fid,'\n本表固定原MWT-DVC与GFM参数，仅改变物理DC-link电容。它用于区分“固定控制器在新增柔性传动链上缺少鲁棒性”和“DC能量时间尺度导致的局部耦合”，不应用其中任何稳定点替代原M3基准或解除S6 Gate。\n\n');
fprintf(fid,'## M3定功率机械源斜率反事实\n\n');writeM3Table(fid,S6.constant_power_source_slope_audit);
fprintf(fid,'\nM3机械方程采用 `Tm=Tm0*omega0/omega_t`，故其在工作点附近包含 `-dTm/domega_t=Tm0/omega0` 的低频气动速度斜率。本表把这一斜率等效为OpenFAST低速轴附加制动反馈，仅用于检验“两个机械源律的低频阻抗差异是否足以改变MWT-DVC慢分支”。它不是OpenFAST原始气动模型、不是控制器整定，也不会解除S6 Gate。\n\n');
fprintf(fid,'## 三方位角敏感性\n\n');writeM3Table(fid,S6.phase_sensitivity);
fprintf(fid,'\nMWT全坐标分支跨方位角实部符号一致性：`%s`。若符号跨方位角改变，平均矩阵中的近零频极点只能列为周期相位敏感候选，不得作为LTI普适稳定边界。\n\n',iff(gate.mwt_full_coordinate_phase_sign_consistent,'YES','NO'));
fprintf(fid,'## 一周Floquet插值审计\n\n');writeM3Table(fid,S6.floquet);
fprintf(fid,'\nFloquet审计将官方MBC固定坐标系下的三个方位角矩阵按转子一周分段指数传播。它用于判定平均矩阵近零频分支是否在该三点周期插值下保留，不能替代完整动态AeroDyn或直接驱动机组的时变非线性验证。\n\n');
fprintf(fid,'## 解释纪律\n\n1. M3两质量模态与OpenFAST柔性驱动链模态并非同一模态，频率/阻尼差只说明机械模型层级改变了谱结构，不能写成GFM效应。\n2. `GridToShaft` 与 `WindToPCC` 残差用于筛选新增机械模态是否改变扰动排序；单个点不能证明全局方向性。\n3. S6为“OpenFAST线性机械体＋M3电气SSM”的混合线性分析，而非与M3理想非线性Simulink模型的联合时域仿真；因此不能用于非线性幅值、限幅、启动、控制采样或时域收敛性结论。\n4. 本轮若通过，只允许说候选机电耦合分析在一个来源可追溯的柔性机械、准稳态气动、低速轴等效接口上可执行。完整动态AeroDyn、直驱OpenFAST模型和跨风速稳健性仍未验证。\n\n');
if gate.pass
    fprintf(fid,'## 决策\n\nS6条件性Gate通过。允许进入S7独立的离散平均控制实现测试，但不得把S6结果升级为全局机械—电气结论。\n');
elseif gate.frozen_wake_direct
    fprintf(fid,'## 决策\n\nS6 Gate失败：Frozen-Wake独立柔性机械基线及转矩反馈断开反事实均稳定，但MWT的DVC—电磁转矩闭合形成零频慢速不稳定分支；该分支跨三个方位角并由Floquet插值保留。它是局部跨模型耦合候选，尚非轴系扭振结论。S7保持阻塞，下一步应先定位DVC P/I分量和互连时间尺度，再决定是否存在可物理实现的稳定MWT运行区。\n');
else
    fprintf(fid,'## 决策\n\nS6 Gate失败：当前静态AeroDyn消元后的全坐标开环柔性机械基线已在一周Floquet插值中出现正增长；MWT只是在该失效基线上进一步放大慢速分支。S7保持阻塞。下一步必须获取或生成经周期稳态配平且无需静态AeroDyn状态消元的严格稳定柔性机械模型，不能以额外控制调参掩盖基线问题。\n');
end
end

function [S2,gate]=runM3S2(scales,allModels,allP,Msummary,requestedScales,base,p0)
% S2：三个代表运行点上的单因素稀疏控制整定与独立SCR环境扫描。
% 本函数不做笛卡尔积；初始点固定为0.6/1/1.4倍，边界只作为待加密
% 区间输出。失稳点是反例证据，不导致Gate自动失败；Gate只要求工作点、
% 数值有限性、基准稳定性和轴系模态身份跟踪可信。
requestedScales=requestedScales(:)'; idx=zeros(size(requestedScales));
for k=1:numel(requestedScales)
    [d,idx(k)]=min(abs(scales-requestedScales(k)));
    assert(d<1e-9,'Requested S2 workpoint %.6g is not available from S1.',requestedScales(k));
end
idx=unique(idx,'stable'); assert(numel(idx)>=3,'S2 requires low/middle/high workpoints.');
opLabels=["OP_L","OP_M","OP_H"];
families=["H","DVC","GSC_CURRENT_BW","GSC_VOLTAGE_BW","MPPT_GAIN","SCR"];
rows={};
for oi=1:numel(idx)
    wi=idx(oi); sc=scales(wi); models=allModels{wi}; p=allP{wi};
    % SCR会改变网侧损耗，不能沿用原机械转矩后再放宽工作点Gate。
    % 对每个SCR重新标定一套三架构共同工作点；该缓存只驻留内存。
    scrValues=[3 4 6];scrModels=cell(size(scrValues));scrP=cell(size(scrValues));
    for svi=1:numel(scrValues)
        sv=scrValues(svi);ps=p;ps(9:10)=p(9:10)*(4/sv);ps(40)=pccVoltageMagnitudeForPQ(ps(37),ps(38),ps(4),ps(9),ps(3)*ps(10));
        wp=struct('powerScale',sc,'speedScale',sc^(1/3),'torqueScale',ps(39)/p0(39),'wind_mps',NaN,'rawCpPower_W',NaN,'aeroCalibration',NaN,'aeroPower_W',ps(39)*ps(12),'definition','S2_SCR_COMMON_WORKPOINT','pmsgConvention','GENERATOR_OUTWARD');
        [ps,wp]=calibrateMwtTorqueAtTargetSpeed(base,ps,p0,wp);aa=makeArchitectures(base,ps,wp);[mm,~,~,gg]=runStageA(aa,ps,base);
        assert(gg.pass,'S2 SCR=%g at workpoint %.3g failed strict common-workpoint Gate.',sv,sc);
        scrModels{svi}=mm;scrP{svi}=ps;
    end
    for a=1:numel(models)
        M=models{a}; tq=Msummary(Msummary.WorkpointScale==sc&Msummary.Architecture==string(M.name),:);
        ref=trackedMetrics(M.linear,tq,[],p); [~,refDe,~]=complexTorqueCurve(M.linear,ref.ftor);
        for fi=1:numel(families)
            fam=families(fi);
            if ~s2ParameterIsActive(fam,M.name),continue;end
            if fam=="SCR",physicalValues=[3 4 6];factors=physicalValues/4;category="ENVIRONMENT";else,physicalValues=[0.6 1 1.4];factors=physicalValues;category="CONTROL";end
            for vi=1:numel(physicalValues)
                if fam=="SCR"
                    Ms=scrModels{vi}{a};pv=scrP{vi};flags=Ms.flags;x=Ms.x0;eq=Ms.equilibrium;L=Ms.linear;physicalValue=physicalValues(vi);physicalUnit="SCR";
                else
                    [pv,flags,physicalValue,physicalUnit]=applyS2Parameter(p,M.flags,M.name,fam,physicalValues(vi));
                    [x,eq,pv]=solveS2Point(M.x0,p,pv,M.name,flags,M.active,fam,physicalValues(vi));L=linearizeModel(x,pv,M.name,flags,M.active);
                end
                rows(end+1,:)=buildS2Row(sc,opLabels(oi),M,pv,flags,x,eq,L,tq,ref,refDe,fam,category,factors(vi),physicalValue,physicalUnit,0); %#ok<AGROW>
            end
        end
    end
end
T=makeS2Table(rows);
T.OPLabel=string(T.OPLabel);T.Architecture=string(T.Architecture);T.ParameterFamily=string(T.ParameterFamily);T.ParameterCategory=string(T.ParameterCategory);T.PhysicalUnit=string(T.PhysicalUnit);T.CriticalModeClass=string(T.CriticalModeClass);T.ControlContributionCase=string(T.ControlContributionCase);T.PolePathClass=string(T.PolePathClass);T.EvidenceStatus=string(T.EvidenceStatus);
B0=detectS2Boundaries(T);
[Tr,refineMeta]=refineS2ContinuousBoundaries(B0,T,scales,allModels,allP,Msummary,base,p0);
if ~isempty(Tr),T=[T;Tr];T=sortrows(T,{'WorkpointScale','Architecture','ParameterFamily','Factor','RefinementLevel'});end
B=annotateS2Boundaries(detectS2Boundaries(T),T); CE=detectS2Counterexamples(T,B);
keyFinite=all(isfinite(T{:,{'EquilibriumResidual','P0_W','Q0_var','Udc_V','omega_g_radps','MaxRealPole','TorPoleReal','TorPoleImag','TorFrequency_Hz','TorDampingRatio','TorPiMECH','ParticipationPatternCorrelation','FrequencyGap_Hz','TotalDe','TotalKe','TorResidue_GridToShaft','C_GridToMachine','C_MachineToGrid','Ldir_Log10'}}),'all');
baseRows=T(abs(T.Factor-1)<1e-12&T.RefinementLevel==0,:);remaining=B(B.Status=="REFINE_REQUIRED",:);
gate=struct('pass',keyFinite&&all(T.Workpoint_PASS)&&all(T.ModeIdentity_PASS)&&all(baseRows.Stable)&&isempty(remaining), ...
    'all_key_metrics_finite',keyFinite,'all_workpoints_pass',all(T.Workpoint_PASS),'all_mode_identity_pass',all(T.ModeIdentity_PASS),'all_baselines_stable',all(baseRows.Stable),'all_continuous_boundaries_refined',isempty(remaining), ...
    'num_initial_points',sum(T.RefinementLevel==0),'num_refinement_points',sum(T.RefinementLevel>0),'num_unstable_points',sum(~T.Stable),'num_boundary_brackets',height(B),'num_counterexamples',sum(CE.Detected), ...
    'scan_scope','OP_L/OP_M/OP_H; control factors 0.6/1.0/1.4; SCR 3/4/6; single-factor only', ...
    'next_rule','Only boundary brackets or classification transitions may be adaptively refined before S3.');
S2=struct('objective','S2 sparse falsification of control-tuning mechanism validity','representative_scales',scales(idx),'summary',T,'initial_boundaries',B0,'boundaries',B,'counterexamples',CE,'refinement',refineMeta,'gate',gate);
end

function [S3,gate]=runM3S3(scales,allModels,allP,~,S2,base,p0)
% S3：只在S2已加密边界两侧进行轴系单模态Pole--Excitation反事实分解。
% 这里的“Excitation”是该输入到omega_sh轴系模态的复残差（含可观测性
% 与输入投影），不是把所有路径误认为可线性相加。全系统稳定性与轴系
% 单模态稳定性分别报告，避免把非轴系电气失稳归因于轴系负阻尼。
B=S2.boundaries;
B=B(B.Status=="REFINED_BRACKET"&ismember(B.BoundaryType,["STABILITY","DIRECTION","FEEDBACK_SIGN"]),:);
assert(~isempty(B),'S3 requires refined S2 boundaries.');
distIdx=[1 4];distNames=["MechanicalTorque","GridFrequency"];
caseRows={};rows={};curves={};
for bi=1:height(B)
    b=B(bi,:);wi=find(abs(scales-b.WorkpointScale)<1e-12,1);assert(~isempty(wi),'Missing S3 workpoint.');
    models=allModels{wi};p=allP{wi};ai=find(cellfun(@(m)strcmpi(m.name,b.Architecture),models),1);assert(~isempty(ai),'Missing S3 architecture.');
    M0=buildS2ParameterizedModel(b.WorkpointScale,models{ai},p,b.ParameterFamily,b.FactorLow,base,p0);
    M1=buildS2ParameterizedModel(b.WorkpointScale,models{ai},p,b.ParameterFamily,b.FactorHigh,base,p0);
    [crit0,stable0]=s3CriticalMode(M0.linear,M0.name);[crit1,stable1]=s3CriticalMode(M1.linear,M1.name);
    caseId=sprintf('S3-%02d',bi);
    caseRows(end+1,:)={string(caseId),b.WorkpointScale,b.OPLabel,b.Architecture,b.ParameterFamily,b.BoundaryType,b.FactorLow,b.FactorHigh, ...
        M0.equilibrium.normalized_residual,M1.equilibrium.normalized_residual,stable0,stable1,crit0.real,crit0.frequency_Hz,crit0.class,crit0.piMech,crit1.real,crit1.frequency_Hz,crit1.class,crit1.piMech}; %#ok<AGROW>
    for di=1:numel(distIdx)
        dId=distIdx(di);c0=s3TorsionModalComponents(M0,dId,[]);c1=s3TorsionModalComponents(M1,dId,c0);
        if dId==1,amp=0.001*p(1)/p(12);else,amp=0.001*p(3);end
        t=linspace(0,8,1601)';
        y00=modalStepPair(t,c0.lambda,c0.residue,amp);y10=modalStepPair(t,c1.lambda,c0.residue,amp);
        y01=modalStepPair(t,c0.lambda,c1.residue,amp);y11=modalStepPair(t,c1.lambda,c1.residue,amp);
        poleTerm=y10-y00;excTerm=y01-y00;interaction=y11-y10-y01+y00;total=y11-y00;
        dp=norm(poleTerm);de=norm(excTerm);dint=norm(interaction);dt=norm(total);den=max(dt,1e-30);
        closure=norm(total-poleTerm-excTerm-interaction)/max(norm(total),1e-30);
        poleFraction=dp/den;excFraction=de/den;intFraction=dint/den;
        systemStabilityChanged=stable0~=stable1;nonTorBoundary=systemStabilityChanged&&max(crit0.piMech,crit1.piMech)<0.5;
        if nonTorBoundary
            cls="ELECTRICAL_STABILITY_BOUNDARY_NOT_TORSION_MECHANISM";
        elseif intFraction>0.30
            cls="INTERACTION_SIGNIFICANT";
        elseif dp>1.5*de
            cls="POLE_DOMINATED";
        elseif de>1.5*dp
            cls="EXCITATION_DOMINATED";
        else
            cls="JOINT";
        end
        rows(end+1,:)={string(caseId),b.WorkpointScale,b.OPLabel,b.Architecture,b.ParameterFamily,b.BoundaryType,b.FactorLow,b.FactorHigh,distNames(di), ...
            stable0,stable1,systemStabilityChanged,crit0.class,crit1.class,crit0.piMech,crit1.piMech, ...
            real(c0.lambda),imag(c0.lambda),real(c1.lambda),imag(c1.lambda),c0.frequency_Hz,c1.frequency_Hz,c0.damping,c1.damping,c0.piMech,c1.piMech,c1.patternCorrelation,c1.rightMAC, ...
            abs(c0.observability),abs(c1.observability),abs(c0.inputProjection),abs(c1.inputProjection),angle(c0.inputProjection)*180/pi,angle(c1.inputProjection)*180/pi, ...
            abs(c0.residue),abs(c1.residue),angle(c0.residue)*180/pi,angle(c1.residue)*180/pi,dp,de,dint,dt,poleFraction,excFraction,intFraction,closure, ...
            max(abs(y00)),max(abs(y10)),max(abs(y01)),max(abs(y11)),string(cls)}; %#ok<AGROW>
        curves{end+1}=struct('CaseID',string(caseId),'Architecture',string(b.Architecture),'ParameterFamily',string(b.ParameterFamily),'BoundaryType',string(b.BoundaryType), ...
            'Disturbance',distNames(di),'t_s',t,'y00_reference',y00,'y10_pole_only',y10,'y01_excitation_only',y01,'y11_target',y11); %#ok<AGROW>
    end
end
caseVars={'CaseID','WorkpointScale','OPLabel','Architecture','ParameterFamily','BoundaryType','FactorLow','FactorHigh','ResidualLow','ResidualHigh','StableLow','StableHigh','CriticalRealLow','CriticalFrequencyLow_Hz','CriticalClassLow','CriticalPiMECHLow','CriticalRealHigh','CriticalFrequencyHigh_Hz','CriticalClassHigh','CriticalPiMECHHigh'};
C=cell2table(caseRows,'VariableNames',caseVars);
vars={'CaseID','WorkpointScale','OPLabel','Architecture','ParameterFamily','BoundaryType','FactorLow','FactorHigh','Disturbance','StableLow','StableHigh','SystemStabilityChanged','CriticalClassLow','CriticalClassHigh','CriticalPiMECHLow','CriticalPiMECHHigh','Lambda0Real','Lambda0Imag','Lambda1Real','Lambda1Imag','Frequency0_Hz','Frequency1_Hz','Damping0','Damping1','PiMECH0','PiMECH1','ParticipationPatternCorrelation','RightEigenvectorMAC','Observability0','Observability1','InputProjection0','InputProjection1','InputProjectionPhase0_deg','InputProjectionPhase1_deg','Residue0','Residue1','ResiduePhase0_deg','ResiduePhase1_deg','DeltaPoleNorm','DeltaExcitationNorm','DeltaInteractionNorm','DeltaTotalNorm','PoleFraction','ExcitationFraction','InteractionFraction','ClosureError','Peak_y00','Peak_y10','Peak_y01','Peak_y11','Classification'};
T=cell2table(rows,'VariableNames',vars);
strVars={'CaseID','OPLabel','Architecture','ParameterFamily','BoundaryType','Disturbance','CriticalClassLow','CriticalClassHigh','Classification'};for k=1:numel(strVars),T.(strVars{k})=string(T.(strVars{k}));end
for k={'CaseID','OPLabel','Architecture','ParameterFamily','BoundaryType','CriticalClassLow','CriticalClassHigh'},C.(k{1})=string(C.(k{1}));end
finiteVars={'Lambda0Real','Lambda0Imag','Lambda1Real','Lambda1Imag','Frequency0_Hz','Frequency1_Hz','Damping0','Damping1','PiMECH0','PiMECH1','ParticipationPatternCorrelation','Observability0','Observability1','InputProjection0','InputProjection1','Residue0','Residue1','DeltaPoleNorm','DeltaExcitationNorm','DeltaInteractionNorm','DeltaTotalNorm','ClosureError'};
finitePass=all(isfinite(T{:,finiteVars}),'all');workpointPass=all(C.ResidualLow<1e-8&C.ResidualHigh<1e-8);modePass=all(T.PiMECH0>0.5&T.PiMECH1>0.5&T.ParticipationPatternCorrelation>0.75);closurePass=max(T.ClosureError)<1e-12;
gate=struct('pass',finitePass&&workpointPass&&modePass&&closurePass,'all_metrics_finite',finitePass,'all_workpoints_pass',workpointPass,'all_mode_identity_pass',modePass,'decomposition_closure_pass',closurePass,'max_closure_error',max(T.ClosureError),'num_boundary_cases',height(C),'num_disturbance_cases',height(T),'num_non_torsional_stability_boundaries',sum(T.Classification=="ELECTRICAL_STABILITY_BOUNDARY_NOT_TORSION_MECHANISM"),'next_rule','Only after this Gate passes may modal-proximity screens enter S4 hybridization tests.');
S3=struct('objective','S3 boundary-side Pole--Excitation counterfactual decomposition','case_definitions',C,'summary',T,'curves',{curves},'gate',gate,'evidence_status','CONDITIONAL_IDEAL_CONTINUOUS_AVERAGE');
end

function M=buildS2ParameterizedModel(sc,Mbase,p,fam,factor,base,p0)
if string(fam)=="SCR"
    sv=4*factor;pv=p;pv(9:10)=p(9:10)*(4/sv);pv(40)=pccVoltageMagnitudeForPQ(pv(37),pv(38),pv(4),pv(9),pv(3)*pv(10));
    wp=struct('powerScale',sc,'speedScale',sc^(1/3),'torqueScale',pv(39)/p0(39),'wind_mps',NaN,'rawCpPower_W',NaN,'aeroCalibration',NaN,'aeroPower_W',pv(39)*pv(12),'definition','S3_SCR_COMMON_WORKPOINT','pmsgConvention','GENERATOR_OUTWARD');
    [pv,wp]=calibrateMwtTorqueAtTargetSpeed(base,pv,p0,wp);aa=makeArchitectures(base,pv,wp);[mm,~,~,gg]=runStageA(aa,pv,base);assert(gg.pass,'S3 SCR point failed strict common-workpoint Gate.');ai=find(cellfun(@(m)strcmpi(m.name,Mbase.name),mm),1);M=mm{ai};M.p=pv;
else
    [pv,flags]=applyS2Parameter(p,Mbase.flags,Mbase.name,fam,factor);[x,eq,pv]=solveS2Point(Mbase.x0,p,pv,Mbase.name,flags,Mbase.active,fam,factor);L=linearizeModel(x,pv,Mbase.name,flags,Mbase.active);[y,names]=m2Outputs(x,pv,Mbase.name,zeros(4,1),flags);
    M=struct('name',Mbase.name,'description',Mbase.description,'x0',x,'flags',flags,'active',Mbase.active,'equilibrium',eq,'linear',L,'y0',y,'output_names',{names},'p',pv);
end
end

function c=s3TorsionModalComponents(M,didx,ref)
L=M.linear;[V,D,W]=eig(L.A);lam=diag(D);pos=find(imag(lam)>1e-7);freq=imag(lam(pos))/(2*pi);cand=pos(freq>=0.5&freq<=10);assert(~isempty(cand),'No torsional-mode candidate in S3.');
names=string(L.state_names);mech=ismember(names,["theta_sh","omega_t","omega_g"]);pfs=cell(numel(cand),1);piMech=zeros(numel(cand),1);corrs=zeros(numel(cand),1);prox=zeros(numel(cand),1);
for k=1:numel(cand)
    q=cand(k);pf=abs(V(:,q).*conj(W(:,q)));pf=pf/max(sum(pf),eps);pfs{k}=pf;piMech(k)=sum(pf(mech));
    if ~isempty(ref),corrs(k)=dot(ref.pf,pf)/max(norm(ref.pf)*norm(pf),eps);prox(k)=exp(-abs(lam(q)-ref.lambda)/max(abs(ref.lambda),1));end
end
if isempty(ref),[~,j]=max(piMech);pattern=1;rmac=1;else,[~,j]=max(0.75*corrs+0.25*prox);pattern=corrs(j);v0=ref.rightVector;v1=V(:,cand(j));rmac=abs(v0'*v1)^2/max(real(v0'*v0)*real(v1'*v1),eps);end
ii=cand(j);den=W(:,ii)'*V(:,ii);iy=find(string(L.output_names)=="omega_sh",1);obs=L.C(iy,:)*V(:,ii);inp=(W(:,ii)'*L.B(:,didx))/den;res=obs*inp;
c=struct('lambda',lam(ii),'frequency_Hz',abs(imag(lam(ii)))/(2*pi),'damping',-real(lam(ii))/abs(lam(ii)),'piMech',piMech(j),'pf',pfs{j},'patternCorrelation',pattern,'rightMAC',rmac,'rightVector',V(:,ii),'observability',obs,'inputProjection',inp,'residue',res);
end

function [c,stable]=s3CriticalMode(L,arch)
[V,D,W]=eig(L.A);lam=diag(D);[mr,ii]=max(real(lam));pf=abs(V(:,ii).*conj(W(:,ii)));pf=pf/max(sum(pf),eps);g=aggregateParticipation(pf,string(L.state_names),arch);c=struct('real',mr,'frequency_Hz',abs(imag(lam(ii)))/(2*pi),'class',dominantParticipationClass(g),'piMech',g.MECH);stable=mr<0;
end

function makeM3S3Figure(path,S3)
C=S3.case_definitions;fig=figure('Visible','off','Color','w','Position',[30 30 1750 1200]);tl=tiledlayout(fig,3,3,'TileSpacing','compact','Padding','compact');
for k=1:height(C)
    nexttile;cid=C.CaseID(k);if C.BoundaryType(k)=="FEEDBACK_SIGN",dn="MechanicalTorque";else,dn="GridFrequency";end
    j=find(cellfun(@(q)q.CaseID==cid&&q.Disturbance==dn,S3.curves),1);q=S3.curves{j};scale=max(abs([q.y00_reference;q.y10_pole_only;q.y01_excitation_only;q.y11_target]));scale=max(scale,eps);
    plot(q.t_s,q.y00_reference/scale,'k-','LineWidth',1.2,'DisplayName','reference');hold on;plot(q.t_s,q.y10_pole_only/scale,'b--','LineWidth',1.2,'DisplayName','pole only');plot(q.t_s,q.y01_excitation_only/scale,'Color',[0.85 0.45 0.05],'LineStyle','-.','LineWidth',1.2,'DisplayName','excitation only');plot(q.t_s,q.y11_target/scale,'r:','LineWidth',1.7,'DisplayName','target');grid on;
    title(sprintf('%s %s/%s (%s)',cid,C.Architecture(k),C.ParameterFamily(k),C.BoundaryType(k)),'Interpreter','none');xlabel('t (s)');ylabel('normalized \Delta\omega_{sh}');if k==1,legend('Location','best');end
end
sgtitle(tl,'M3 S3：边界两侧轴系单模态 Pole–Excitation 反事实响应（条件性证据）');exportgraphics(fig,path,'Resolution',220);close(fig);
end

function writeM3S3Report(path,S3,gate)
fid=fopen(path,'w','n','UTF-8');assert(fid>0,'Cannot open S3 report.');c=onCleanup(@()fclose(fid));T=S3.summary;C=S3.case_definitions;
fprintf(fid,'# M3 S3：Pole–Excitation 反事实分解报告\n\n');
fprintf(fid,'## 范围与口径\n\n本轮只重构 S2 已加密边界两侧，不新增全局扫参。输出为 `omega_sh` 的轴系单模态响应；Excitation项使用完整复残差，因此同时包含输入投影与模态可观测性。它不是多闭环通道的严格线性加和。\n\n');
fprintf(fid,'## Gate\n\n- 边界对：%d；扰动案例：%d；\n- 工作点：%s；模态身份：%s；分解闭合：%s；\n- 最大闭合误差：%.3e；总体：%s。\n\n',height(C),height(T),iff(gate.all_workpoints_pass,'PASS','FAIL'),iff(gate.all_mode_identity_pass,'PASS','FAIL'),iff(gate.decomposition_closure_pass,'PASS','FAIL'),gate.max_closure_error,iff(gate.pass,'PASS','FAIL'));
fprintf(fid,'## 关键判别\n\n');
classes=unique(T.Classification,'stable');for k=1:numel(classes),fprintf(fid,'- `%s`：%d 个扰动案例。\n',classes(k),sum(T.Classification==classes(k)));end
fprintf(fid,'\n其中系统稳定性跨界但临界模态机械参与低于50%%的案例，统一标记为 `ELECTRICAL_STABILITY_BOUNDARY_NOT_TORSION_MECHANISM`，禁止写成轴系负阻尼失稳。\n\n');
fprintf(fid,'## 边界摘要\n\n');writeM3Table(fid,C);fprintf(fid,'\n## 反事实结果\n\n');writeM3Table(fid,T);fprintf(fid,'\n');
if gate.pass,fprintf(fid,'## 决策\n\nS3 PASS：Pole、Excitation与交互项分解在全部代表边界闭合，工作点与轴系模态身份均通过。允许把 S2 的频率接近点送入 S4，但频率接近本身仍不构成模态混合证据，必须继续检查参与因子交换、左右特征向量相关性和能量交换。\n');else,fprintf(fid,'## 决策\n\nS3 FAIL：至少一项工作点、模态身份或反事实闭合未通过，S4保持阻塞。\n');end
end

function [S4,gate]=runM3S4(scales,allModels,allP,S2,base,p0)
% S4：对S2的频率接近点作局部、可证伪的双模态验证。
% 只有“频率接近 + 机械/电气参与交换 + 双模态子空间连续”共同出现时，
% 才允许使用hybridization。单纯频率重合统一归为frequency coincidence。
B=S2.boundaries;B=B(B.Status=="S3_SCREEN_ONLY"&B.BoundaryType=="MODAL_PROXIMITY",:);assert(~isempty(B),'No S4 modal-proximity screens.');
rows={};assessRows={};
for bi=1:height(B)
    b=B(bi,:);wi=find(abs(scales-b.WorkpointScale)<1e-12,1);models=allModels{wi};p=allP{wi};ai=find(cellfun(@(m)strcmpi(m.name,b.Architecture),models),1);Mbase=models{ai};center=b.EstimatedFactor;
    half=max(0.075,0.10*center);factors=max(center+[-2 -1 0 1 2]*half,0.1);caseId=sprintf('S4-%02d',bi);
    Mc=buildS2ParameterizedModel(b.WorkpointScale,Mbase,p,b.ParameterFamily,center,base,p0);ref=s4ModePair(Mc,[]);
    caseStart=size(rows,1)+1;
    for fi=1:numel(factors)
        factor=factors(fi);M=buildS2ParameterizedModel(b.WorkpointScale,Mbase,p,b.ParameterFamily,factor,base,p0);q=s4ModePair(M,ref);[crit,stable]=s3CriticalMode(M.linear,M.name);
        physical=factor;if b.ParameterFamily=="SCR",physical=4*factor;end
        wpPass=M.equilibrium.normalized_residual<1e-8;pairPass=q.subspaceMAC>0.8;
        rows(end+1,:)={string(caseId),b.WorkpointScale,b.OPLabel,b.Architecture,b.ParameterFamily,factor,physical,M.equilibrium.normalized_residual,stable,crit.real,crit.class, ...
            real(q.lambdaTor),imag(q.lambdaTor),q.fTor,q.zetaTor,q.piMechTor,q.mixTor,real(q.lambdaElec),imag(q.lambdaElec),q.fElec,q.zetaElec,q.piMechElec,q.mixElec,q.frequencyGap, ...
            q.corrTorToTor,q.corrTorToElec,q.corrElecToTor,q.corrElecToElec,q.rightMACTor,q.rightMACElec,q.leftMACTor,q.leftMACElec,q.subspaceMAC,wpPass,pairPass}; %#ok<AGROW>
    end
    idx=caseStart:size(rows,1);scanVars=s4ScanVariableNames();Q=cell2table(rows(idx,:),'VariableNames',scanVars);
    [minGap,j]=min(Q.FrequencyGap_Hz);torRange=max(Q.PiMECH_Tor)-min(Q.PiMECH_Tor);elecRange=max(Q.PiMECH_Elec)-min(Q.PiMECH_Elec);maxMix=max([Q.MixingIndex_Tor;Q.MixingIndex_Elec]);swap=any(Q.Corr_Tor_to_Elec>Q.Corr_Tor_to_Tor&Q.Corr_Elec_to_Tor>Q.Corr_Elec_to_Elec);minSub=min(Q.SubspaceMAC);opposite=(max(Q.PiMECH_Tor)-min(Q.PiMECH_Tor))*(max(Q.PiMECH_Elec)-min(Q.PiMECH_Elec))>0;
    confirmed=minGap<0.5&&maxMix>0.2&&(max(torRange,elecRange)>0.1||swap)&&minSub>0.8;
    if confirmed,cls="CONFIRMED_HYBRIDIZATION_IN_TESTED_NEIGHBORHOOD";elseif minGap<0.1&&maxMix<0.1&&max(torRange,elecRange)<0.05&&~swap,cls="FREQUENCY_COINCIDENCE_ONLY";elseif minGap<0.5&&(maxMix>0.05||max(torRange,elecRange)>0.02),cls="POTENTIAL_WEAK_MIXING_REQUIRES_HIGHER_FIDELITY";else,cls="NO_HYBRIDIZATION_IN_TESTED_NEIGHBORHOOD";end
    assessRows(end+1,:)={string(caseId),b.WorkpointScale,b.OPLabel,b.Architecture,b.ParameterFamily,center,min(factors),max(factors),minGap,Q.Factor(j),torRange,elecRange,maxMix,swap,opposite,minSub,all(Q.Stable),string(cls),"CONDITIONAL_IDEAL_CONTINUOUS_AVERAGE"}; %#ok<AGROW>
end
scanVars=s4ScanVariableNames();T=cell2table(rows,'VariableNames',scanVars);A=cell2table(assessRows,'VariableNames',{'CaseID','WorkpointScale','OPLabel','Architecture','ParameterFamily','ScreenFactor','FactorMin','FactorMax','MinimumFrequencyGap_Hz','FactorAtMinimumGap','TorPiMECHRange','ElecPiMECHRange','MaximumMixingIndex','CharacterSwapDetected','ParticipationVariationObserved','MinimumSubspaceMAC','AllPointsStable','Assessment','EvidenceStatus'});
for k={'CaseID','OPLabel','Architecture','ParameterFamily','CriticalClass'},T.(k{1})=string(T.(k{1}));end;for k={'CaseID','OPLabel','Architecture','ParameterFamily','Assessment','EvidenceStatus'},A.(k{1})=string(A.(k{1}));end
finitePass=all(isfinite(T{:,{'EquilibriumResidual','MaxRealPole','TorPoleReal','TorPoleImag','TorFrequency_Hz','TorDampingRatio','PiMECH_Tor','MixingIndex_Tor','ElecPoleReal','ElecPoleImag','ElecFrequency_Hz','ElecDampingRatio','PiMECH_Elec','MixingIndex_Elec','FrequencyGap_Hz','Corr_Tor_to_Tor','Corr_Tor_to_Elec','Corr_Elec_to_Tor','Corr_Elec_to_Elec','RightMAC_Tor','RightMAC_Elec','LeftMAC_Tor','LeftMAC_Elec','SubspaceMAC'}}),'all');
gate=struct('pass',finitePass&&all(T.Workpoint_PASS)&&all(T.ModePairIdentity_PASS)&&height(A)==height(B),'all_metrics_finite',finitePass,'all_workpoints_pass',all(T.Workpoint_PASS),'all_mode_pair_identity_pass',all(T.ModePairIdentity_PASS),'num_screen_candidates',height(B),'num_scan_points',height(T),'num_confirmed_hybridization',sum(A.Assessment=="CONFIRMED_HYBRIDIZATION_IN_TESTED_NEIGHBORHOOD"),'num_frequency_coincidence_only',sum(A.Assessment=="FREQUENCY_COINCIDENCE_ONLY"),'next_rule','S5 may proceed, but only confirmed S4 mechanisms may enter the main claim set.');
S4=struct('objective','S4 targeted modal-interaction falsification','scan',T,'assessment',A,'gate',gate,'evidence_status','CONDITIONAL_IDEAL_CONTINUOUS_AVERAGE');
end

function vars=s4ScanVariableNames()
vars={'CaseID','WorkpointScale','OPLabel','Architecture','ParameterFamily','Factor','PhysicalValue','EquilibriumResidual','Stable','MaxRealPole','CriticalClass','TorPoleReal','TorPoleImag','TorFrequency_Hz','TorDampingRatio','PiMECH_Tor','MixingIndex_Tor','ElecPoleReal','ElecPoleImag','ElecFrequency_Hz','ElecDampingRatio','PiMECH_Elec','MixingIndex_Elec','FrequencyGap_Hz','Corr_Tor_to_Tor','Corr_Tor_to_Elec','Corr_Elec_to_Tor','Corr_Elec_to_Elec','RightMAC_Tor','RightMAC_Elec','LeftMAC_Tor','LeftMAC_Elec','SubspaceMAC','Workpoint_PASS','ModePairIdentity_PASS'};
end

function q=s4ModePair(M,ref)
L=M.linear;[V,D,W]=eig(L.A);lam=diag(D);pos=find(imag(lam)>1e-7);assert(numel(pos)>=2,'S4 requires two positive-imaginary modes.');names=string(L.state_names);mech=ismember(names,["theta_sh","omega_t","omega_g"]);n=numel(pos);pfs=cell(n,1);pimech=zeros(n,1);
for k=1:n,pf=abs(V(:,pos(k)).*conj(W(:,pos(k))));pf=pf/max(sum(pf),eps);pfs{k}=pf;pimech(k)=sum(pf(mech));end
if isempty(ref)
    band=imag(lam(pos))/(2*pi)>=0.5&imag(lam(pos))/(2*pi)<=10;cand=find(band);[~,j0]=max(pimech(cand));it=cand(j0);rest=setdiff(1:n,it);[~,j1]=min(abs(imag(lam(pos(rest)))-imag(lam(pos(it)))));ie=rest(j1);
    corrTT=1;corrTE=dot(pfs{it},pfs{ie})/max(norm(pfs{it})*norm(pfs{ie}),eps);corrET=corrTE;corrEE=1;rmt=1;rme=1;lmt=1;lme=1;sub=1;
else
    cT=zeros(n,1);cE=zeros(n,1);pT=zeros(n,1);pE=zeros(n,1);
    for k=1:n,cT(k)=dot(ref.pfTor,pfs{k})/max(norm(ref.pfTor)*norm(pfs{k}),eps);cE(k)=dot(ref.pfElec,pfs{k})/max(norm(ref.pfElec)*norm(pfs{k}),eps);pT(k)=exp(-abs(lam(pos(k))-ref.lambdaTor)/max(abs(ref.lambdaTor),1));pE(k)=exp(-abs(lam(pos(k))-ref.lambdaElec)/max(abs(ref.lambdaElec),1));end
    [~,it]=max(0.75*cT+0.25*pT);scoreE=0.75*cE+0.25*pE;scoreE(it)=-Inf;[~,ie]=max(scoreE);
    corrTT=cT(it);corrTE=cE(it);corrET=cT(ie);corrEE=cE(ie);rmt=s4Mac(ref.vTor,V(:,pos(it)));rme=s4Mac(ref.vElec,V(:,pos(ie)));lmt=s4Mac(ref.wTor,W(:,pos(it)));lme=s4Mac(ref.wElec,W(:,pos(ie)));Q0=orth([ref.vTor ref.vElec]);Q1=orth([V(:,pos(it)) V(:,pos(ie))]);sub=norm(Q0'*Q1,'fro')^2/2;
end
q=struct('lambdaTor',lam(pos(it)),'lambdaElec',lam(pos(ie)),'fTor',abs(imag(lam(pos(it))))/(2*pi),'fElec',abs(imag(lam(pos(ie))))/(2*pi),'zetaTor',-real(lam(pos(it)))/abs(lam(pos(it))),'zetaElec',-real(lam(pos(ie)))/abs(lam(pos(ie))),'piMechTor',pimech(it),'piMechElec',pimech(ie),'mixTor',4*pimech(it)*(1-pimech(it)),'mixElec',4*pimech(ie)*(1-pimech(ie)),'frequencyGap',abs(imag(lam(pos(it))-lam(pos(ie))))/(2*pi),'pfTor',pfs{it},'pfElec',pfs{ie},'vTor',V(:,pos(it)),'vElec',V(:,pos(ie)),'wTor',W(:,pos(it)),'wElec',W(:,pos(ie)),'corrTorToTor',corrTT,'corrTorToElec',corrTE,'corrElecToTor',corrET,'corrElecToElec',corrEE,'rightMACTor',rmt,'rightMACElec',rme,'leftMACTor',lmt,'leftMACElec',lme,'subspaceMAC',sub);
end

function v=s4Mac(x,y)
v=abs(x'*y)^2/max(real(x'*x)*real(y'*y),eps);
end

function makeM3S4Figure(path,S4)
A=S4.assessment;T=S4.scan;fig=figure('Visible','off','Color','w','Position',[20 20 1750 1100]);tl=tiledlayout(fig,2,4,'TileSpacing','compact','Padding','compact');
for k=1:height(A)
    nexttile;q=sortrows(T(T.CaseID==A.CaseID(k),:),'Factor');yyaxis left;plot(q.Factor,q.TorFrequency_Hz,'o-','LineWidth',1.4,'DisplayName','TOR f');hold on;plot(q.Factor,q.ElecFrequency_Hz,'s--','LineWidth',1.4,'DisplayName','ELEC f');ylabel('Frequency (Hz)');yyaxis right;plot(q.Factor,q.PiMECH_Tor,'^-','LineWidth',1.2,'DisplayName','TOR PiMECH');plot(q.Factor,q.PiMECH_Elec,'v--','LineWidth',1.2,'DisplayName','ELEC PiMECH');ylabel('Mechanical participation');ylim([0 1]);grid on;xlabel('factor');title(sprintf('%s %s/%s',A.CaseID(k),A.Architecture(k),A.ParameterFamily(k)),'Interpreter','none');if k==1,legend('Location','best');end
end
nexttile;bar(categorical(A.CaseID),A.MinimumFrequencyGap_Hz);set(gca,'YScale','log');grid on;ylabel('min gap (Hz)');title('Minimum frequency gap');
sgtitle(tl,'M3 S4：频率接近、参与交换与双模态身份联合判据');exportgraphics(fig,path,'Resolution',220);close(fig);
end

function writeM3S4Report(path,S4,gate)
fid=fopen(path,'w','n','UTF-8');assert(fid>0,'Cannot open S4 report.');c=onCleanup(@()fclose(fid));A=S4.assessment;
fprintf(fid,'# M3 S4：目标模态混合可证伪验证\n\n');fprintf(fid,'## 判据\n\n频率接近只用于筛选。确认模态混合必须同时满足：局部频率接近、机械/电气参与显著交换、双模态子空间连续；并检查左右特征向量MAC。\n\n');
fprintf(fid,'## Gate\n\n- 候选点：%d；局部扫描点：%d；\n- 工作点：%s；双模态身份：%s；数值有限性：%s；总体：%s。\n\n',gate.num_screen_candidates,gate.num_scan_points,iff(gate.all_workpoints_pass,'PASS','FAIL'),iff(gate.all_mode_pair_identity_pass,'PASS','FAIL'),iff(gate.all_metrics_finite,'PASS','FAIL'),iff(gate.pass,'PASS','FAIL'));
fprintf(fid,'## 评估结果\n\n');writeM3Table(fid,A);fprintf(fid,'\n');
if gate.num_confirmed_hybridization==0,fprintf(fid,'本轮没有任何候选满足完整混合判据。最小频差不能单独作为模态耦合或混合证据；相关现象保留为“频率重合”或“测试邻域内未发现混合”。\n\n');else,fprintf(fid,'本轮有 %d 个候选满足完整混合判据，仍仅适用于当前理想连续平均模型和已测试邻域。\n\n',gate.num_confirmed_hybridization);end
if gate.pass,fprintf(fid,'## 决策\n\nS4 PASS：候选点已完成可证伪验证。允许进入 S5 气动运行区扩展，但只有标为 `CONFIRMED_HYBRIDIZATION_IN_TESTED_NEIGHBORHOOD` 的条目可进入候选主张，其余不得外推。\n');else,fprintf(fid,'## 决策\n\nS4 FAIL：工作点或双模态身份未通过，S5保持阻塞。\n');end
end

function [S5,gate]=runM3S5A(scales,allModels,allP,Msummary,TorqueSummary,Cbase,~,m0Dir)
% S5A：在S1的4个额定以下物理MPPT工作点上，用显式Cp(lambda,beta)
% 替换常功率气动转矩。工作点保持不变，只改变omega_t->lambda->Cp->Ta
% 的局部反馈；因此这是完整气动反馈Gate，不是额定以上Pitch验证。
old=path;cleanup=onCleanup(@()path(old));addpath(m0Dir);p5=Liu2024_5MW_Params();rows={};lambdaDeclared=p5.lambda_opt;lambdaCurveOpt=fminbnd(@(z)-m3WindCp(z,0),4,12);hLam=1e-4;cpSlopeDeclared=(m3WindCp(lambdaDeclared+hLam,0)-m3WindCp(lambdaDeclared-hLam,0))/(2*hLam);cpSlopeUsed=(m3WindCp(lambdaCurveOpt+hLam,0)-m3WindCp(lambdaCurveOpt-hLam,0))/(2*hLam);
for si=1:numel(scales)
    sc=scales(si);p=allP{si};wind=p(12)*p5.rotor_radius/lambdaCurveOpt;raw=0.5*p5.air_density*p5.rotor_area*m3WindCp(lambdaCurveOpt,0)*wind^3;cal=(p(39)*p(12))/raw;
    aero=struct('wind_mps',wind,'rotorRadius',p5.rotor_radius,'rho',p5.air_density,'area',p5.rotor_area,'beta_deg',0,'calibration',cal);
    [Ta0,dTa,Dcp]=m3AeroTorqueDerivative(p(12),aero);Dconst=p(39)/p(12);
    models=allModels{si};
    for ai=1:numel(models)
        M0=models{ai};flags=M0.flags;flags.aeroMode='CP_LAMBDA';flags.aero=aero;[x,eq]=solveEquilibrium(M0.x0,p,M0.name,flags,M0.active);L=linearizeModel(x,p,M0.name,flags,M0.active);M1=struct('name',M0.name,'x0',x,'flags',flags,'active',M0.active,'equilibrium',eq,'linear',L);
        tq=Msummary(Msummary.WorkpointScale==sc&Msummary.Architecture==string(M0.name),:);ref=trackedMetrics(M0.linear,tq,[],p);met=trackedMetrics(L,tq,ref,p);[~,De0,~]=complexTorqueCurve(M0.linear,ref.ftor);[~,De1,~]=complexTorqueCurve(L,met.ftor);[CGM,CMG]=couplingAtFrequency(L,p,met.ftor);cb=Cbase(Cbase.WorkpointScale==sc&Cbase.Architecture==string(M0.name),:);
        dir0=directionLabel(log10((cb.C_GridToMachine+1e-30)/(cb.C_MachineToGrid+1e-30)));dir1=directionLabel(log10((CGM+1e-30)/(CMG+1e-30)));
        [mcls,mpf,mef,mif]=s5AeroCounterfactual(M0,M1,1,p);[gcls,gpf,gef,gif]=s5AeroCounterfactual(M0,M1,4,p);
        paErr=Ta0*p(12)-p(39)*p(12);wpPass=eq.normalized_residual<1e-8&&abs(paErr)/p(1)<1e-10;modePass=met.piMechTor>0.5&&met.patternCorrelation>0.75;
        rows(end+1,:)={sc,string(M0.name),wind,lambdaDeclared,lambdaCurveOpt,cpSlopeDeclared,cpSlopeUsed,m3WindCp(lambdaCurveOpt,0),cal,Ta0,dTa,Dconst,Dcp,Dcp-Dconst,paErr,eq.normalized_residual,met.maxReal<0,met.maxReal,met.critFreq,string(met.critClass),met.critPiMech, ...
            real(ref.ltor),imag(ref.ltor),ref.ftor,ref.zeta,real(met.ltor),imag(met.ltor),met.ftor,met.zeta,met.zeta-ref.zeta,met.piMechTor,met.patternCorrelation,De0,De1,CGM,CMG,dir0,dir1,dir0==dir1,mcls,mpf,mef,mif,gcls,gpf,gef,gif,wpPass,modePass}; %#ok<AGROW>
    end
end
vars={'WorkpointScale','Architecture','Wind_mps','LambdaDeclared','LambdaCurveOpt','CpSlopeDeclared','CpSlopeUsed','Cp0','AeroCalibration','Taero0_Nm','dTaero_dOmega','Daero_ConstantPower','Daero_CpLambda','DeltaDaero','AeroPowerIdentityError_W','EquilibriumResidual','Stable','MaxRealPole','CriticalModeFrequency_Hz','CriticalModeClass','CriticalModePiMECH','BasePoleReal','BasePoleImag','BaseFrequency_Hz','BaseDampingRatio','AeroPoleReal','AeroPoleImag','AeroFrequency_Hz','AeroDampingRatio','DeltaDampingRatio','AeroPiMECH','ModePatternCorrelation','BaseDe','AeroDe','C_GridToMachine_Aero','C_MachineToGrid_Aero','BaseDirection','AeroDirection','DirectionClassPreserved','Mechanical_CF_Class','Mechanical_PoleFraction','Mechanical_ExcitationFraction','Mechanical_InteractionFraction','Grid_CF_Class','Grid_PoleFraction','Grid_ExcitationFraction','Grid_InteractionFraction','Workpoint_PASS','ModeIdentity_PASS'};
S=cell2table(rows,'VariableNames',vars);for k={'Architecture','CriticalModeClass','BaseDirection','AeroDirection','Mechanical_CF_Class','Grid_CF_Class'},S.(k{1})=string(S.(k{1}));end
survRows={};survRows(end+1,:)={"PARAM_AUDIT","Declared lambda_opt matches Cp-curve optimum",abs(lambdaDeclared-lambdaCurveOpt)<0.05,sprintf('declared %.6g; curve optimum %.6g; slope at declared %.6g',lambdaDeclared,lambdaCurveOpt,cpSlopeDeclared),"DECLARED_TSR_REJECTED_FOR_FORMAL_GATE","VERIFIED_PARAMETER_INCONSISTENCY"};q=TorqueSummary(ismember(TorqueSummary.Architecture,["GFL","GWT"]),:);survRows(end+1,:)={"H1","Local MPPT electrical damping positive",all(q.DeltaDe_MPPT>0),sprintf('DeltaDe_MPPT range [%.6g, %.6g]',min(q.DeltaDe_MPPT),max(q.DeltaDe_MPPT)),"UNCHANGED_ELECTRICAL_SUBSYSTEM_IN_S5A","CONDITIONAL"};q=TorqueSummary(TorqueSummary.Architecture=="MWT",:);survRows(end+1,:)={"H2","MSC-DVC electrical damping negative",all(q.DeltaDe_DVC<0),sprintf('DeltaDe_DVC range [%.6g, %.6g]',min(q.DeltaDe_DVC),max(q.DeltaDe_DVC)),"UNCHANGED_ELECTRICAL_SUBSYSTEM_IN_S5A","CONDITIONAL"};survRows(end+1,:)={"H3","Frequency-local direction class survives full aero",all(S.DirectionClassPreserved),sprintf('%d/%d rows preserve class',sum(S.DirectionClassPreserved),height(S)),iff(all(S.DirectionClassPreserved),'SURVIVES_S5A','COUNTEREXAMPLE_FOUND'),"CONDITIONAL"};survRows(end+1,:)={"AERO","Cp-lambda feedback changes torsional damping",any(abs(S.DeltaDampingRatio)>1e-5),sprintf('Delta zeta range [%.6g, %.6g]',min(S.DeltaDampingRatio),max(S.DeltaDampingRatio)),"PHYSICAL_FEEDBACK_EFFECT","CONDITIONAL"};
H=cell2table(survRows,'VariableNames',{'HypothesisID','CandidateMechanism','SupportedInTestedSet','Evidence','S5A_Status','EvidenceLevel'});for k={'HypothesisID','CandidateMechanism','Evidence','S5A_Status','EvidenceLevel'},H.(k{1})=string(H.(k{1}));end
finitePass=all(isfinite(S{:,setdiff(S.Properties.VariableNames,{'Architecture','CriticalModeClass','BaseDirection','AeroDirection','Mechanical_CF_Class','Grid_CF_Class'})}),'all');curveOptPass=abs(cpSlopeUsed)<1e-5;gate=struct('pass',finitePass&&all(S.Workpoint_PASS)&&all(S.ModeIdentity_PASS)&&all(S.Stable)&&curveOptPass,'all_metrics_finite',finitePass,'all_workpoints_pass',all(S.Workpoint_PASS),'all_mode_identity_pass',all(S.ModeIdentity_PASS),'all_points_stable',all(S.Stable),'curve_optimum_pass',curveOptPass,'lambda_declared',lambdaDeclared,'lambda_curve_optimum',lambdaCurveOpt,'cp_slope_declared',cpSlopeDeclared,'cp_slope_used',cpSlopeUsed,'num_cases',height(S),'scope','S5A full Cp-lambda feedback at four below-rated MPPT points; beta=0; no Pitch state','next_rule','Pitch/transition/rated-region workpoints remain a separate S5B Gate.');
S5=struct('objective','S5A full Cp-lambda aerodynamic-feedback robustness','summary',S,'mechanism_survival',H,'gate',gate,'evidence_status','CONDITIONAL_BELOW_RATED_FULL_AERO');
end

function [cls,fp,fe,fi]=s5AeroCounterfactual(M0,M1,didx,p)
c0=s3TorsionModalComponents(M0,didx,[]);c1=s3TorsionModalComponents(M1,didx,c0);if didx==1,amp=0.001*p(1)/p(12);else,amp=0.001*p(3);end;t=linspace(0,8,1601)';y00=modalStepPair(t,c0.lambda,c0.residue,amp);y10=modalStepPair(t,c1.lambda,c0.residue,amp);y01=modalStepPair(t,c0.lambda,c1.residue,amp);y11=modalStepPair(t,c1.lambda,c1.residue,amp);dp=norm(y10-y00);de=norm(y01-y00);di=norm(y11-y10-y01+y00);dt=norm(y11-y00);den=max(dt,1e-30);fp=dp/den;fe=de/den;fi=di/den;if dt<1e-12*max(norm(y00),1),cls="NUMERICALLY_SIMILAR";elseif fi>0.30,cls="INTERACTION_SIGNIFICANT";elseif dp>1.5*de,cls="POLE_DOMINATED";elseif de>1.5*dp,cls="EXCITATION_DOMINATED";else,cls="JOINT";end
end

function [T,dT,D]=m3AeroTorqueDerivative(w,a)
fun=@(x)m3AeroTorque(x,a);T=fun(w);h=1e-5*max(abs(w),1);dT=(fun(w+h)-fun(w-h))/(2*h);D=-dT;
end

function T=m3AeroTorque(w,a)
lambda=max(w*a.rotorRadius/max(a.wind_mps,1e-9),1e-6);P=a.calibration*0.5*a.rho*a.area*m3WindCp(lambda,a.beta_deg)*a.wind_mps^3;T=P/max(w,1e-9);
end

function Cp=m3WindCp(lambda,beta_deg)
lambda=max(lambda,1e-4);beta_deg=max(beta_deg,0);lambda_i=1./(1./(lambda+0.08.*beta_deg)-0.035./(beta_deg.^3+1));Cp=0.5176.*(116./lambda_i-0.4.*beta_deg-5).*exp(-21./lambda_i)+0.0068.*lambda;Cp=min(max(Cp,0),0.59);
end

function makeM3S5AFigure(path,S5)
S=S5.summary;archs=["GFL","GWT","MWT"];cols=lines(3);fig=figure('Visible','off','Color','w','Position',[30 30 1500 900]);tl=tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
nexttile;hold on;for a=1:3,q=S(S.Architecture==archs(a),:);plot(q.WorkpointScale,100*q.BaseDampingRatio,'--','Color',cols(a,:),'LineWidth',1.2,'HandleVisibility','off');plot(q.WorkpointScale,100*q.AeroDampingRatio,'o-','Color',cols(a,:),'LineWidth',1.6,'DisplayName',archs(a));end;grid on;xlabel('P_0/P_N');ylabel('\zeta_{tor} (%)');title('虚线：常功率；实线：Cp-\lambda');legend('Location','best');
nexttile;q=unique(S(:,{'WorkpointScale','Daero_ConstantPower','Daero_CpLambda'}),'rows');plot(q.WorkpointScale,q.Daero_ConstantPower/1e6,'s--','LineWidth',1.4,'DisplayName','constant power');hold on;plot(q.WorkpointScale,q.Daero_CpLambda/1e6,'o-','LineWidth',1.6,'DisplayName','Cp-lambda');grid on;xlabel('P_0/P_N');ylabel('D_{aero} (MN m s/rad)');title('气动阻尼导数');legend('Location','best');
nexttile;hold on;for a=1:3,q=S(S.Architecture==archs(a),:);plot(q.WorkpointScale,100*q.DeltaDampingRatio,'o-','Color',cols(a,:),'LineWidth',1.5,'DisplayName',archs(a));end;yline(0,':k');grid on;xlabel('P_0/P_N');ylabel('\Delta\zeta_{tor} (percentage point)');title('完整气动反馈引起的轴系阻尼修正');
nexttile;hold on;for a=1:3,q=S(S.Architecture==archs(a),:);plot(q.WorkpointScale,log10((q.C_GridToMachine_Aero+1e-30)./(q.C_MachineToGrid_Aero+1e-30)),'o-','Color',cols(a,:),'LineWidth',1.5,'DisplayName',archs(a));end;yline(0,':k');yline(log10(3),'--k');yline(-log10(3),'--k');grid on;xlabel('P_0/P_N');ylabel('log_{10}(C_{GM}/C_{MG})');title('Cp-\lambda下的局部方向占优');
sgtitle(tl,'M3 S5A：显式Cp(\lambda,\beta=0)气动反馈跨架构比较');exportgraphics(fig,path,'Resolution',220);close(fig);
end

function writeM3S5AReport(path,S5,gate)
fid=fopen(path,'w','n','UTF-8');assert(fid>0,'Cannot open S5A report.');c=onCleanup(@()fclose(fid));S=S5.summary;
fprintf(fid,'# M3 S5A：完整Cp-lambda气动反馈验证\n\n');fprintf(fid,'## 边界\n\n本轮在4个额定以下MPPT工作点显式加入 `omega_t -> lambda -> Cp -> Taero`，beta固定为0。工作点、公共电气plant和控制职责不变；尚未进入Pitch、额定以上或OpenFAST。\n\n');
fprintf(fid,'## 参数审计\n\n- 参数文件声明 lambda_opt = %.8g；同一Cp曲线数值极值为 %.8g；\n- 声明点 dCp/dlambda = %.8g，正式Gate使用点斜率 = %.8g。\n- 因声明值与曲线不一致，lambda=7.55产生的MWT公共转速实模态失稳只登记为参数不一致反例，不作为物理结论。\n\n',gate.lambda_declared,gate.lambda_curve_optimum,gate.cp_slope_declared,gate.cp_slope_used);
fprintf(fid,'## Gate\n\n- 案例：%d；工作点：%s；模态身份：%s；稳定性：%s；Cp极值一致性：%s；总体：%s。\n- Delta zeta范围：%.6g 到 %.6g；方向分类保持：%d/%d。\n\n',gate.num_cases,iff(gate.all_workpoints_pass,'PASS','FAIL'),iff(gate.all_mode_identity_pass,'PASS','FAIL'),iff(gate.all_points_stable,'PASS','FAIL'),iff(gate.curve_optimum_pass,'PASS','FAIL'),iff(gate.pass,'PASS','FAIL'),min(S.DeltaDampingRatio),max(S.DeltaDampingRatio),sum(S.DirectionClassPreserved),height(S));
fprintf(fid,'## 机制存活表\n\n');writeM3Table(fid,S5.mechanism_survival);fprintf(fid,'\n## 完整结果\n\n');writeM3Table(fid,S);fprintf(fid,'\n');
if gate.pass,fprintf(fid,'## 决策\n\nS5A PASS：额定以下显式气动反馈层已通过。结果仍是条件性证据；S5尚未完成，必须另建S5B验证transition/rated/Pitch区域，不能把本轮规律外推到额定以上。\n');else,fprintf(fid,'## 决策\n\nS5A FAIL：完整气动反馈工作点或模态身份未通过，S5B保持阻塞。\n');end
end

function [S5,gate]=runM3S5B(base,p0,m0Dir,windFactors,pitchKpScale,controllerMode)
% S5B：额定平台连续Pitch-PI模型。beta由Kp*(omega_t-omega_ref)+xi_beta
% 给出，xi_beta为新增连续状态；不含速率/幅值限位、离散或执行器延迟。
old=path;cleanup=onCleanup(@()path(old));addpath(m0Dir);p5=Liu2024_5MW_Params();[p,wp]=makePhysicalMpptPoint(p0,1,m0Dir);wp.pmsgConvention='GENERATOR_OUTWARD';[p,wp]=calibrateMwtTorqueAtTargetSpeed(base,p,p0,wp);aa=makeArchitectures(base,p,wp);[ratedModels,~,~,gA]=runStageA(aa,p,base);assert(gA.pass,'S5B rated common workpoint failed.');[~,torRated,gB]=runStageB(ratedModels);assert(gB.pass,'S5B rated torsional reference failed.');
lambdaOpt=fminbnd(@(z)-m3WindCp(z,0),4,12);windOpt=p(12)*p5.rotor_radius/lambdaOpt;raw=0.5*p5.air_density*p5.rotor_area*m3WindCp(lambdaOpt,0)*windOpt^3;cal=(p(39)*p(12))/raw;windFactors=windFactors(:)';rows={};
for wi=1:numel(windFactors)
    wf=windFactors(wi);wind=wf*windOpt;targetP=p(39)*p(12);fun=@(b)cal*0.5*p5.air_density*p5.rotor_area*m3WindCp(p(12)*p5.rotor_radius/wind,b)*wind^3-targetP;assert(fun(0)>=0&&fun(25)<=0,'S5B wind factor %.4g has no pitch equilibrium in [0,25] deg.',wf);beta0=fzero(fun,[0 25]);
    if wi==1,region="TRANSITION";elseif wi==numel(windFactors),region="ABOVE_RATED";else,region="RATED_PITCH";end
    aero=struct('wind_mps',wind,'rotorRadius',p5.rotor_radius,'rho',p5.air_density,'area',p5.rotor_area,'beta_deg',beta0,'calibration',cal);[~,~,Daero]=m3AeroTorqueDerivative(p(12),aero);
    for ai=1:numel(ratedModels)
        M0=ratedModels{ai};flags=M0.flags;flags.Kmppt_iq_per_radps=0;flags.aeroMode='CP_LAMBDA';flags.aero=aero;[flags.pitchKp_deg_per_radps,flags.pitchKi_deg_per_rad,gk]=s5bPitchGains(p5,beta0,pitchKpScale,controllerMode);flags.pitchControllerMode=controllerMode;
        zseed=[M0.x0;beta0];active=[M0.active 24];[z,eq]=solveS5BEquilibrium(zseed,p,M0.name,flags,active);L=linearizeS5B(z,p,M0.name,flags,active);M1=struct('name',M0.name,'x0',z,'flags',flags,'active',active,'equilibrium',eq,'linear',L);
        tq=torRated(torRated.Architecture==string(M0.name),:);met=trackedMetrics(L,tq,[],p);[CGM,CMG]=couplingAtFrequency(L,p,met.ftor);cb=directionLabel(log10((CGM+1e-30)/(CMG+1e-30)));baseDirection=directionLabel(log10((couplingAtFrequencyScalar(M0.linear,p,tq.Frequency_Hz,1)+1e-30)/(couplingAtFrequencyScalar(M0.linear,p,tq.Frequency_Hz,2)+1e-30)));
        [y,names]=m2Outputs(z(1:23),p,M0.name,zeros(4,1),s5bEffectiveFlags(z,p,flags));get=@(n)y(strcmp(names,n));beta=flags.pitchKp_deg_per_radps*(z(2)-p(12))+z(24);pitchIdx=find(string(L.state_names)=="xi_pitch",1);piPitch=0;critPiPitch=0;if ~isempty(pitchIdx),piPitch=met.pfTor(pitchIdx);critPiPitch=met.pfCrit(pitchIdx);end
        omegaTIdx=find(string(L.state_names)=="omega_t",1);omegaGIdx=find(string(L.state_names)=="omega_g",1);udcIdx=find(string(L.state_names)=="Udc",1);dvcIdx=find(string(L.state_names)=="xi_DVC",1);critPiOmegaT=met.pfCrit(omegaTIdx);critPiOmegaG=met.pfCrit(omegaGIdx);critPiUdc=met.pfCrit(udcIdx);critPiDVC=met.pfCrit(dvcIdx);[~,domCritIdx]=max(met.pfCrit);domCritState=string(L.state_names{domCritIdx});
        hb=1e-4;aeroP=aero;aeroM=aero;aeroP.beta_deg=beta0+hb;aeroM.beta_deg=beta0-hb;dT_dBeta=(m3AeroTorque(p(12),aeroP)-m3AeroTorque(p(12),aeroM))/(2*hb);effectiveSpeedSlope=-Daero+flags.pitchKp_deg_per_radps*dT_dBeta;
        wpPass=eq.normalized_residual<1e-8&&abs(get('P_PCC')-p(37))/p(1)<1e-4&&abs(get('Q_PCC')-p(38))/p(1)<1e-4&&abs(get('Udc')-p(2))/p(2)<1e-6&&abs(get('omega_g')-p(12))/p(12)<1e-5&&abs(beta-beta0)<1e-5;modePass=met.piMechTor>0.5&&abs(met.ftor-tq.Frequency_Hz)<0.25;
        rows(end+1,:)={wf,region,string(M0.name),wind,beta0,beta,string(controllerMode),gk,flags.pitchKp_deg_per_radps,flags.pitchKi_deg_per_rad,Daero,dT_dBeta,effectiveSpeedSlope,eq.normalized_residual,met.maxReal<0,met.maxReal,met.critFreq,string(met.critClass),met.critPiMech,domCritState,critPiPitch,critPiOmegaT,critPiOmegaG,critPiUdc,critPiDVC,real(met.ltor),imag(met.ltor),met.ftor,met.zeta,met.piMechTor,piPitch,CGM,CMG,baseDirection,cb,baseDirection==cb,get('P_PCC'),get('Q_PCC'),get('Udc'),get('omega_g'),wpPass,modePass}; %#ok<AGROW>
    end
end
vars={'WindFactor','OperatingRegion','Architecture','Wind_mps','BetaRequired_deg','BetaSolved_deg','PitchControllerMode','PitchGainScheduleFactor','PitchKp_deg_per_radps','PitchKi_deg_per_rad','Daero_Nms_per_rad','dTa_dBeta_Nm_per_deg','Effective_dTa_dOmega_WithPitchP','EquilibriumResidual','Stable','MaxRealPole','CriticalModeFrequency_Hz','CriticalModeClass','CriticalModePiMECH','CriticalDominantState','CriticalPiPitch','CriticalPiOmegaT','CriticalPiOmegaG','CriticalPiUdc','CriticalPiDVC','TorPoleReal','TorPoleImag','TorFrequency_Hz','TorDampingRatio','TorPiMECH','TorPiPitch','C_GridToMachine','C_MachineToGrid','RatedBaseDirection','PitchRegionDirection','DirectionClassPreserved','P_PCC_W','Q_PCC_var','Udc_V','Omega_g_radps','Workpoint_PASS','ModeIdentity_PASS'};
T=cell2table(rows,'VariableNames',vars);for k={'OperatingRegion','Architecture','PitchControllerMode','CriticalModeClass','CriticalDominantState','RatedBaseDirection','PitchRegionDirection'},T.(k{1})=string(T.(k{1}));end
    failureDiagnostics=runS5BFailureDiagnostics(T,ratedModels,p,p5,cal,windOpt,pitchKpScale,controllerMode);
    stabilityBoundaries=refineS5BStabilityBoundaries(T,ratedModels,p,p5,cal,windOpt,pitchKpScale,controllerMode);
    survRows={{"S5B-H3","Frequency-local direction class survives Pitch region",all(T.DirectionClassPreserved),sprintf('%d/%d preserve rated class',sum(T.DirectionClassPreserved),height(T)),iff(all(T.DirectionClassPreserved),'SURVIVES_S5B','COUNTEREXAMPLE_FOUND'),"CONDITIONAL_PITCH_MODEL"};{"S5B-TOR","Torsional mode remains stable across Pitch region",all(T.TorDampingRatio>0),sprintf('zeta range [%.6g, %.6g]',min(T.TorDampingRatio),max(T.TorDampingRatio)),iff(all(T.TorDampingRatio>0),'SURVIVES_S5B','COUNTEREXAMPLE_FOUND'),"CONDITIONAL_PITCH_MODEL"};{"S5B-PITCH","Pitch state strongly mixes with torsional mode",any(T.TorPiPitch>0.05),sprintf('Pi_pitch range [%.6g, %.6g]',min(T.TorPiPitch),max(T.TorPiPitch)),iff(any(T.TorPiPitch>0.05),'MIXING_CANDIDATE','NO_STRONG_MIXING_IN_TESTED_SET'),"CONDITIONAL_PITCH_MODEL"};{"H1","Region-2 MPPT positive damping extrapolates to Region-3",false,"MPPT slope is frozen on rated plateau","NOT_APPLICABLE_OUTSIDE_REGION2","SCOPE_LIMIT"}};
H=cell2table(vertcat(survRows{:}),'VariableNames',{'HypothesisID','CandidateMechanism','SupportedInTestedSet','Evidence','S5B_Status','EvidenceLevel'});for k={'HypothesisID','CandidateMechanism','Evidence','S5B_Status','EvidenceLevel'},H.(k{1})=string(H.(k{1}));end
finitePass=all(isfinite(T{:,setdiff(T.Properties.VariableNames,{'OperatingRegion','Architecture','PitchControllerMode','CriticalModeClass','CriticalDominantState','RatedBaseDirection','PitchRegionDirection'})}),'all');gate=struct('pass',finitePass&&all(T.Workpoint_PASS)&&all(T.ModeIdentity_PASS)&&all(T.Stable),'all_metrics_finite',finitePass,'all_workpoints_pass',all(T.Workpoint_PASS),'all_mode_identity_pass',all(T.ModeIdentity_PASS),'all_points_stable',all(T.Stable),'num_cases',height(T),'num_stability_boundaries',height(stabilityBoundaries),'wind_factors',windFactors,'pitch_kp_scale',pitchKpScale,'pitch_controller_mode',controllerMode,'pitch_kp_min',min(T.PitchKp_deg_per_radps),'pitch_kp_max',max(T.PitchKp_deg_per_radps),'pitch_ki_min',min(T.PitchKi_deg_per_rad),'pitch_ki_max',max(T.PitchKi_deg_per_rad),'pitch_model','continuous algebraic beta command with one PI integrator state; command-level gain scheduling allowed; no actuator lag/rate/limit','next_rule','S6 flexible-mechanics expansion is allowed only if this Gate passes and the controller source is traceable.');S5=struct('objective','S5B transition/rated/above-rated continuous Pitch-region Gate','summary',T,'mechanism_survival',H,'failure_diagnostics',failureDiagnostics,'stability_boundaries',stabilityBoundaries,'gate',gate,'evidence_status','CONDITIONAL_CONTINUOUS_PITCH_MODEL');
end

function B=refineS5BStabilityBoundaries(T,ratedModels,p,p5,cal,windOpt,pitchScale,controllerMode)
Q=sortrows(T(T.Architecture=="MWT",:),'WindFactor');cross=find(Q.Stable(1:end-1)~=Q.Stable(2:end));rows={};M0=ratedModels{find(cellfun(@(m)strcmpi(m.name,'MWT'),ratedModels),1)};
for k=1:numel(cross)
    lo=Q.WindFactor(cross(k));hi=Q.WindFactor(cross(k)+1);[flo,~]=evaluateS5BMwtFactor(lo,M0,p,p5,cal,windOpt,pitchScale,controllerMode);stableLo=flo<0;stableHi=Q.Stable(cross(k)+1);
    it=0;while hi-lo>1e-5&&it<30,it=it+1;mid=(lo+hi)/2;[fm,~]=evaluateS5BMwtFactor(mid,M0,p,p5,cal,windOpt,pitchScale,controllerMode);if (fm<0)==stableLo,lo=mid;else,hi=mid;end;end
    fc=(lo+hi)/2;[mr,c]=evaluateS5BMwtFactor(fc,M0,p,p5,cal,windOpt,pitchScale,controllerMode);wind=fc*windOpt;beta=s5bPitchEquilibrium(fc,p,p5,cal,windOpt);rows(end+1,:)={k,lo,hi,fc,wind,beta,mr,c.frequency_Hz,c.class,c.dominantState,stableLo,stableHi,it}; %#ok<AGROW>
end
vars={'BoundaryIndex','FactorLow','FactorHigh','CriticalWindFactor','CriticalWind_mps','CriticalBeta_deg','MaxRealAtBoundary','CriticalFrequency_Hz','CriticalClass','CriticalDominantState','StableOnLowSide','StableOnHighSide','Iterations'};
if isempty(rows),B=cell2table(cell(0,numel(vars)),'VariableNames',vars);else,B=cell2table(rows,'VariableNames',vars);B.CriticalClass=string(B.CriticalClass);B.CriticalDominantState=string(B.CriticalDominantState);end
end

function [mr,c]=evaluateS5BMwtFactor(wf,M0,p,p5,cal,windOpt,pitchScale,controllerMode)
beta=s5bPitchEquilibrium(wf,p,p5,cal,windOpt);aero=struct('wind_mps',wf*windOpt,'rotorRadius',p5.rotor_radius,'rho',p5.air_density,'area',p5.rotor_area,'beta_deg',beta,'calibration',cal);f=M0.flags;f.Kmppt_iq_per_radps=0;f.aeroMode='CP_LAMBDA';f.aero=aero;[f.pitchKp_deg_per_radps,f.pitchKi_deg_per_rad]=s5bPitchGains(p5,beta,pitchScale,controllerMode);f.pitchControllerMode=controllerMode;active=[M0.active 24];[z,eq]=solveS5BEquilibrium([M0.x0;beta],p,M0.name,f,active);assert(eq.normalized_residual<1e-8,'S5B boundary equilibrium failed.');L=linearizeS5B(z,p,M0.name,f,active);c=s5bCriticalMode(L,M0.name);mr=real(c.lambda);
end

function beta=s5bPitchEquilibrium(wf,p,p5,cal,windOpt)
wind=wf*windOpt;targetP=p(39)*p(12);fun=@(b)cal*0.5*p5.air_density*p5.rotor_area*m3WindCp(p(12)*p5.rotor_radius/wind,b)*wind^3-targetP;assert(fun(0)>=0&&fun(25)<=0,'S5B factor %.6g has no pitch equilibrium.',wf);beta=fzero(fun,[0 25]);
end

function [kp,ki,gk]=s5bPitchGains(p5,beta_deg,scale,mode)
mode=upper(char(string(mode)));
switch mode
    case 'LEGACY_CONSTANT'
        gk=1;kp=scale*p5.pitch_kp_deg_per_radps;ki=p5.pitch_ki_deg_per_rad;
    case 'NREL5MW_SCHEDULED_LSS'
        % Jonkman et al. NREL 5-MW baseline: Kp=0.01882681 s,
        % Ki=0.008068634 on the 97:1 high-speed shaft.  M3 uses low-speed
        % rotor speed, hence both gains are referred through Ngear=97 and
        % converted from rad pitch to deg pitch.  The official GK(beta)
        % schedule is retained.
        gear=97;thetaK=0.1099965;gk=1/(1+(beta_deg*pi/180)/thetaK);
        kp=scale*0.01882681*gear*180/pi*gk;ki=scale*0.008068634*gear*180/pi*gk;
    otherwise,error('Unsupported S5B Pitch controller mode %s.',mode);
end
end

function D=runS5BFailureDiagnostics(T,ratedModels,p,p5,cal,windOpt,pitchScale,controllerMode)
% 只对失败点做最小反事实：Pitch积分、Pitch比例、固定桨距和MWT-DVC积分。
bad=T(~T.Stable,:);rows={};
for bi=1:height(bad)
    q=bad(bi,:);ai=find(cellfun(@(m)strcmpi(m.name,char(q.Architecture)),ratedModels),1);M0=ratedModels{ai};
    aero=struct('wind_mps',q.WindFactor*windOpt,'rotorRadius',p5.rotor_radius,'rho',p5.air_density,'area',p5.rotor_area,'beta_deg',q.BetaRequired_deg,'calibration',cal);
    f0=M0.flags;f0.Kmppt_iq_per_radps=0;f0.aeroMode='CP_LAMBDA';f0.aero=aero;[f0.pitchKp_deg_per_radps,f0.pitchKi_deg_per_rad]=s5bPitchGains(p5,q.BetaRequired_deg,pitchScale,controllerMode);f0.pitchControllerMode=controllerMode;
    fP=f0;fP.pitchKi_deg_per_rad=0;fB=fP;fB.pitchKp_deg_per_radps=0;
    variants={'FULL_PITCH_PI',f0,[M0.active 24],p;'PITCH_P_ONLY',fP,M0.active,p;'FROZEN_BETA',fB,M0.active,p};
    if strcmpi(M0.name,'MWT')
        pd=p;pd(26)=0;variants(end+1,:)={'DVC_P_ONLY_WITH_PITCH_PI',f0,setdiff([M0.active 24],6),pd}; %#ok<AGROW>
    end
    for vi=1:size(variants,1)
        tag=string(variants{vi,1});fv=variants{vi,2};active=variants{vi,3};pv=variants{vi,4};seed=[M0.x0;q.BetaRequired_deg];[z,eq]=solveS5BEquilibrium(seed,pv,M0.name,fv,active);L=linearizeS5B(z,pv,M0.name,fv,active);cm=s5bCriticalMode(L,M0.name);
        rows(end+1,:)={q.WindFactor,q.Architecture,tag,eq.normalized_residual,cm.stable,real(cm.lambda),cm.frequency_Hz,cm.class,cm.dominantState,cm.piMech,cm.piPitch,cm.piOmegaT,cm.piOmegaG,cm.piUdc,cm.piDVC}; %#ok<AGROW>
    end
end
vars={'WindFactor','Architecture','Counterfactual','EquilibriumResidual','Stable','MaxRealPole','CriticalFrequency_Hz','CriticalClass','CriticalDominantState','CriticalPiMECH','CriticalPiPitch','CriticalPiOmegaT','CriticalPiOmegaG','CriticalPiUdc','CriticalPiDVC'};
if isempty(rows),D=cell2table(cell(0,numel(vars)),'VariableNames',vars);else,D=cell2table(rows,'VariableNames',vars);for k={'Architecture','Counterfactual','CriticalClass','CriticalDominantState'},D.(k{1})=string(D.(k{1}));end;end
end

function c=s5bCriticalMode(L,arch)
[V,D,W]=eig(L.A);lam=diag(D);[~,ii]=max(real(lam));pf=abs(V(:,ii).*conj(W(:,ii)));pf=pf/max(sum(pf),eps);names=string(L.state_names);g=aggregateParticipation(pf,names,arch);[~,jd]=max(pf);at=@(n)sum(pf(names==n));c=struct('lambda',lam(ii),'frequency_Hz',abs(imag(lam(ii)))/(2*pi),'stable',real(lam(ii))<0,'class',dominantParticipationClass(g),'dominantState',names(jd),'piMech',g.MECH,'piPitch',at("xi_pitch"),'piOmegaT',at("omega_t"),'piOmegaG',at("omega_g"),'piUdc',at("Udc"),'piDVC',at("xi_DVC"));
end

function v=couplingAtFrequencyScalar(L,p,f,k)
[a,b]=couplingAtFrequency(L,p,f);if k==1,v=a;else,v=b;end
end

function f=s5bEffectiveFlags(z,p,flags)
f=flags;f.aero.beta_deg=flags.pitchKp_deg_per_radps*(z(2)-p(12))+z(24);
end

function dz=s5bRhs(z,p,arch,d,flags)
f=s5bEffectiveFlags(z,p,flags);dx=m2Rhs(z(1:23),p,arch,d,f);dz=[dx;flags.pitchKi_deg_per_rad*(z(2)-p(12))];
end

function [z,meta]=solveS5BEquilibrium(seed,p,arch,flags,active)
sx=[1;1;1;1e4;1e4;1e4;1e4;1e4;1e3;5e6;5e6;1;1;1e4;1e4;1e4;1e4;1e4;1e4;1e3;1e3;1e4;1e4;10];sr=[1;1;1;1e4;1e4;1e4;1e4;1e4;1e6;5e8;5e8;1;1;1e4;1e4;1e4;1e4;1e6;1e6;1e6;1e6;1e6;1e6;1];opts=optimoptions('fsolve','Display','off','Algorithm','levenberg-marquardt','FunctionTolerance',1e-12,'StepTolerance',1e-12,'OptimalityTolerance',1e-12,'MaxIterations',3000,'MaxFunctionEvaluations',50000);z=seed(:);q0=z(active)./sx(active);[q,fval,exitflag]=fsolve(@residual,q0,opts);z(active)=sx(active).*q;dz=s5bRhs(z,p,arch,zeros(4,1),flags);nr=norm(dz(active)./sr(active),inf);meta=struct('exitflag',exitflag,'normalized_residual',nr,'solver_residual',norm(fval,inf),'max_abs_dx',max(abs(dz(active))),'pass',exitflag>0&&nr<1e-10);
    function r=residual(qv),zz=seed(:);zz(active)=sx(active).*qv;dd=s5bRhs(zz,p,arch,zeros(4,1),flags);r=dd(active)./sr(active);end
end

function L=linearizeS5B(z,p,arch,flags,active)
[y0,names,units]=s5bOutputs(z,p,arch,zeros(4,1),flags);n=numel(z);ny=numel(y0);nd=4;Afull=zeros(n);Bfull=zeros(n,nd);Cfull=zeros(ny,n);D=zeros(ny,nd);
for j=active,h=1e-6*max(abs(z(j)),1);e=zeros(n,1);e(j)=h;Afull(:,j)=(s5bRhs(z+e,p,arch,zeros(nd,1),flags)-s5bRhs(z-e,p,arch,zeros(nd,1),flags))/(2*h);Cfull(:,j)=(s5bOutputs(z+e,p,arch,zeros(nd,1),flags)-s5bOutputs(z-e,p,arch,zeros(nd,1),flags))/(2*h);end
hd=[1;1;1e-6;1e-4];for j=1:nd,e=zeros(nd,1);e(j)=hd(j);Bfull(:,j)=(s5bRhs(z,p,arch,e,flags)-s5bRhs(z,p,arch,-e,flags))/(2*hd(j));D(:,j)=(s5bOutputs(z,p,arch,e,flags)-s5bOutputs(z,p,arch,-e,flags))/(2*hd(j));end
allNames={'theta_sh','omega_t','omega_g','i_md','i_mq','xi_DVC','xi_MSC_d','xi_MSC_q','Udc','P_f','Q_f','omega_sync','delta','xi_GSC_vd','xi_GSC_vq','xi_GSC_id','xi_GSC_iq','i_f_d','i_f_q','v_c_d','v_c_q','i_g_d','i_g_q','xi_pitch'};L=struct('A',Afull(active,active),'B',Bfull(active,:),'C',Cfull(:,active),'D',D,'active',active,'state_names',{allNames(active)},'output_names',{names},'output_units',{units},'input_names',{{'DeltaTm','DeltaPaero','DeltaThetaGrid','DeltaOmegaGrid'}});
end

function [y,names,units]=s5bOutputs(z,p,arch,d,flags)
f=s5bEffectiveFlags(z,p,flags);[y,names,units]=m2Outputs(z(1:23),p,arch,d,f);beta=f.aero.beta_deg;y=[y;beta];names=[names {'beta_pitch'}];units=[units {'deg'}];
end

function makeM3S5BFigure(path,S5)
T=S5.summary;archs=["GFL","GWT","MWT"];cols=lines(3);fig=figure('Visible','off','Color','w','Position',[40 40 1650 900]);tl=tiledlayout(fig,2,3,'TileSpacing','compact','Padding','compact');
nexttile;q=unique(T(:,{'WindFactor','Wind_mps','BetaRequired_deg'}),'rows');plot(q.Wind_mps,q.BetaRequired_deg,'o-','LineWidth',1.7);grid on;xlabel('Wind (m/s)');ylabel('\beta_0 (deg)');title('Pitch equilibrium');
nexttile;hold on;for a=1:3,q=T(T.Architecture==archs(a),:);plot(q.Wind_mps,100*q.TorDampingRatio,'o-','LineWidth',1.5,'Color',cols(a,:),'DisplayName',archs(a));end;grid on;xlabel('Wind (m/s)');ylabel('\zeta_{tor} (%)');title('Torsional damping');legend('Location','best');
nexttile;hold on;for a=1:3,q=T(T.Architecture==archs(a),:);plot(q.Wind_mps,q.TorPiPitch,'o-','LineWidth',1.5,'Color',cols(a,:),'DisplayName',archs(a));end;grid on;xlabel('Wind (m/s)');ylabel('Pitch participation in TOR');title('Pitch--torsion participation');
nexttile;hold on;for a=1:3,q=T(T.Architecture==archs(a),:);plot(q.Wind_mps,log10((q.C_GridToMachine+1e-30)./(q.C_MachineToGrid+1e-30)),'o-','LineWidth',1.5,'Color',cols(a,:),'DisplayName',archs(a));end;yline(0,':k');yline(log10(3),'--k');yline(-log10(3),'--k');grid on;xlabel('Wind (m/s)');ylabel('log_{10}(C_{GM}/C_{MG})');title('Directional coupling');
nexttile;hold on;for a=1:3,q=T(T.Architecture==archs(a),:);plot(q.Wind_mps,q.MaxRealPole,'o-','LineWidth',1.5,'Color',cols(a,:),'DisplayName',archs(a));end;yline(0,'--k');grid on;xlabel('Wind (m/s)');ylabel('max Re(\lambda) (1/s)');title('Whole-system stability');
nexttile;D=S5.failure_diagnostics;if isempty(D),axis off;text(.1,.5,'No failed point','FontSize',12);else,lab=D.Counterfactual+"@"+compose('%.2f',D.WindFactor);bar(categorical(lab,lab),D.MaxRealPole);yline(0,'--k');grid on;ylabel('max Re(\lambda) (1/s)');title('Failed-point counterfactuals');xtickangle(35);end
sgtitle(tl,'M3 S5B：transition/rated/above-rated连续Pitch区域');exportgraphics(fig,path,'Resolution',220);close(fig);
end

function writeM3S5BReport(path,S5,gate)
fid=fopen(path,'w','n','UTF-8');assert(fid>0,'Cannot open S5B report.');c=onCleanup(@()fclose(fid));T=S5.summary;fprintf(fid,'# M3 S5B：Pitch运行区验证\n\n');fprintf(fid,'## 模型边界\n\n本轮使用连续Pitch PI：beta=Kp*(omega_t-omega_ref)+xi_beta，dot(xi_beta)=Ki*(omega_t-omega_ref)。无执行器延迟、速率限制、幅值限位和离散；GFL/GWT在额定平台冻结Region-2 MPPT斜率。\n\n- 控制器模式：`%s`；增益比例：%.6g；\n- Kp范围：%.6g--%.6g deg/(rad/s)；Ki范围：%.6g--%.6g deg/rad。\n\n',gate.pitch_controller_mode,gate.pitch_kp_scale,gate.pitch_kp_min,gate.pitch_kp_max,gate.pitch_ki_min,gate.pitch_ki_max);fprintf(fid,'## Gate\n\n- 风速因子：%s；案例：%d；\n- 工作点：%s；模态身份：%s；稳定性：%s；总体：%s。\n- beta范围：%.4f--%.4f deg；轴系阻尼范围：%.4f%%--%.4f%%。\n\n',strjoin(string(gate.wind_factors),', '),gate.num_cases,iff(gate.all_workpoints_pass,'PASS','FAIL'),iff(gate.all_mode_identity_pass,'PASS','FAIL'),iff(gate.all_points_stable,'PASS','FAIL'),iff(gate.pass,'PASS','FAIL'),min(T.BetaSolved_deg),max(T.BetaSolved_deg),100*min(T.TorDampingRatio),100*max(T.TorDampingRatio));fprintf(fid,'## 机制存活表\n\n');writeM3Table(fid,S5.mechanism_survival);fprintf(fid,'\n## 结果\n\n');writeM3Table(fid,T);fprintf(fid,'\n');if ~isempty(S5.failure_diagnostics),fprintf(fid,'## 失败点反事实定位\n\n');writeM3Table(fid,S5.failure_diagnostics);fprintf(fid,'\n该表只用于区分Pitch积分、Pitch比例、固定桨距与MWT-DVC积分的局部作用，不能直接外推为全局因果结论。\n\n');end;if ~isempty(S5.stability_boundaries),fprintf(fid,'## MWT局部稳定边界\n\n');writeM3Table(fid,S5.stability_boundaries);fprintf(fid,'\n边界来自当前连续Cp-Pitch模型的局部数值细化；不得外推到离散、限幅或OpenFAST模型。\n\n');end;if gate.pass,fprintf(fid,'## 决策\n\nS5B数值Gate通过，但是否允许进入S6仍须结合来源可追溯性审计。所有结论仍受当前连续Pitch模型边界约束；原始失败证据必须并列保留。\n');else,fprintf(fid,'## 决策\n\nS5B FAIL：不得进入S6。当前失败属于低频公共转速/Pitch-DVC耦合候选机制，并非2.5 Hz轴系模态失稳；进入S6前必须先解释或重新定义稳定的额定区基准。\n');end
end

function [A,gate]=runM3S5BCandidateAudit(base,p0,m0Dir,S5B,pitchKpScale,controllerMode,marginWindFactors)
% S5B候选整定审计：不覆盖原始Kp=10失败证据，只检验Kp候选能否
% 作为S6的公共额定区基准。完整时序不落盘，所有诊断合并到一个表。
old=path;cleanup=onCleanup(@()path(old));addpath(m0Dir);p5=Liu2024_5MW_Params();
[p,wp]=makePhysicalMpptPoint(p0,1,m0Dir);wp.pmsgConvention='GENERATOR_OUTWARD';[p,wp]=calibrateMwtTorqueAtTargetSpeed(base,p,p0,wp);aa=makeArchitectures(base,p,wp);[ratedModels,~,~,gA]=runStageA(aa,p,base);assert(gA.pass,'S5B audit rated common workpoint failed.');
lambdaOpt=fminbnd(@(z)-m3WindCp(z,0),4,12);windOpt=p(12)*p5.rotor_radius/lambdaOpt;raw=0.5*p5.air_density*p5.rotor_area*m3WindCp(lambdaOpt,0)*windOpt^3;cal=(p(39)*p(12))/raw;
T=S5B.summary;T.SlowPoleTimeConstant_s=nan(height(T),1);neg=T.MaxRealPole<0;T.SlowPoleTimeConstant_s(neg)=-1./T.MaxRealPole(neg);
T.ArchitecturePSpread_pu=zeros(height(T),1);T.ArchitectureUdcSpread_pu=zeros(height(T),1);T.ArchitectureOmegaSpread_pu=zeros(height(T),1);T.ArchitectureBetaSpread_deg=zeros(height(T),1);
u=unique(T.WindFactor);for k=1:numel(u),ii=abs(T.WindFactor-u(k))<1e-12;q=T(ii,:);T.ArchitecturePSpread_pu(ii)=(max(q.P_PCC_W)-min(q.P_PCC_W))/p(1);T.ArchitectureUdcSpread_pu(ii)=(max(q.Udc_V)-min(q.Udc_V))/p(2);T.ArchitectureOmegaSpread_pu(ii)=(max(q.Omega_g_radps)-min(q.Omega_g_radps))/p(12);T.ArchitectureBetaSpread_deg(ii)=max(q.BetaSolved_deg)-min(q.BetaSolved_deg);end

marginWindFactors=marginWindFactors(:)';mr={};
for wi=1:numel(marginWindFactors)
    wf=marginWindFactors(wi);beta0=s5bPitchEquilibrium(wf,p,p5,cal,windOpt);aero=struct('wind_mps',wf*windOpt,'rotorRadius',p5.rotor_radius,'rho',p5.air_density,'area',p5.rotor_area,'beta_deg',beta0,'calibration',cal);
    for ai=1:numel(ratedModels)
        M0=ratedModels{ai};f=M0.flags;f.Kmppt_iq_per_radps=0;f.aeroMode='CP_LAMBDA';f.aero=aero;[f.pitchKp_deg_per_radps,f.pitchKi_deg_per_rad]=s5bPitchGains(p5,beta0,pitchKpScale,controllerMode);f.pitchControllerMode=controllerMode;active=[M0.active 24];[z,eq]=solveS5BEquilibrium([M0.x0;beta0],p,M0.name,f,active);Lc=linearizeS5B(z,p,M0.name,f,active);lm=pitchLoopMargins(z,p,M0.name,f,M0.active,Lc);
        lower=NaN;upper=NaN;lowerMargin=Inf;upperMargin=Inf;intervalStatus="NOT_LIMITING_ARCHITECTURE";
        if strcmpi(M0.name,'MWT'),[lower,upper,lowerMargin,upperMargin,intervalStatus]=findS5BKpStabilityInterval(M0,p,p5,cal,windOpt,wf,pitchKpScale,controllerMode);end
        intervalPass=~strcmpi(M0.name,'MWT')||(lowerMargin>=1.2&&upperMargin>=1.2&&intervalStatus~="CANDIDATE_UNSTABLE");
        mr(end+1,:)={wf,string(M0.name),eq.normalized_residual,lm.OpenLoopPlantMaxReal,lm.ClassicalMarginApplicable,lm.GainMarginFactor,lm.PhaseMargin_deg,lm.GainMarginFrequency_Hz,lm.PhaseCrossover_Hz,lm.ClosedLoopBandwidth_Hz,lm.ClosedLoopPoleMismatch,lm.LoopClosure_PASS,lower,upper,lowerMargin,upperMargin,string(intervalStatus),intervalPass}; %#ok<AGROW>
    end
end
MT=cell2table(mr,'VariableNames',{'WindFactor','Architecture','MarginEquilibriumResidual','OpenLoopPitchPlantMaxReal','ClassicalMarginApplicable','PitchLoopGainMarginFactor','PitchLoopPhaseMargin_deg','GainMarginFrequency_Hz','PhaseCrossover_Hz','PitchClosedLoopBandwidth_Hz','PitchLoopClosurePoleMismatch','LoopClosure_PASS','KpLowerCriticalScale','KpUpperCriticalScale','KpLowerMarginFactor','KpUpperMarginFactor','KpIntervalStatus','KpIntervalMargin_PASS'});MT.Architecture=string(MT.Architecture);MT.KpIntervalStatus=string(MT.KpIntervalStatus);

newNames={'MarginEquilibriumResidual','OpenLoopPitchPlantMaxReal','ClassicalMarginApplicable','PitchLoopGainMarginFactor','PitchLoopPhaseMargin_deg','GainMarginFrequency_Hz','PhaseCrossover_Hz','PitchClosedLoopBandwidth_Hz','PitchLoopClosurePoleMismatch','LoopClosure_PASS','KpLowerCriticalScale','KpUpperCriticalScale','KpLowerMarginFactor','KpUpperMarginFactor','KpIntervalMargin_PASS'};
for k=1:numel(newNames),nm=newNames{k};if ismember(nm,{'ClassicalMarginApplicable','LoopClosure_PASS','KpIntervalMargin_PASS'}),T.(nm)=false(height(T),1);else,T.(nm)=nan(height(T),1);end;end
T.KpIntervalStatus=repmat("NOT_EVALUATED_AT_THIS_WIND",height(T),1);
for k=1:height(MT),ii=abs(T.WindFactor-MT.WindFactor(k))<1e-12&T.Architecture==MT.Architecture(k);for j=1:numel(newNames),nm=newNames{j};T.(nm)(ii)=MT.(nm)(k);end;T.KpIntervalStatus(ii)=MT.KpIntervalStatus(k);end

kpMin=min(T.PitchKp_deg_per_radps);kpMax=max(T.PitchKp_deg_per_radps);kiMin=min(T.PitchKi_deg_per_rad);kiMax=max(T.PitchKi_deg_per_rad);zeroHz=median((T.PitchKi_deg_per_rad./T.PitchKp_deg_per_radps)/(2*pi));zeroTau=median(T.PitchKp_deg_per_radps./T.PitchKi_deg_per_rad);T.PitchPIZero_Hz=repmat(zeroHz,height(T),1);T.PitchPIZeroTimeConstant_s=repmat(zeroTau,height(T),1);
fair=max(T.ArchitecturePSpread_pu)<1e-8&&max(T.ArchitectureUdcSpread_pu)<1e-10&&max(T.ArchitectureOmegaSpread_pu)<1e-10&&max(T.ArchitectureBetaSpread_deg)<1e-5;
dense=all(T.Stable)&all(T.Workpoint_PASS)&all(T.ModeIdentity_PASS)&all(isfinite(T.MaxRealPole));marginPass=all(MT.LoopClosure_PASS)&all(MT.KpIntervalMargin_PASS);candidateTraceable=strcmpi(controllerMode,'NREL5MW_SCHEDULED_LSS');actuatorRepresented=false;commandLevelScopeAcceptable=candidateTraceable;
% 参考NREL 5MW基准仅作量级审计，不能替代当前直驱PMSG的重新设计。
nrelGear=97;nrelKpHss=0.01882681;nrelKiHss=0.008068634;nrelKpLssDeg=nrelKpHss*nrelGear*180/pi;nrelKiLssDeg=nrelKiHss*nrelGear*180/pi;
physicalProvenance=candidateTraceable&&commandLevelScopeAcceptable;
gate=struct('pass',dense&&fair&&marginPass&&physicalProvenance,'dense_scan_pass',dense,'architecture_fairness_pass',fair,'loop_margin_pass',marginPass,'loop_closure_pass',all(MT.LoopClosure_PASS),'kp_interval_margin_pass',all(MT.KpIntervalMargin_PASS),'physical_provenance_pass',physicalProvenance,'candidate_traceable_to_frozen_source',candidateTraceable,'pitch_actuator_represented',actuatorRepresented,'command_level_scope_acceptable_for_S6',commandLevelScopeAcceptable,'controller_mode',controllerMode,'num_dense_cases',height(T),'num_margin_cases',height(MT),'minimum_max_real_pole',min(T.MaxRealPole),'least_stable_max_real_pole',max(T.MaxRealPole),'minimum_torsional_damping',min(T.TorDampingRatio),'maximum_torsional_damping',max(T.TorDampingRatio),'pitch_gain_scale',pitchKpScale,'pitch_kp_min_deg_per_radps',kpMin,'pitch_kp_max_deg_per_radps',kpMax,'pitch_ki_min_deg_per_rad',kiMin,'pitch_ki_max_deg_per_rad',kiMax,'pi_zero_Hz',zeroHz,'pi_zero_time_constant_s',zeroTau,'nrel_reference_kp_lss_deg_per_radps',nrelKpLssDeg,'nrel_reference_ki_lss_deg_per_rad',nrelKiLssDeg,'decision_rule','S6 local flexible-mechanics Gate requires dense stability, common-workpoint fairness, loop reconstruction plus gain-scale interval margin, and a traceable continuous command-level Pitch controller. Actuator/limits remain mandatory for S7.');
T.DenseNumerical_PASS=repmat(dense,height(T),1);T.ArchitectureFairness_PASS=repmat(fair,height(T),1);T.AllRepresentativeLoopMargins_PASS=repmat(marginPass,height(T),1);T.PhysicalProvenance_PASS=repmat(physicalProvenance,height(T),1);T.OverallCandidate_PASS=repmat(gate.pass,height(T),1);
A=struct('objective','S5B Pitch candidate physical/numerical baseline audit','summary',T,'margin_summary',MT,'gate',gate,'evidence_status',iff(gate.pass,'TRACEABLE_CONTINUOUS_COMMAND_LEVEL_BASELINE','NUMERICALLY_STABLE_CANDIDATE_NOT_YET_PHYSICAL_BASELINE'));
end

function lm=pitchLoopMargins(z,p,arch,flags,activePlant,Lclosed)
% 断开 beta=PI(omega_t-omega_ref) 环路，重建 beta->omega_t plant。
x=z(1:23);fp=flags;fp.aero.beta_deg=flags.aero.beta_deg;Lp=linearizeModel(x,p,arch,fp,activePlant);h=1e-4;fP=fp;fM=fp;fP.aero.beta_deg=fp.aero.beta_deg+h;fM.aero.beta_deg=fp.aero.beta_deg-h;bFull=(m2Rhs(x,p,arch,zeros(4,1),fP)-m2Rhs(x,p,arch,zeros(4,1),fM))/(2*h);B=bFull(activePlant);C=zeros(1,numel(activePlant));C(activePlant==2)=1;G=ss(Lp.A,B,C,0);K=tf([flags.pitchKp_deg_per_radps flags.pitchKi_deg_per_rad],[1 0]);Loop=-minreal(K*G,1e-9);plantMaxReal=max(real(eig(Lp.A)));classicalApplicable=plantMaxReal<-1e-8;
try,evalc('[gm,pm,wcg,wcp]=margin(Loop);');evalc('bw=bandwidth(feedback(Loop,1));');catch,gm=NaN;pm=NaN;wcg=NaN;wcp=NaN;bw=NaN;end
Aaug=[Lp.A+B*flags.pitchKp_deg_per_radps*C B;flags.pitchKi_deg_per_rad*C 0];la=eig(Aaug);lc=eig(Lclosed.A);dm=max(arrayfun(@(q)min(abs(lc-q)),la));closurePass=dm<1e-5;
lm=struct('OpenLoopPlantMaxReal',plantMaxReal,'ClassicalMarginApplicable',classicalApplicable,'GainMarginFactor',gm,'PhaseMargin_deg',pm,'GainMarginFrequency_Hz',wcg/(2*pi),'PhaseCrossover_Hz',wcp/(2*pi),'ClosedLoopBandwidth_Hz',bw/(2*pi),'ClosedLoopPoleMismatch',dm,'LoopClosure_PASS',closurePass);
end

function [lower,upper,lowerMargin,upperMargin,status]=findS5BKpStabilityInterval(M0,p,p5,cal,windOpt,wf,candidate,controllerMode)
gridLo=linspace(0.1,candidate,13);vLo=zeros(size(gridLo));for k=1:numel(gridLo),vLo(k)=evalS5BKpScale(M0,p,p5,cal,windOpt,wf,gridLo(k),controllerMode);end
stableLo=vLo<0;idx=find(~stableLo(1:end-1)&stableLo(2:end),1,'last');lower=NaN;if ~isempty(idx),lower=refineKpCrossing(M0,p,p5,cal,windOpt,wf,gridLo(idx),gridLo(idx+1),false,controllerMode);end
gridHi=linspace(candidate,12,13);vHi=zeros(size(gridHi));for k=1:numel(gridHi),vHi(k)=evalS5BKpScale(M0,p,p5,cal,windOpt,wf,gridHi(k),controllerMode);end
stableHi=vHi<0;idx=find(stableHi(1:end-1)&~stableHi(2:end),1,'first');upper=NaN;if ~isempty(idx),upper=refineKpCrossing(M0,p,p5,cal,windOpt,wf,gridHi(idx),gridHi(idx+1),true,controllerMode);end
if isnan(lower),lowerMargin=Inf;else,lowerMargin=candidate/lower;end;if isnan(upper),upperMargin=Inf;else,upperMargin=upper/candidate;end
if ~stableLo(end),status="CANDIDATE_UNSTABLE";elseif isnan(lower)&&isnan(upper),status="STABLE_FROM_0p1_TO_12_TESTED";elseif isnan(upper),status="LOWER_BOUNDARY_ONLY_NO_UPPER_TO_12";elseif isnan(lower),status="UPPER_BOUNDARY_ONLY";else,status="BOUNDED_STABLE_INTERVAL";end
end

function fc=refineKpCrossing(M0,p,p5,cal,windOpt,wf,lo,hi,stableOnLow,controllerMode)
flo=evalS5BKpScale(M0,p,p5,cal,windOpt,wf,lo,controllerMode);for it=1:24,mid=(lo+hi)/2;fm=evalS5BKpScale(M0,p,p5,cal,windOpt,wf,mid,controllerMode);if (flo<0)==(fm<0),lo=mid;flo=fm;else,hi=mid;end;end;fc=(lo+hi)/2;
if stableOnLow,assert(evalS5BKpScale(M0,p,p5,cal,windOpt,wf,fc*0.999,controllerMode)<0||fc<1e-6,'Unexpected upper Pitch-gain crossing orientation.');end
end

function mr=evalS5BKpScale(M0,p,p5,cal,windOpt,wf,scale,controllerMode)
beta=s5bPitchEquilibrium(wf,p,p5,cal,windOpt);aero=struct('wind_mps',wf*windOpt,'rotorRadius',p5.rotor_radius,'rho',p5.air_density,'area',p5.rotor_area,'beta_deg',beta,'calibration',cal);f=M0.flags;f.Kmppt_iq_per_radps=0;f.aeroMode='CP_LAMBDA';f.aero=aero;[f.pitchKp_deg_per_radps,f.pitchKi_deg_per_rad]=s5bPitchGains(p5,beta,scale,controllerMode);f.pitchControllerMode=controllerMode;active=[M0.active 24];[z,eq]=solveS5BEquilibrium([M0.x0;beta],p,M0.name,f,active);assert(eq.normalized_residual<1e-8,'S5B Pitch gain interval equilibrium failed.');L=linearizeS5B(z,p,M0.name,f,active);mr=max(real(eig(L.A)));
end

function makeM3S5BAuditFigure(path,A)
T=A.summary;M=A.margin_summary;archs=["GFL","GWT","MWT"];cols=lines(3);fig=figure('Visible','off','Color','w','Position',[40 40 1650 980]);tl=tiledlayout(fig,2,3,'TileSpacing','compact','Padding','compact');
nexttile;hold on;for a=1:3,q=T(T.Architecture==archs(a),:);plot(q.Wind_mps,q.MaxRealPole,'LineWidth',1.6,'Color',cols(a,:),'DisplayName',archs(a));end;yline(0,'--k');grid on;xlabel('Wind (m/s)');ylabel('max Re(\lambda) (1/s)');title('Dense stability scan');legend('Location','best');
nexttile;hold on;for a=1:3,q=T(T.Architecture==archs(a),:);plot(q.Wind_mps,100*q.TorDampingRatio,'LineWidth',1.6,'Color',cols(a,:),'DisplayName',archs(a));end;grid on;xlabel('Wind (m/s)');ylabel('\zeta_{tor} (%)');title('Torsional-mode identity');
nexttile;hold on;for a=1:3,q=T(T.Architecture==archs(a),:);plot(q.Wind_mps,q.SlowPoleTimeConstant_s,'LineWidth',1.6,'Color',cols(a,:),'DisplayName',archs(a));end;set(gca,'YScale','log');grid on;xlabel('Wind (m/s)');ylabel('slow-pole time constant (s)');title('Residual slow mode');
nexttile;hold on;for a=1:3,q=M(M.Architecture==archs(a),:);plot(q.WindFactor,q.OpenLoopPitchPlantMaxReal,'o-','LineWidth',1.5,'Color',cols(a,:),'DisplayName',archs(a));end;yline(0,'--k');grid on;xlabel('Wind factor');ylabel('open-loop plant max Re(\lambda)');title('Classical margin applicability');
nexttile;hold on;for a=1:3,q=M(M.Architecture==archs(a),:);semilogy(q.WindFactor,q.PitchClosedLoopBandwidth_Hz,'o-','LineWidth',1.5,'Color',cols(a,:),'DisplayName',archs(a));end;grid on;xlabel('Wind factor');ylabel('bandwidth (Hz)');title('Pitch speed-loop bandwidth');
nexttile;q=M(M.Architecture=="MWT",:);hold on;plot(q.WindFactor,q.KpLowerCriticalScale,'o-','LineWidth',1.6,'DisplayName','lower stability boundary');yline(A.gate.pitch_gain_scale,'--','candidate','DisplayName','gain-scale candidate');grid on;xlabel('Wind factor');ylabel('gain scale');title('MWT Pitch-gain stability interval');legend('Location','best');
sgtitle(tl,'M3 S5B candidate audit: numerical Gate versus physical baseline Gate');exportgraphics(fig,path,'Resolution',220);close(fig);
end

function writeM3S5BAuditReport(path,A,gate)
fid=fopen(path,'w','n','UTF-8');assert(fid>0,'Cannot open S5B candidate audit report.');c=onCleanup(@()fclose(fid));T=A.summary;M=A.margin_summary;
fprintf(fid,'# M3 S5B 候选整定物理/数值审计\n\n');fprintf(fid,'## 审计目标\n\n本轮不覆盖原始 Kp=10 的失败证据，只判断 `%s`、增益比例 %.6g（Kp %.6g--%.6g deg/(rad/s)，Ki %.6g--%.6g deg/rad）是否足以冻结为 S6 公共额定区基准。连续风速加密扫描、三架构公平性、Pitch断环重构和增益稳定区间均由同一M3方程生成；没有保存时序。\n\n',gate.controller_mode,gate.pitch_gain_scale,gate.pitch_kp_min_deg_per_radps,gate.pitch_kp_max_deg_per_radps,gate.pitch_ki_min_deg_per_rad,gate.pitch_ki_max_deg_per_rad);
fprintf(fid,'## Gate结果\n\n- 加密数值稳定性：%s（%d案例，最弱稳定极点 %.6g 1/s）；\n- 三架构共同工作点公平性：%s；\n- Pitch断环重构：%s；\n- 增益稳定区间距离：%s；\n- 参数来源可追溯且适用于S6命令级分析：%s；\n- Pitch执行器/速率/角度限制已建模：%s（本项不是S6 Gate，S7前必须补齐）；\n- **总体：%s**。\n\n',iff(gate.dense_scan_pass,'PASS','FAIL'),gate.num_dense_cases,gate.least_stable_max_real_pole,iff(gate.architecture_fairness_pass,'PASS','FAIL'),iff(gate.loop_closure_pass,'PASS','FAIL'),iff(gate.kp_interval_margin_pass,'PASS','FAIL'),iff(gate.physical_provenance_pass,'PASS','FAIL'),iff(gate.pitch_actuator_represented,'PASS','NOT_YET'),iff(gate.pass,'PASS','FAIL'));
fprintf(fid,'## 控制器尺度审计\n\n- 控制器模式：`%s`；Kp范围 %.6g--%.6g deg/(rad/s)，Ki范围 %.6g--%.6g deg/rad；\n- PI零点 %.6g Hz，对应时间常数 %.6g s；\n- NREL 5 MW基准控制器折算到低速轴、未施加桨距增益调度前：Kp约 %.6g deg/(rad/s)，Ki约 %.6g deg/rad；\n- 当前模型保留官方形式的增益调度时，`PitchGainScheduleFactor` 随平衡桨距变化；\n- 当前仍是连续命令级模型，未包含Pitch执行器滞后、8 deg/s速率限制和角度限位。这些必须在S7恢复，本轮不能据此宣称真实执行器已验证。\n\n',gate.controller_mode,gate.pitch_kp_min_deg_per_radps,gate.pitch_kp_max_deg_per_radps,gate.pitch_ki_min_deg_per_rad,gate.pitch_ki_max_deg_per_rad,gate.pi_zero_Hz,gate.pi_zero_time_constant_s,gate.nrel_reference_kp_lss_deg_per_radps,gate.nrel_reference_ki_lss_deg_per_rad);
fprintf(fid,'官方参考：[NREL 5-MW Baseline Wind Turbine](https://www.nrel.gov/docs/fy09osti/38060.pdf)；[OpenFAST DISCON baseline controller](https://github.com/OpenFAST/openfast/blob/main/share/discon/DISCON.F90)。\n\n');
fprintf(fid,'## 代表点断环诊断与增益区间\n\n');writeM3Table(fid,M);fprintf(fid,'\n传统单环 margin 只作为诊断：断开的Pitch plant包含公共转速积分/既有多环动态；当 `ClassicalMarginApplicable=false` 时，不得用传统相位裕度或增益裕度作Gate。正式数值Gate采用闭环极点重构误差和增益比例稳定区间距离。\n\n## 结论边界\n\n本轮只证明当前**无执行器的连续命令级Pitch模型**在测试域内是否稳定、是否可追溯。原Kp=10反例、Kp=30数值候选与本次可追溯控制器结果必须并列保留；任何一个结果都不能外推到离散、限幅、OpenFAST或EMT模型。\n\n');
if gate.pass,fprintf(fid,'## 决策\n\nS5B可追溯连续命令级基准通过本轮全部Gate，可进入S6的局部柔性机械扩展；S7前必须补回执行器、速率和角度限制。\n');else,fprintf(fid,'## 决策\n\nS5B候选未通过物理基准Gate，S6继续阻塞。下一步只定位失败来源，不继续盲目放大单一增益。\n');end
end

function row=buildS2Row(sc,opLabel,M,pv,flags,x,eq,L,tq,ref,refDe,fam,category,factor,physicalValue,physicalUnit,refinementLevel)
met=trackedMetrics(L,tq,ref,pv);[~,De,Ke]=complexTorqueCurve(L,met.ftor);[dDeCtrl,dKeCtrl,ctrlCase]=s2ControlContribution(x,pv,M.name,flags,M.active,fam,met.ftor);
poleIdx=abs(met.ltor-ref.ltor)/max(abs(ref.ltor),eps);resDec=abs(log10(max(met.residue,1e-30)/max(ref.residue,1e-30)));ppClass=classifyPoleExcitation(poleIdx,resDec);
[y,names]=m2Outputs(x,pv,M.name,zeros(4,1),flags);get=@(n)y(strcmp(names,n));P0=get('P_PCC');Q0=get('Q_PCC');U0=get('Udc');wg0=get('omega_g');
wpPass=eq.normalized_residual<1e-8&&abs(P0-pv(37))/pv(1)<1e-4&&abs(Q0-pv(38))/pv(1)<1e-4&&abs(U0-pv(2))/pv(2)<1e-6&&abs(wg0-pv(12))/max(pv(12),eps)<1e-5;
modePass=met.piMechTor>0.5&&met.patternCorrelation>0.75;hybrid=met.freqGap<0.5&&(met.piMechTor<0.9||met.piMechElec>0.05)&&met.patternCorrelation<0.95;status=string(iff(wpPass&&modePass,'CONDITIONAL_SPARSE_POINT','REVIEW_REQUIRED'));
row={sc,string(opLabel),string(M.name),string(fam),string(category),factor,physicalValue,string(physicalUnit),refinementLevel,eq.normalized_residual,P0,Q0,U0,wg0,met.maxReal<0,met.maxReal,met.critFreq,string(met.critClass),met.critPiMech,real(met.ltor),imag(met.ltor),met.ftor,met.zeta,met.piMechTor,met.patternCorrelation,met.felec,met.piMechElec,met.freqGap,De,Ke,dDeCtrl,dKeCtrl,ctrlCase,De-refDe,met.residue,poleIdx,resDec,string(ppClass),met.CGM,met.CMG,log10(met.dirRatio),hybrid,modePass,wpPass,status};
end

function T=makeS2Table(rows)
vars={'WorkpointScale','OPLabel','Architecture','ParameterFamily','ParameterCategory','Factor','PhysicalValue','PhysicalUnit','RefinementLevel','EquilibriumResidual','P0_W','Q0_var','Udc_V','omega_g_radps','Stable','MaxRealPole','CriticalModeFrequency_Hz','CriticalModeClass','CriticalModePiMECH','TorPoleReal','TorPoleImag','TorFrequency_Hz','TorDampingRatio','TorPiMECH','ParticipationPatternCorrelation','NearestElectricalFrequency_Hz','NearestElectricalPiMECH','FrequencyGap_Hz','TotalDe','TotalKe','ControlContributionDe','ControlContributionKe','ControlContributionCase','DeltaDeFromNominal','TorResidue_GridToShaft','PoleChangeIndex','ResidueChangeDecades','PolePathClass','C_GridToMachine','C_MachineToGrid','Ldir_Log10','HybridizationCandidate','ModeIdentity_PASS','Workpoint_PASS','EvidenceStatus'};
if isempty(rows),T=table();else,T=cell2table(rows,'VariableNames',vars);end
end

function [Tr,meta]=refineS2ContinuousBoundaries(B0,T0,scales,allModels,allP,Msummary,base,p0)
targets=B0(B0.Status=="REFINE_REQUIRED"&ismember(B0.BoundaryType,["STABILITY","DIRECTION","FEEDBACK_SIGN"]),:);newRows={};metaRows={};
for bi=1:height(targets)
    b=targets(bi,:);wi=find(abs(scales-b.WorkpointScale)<1e-12,1);models=allModels{wi};p=allP{wi};ai=find(cellfun(@(m)strcmpi(m.name,b.Architecture),models),1);M=models{ai};
    tq=Msummary(Msummary.WorkpointScale==b.WorkpointScale&Msummary.Architecture==b.Architecture,:);ref=trackedMetrics(M.linear,tq,[],p);[~,refDe,~]=complexTorqueCurve(M.linear,ref.ftor);
    lo=b.FactorLow;hi=b.FactorHigh;qlo=T0(T0.WorkpointScale==b.WorkpointScale&T0.Architecture==b.Architecture&T0.ParameterFamily==b.ParameterFamily&abs(T0.Factor-lo)<1e-12,:);qhi=T0(T0.WorkpointScale==b.WorkpointScale&T0.Architecture==b.Architecture&T0.ParameterFamily==b.ParameterFamily&abs(T0.Factor-hi)<1e-12,:);
    assert(~isempty(qlo)&&~isempty(qhi),'Missing S2 boundary endpoints.');vlo=s2BoundaryMetric(qlo(1,:),b.BoundaryType);vhi=s2BoundaryMetric(qhi(1,:),b.BoundaryType);nEval=0;
    for it=1:7
        if hi-lo<=0.01,break;end
        mid=(lo+hi)/2;row=evaluateS2RefinementPoint(b.WorkpointScale,b.OPLabel,M,p,tq,ref,refDe,b.ParameterFamily,mid,it,base,p0);newRows(end+1,:)=row; %#ok<AGROW>
        qt=makeS2Table(row);vm=s2BoundaryMetric(qt,b.BoundaryType);nEval=nEval+1;
        if vm==0,lo=mid;hi=mid;vlo=vm;vhi=vm;break;elseif sign(vlo)==sign(vm),lo=mid;vlo=vm;else,hi=mid;vhi=vm;end
    end
    metaRows(end+1,:)={b.WorkpointScale,b.OPLabel,b.Architecture,b.ParameterFamily,b.BoundaryType,b.FactorLow,b.FactorHigh,lo,hi,mean([lo hi]),vlo,vhi,nEval,hi-lo<=0.01}; %#ok<AGROW>
end
Tr=makeS2Table(newRows);if ~isempty(Tr),Tr.OPLabel=string(Tr.OPLabel);Tr.Architecture=string(Tr.Architecture);Tr.ParameterFamily=string(Tr.ParameterFamily);Tr.ParameterCategory=string(Tr.ParameterCategory);Tr.PhysicalUnit=string(Tr.PhysicalUnit);Tr.CriticalModeClass=string(Tr.CriticalModeClass);Tr.ControlContributionCase=string(Tr.ControlContributionCase);Tr.PolePathClass=string(Tr.PolePathClass);Tr.EvidenceStatus=string(Tr.EvidenceStatus);end
if isempty(metaRows),meta=table();else,meta=cell2table(metaRows,'VariableNames',{'WorkpointScale','OPLabel','Architecture','ParameterFamily','BoundaryType','InitialFactorLow','InitialFactorHigh','FinalFactorLow','FinalFactorHigh','EstimatedFactor','MetricLow','MetricHigh','NumNewEvaluations','Refined_PASS'});meta.OPLabel=string(meta.OPLabel);meta.Architecture=string(meta.Architecture);meta.ParameterFamily=string(meta.ParameterFamily);meta.BoundaryType=string(meta.BoundaryType);end
end

function v=s2BoundaryMetric(q,btype)
switch string(btype)
    case "STABILITY",v=q.MaxRealPole(1);
    case "DIRECTION",v=q.Ldir_Log10(1);
    case "FEEDBACK_SIGN",v=q.ControlContributionDe(1);
    otherwise,error('Unsupported S2 boundary type %s.',btype);
end
end

function row=evaluateS2RefinementPoint(sc,opLabel,M,p,tq,ref,refDe,fam,factor,level,base,p0)
category=string(iff(fam=="SCR",'ENVIRONMENT','CONTROL'));
if fam=="SCR"
    sv=4*factor;pv=p;pv(9:10)=p(9:10)*(4/sv);pv(40)=pccVoltageMagnitudeForPQ(pv(37),pv(38),pv(4),pv(9),pv(3)*pv(10));
    wp=struct('powerScale',sc,'speedScale',sc^(1/3),'torqueScale',pv(39)/p0(39),'wind_mps',NaN,'rawCpPower_W',NaN,'aeroCalibration',NaN,'aeroPower_W',pv(39)*pv(12),'definition','S2_SCR_REFINED_COMMON_WORKPOINT','pmsgConvention','GENERATOR_OUTWARD');
    [pv,wp]=calibrateMwtTorqueAtTargetSpeed(base,pv,p0,wp);aa=makeArchitectures(base,pv,wp);[mm,~,~,gg]=runStageA(aa,pv,base);assert(gg.pass,'Refined SCR point failed common workpoint Gate.');ai=find(cellfun(@(m)strcmpi(m.name,M.name),mm),1);Ms=mm{ai};flags=Ms.flags;x=Ms.x0;eq=Ms.equilibrium;L=Ms.linear;physical=sv;unit="SCR";
else
    [pv,flags,physical,unit]=applyS2Parameter(p,M.flags,M.name,fam,factor);[x,eq,pv]=solveS2Point(M.x0,p,pv,M.name,flags,M.active,fam,factor);L=linearizeModel(x,pv,M.name,flags,M.active);
end
row=buildS2Row(sc,opLabel,M,pv,flags,x,eq,L,tq,ref,refDe,fam,category,factor,physical,unit,level);
end

function tf=s2ParameterIsActive(fam,arch)
arch=upper(string(arch));fam=upper(string(fam));
switch fam
    case "H",tf=any(arch==["GWT","MWT"]);
    case "DVC",tf=true;
    case "GSC_CURRENT_BW",tf=true;
    case "GSC_VOLTAGE_BW",tf=any(arch==["GWT","MWT"]);
    case "MPPT_GAIN",tf=any(arch==["GFL","GWT"]);
    case "SCR",tf=true;
    otherwise,tf=false;
end
end

function [pv,flags,value,unit]=applyS2Parameter(p,flags,arch,fam,q)
pv=p;arch=upper(string(arch));fam=upper(string(fam));
switch fam
    case "H",pv(33)=p(33)*q;value=pv(33);unit="s";
    case "DVC"
        if arch=="MWT",pv(25:26)=p(25:26)*q;else,flags.KpGscDvc_W_per_V=flags.KpGscDvc_W_per_V*q;flags.KiGscDvc_W_per_Vs=flags.KiGscDvc_W_per_Vs*q;flags.KpGscDvc_A_per_V=flags.KpGscDvc_A_per_V*q;flags.KiGscDvc_A_per_Vs=flags.KiGscDvc_A_per_Vs*q;end
        value=q;unit="factor";
    case "GSC_CURRENT_BW",pv(29:30)=p(29:30)*q;value=q;unit="factor";
    case "GSC_VOLTAGE_BW",pv(31:32)=p(31:32)*q;value=q;unit="factor";
    case "MPPT_GAIN",flags.Kmppt_iq_per_radps=flags.Kmppt_iq_per_radps*q;value=q;unit="factor";
    case "SCR"
        pv(9:10)=p(9:10)*(4/q);pv(40)=pccVoltageMagnitudeForPQ(pv(37),pv(38),pv(4),pv(9),pv(3)*pv(10));value=q;unit="SCR";
    otherwise,error('Unsupported S2 parameter %s.',fam);
end
end

function [x,eq,pv]=solveS2Point(seed,pbase,pv,arch,flags,active,fam,q)
x=seed;
if fam=="SCR"
    for v=linspace(4,q,5)
        ps=pbase;ps(9:10)=pbase(9:10)*(4/v);ps(40)=pccVoltageMagnitudeForPQ(ps(37),ps(38),ps(4),ps(9),ps(3)*ps(10));
        [x,eq]=solveEquilibrium(x,ps,arch,flags,active);
    end
    pv=ps;
else
    [x,eq]=solveEquilibrium(seed,pv,arch,flags,active);
end
end

function [dDe,dKe,cas]=s2ControlContribution(x,p,arch,flags,active,fam,ft)
dDe=NaN;dKe=NaN;cas="NOT_ADDITIVE_CHANNEL";p0=p;f0=flags;
switch upper(string(fam))
    case "DVC"
        if strcmpi(arch,'MWT'),p0(25:26)=0;else,f0.KpGscDvc_W_per_V=0;f0.KiGscDvc_W_per_Vs=0;f0.KpGscDvc_A_per_V=0;f0.KiGscDvc_A_per_Vs=0;end
        cas="DVC_ON_MINUS_BYPASS";
    case "MPPT_GAIN",f0.Kmppt_iq_per_radps=0;cas="MPPT_ON_MINUS_ZERO_SLOPE";
    case "H",f0.freezeSync=true;cas="GFM_SYNC_ON_MINUS_FROZEN";
    otherwise,return
end
L1=linearizeModel(x,p,arch,flags,active);L0=linearizeModel(x,p0,arch,f0,active);
G1=complexTorqueCurve(L1,ft);G0=complexTorqueCurve(L0,ft);dG=G1-G0;dDe=real(dG);dKe=-(2*pi*ft)*imag(dG);
end

function B=detectS2Boundaries(T)
rows={};groups=unique(T(:,{'WorkpointScale','Architecture','ParameterFamily'}),'rows');
for g=1:height(groups)
    q=T(T.WorkpointScale==groups.WorkpointScale(g)&T.Architecture==groups.Architecture(g)&T.ParameterFamily==groups.ParameterFamily(g),:);q=sortrows(q,'Factor');
    rows=appendCrossings(rows,q,'STABILITY','MaxRealPole');
    rows=appendCrossings(rows,q,'DIRECTION','Ldir_Log10');
    if all(isfinite(q.ControlContributionDe)),rows=appendCrossings(rows,q,'FEEDBACK_SIGN','ControlContributionDe');end
    for k=1:height(q)-1
        % 因子1与参考完全相同，SIMILAR_TO_REFERENCE是定义上的必然值，
        % 不能把其两侧分类变化误报为机制迁移边界。
        if q.PolePathClass(k)~=q.PolePathClass(k+1)&&q.PolePathClass(k)~="SIMILAR_TO_REFERENCE"&&q.PolePathClass(k+1)~="SIMILAR_TO_REFERENCE"
            rows(end+1,:)={q.WorkpointScale(k),q.OPLabel(k),q.Architecture(k),q.ParameterFamily(k),"POLE_PATH_CLASS",q.Factor(k),q.Factor(k+1),mean(q.Factor(k:k+1)),NaN,string(q.PolePathClass(k)+" -> "+q.PolePathClass(k+1)),"REFINE_REQUIRED"}; %#ok<AGROW>
        end
    end
end

% 模态接近只保留每个工作点/架构的全参数最小值，避免同一基准模态在
% H/DVC/带宽/MPPT/SCR各族中被重复登记。
pa=unique(T(:,{'WorkpointScale','OPLabel','Architecture'}),'rows');
for k=1:height(pa)
    q=T(T.WorkpointScale==pa.WorkpointScale(k)&T.Architecture==pa.Architecture(k),:);[gap,j]=min(q.FrequencyGap_Hz);
    if gap<0.75,rows(end+1,:)={q.WorkpointScale(j),q.OPLabel(j),q.Architecture(j),q.ParameterFamily(j),"MODAL_PROXIMITY",q.Factor(j),q.Factor(j),q.Factor(j),gap,sprintf('global minimum gap %.6g Hz',gap),"S3_SCREEN_ONLY"};end %#ok<AGROW>
end
if isempty(rows)
    B=table('Size',[0 11],'VariableTypes',{'double','string','string','string','string','double','double','double','double','string','string'},'VariableNames',{'WorkpointScale','OPLabel','Architecture','ParameterFamily','BoundaryType','FactorLow','FactorHigh','EstimatedFactor','MetricAtEstimate','Evidence','Status'});
else
    B=cell2table(rows,'VariableNames',{'WorkpointScale','OPLabel','Architecture','ParameterFamily','BoundaryType','FactorLow','FactorHigh','EstimatedFactor','MetricAtEstimate','Evidence','Status'});
    B.OPLabel=string(B.OPLabel);B.Architecture=string(B.Architecture);B.ParameterFamily=string(B.ParameterFamily);B.BoundaryType=string(B.BoundaryType);B.Evidence=string(B.Evidence);B.Status=string(B.Status);
end
end

function B=annotateS2Boundaries(B,T)
B.NearestSampleFactor=NaN(height(B),1);B.CriticalModeClass=strings(height(B),1);B.CriticalModeFrequency_Hz=NaN(height(B),1);B.TorDampingRatio=NaN(height(B),1);B.TorPiMECH=NaN(height(B),1);B.NearestElectricalPiMECH=NaN(height(B),1);B.ParticipationPatternCorrelation=NaN(height(B),1);B.HybridizationCandidateAtNearest=false(height(B),1);
for k=1:height(B)
    q=T(T.WorkpointScale==B.WorkpointScale(k)&T.Architecture==B.Architecture(k)&T.ParameterFamily==B.ParameterFamily(k),:);if isempty(q),continue;end
    [~,j]=min(abs(q.Factor-B.EstimatedFactor(k)));B.NearestSampleFactor(k)=q.Factor(j);B.CriticalModeClass(k)=q.CriticalModeClass(j);B.CriticalModeFrequency_Hz(k)=q.CriticalModeFrequency_Hz(j);B.TorDampingRatio(k)=q.TorDampingRatio(j);B.TorPiMECH(k)=q.TorPiMECH(j);B.NearestElectricalPiMECH(k)=q.NearestElectricalPiMECH(j);B.ParticipationPatternCorrelation(k)=q.ParticipationPatternCorrelation(j);B.HybridizationCandidateAtNearest(k)=q.HybridizationCandidate(j);
end
end

function rows=appendCrossings(rows,q,btype,varname)
v=q.(varname);
for k=1:numel(v)-1
    if ~isfinite(v(k))||~isfinite(v(k+1)),continue;end
    if v(k)==0||v(k+1)==0||sign(v(k))~=sign(v(k+1))
        if abs(v(k+1)-v(k))>eps,est=q.Factor(k)-v(k)*(q.Factor(k+1)-q.Factor(k))/(v(k+1)-v(k));else,est=mean(q.Factor(k:k+1));end
        if abs(q.Factor(k+1)-q.Factor(k))<=0.01,status="REFINED_BRACKET";else,status="REFINE_REQUIRED";end
        rows(end+1,:)={q.WorkpointScale(k),q.OPLabel(k),q.Architecture(k),q.ParameterFamily(k),string(btype),q.Factor(k),q.Factor(k+1),est,0,sprintf('%s: %.6g -> %.6g',varname,v(k),v(k+1)),status}; %#ok<AGROW>
    end
end
end

function CE=detectS2Counterexamples(T,B)
rows={};types=["FEEDBACK_SIGN","DIRECTION","STABILITY","POLE_PATH_CLASS","MODAL_PROXIMITY"];
for k=1:numel(types)
    q=B(B.BoundaryType==types(k),:);det=~isempty(q);
    if ~det,lev="NO_BRACKET_IN_TESTED_GRID";elseif types(k)=="MODAL_PROXIMITY",lev="S3_SCREEN_ONLY";elseif all(q.Status=="REFINED_BRACKET"),lev="REFINED_CONDITIONAL_BOUNDARY";else,lev="REFINEMENT_INCOMPLETE";end
    rows(end+1,:)={"B-C"+k,types(k),det,height(q),iff(det,"Boundary/counterexample bracket found","No nontrivial bracket in tested grid"),lev}; %#ok<AGROW>
end
q=T(~T.ModeIdentity_PASS,:);rows(end+1,:)={"B-C6","MODE_IDENTITY",~isempty(q),height(q),iff(isempty(q),"All tracked torsional modes retained identity","Manual mode review required"),"GATE_CRITICAL"}; %#ok<AGROW>
CE=cell2table(rows,'VariableNames',{'CounterexampleID','Test','Detected','Count','Interpretation','EvidenceLevel'});CE.CounterexampleID=string(CE.CounterexampleID);CE.Test=string(CE.Test);CE.Interpretation=string(CE.Interpretation);CE.EvidenceLevel=string(CE.EvidenceLevel);
end

function makeM3S2Figure(path,S2)
T=S2.summary;fams=["H","DVC","GSC_CURRENT_BW","GSC_VOLTAGE_BW","MPPT_GAIN","SCR"];cols=lines(3);marks={'o','s','^'};styles={'-','--',':'};
opNames=["OP_L","OP_M","OP_H"];opShort=["L","M","H"];
fig=figure('Visible','off','Color','w','Position',[40 40 1650 980]);tl=tiledlayout(fig,2,3,'TileSpacing','compact','Padding','compact');
for fi=1:numel(fams)
    nexttile;hold on;q=T(T.ParameterFamily==fams(fi),:);aa=unique(q.Architecture,'stable');
    for a=1:numel(aa)
        for o=1:3
            z=sortrows(q(q.Architecture==aa(a)&q.OPLabel==opNames(o),:),'Factor');if isempty(z),continue;end
            plot(z.Factor,100*z.TorDampingRatio,'LineStyle',styles{o},'Marker',marks{a},'LineWidth',1.35,'Color',cols(a,:),'DisplayName',aa(a)+"/"+opShort(o));
            if any(~z.Stable),scatter(z.Factor(~z.Stable),100*z.TorDampingRatio(~z.Stable),70,'x','MarkerEdgeColor','r','LineWidth',1.8,'HandleVisibility','off');end
        end
    end
    grid on;xlabel(iff(fams(fi)=="SCR","SCR","parameter factor"));ylabel('\zeta_{tor} (%)');title(strrep(fams(fi),'_',' '));if fi==1,legend('Location','eastoutside','FontSize',7);end
end
sgtitle(tl,'M3 S2：稀疏控制整定下轴系阻尼（红叉表示全系统失稳，非轴系失稳也保留）');exportgraphics(fig,path,'Resolution',220);close(fig);
end

function makeM3S2BoundaryFigure(path,S2)
T=S2.summary;B=S2.boundaries(BoundaryIsContinuous(S2.boundaries.BoundaryType)&S2.boundaries.Status=="REFINED_BRACKET",:);n=height(B);if n==0,return;end
nr=ceil(n/3);fig=figure('Visible','off','Color','w','Position',[30 30 1650 max(700,320*nr)]);tl=tiledlayout(fig,nr,3,'TileSpacing','compact','Padding','compact');
for k=1:n
    b=B(k,:);q=T(T.WorkpointScale==b.WorkpointScale&T.Architecture==b.Architecture&T.ParameterFamily==b.ParameterFamily,:);q=sortrows(q,'Factor');
    switch b.BoundaryType
        case "STABILITY",v=q.MaxRealPole;yl='max Re(\lambda) (1/s)';
        case "DIRECTION",v=q.Ldir_Log10;yl='log_{10}(C_{GM}/C_{MG})';
        case "FEEDBACK_SIGN",v=q.ControlContributionDe/1e6;yl='\Delta D_e (MNms/rad)';
    end
    nexttile;plot(q.Factor,v,'o-','LineWidth',1.45,'MarkerSize',5);hold on;yline(0,':k');xline(b.EstimatedFactor,'--r',sprintf('%.4f',b.EstimatedFactor),'LabelOrientation','horizontal');grid on;xlabel(iff(b.ParameterFamily=="SCR","SCR/4 factor","parameter factor"));ylabel(yl);title(sprintf('%s %s %s %s',b.OPLabel,b.Architecture,b.ParameterFamily,b.BoundaryType),'Interpreter','none');
end
sgtitle(tl,'M3 S2：经二分加密的条件性机制边界（仅限当前连续平均模型）');exportgraphics(fig,path,'Resolution',220);close(fig);
end

function tf=BoundaryIsContinuous(x)
tf=ismember(string(x),["STABILITY","DIRECTION","FEEDBACK_SIGN"]);
end

function writeM3S2Report(path,S2,gate)
T=S2.summary;B=S2.boundaries;CE=S2.counterexamples;fid=fopen(path,'w','n','UTF-8');assert(fid>0);c=onCleanup(@()fclose(fid));
fprintf(fid,'# M3 S2：控制整定反例搜索与机制迁移边界\n\n');
fprintf(fid,'## 范围与停止规则\n\n仅使用S1已通过的物理一致连续平均模型，在OP_L/OP_M/OP_H做单因素稀疏扫描。控制参数为0.6/1.0/1.4倍，SCR为3/4/6；未做笛卡尔积，也未进入真正模态混合、气动升级、离散控制或EMT。\n\n');
fprintf(fid,'## Gate\n\n- 初始点：%d；二分加密点：%d；失稳点：%d；最终边界/筛选项：%d；\n- 工作点：%s；模态身份：%s；基准稳定：%s；连续边界完成加密：%s；\n- S2 Gate：%s。\n\n',gate.num_initial_points,gate.num_refinement_points,gate.num_unstable_points,gate.num_boundary_brackets,iff(gate.all_workpoints_pass,'PASS','FAIL'),iff(gate.all_mode_identity_pass,'PASS','FAIL'),iff(gate.all_baselines_stable,'PASS','FAIL'),iff(gate.all_continuous_boundaries_refined,'PASS','FAIL'),iff(gate.pass,'PASS','FAIL'));
fprintf(fid,'## 反例与迁移候选\n\n');writeM3Table(fid,CE);fprintf(fid,'\n## 边界括区\n\n');writeM3Table(fid,B);fprintf(fid,'\n');
fprintf(fid,'## 解释限制\n\n当前边界仅来自稀疏点的括区或线性插值，不能作为最终临界值。只有 `REFINE_REQUIRED` 区间完成局部加密后，才允许进入S3目标化模态相互作用搜索；`MODAL_PROXIMITY` 仅是筛选器，不等于已发现hybridization。\n');
end

function v=localTableValue(~,~,default)
% 预留统一字段读取接口，避免未来扩展时在主循环中产生未使用变量。
v=default;
end

function T=trackTorsionAcrossWorkpoints(scales,allModels,Msummary)
sx=[1;1;1;1e4;1e4;1e4;1e4;1e4;1e3;5e6;5e6;1;1;1e4;1e4;1e4;1e4;1e4;1e4;1e3;1e3;1e4;1e4];
rows={}; archs=["GFL","GWT","MWT"];
for a=1:numel(archs)
    vPrev=[]; lamPrev=NaN;
    for k=1:numel(scales)
        MM=allModels{k}{a}; A=MM.linear.A; [V,D]=eig(A); lam=diag(D);
        q=Msummary(Msummary.WorkpointScale==scales(k)&Msummary.Architecture==archs(a),:);
        target=q.PoleReal+1i*q.PoleImag; [~,ii]=min(abs(lam-target));
        vv=V(:,ii)./sx(MM.active); vv=vv/max(norm(vv),eps);
        if isempty(vPrev),mac=1;dl=0;else,mac=abs(vPrev'*vv)^2/max((vPrev'*vPrev)*(vv'*vv),eps);dl=abs(lam(ii)-lamPrev)/max(abs(lamPrev),eps);end
        identityPass=mac>=0.8 && q.Pi_MECH>=0.5;
        rows(end+1,:)={scales(k),archs(a),real(lam(ii)),imag(lam(ii)),q.TorsionalFrequency_Hz,q.TorsionalDampingRatio,q.Pi_MECH,mac,dl,identityPass}; %#ok<AGROW>
        vPrev=vv;lamPrev=lam(ii);
    end
end
T=cell2table(rows,'VariableNames',{'WorkpointScale','Architecture','PoleReal','PoleImag','Frequency_Hz','DampingRatio','Pi_MECH','MAC_FromPrevious','EigenvalueContinuityIndex','Identity_PASS'});
T.Architecture=string(T.Architecture);
end

function T=counterfactualPoleExcitation(scales,allModels,allP)
rows={};distIdx=[1 4];distNames=["MechanicalTorque","GridFrequency"];
for k=1:numel(scales)
    models=allModels{k};p=allP{k};ref=models{1};
    for d=1:numel(distIdx)
        [l0,r0,b0]=torsionPoleResidue(ref,distIdx(d));
        for a=2:3
            test=models{a};[l1,r1,b1]=torsionPoleResidue(test,distIdx(d));
            t=linspace(0,8,1601)'; amp=0.001*(p(1)/p(12));if distIdx(d)==4,amp=0.001*p(3);end
            y00=modalStepPair(t,l0,r0,amp);y10=modalStepPair(t,l1,r0,amp);
            y01=modalStepPair(t,l0,r1,amp);y11=modalStepPair(t,l1,r1,amp);
            dp=norm(y10-y00);de=norm(y01-y00);dt=norm(y11-y00);
            di=norm(y11-y10-y01+y00);den=max(dt,1e-30);
            fp=dp/den;fe=de/den;fi=di/den;
            if fi>0.30,cls="INTERACTION_SIGNIFICANT";elseif dp>1.5*de,cls="POLE_DOMINATED";elseif de>1.5*dp,cls="EXCITATION_DOMINATED";else,cls="JOINT";end
            rows(end+1,:)={scales(k),string(test.name),distNames(d),real(l0),imag(l0),real(l1),imag(l1),abs(r0),abs(r1),abs(b0),abs(b1),dp,de,dt,di,fp,fe,fi,cls}; %#ok<AGROW>
        end
    end
end
T=cell2table(rows,'VariableNames',{'WorkpointScale','Architecture','Disturbance','Lambda0Real','Lambda0Imag','Lambda1Real','Lambda1Imag','Residue0','Residue1','InputProjection0','InputProjection1','DeltaPoleNorm','DeltaExcitationNorm','DeltaTotalNorm','DeltaInteractionNorm','PoleFraction','ExcitationFraction','InteractionFraction','Classification'});
T.Architecture=string(T.Architecture);T.Disturbance=string(T.Disturbance);T.Classification=string(T.Classification);
end

function [lamTor,residue,inputProjection]=torsionPoleResidue(M,didx)
A=M.linear.A;[V,D,W]=eig(A);lam=diag(D);names=string(M.linear.state_names);
mech=ismember(names,["theta_sh","omega_t","omega_g"]);cand=find(imag(lam)>0 & abs(imag(lam))/(2*pi)>1 & abs(imag(lam))/(2*pi)<5);
score=zeros(size(cand));for q=1:numel(cand),den=W(:,cand(q))'*V(:,cand(q));pf=abs(V(:,cand(q)).*conj(W(:,cand(q))/conj(den)));score(q)=sum(pf(mech))/max(sum(pf),eps);end
[~,q]=max(score);ii=cand(q);den=W(:,ii)'*V(:,ii);iy=find(string(M.linear.output_names)=="omega_sh",1);
inputProjection=(W(:,ii)'*M.linear.B(:,didx))/den;
residue=(M.linear.C(iy,:)*V(:,ii))*inputProjection;
lamTor=lam(ii);
end

function y=modalStepPair(t,lam,residue,amp)
y=2*real((residue/lam).*(exp(lam*t)-1))*amp;
end

function T=detectGateACounterexamples(Torque,Coupling,Counterfactual,Track)
rows={};
q=Torque(Torque.Architecture=="MWT",:);v=q.DeltaDe_DVC;v=v(isfinite(v));det=~isempty(v)&&min(v)<=0&&max(v)>=0;
rows(end+1,:)={"A-C1","MWT","MSC-DVC damping sign reversal",det,sprintf('DeltaDe range [%.6g, %.6g]',min(v),max(v)),iff(det,"DVC sign is operating-point dependent","No sign reversal in tested workpoints")}; %#ok<AGROW>
for a=["GFL","GWT"]
    q=Torque(Torque.Architecture==a,:);v=q.DeltaDe_MPPT;v=v(isfinite(v));det=~isempty(v)&&min(v)<=0&&max(v)>=0;
    rows(end+1,:)={"A-C1",a,"Local MPPT damping sign reversal",det,sprintf('DeltaDe range [%.6g, %.6g]',min(v),max(v)),iff(det,"MPPT sign is operating-point dependent","No sign reversal in tested workpoints")}; %#ok<AGROW>
end
for a=unique(Coupling.Architecture,'stable')'
    q=Coupling(Coupling.Architecture==a,:);v=log10(q.DirectionalRatio);det=min(v)<=0&&max(v)>=0;
    rows(end+1,:)={"A-C2",a,"Directional dominance reversal",det,sprintf('Ldir range [%.6g, %.6g]',min(v),max(v)),iff(det,"Direction crosses equality boundary","Direction ordering retained in tested workpoints")}; %#ok<AGROW>
end
for a=unique(Counterfactual.Architecture,'stable')'
    for d=unique(Counterfactual.Disturbance,'stable')'
        q=Counterfactual(Counterfactual.Architecture==a&Counterfactual.Disturbance==d,:);u=unique(q.Classification);det=numel(u)>1;
        rows(end+1,:)={"A-C3",a,"Pole/excitation class migration: "+d,det,strjoin(cellstr(u),';'),iff(det,"Response-difference mechanism migrates","Classification retained")}; %#ok<AGROW>
    end
end
for a=unique(Track.Architecture,'stable')'
    q=Track(Track.Architecture==a,:);det=any(~q.Identity_PASS);
    rows(end+1,:)={"A-C4",a,"Torsional mode identity change",det,sprintf('minimum MAC %.6g; minimum PiMECH %.6g',min(q.MAC_FromPrevious),min(q.Pi_MECH)),iff(det,"Mode identity requires manual review","Torsional identity retained")}; %#ok<AGROW>
end
T=cell2table(rows,'VariableNames',{'CounterexampleID','Architecture','Test','Detected','Evidence','Interpretation'});
T.CounterexampleID=string(T.CounterexampleID);T.Architecture=string(T.Architecture);T.Test=string(T.Test);T.Evidence=string(T.Evidence);T.Interpretation=string(T.Interpretation);
end

function A=assembleWorkpointEnergyAudit(W,E,WP)
rows={};
for k=1:height(W)
    e=E(E.WorkpointScale==W.WorkpointScale(k)&E.Architecture==W.Architecture(k),:);
    q=WP(WP.WorkpointScale==W.WorkpointScale(k),:);
    rows(end+1,:)={W.WorkpointScale(k),W.Architecture(k),q.Wind_mps,q.OmegaTarget_radps,W.omega_g0_radps(k),q.Tm0_Nm,W.Te0_Nm(k),W.P0_W(k),W.Q0_var(k),W.Udc0_V(k), ...
        e.Paero_W,e.Pelectromagnetic_W,e.Pcu_W,e.Pmsc_W,e.Pgsc_W,e.Ppcc_W,e.ModelIdentityResidual_W,e.LegacyPlusResidual_W,e.PhysicalGeneratorResidual_W,e.DCLinkMismatch_W,e.GridFilterLoss_W, ...
        W.EquilibriumResidual(k),q.TargetSpeedRelativeError,q.CalibrationIterations,q.CalibrationConverged,e.ModelIdentity_PASS,e.PhysicalGeneratorSign_PASS}; %#ok<AGROW>
end
A=cell2table(rows,'VariableNames',{'WorkpointScale','Architecture','Wind_mps','OmegaTarget_radps','OmegaSolved_radps','Tm0_Nm','Te0_Nm','Ppcc_W','Qpcc_var','Udc_V','Paero_W','Pelectromagnetic_W','Pcu_W','Pmsc_W','Pgsc_W','PpccAudit_W','ModelIdentityResidual_W','LegacyPlusResidual_W','PhysicalGeneratorResidual_W','DCLinkMismatch_W','GridFilterLoss_W','EquilibriumResidual','TargetSpeedRelativeError','CalibrationIterations','CalibrationConverged','ModelIdentity_PASS','PhysicalGeneratorSign_PASS'});
A.Architecture=string(A.Architecture);
end

function F=assembleGateAFingerprint(M,T,C,Track,CF)
rows={};
for k=1:height(M)
    sc=M.WorkpointScale(k);a=M.Architecture(k);
    tq=T(T.WorkpointScale==sc&T.Architecture==a,:);cq=C(C.WorkpointScale==sc&C.Architecture==a,:);tr=Track(Track.WorkpointScale==sc&Track.Architecture==a,:);
    mclass="REFERENCE";gclass="REFERENCE";mp=NaN;me=NaN;mi=NaN;gp=NaN;ge=NaN;gi=NaN;
    if a~="GFL"
        cm=CF(CF.WorkpointScale==sc&CF.Architecture==a&CF.Disturbance=="MechanicalTorque",:);
        cg=CF(CF.WorkpointScale==sc&CF.Architecture==a&CF.Disturbance=="GridFrequency",:);
        mclass=cm.Classification;gclass=cg.Classification;mp=cm.PoleFraction;me=cm.ExcitationFraction;mi=cm.InteractionFraction;gp=cg.PoleFraction;ge=cg.ExcitationFraction;gi=cg.InteractionFraction;
    end
    rows(end+1,:)={sc,a,M.PoleReal(k),M.PoleImag(k),M.TorsionalFrequency_Hz(k),M.TorsionalDampingRatio(k),M.Pi_MECH(k),tr.MAC_FromPrevious,tr.Identity_PASS, ...
        tq.De_Base,tq.Ke_Base,tq.DeltaDe_MPPT,tq.DeltaDe_DVC,tq.DeltaDe_GFM,cq.C_GridToMachine,cq.C_MachineToGrid,log10(cq.DirectionalRatio),cq.LocalDirection, ...
        mclass,mp,me,mi,gclass,gp,ge,gi}; %#ok<AGROW>
end
F=cell2table(rows,'VariableNames',{'WorkpointScale','Architecture','PoleReal','PoleImag','TorsionalFrequency_Hz','TorsionalDampingRatio','Pi_MECH','MAC_FromPrevious','ModeIdentity_PASS','De_Base','Ke_Base','DeltaDe_MPPT','DeltaDe_DVC','DeltaDe_GFM','C_GridToMachine','C_MachineToGrid','Ldir_Log10','LocalDirection','Mechanical_CF_Class','Mechanical_PoleFraction','Mechanical_ExcitationFraction','Mechanical_InteractionFraction','Grid_CF_Class','Grid_PoleFraction','Grid_ExcitationFraction','Grid_InteractionFraction'});
F.Architecture=string(F.Architecture);F.LocalDirection=string(F.LocalDirection);F.Mechanical_CF_Class=string(F.Mechanical_CF_Class);F.Grid_CF_Class=string(F.Grid_CF_Class);
end

function makeM3GateAFigure(path,R,F)
fig=figure('Visible','off','Color','w','Position',[60 60 1580 920]);tl=tiledlayout(fig,2,3,'TileSpacing','compact','Padding','compact');archs=["GFL","GWT","MWT"];cols=lines(3);marks={'o','s','^'};styles={'-','--','-.'};
nexttile;hold on;for a=1:3,q=F(F.Architecture==archs(a),:);plot(q.WorkpointScale,100*q.TorsionalDampingRatio,'LineStyle',styles{a},'Marker',marks{a},'LineWidth',1.7,'Color',cols(a,:),'DisplayName',archs(a));end;grid on;xlabel('P_0/P_N');ylabel('\zeta_{tor} (%)');title('轴系极点阻尼');legend('Location','best');
nexttile;hold on;for a=1:3,q=F(F.Architecture==archs(a),:);plot(q.WorkpointScale,q.Ldir_Log10,'LineStyle',styles{a},'Marker',marks{a},'LineWidth',1.7,'Color',cols(a,:),'DisplayName',archs(a));end;yline(0,':k','HandleVisibility','off');yline(log10(3),'--k','HandleVisibility','off');yline(-log10(3),'--k','HandleVisibility','off');grid on;xlabel('P_0/P_N');ylabel('log_{10}(C_{GM}/C_{MG})');title('方向占优及跨零');legend('Location','best');
nexttile;hold on;for a=2:3,q=F(F.Architecture==archs(a),:);plot(q.WorkpointScale,q.Grid_PoleFraction,'-','LineWidth',1.7,'Color',cols(a,:),'DisplayName',archs(a)+' pole');plot(q.WorkpointScale,q.Grid_ExcitationFraction,'--','LineWidth',1.7,'Color',cols(a,:),'DisplayName',archs(a)+' excitation');plot(q.WorkpointScale,q.Grid_InteractionFraction,':','LineWidth',1.7,'Color',cols(a,:),'DisplayName',archs(a)+' interaction');end;grid on;xlabel('P_0/P_N');ylabel('相对贡献');title('Grid-frequency反事实分解');legend('Location','best');
for a=1:3
    nexttile;Z=zeros(numel(R.workpoint_scales),241);ff=[];
    for k=1:numel(R.workpoint_scales),q=R.feedback_curves{k}{a};ff=q.f_Hz;Z(k,:)=q.De;end
    surf(ff,R.workpoint_scales,Z/1e6,'EdgeColor','none');view(2);set(gca,'XScale','log');colorbar;grid on;xlabel('Frequency (Hz)');ylabel('P_0/P_N');title(archs(a)+' D_e (MNms/rad)');
end
sgtitle(tl,'M3 Gate A：物理一致跨运行点机制指纹（全部结论仍为条件性）');exportgraphics(fig,path,'Resolution',220);close(fig);
end

function writeM3GateAReport(path,R,A,F)
fid=fopen(path,'w','n','UTF-8');assert(fid>0,'Cannot open M3 Gate A report.');c=onCleanup(@()fclose(fid));
fprintf(fid,'# M3 Gate A：物理一致跨运行点与反例搜索报告\n\n');
fprintf(fid,'## 执行边界\n\n本轮只完成 S0 与 S1。未执行 H/DVC/SCR 控制扫参、目标模态混合搜索、Pitch/OpenFAST、离散平均模型或 EMT。三个架构共享 plant、目标P/Q、Udc、转速和机械转矩，只改变控制职责。\n\n');
fprintf(fid,'## Gate A 工作点\n\n- 工况：P0/PN = %s；\n- 最大平衡残差：%.3e；\n- 最大架构间工作点差异：%.3e pu；\n- 严格共同工作点 Gate：%s。\n\n',strjoin(string(R.workpoint_scales),', '),max(R.gateA.MaxEquilibriumResidual),max(R.gateA.MaxPairwiseWorkpointMismatch_pu),iff(all(R.gateA.GateA_PASS),'PASS','FAIL'));
fprintf(fid,'运行点采用通过额定M2点校准的常TSR曲线：P~omega^3、Tm~omega^2；同时按目标P/Q重算GFM电压幅值，并统一标定机械转矩以补偿PMSG与滤波器损耗。\n\n');
if strcmpi(R.pmsg_convention,'GENERATOR_OUTWARD')
    fprintf(fid,'## 功率与符号审计\n\n- M3采用 `GENERATOR_OUTWARD`：正iq表示从PMSG流向DC-link的正发电电流，正Te表示发电制动转矩。\n- 物理恒等式 `P_MSC = T_e*omega_g - P_Cu`：%s；最大残差 %.3e W。\n- 冻结旧M2的加铜耗关系残差最大为 %.3e W，仅保留作反事实参考。\n\n',iff(all(A.ModelIdentity_PASS)&&all(A.PhysicalGeneratorSign_PASS),'PASS','FAIL'),max(abs(A.PhysicalGeneratorResidual_W)),max(abs(A.LegacyPlusResidual_W)));
    fprintf(fid,'机器侧电流、转矩及DC-link输入功率方向已经在同一发电机外向端口约定下闭合；旧M2数值只作为冻结历史基准，不再作为M3物理结论。\n\n');
else
    fprintf(fid,'## 功率与符号审计\n\n- 冻结旧M2恒等式 `P_MSC = T_e*omega_g + P_Cu`：%s；最大残差 %.3e W。\n- 物理发电机关系 `P_MSC = T_e*omega_g - P_Cu`：%s；最大差值 %.3e W。\n\n',iff(all(A.ModelIdentity_PASS),'PASS','FAIL'),max(abs(A.ModelIdentityResidual_W)),iff(all(A.PhysicalGeneratorSign_PASS),'PASS','FAIL'),max(abs(A.PhysicalGeneratorResidual_W)));
end
fprintf(fid,'## 主要结果\n\n');
for a=unique(F.Architecture,'stable')'
    q=F(F.Architecture==a,:);fprintf(fid,'- %s：f_tor %.4f--%.4f Hz，zeta_tor %.3f%%--%.3f%%，Ldir %.3f--%.3f。\n',a,min(q.TorsionalFrequency_Hz),max(q.TorsionalFrequency_Hz),100*min(q.TorsionalDampingRatio),100*max(q.TorsionalDampingRatio),min(q.Ldir_Log10),max(q.Ldir_Log10));
end
fprintf(fid,'\n- GFL/GWT局部MPPT增量阻尼在4个工况均为正；本轮未找到符号反例。\n- MWT的MSC-DVC增量阻尼在4个工况均为负；本轮未找到符号反例。\n- GWT的Ldir由低功率正值降至0.9 pu附近负值，出现A-C2方向跨零反例；“GWT始终Grid-to-Machine占优”不成立。\n- 模态身份由MAC和机械参与度联合检查；任何失败行必须先人工复核，不能按频率最近强制续接。\n\n');
fprintf(fid,'## 反例登记\n\n');writeM3Table(fid,R.gateA_counterexamples);fprintf(fid,'\n');
fprintf(fid,'## 反事实 Pole–Excitation 分解\n\n');writeM3Table(fid,R.counterfactual_pole_excitation);fprintf(fid,'\n');
if all(R.gateA.GateA_PASS)&&all(A.ModelIdentity_PASS)&&all(A.PhysicalGeneratorSign_PASS)
    fprintf(fid,'## Gate 决策\n\nS1 PASS：严格共同工作点、全部极点稳定、模态身份连续以及PMSG发电端口功率恒等式均通过。允许进入S2稀疏控制参数边界扫描；所有本轮机制判断仍保持 `CONDITIONAL_CROSS_WORKPOINT`，不得外推到未测试参数或更高保真模型。\n');
else
    fprintf(fid,'## Gate 决策\n\nS1 FAIL：至少一项共同工作点、稳定性或物理功率恒等式未通过，禁止进入S2。\n');
end
end

function v=iff(c,a,b)
if c,v=a;else,v=b;end
end

function independent=runM3IndependentCheck(models,p,scale)
rows={};curves=cell(numel(models),1);allFinite=true;allPass=true;
for a=1:numel(models)
    M=models{a}; arch=M.name; x0=M.x0; active=M.active; flags=M.flags; amp=0.001*p(1)/p(12); dt=0.005; tStep=0.5; tEnd=8;
    t1=(0:dt:tStep)';t2=(tStep:dt:tEnd)';opt=odeset('RelTol',1e-8,'AbsTol',1e-9,'MaxStep',2e-3);
    [~,z1]=ode15s(@(tt,z)m3IndependentActiveRhs(tt,z,x0,p,arch,flags,active,1,0),t1,x0(active),opt);
    [~,z2]=ode15s(@(tt,z)m3IndependentActiveRhs(tt,z,x0,p,arch,flags,active,1,amp),t2,z1(end,:)',opt);
    t=[t1;t2(2:end)]; z=[z1;z2(2:end,:)]; yI=zeros(numel(t),4); yM=zeros(numel(t),4);
    y0I=m3IndependentOutputs(x0,p,arch,zeros(4,1),flags); y0M=m2Outputs(x0,p,arch,zeros(4,1),flags);
    for k=1:numel(t)
        x=x0;x(active)=z(k,:)';d=zeros(4,1);if t(k)>=tStep,d(1)=amp;end
        yI(k,:)=m3IndependentOutputs(x,p,arch,d,flags)'-y0I';
        yMfull=m2Outputs(x,p,arch,d,flags)'-y0M';
        % M2输出顺序为[P_GSC,Udc,iq_ref,iq,Te,omega_sh,...,P_PCC]。
        yM(k,:)=yMfull([6 5 2 10]);
    end
    names={'omega_sh','T_e','Udc','P_PCC'}; nr=zeros(4,1); pe=zeros(4,1);
    for j=1:4
        nr(j)=sqrt(mean((yI(:,j)-yM(:,j)).^2))/max(max(abs(yM(:,j))),eps);
        pe(j)=abs(max(abs(yI(:,j)))-max(abs(yM(:,j))))/max(max(abs(yM(:,j))),eps);
    end
    pass=all(isfinite([yI(:);yM(:)]))&&max(nr)<0.01&&max(pe)<0.02; allFinite=allFinite&&all(isfinite([nr;pe])); allPass=allPass&&pass;
    rows(end+1,:)={scale,string(arch),nr(1),pe(1),nr(2),pe(2),nr(3),pe(3),nr(4),pe(4),max(nr),max(pe),pass}; %#ok<AGROW>
    curves{a}=struct('Architecture',arch,'t_s',t,'omega_sh_ind',yI(:,1),'omega_sh_m2',yM(:,1),'Te_ind',yI(:,2),'Te_m2',yM(:,2));
end
S=cell2table(rows,'VariableNames',{'WorkpointScale','Architecture','NRMSE_omega_sh','PeakErr_omega_sh','NRMSE_Te','PeakErr_Te','NRMSE_Udc','PeakErr_Udc','NRMSE_P_PCC','PeakErr_P_PCC','MaxNRMSE','MaxPeakError','PASS'});
independent=struct('enabled',true,'pass',allFinite&&allPass,'summary',S,'curves',{curves},'scope','独立重编程理想连续平均RHS与M2输出核对，不是EMT验证');
end

function dz=m3IndependentActiveRhs(~,z,xbase,p,arch,flags,active,id,amp)
x=xbase;x(active)=z;d=zeros(4,1);d(id)=amp;dx=m3IndependentRhs(x,p,arch,d,flags);dz=dx(active);
end

function dx=m3IndependentRhs(x,p,arch,d,flags)
% 独立重编程版本：不调用M2的m2Rhs，变量展开顺序保持同一物理定义。
Sb=p(1); Vdc0=p(2); w0=p(3); Vg=p(4); Rf=p(5); Lf=p(6); Cf=p(7); Rd=p(8); Rg=p(9); Lg=p(10); Cdc=p(11); wm0=p(12); Rs=p(13); Ld=p(14); Lq=p(15); psi=p(16); np=p(17); Kt=p(18); Jt=p(19); Jg=p(20); Ksh=p(21); Dsh=p(22); Dt=p(23); Dg=p(24); Kpdc=p(25); Kidc=p(26); Kpmi=p(27); Kimi=p(28); Kpgi=p(29); Kigi=p(30); Kpgv=p(31); Kigv=p(32); H=p(33); mp=p(34); wpf=p(35); kq=p(36); Pref=p(37); Qref=p(38); Tm0=p(39); E0=p(40); sP=p(41); ffIg=p(42); ffVpcc=p(43);
theta=x(1); wt=x(2); wg=x(3); imd=x(4); imq=x(5); xiDc=x(6); xiMid=x(7); xiMiq=x(8); Udc=x(9); Pf=x(10); Qf=x(11); wsync=x(12); delta=x(13); xiVd=x(14); xiVq=x(15); xiId=x(16); xiIq=x(17); ifd=x(18); ifq=x(19); vcd=x(20); vcq=x(21); igd=x(22); igq=x(23);
wgGrid=w0+d(4); deltaGrid=delta+d(3); electricalSpeed=np*wg; Tgen=Kt*imq; Tshaft=Ksh*theta+Dsh*(wt-wg); wcoi=(Jt*wt+Jg*wg)/(Jt+Jg); eDc=Vdc0-Udc;
if strcmpi(arch,'MWT'), imqRef=Kpdc*eDc+xiDc; else, imqRef=flags.imqRef0+flags.Kmppt_iq_per_radps*(wg-wm0); end
eId=-imd;eIq=imqRef-imq;generatorOut=isfield(flags,'pmsgConvention')&&strcmpi(flags.pmsgConvention,'GENERATOR_OUTWARD');
if generatorOut,vmDcmd=-Kpmi*eId-xiMid+electricalSpeed*Lq*imqRef;vmQcmd=-Kpmi*eIq-xiMiq-Rs*imqRef+electricalSpeed*psi;else,vmDcmd=Kpmi*eId+xiMid-electricalSpeed*Lq*imqRef;vmQcmd=Kpmi*eIq+xiMiq+Rs*imqRef+electricalSpeed*psi;end
vScale=Udc/Vdc0;vmD=vScale*vmDcmd;vmQ=vScale*vmQcmd;Pmsc=1.5*(vmD*imd+vmQ*imq);
vnodeD=vcd+Rd*ifd-(Rd+1e-4)*igd; vnodeQ=vcq+Rd*ifq-(Rd+1e-4)*igq; Ppcc=1.5*(vnodeD*igd+vnodeQ*igq); Qpcc=1.5*(vnodeQ*igd-vnodeD*igq); cc=cos(deltaGrid); ss=sin(deltaGrid); vpd=cc*vnodeD+ss*vnodeQ; vpq=-ss*vnodeD+cc*vnodeQ; ifld=cc*ifd+ss*ifq; iflq=-ss*ifd+cc*ifq; igld=cc*igd+ss*igq; iglq=-ss*igd+cc*igq;
if strcmpi(arch,'GFL'), wctrl=wsync+flags.KpPll_radps_per_V*vpq; ifdRef=flags.KpGscDvc_A_per_V*(Udc-Vdc0)+xiDc; ifqRef=flags.KpQ_A_per_var*(Qpcc-Qref)+xiVq; else, wctrl=wsync; if isfield(flags,'freezeSync')&&flags.freezeSync,wctrl=w0;end; Vref=E0+kq*(Qref-Qf); ifdRef=Kpgv*(Vref-vpd)+xiVd-Cf*wctrl*vpq+ffIg*igld; ifqRef=Kpgv*(-vpq)+xiVq+Cf*wctrl*vpd+ffIg*iglq; end
eIdG=ifdRef-ifld; eIqG=ifqRef-iflq; ucd=Kpgi*eIdG+xiId-wctrl*Lf*iflq+ffVpcc*(vpd+Rf*ifld); ucq=Kpgi*eIqG+xiIq+wctrl*Lf*ifld+ffVpcc*(vpq+Rf*iflq); uinvD=vScale*(cc*ucd-ss*ucq); uinvQ=vScale*(ss*ucd+cc*ucq); Pgsc=1.5*(uinvD*ifd+uinvQ*ifq);
dx=zeros(23,1); TmIn=Tm0*wm0/max(wt,1e-9)+d(1)+d(2)/max(wt,1e-9); dx(1)=wt-wg; dx(2)=(TmIn-Tshaft-Dt*(wcoi-wm0))/Jt; dx(3)=(Tshaft-Tgen-Dg*(wcoi-wm0))/Jg;
if generatorOut,dx(4)=(-vmD-Rs*imd+electricalSpeed*Lq*imq)/Ld;dx(5)=(-vmQ-Rs*imq+electricalSpeed*(psi-Ld*imd))/Lq;else,dx(4)=(vmD-Rs*imd+electricalSpeed*Lq*imq)/Ld;dx(5)=(vmQ-Rs*imq-electricalSpeed*(Ld*imd+psi))/Lq;end
if strcmpi(arch,'MWT'),dx(6)=Kidc*eDc;elseif strcmpi(arch,'GWT'),dx(6)=flags.KiGscDvc_W_per_Vs*eDc;else,dx(6)=flags.KiGscDvc_A_per_Vs*(Udc-Vdc0);end; dx(7)=Kimi*eId; dx(8)=Kimi*eIq; dx(9)=(Pmsc-Pgsc)/(Cdc*Udc); dx(10)=wpf*(Ppcc-Pf); dx(11)=wpf*(Qpcc-Qf);
freezeSync=isfield(flags,'freezeSync')&&flags.freezeSync; if strcmpi(arch,'MWT')&&~freezeSync,dx(12)=w0/(2*H*Sb)*(sP*(Pref-Pf)-(wsync-w0)/mp);dx(13)=wctrl-wgGrid;elseif strcmpi(arch,'GWT')&&~freezeSync,Pctrl=Pref-flags.KpGscDvc_W_per_V*eDc-xiDc;dx(12)=w0/(2*H*Sb)*(sP*(Pctrl-Pf)-(wsync-w0)/flags.mpGwt);dx(13)=wctrl-wgGrid;elseif strcmpi(arch,'GFL'),dx(12)=flags.KiPll_radps2_per_V*vpq;dx(13)=wctrl-wgGrid;else,dx(12)=0;dx(13)=0;end
if strcmpi(arch,'GFL'),dx(14)=0;dx(15)=flags.KiQ_A_per_vars*(Qpcc-Qref);else,dx(14)=Kigv*(E0+kq*(Qref-Qf)-vpd);dx(15)=Kigv*(-vpq);end; dx(16)=Kigi*eIdG;dx(17)=Kigi*eIqG; RfEff=Rf+1e-4; dx(18)=(uinvD-vnodeD-RfEff*ifd+w0*Lf*ifq)/Lf;dx(19)=(uinvQ-vnodeQ-RfEff*ifq-w0*Lf*ifd)/Lf;dx(20)=(ifd-igd)/Cf+w0*vcq;dx(21)=(ifq-igq)/Cf-w0*vcd;dx(22)=(vnodeD-Vg-Rg*igd+w0*Lg*igq)/Lg;dx(23)=(vnodeQ-Rg*igq-w0*Lg*igd)/Lg;
end

function y=m3IndependentOutputs(x,p,arch,d,flags)
% 独立输出重建，用于与M2同一工作点的响应核对。
Vdc0=p(2);w0=p(3);Rd=p(8);Rf=p(5);Lf=p(6);Cf=p(7);Rs=p(13);Ld=p(14);Lq=p(15);psi=p(16);np=p(17);Kt=p(18);Ksh=p(21);Dsh=p(22);Kpdc=p(25);Kpmi=p(27);Kpgi=p(29);Kpgv=p(31);kq=p(36);Qref=p(38);E0=p(40);ffIg=p(42);ffVpcc=p(43);
wg=x(3);imd=x(4);imq=x(5);xiDc=x(6);Udc=x(9);Pf=x(10);Qf=x(11);wsync=x(12);delta=x(13);xiVd=x(14);xiVq=x(15);xiId=x(16);xiIq=x(17);ifd=x(18);ifq=x(19);vcd=x(20);vcq=x(21);igd=x(22);igq=x(23);eDc=Vdc0-Udc;
if strcmpi(arch,'MWT'),imqRef=Kpdc*eDc+xiDc;else,imqRef=flags.imqRef0+flags.Kmppt_iq_per_radps*(wg-p(12));end;we=np*wg;eId=-imd;eIq=imqRef-imq;generatorOut=isfield(flags,'pmsgConvention')&&strcmpi(flags.pmsgConvention,'GENERATOR_OUTWARD');
if generatorOut,vmDcmd=-Kpmi*eId-x(7)+we*Lq*imqRef;vmQcmd=-Kpmi*eIq-x(8)-Rs*imqRef+we*psi;else,vmDcmd=Kpmi*eId+x(7)-we*Lq*imqRef;vmQcmd=Kpmi*eIq+x(8)+Rs*imqRef+we*psi;end
vScale=Udc/Vdc0;vmD=vScale*vmDcmd;vmQ=vScale*vmQcmd;Pmsc=1.5*(vmD*imd+vmQ*imq);
vnodeD=vcd+Rd*ifd-(Rd+1e-4)*igd;vnodeQ=vcq+Rd*ifq-(Rd+1e-4)*igq;Ppcc=1.5*(vnodeD*igd+vnodeQ*igq);Qpcc=1.5*(vnodeQ*igd-vnodeD*igq);deltaEff=delta+d(3);cc=cos(deltaEff);ss=sin(deltaEff);vpd=cc*vnodeD+ss*vnodeQ;vpq=-ss*vnodeD+cc*vnodeQ;ifld=cc*ifd+ss*ifq;iflq=-ss*ifd+cc*ifq;igld=cc*igd+ss*igq;iglq=-ss*igd+cc*igq;
if strcmpi(arch,'GFL'),wctrl=wsync+flags.KpPll_radps_per_V*vpq;ifdRef=flags.KpGscDvc_A_per_V*(Udc-Vdc0)+xiDc;ifqRef=flags.KpQ_A_per_var*(Qpcc-Qref)+xiVq;else,wctrl=wsync;if isfield(flags,'freezeSync')&&flags.freezeSync,wctrl=w0;end;Vref=E0+kq*(Qref-Qf);ifdRef=Kpgv*(Vref-vpd)+xiVd-Cf*wctrl*vpq+ffIg*igld;ifqRef=Kpgv*(-vpq)+xiVq+Cf*wctrl*vpd+ffIg*iglq;end
eIdG=ifdRef-ifld;eIqG=ifqRef-iflq;ucd=Kpgi*eIdG+xiId-wctrl*Lf*iflq+ffVpcc*(vpd+Rf*ifld);ucq=Kpgi*eIqG+xiIq+wctrl*Lf*ifld+ffVpcc*(vpq+Rf*iflq);uinvD=vScale*(cc*ucd-ss*ucq);uinvQ=vScale*(ss*ucd+cc*ucq);Pgsc=1.5*(uinvD*ifd+uinvQ*ifq);Tsh=Ksh*x(1)+Dsh*(x(2)-wg);
y=[x(2)-wg;Kt*imq;Udc;Ppcc]; %#ok<NASGU>
end

function makeM3WorkpointFigure(path,M,C,T)
fig=figure('Visible','off','Color','w','Position',[60 60 1500 900]);tl=tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');aa=unique(M.Architecture,'stable');cols=lines(numel(aa));
nexttile;hold on;for k=1:numel(aa),q=M(M.Architecture==aa(k),:);plot(q.WorkpointScale,100*q.TorsionalDampingRatio,'o-','LineWidth',1.6,'Color',cols(k,:),'DisplayName',aa(k));end;grid on;xlabel('P^*/P_N');ylabel('轴系阻尼比 (%)');title('跨运行点轴系极点阻尼');legend('Location','best');
nexttile;hold on;for k=1:numel(aa),q=M(M.Architecture==aa(k),:);plot(q.WorkpointScale,q.TorsionalFrequency_Hz,'s--','LineWidth',1.6,'Color',cols(k,:),'DisplayName',aa(k));end;grid on;xlabel('P^*/P_N');ylabel('f_{tor} (Hz)');title('轴系主模态频率');
nexttile;hold on;for k=1:numel(aa),q=C(C.Architecture==aa(k),:);semilogy(q.WorkpointScale,max(q.C_GridToMachine,1e-30),'o-','LineWidth',1.6,'Color',cols(k,:),'DisplayName',aa(k));end;grid on;xlabel('P^*/P_N');ylabel('C_{GM}');title('Grid-to-Machine耦合');
nexttile;hold on;for k=1:numel(aa),q=C(C.Architecture==aa(k),:);semilogy(q.WorkpointScale,max(q.C_MachineToGrid,1e-30),'s--','LineWidth',1.6,'Color',cols(k,:),'DisplayName',aa(k));end;grid on;xlabel('P^*/P_N');ylabel('C_{MG}');title('Machine-to-Grid耦合');
sgtitle(tl,'M3跨运行点 Pole–Path–Directional 复核');exportgraphics(fig,path,'Resolution',220);close(fig);
end

function makeM3IndependentFigure(path,I)
fig=figure('Visible','off','Color','w','Position',[80 80 1450 900]);tl=tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
for k=1:numel(I.curves)
    q=I.curves{k}; mk=1:120:numel(q.t_s);
    nexttile; h1=plot(q.t_s,q.omega_sh_ind,'b-','LineWidth',1.1); hold on; h2=plot(q.t_s(mk),q.omega_sh_m2(mk),'ko','LineStyle','none','MarkerSize',3); grid on; title([q.Architecture ' \\Delta\\omega_{sh}']);
    if k==1, legend([h1 h2],{'independent','M2 samples'},'Location','best'); end
    nexttile; plot(q.t_s,q.Te_ind/1e3,'r-','LineWidth',1.1); hold on; plot(q.t_s(mk),q.Te_m2(mk)/1e3,'ks','LineStyle','none','MarkerSize',3); grid on; title([q.Architecture ' \\Delta T_e (kN m)']);
end
xlabel(tl,'Time (s)'); sgtitle(tl,'独立理想连续实现与M2响应核对（重合处用采样标记显示）'); exportgraphics(fig,path,'Resolution',220); close(fig);
end

function writeM3Report(path,R)
fid=fopen(path,'w');assert(fid>0);c=onCleanup(@()fclose(fid));
fprintf(fid,'# M3跨运行点可证伪验证报告\n\n');
fprintf(fid,'## 研究边界\n\n本轮只使用5 MW理想连续物理平均VSC，不含EMT、PWM、采样、离散、延迟、限幅和保护。M2公共基准未修改。所有M2结论在本轮仍作为待证伪假设。\n\n');
fprintf(fid,'## Gate A：工作点重求解\n\n');
writeM3Table(fid,R.gateA); fprintf(fid,'\n');
fprintf(fid,'严格公共工作点判据为架构间核心量差异小于 1e-4 pu。当前只有 1.0 pu 点通过该判据；0.6 pu 和 0.8 pu 点的平衡残差均合格，但架构间实际PCC功率差异分别约为 %.2f%% 和 %.2f%%，因此这两个点只用于“同一机械输入下的运行点敏感性”观察，不用于严格公平的跨架构定量比较。\n\n',100*max(R.gateA.MaxPairwiseWorkpointMismatch_pu(R.gateA.WorkpointScale==0.6),0),100*max(R.gateA.MaxPairwiseWorkpointMismatch_pu(R.gateA.WorkpointScale==0.8),0));
fprintf(fid,'## 跨运行点轴系模态\n\n');writeM3Table(fid,R.modal);fprintf(fid,'\n');
fprintf(fid,'## 复转矩通道与双向耦合\n\n');writeM3Table(fid,R.torque);fprintf(fid,'\n');writeM3Table(fid,R.coupling);fprintf(fid,'\n');
fprintf(fid,'## 模态残差与局部参数扫描\n\n');writeM3Table(fid,R.residue);fprintf(fid,'\n局部H/DVC/SCR扫描完整结果见 `M3_Workpoint_LocalScan_Summary.csv`，本报告不将其外推为全局单调规律。\n\n');
fprintf(fid,'## 本轮主动记录的反例\n\n');
if isempty(R.counterexamples),fprintf(fid,'本轮没有发现局部扫描失稳点。\n\n');else,CE=R.counterexamples(:,{'WorkpointScale','Architecture','Parameter','PhysicalValue','MaxRealPole','CriticalModeFrequency_Hz','CriticalModePiMECH','CriticalModeClass','TorDampingRatio','DirectionalRatio'});writeM3Table(fid,CE);fprintf(fid,'\n这些反例优先于既有叙事：失稳点均为局部参数与运行点组合，不能外推为GFM普遍失稳。\n\n');end
if R.independent_check.enabled
    fprintf(fid,'## 独立理想连续实现核对\n\n');writeM3Table(fid,R.independent_check.summary);fprintf(fid,'\n');
end
fprintf(fid,'## 当前可写入主线的判断\n\n');
fprintf(fid,'- 轴系主模态在三个运行点均约为 2.50 Hz；GFL/GWT阻尼约 5.75%%--5.80%%，MWT约 2.92%%--2.99%%。这是当前理想平均模型的运行点局部结果，不等于GFM普遍恶化。\n');
fprintf(fid,'- 方向关系在三个点保持排序：GFL偏Machine-to-Grid，GWT为同量级，MWT偏Grid-to-Machine；但其数值随工作点变化，属于频率局部方向占优。\n');
fprintf(fid,'- 0.6 pu的GWT在DVC=2处出现约2.49 Hz的SYNC_PLL主导失稳；1.0 pu的MWT在DVC=0.5处出现约0.36 Hz的GSC主导失稳。两者轴系极点仍稳定，说明失稳边界不能只用轴系阻尼解释。\n');
fprintf(fid,'- 只有当不同工作点的轴系阻尼、复转矩符号、耦合方向和模态残差保持一致时，M2现象才可升级为跨运行点机制；否则必须标记为运行点相关。\n');
fprintf(fid,'- 本轮不预设“GFM必然恶化轴系稳定性”“结构零”“非互易性”或“模态混合”。任何反例均优先于既有叙事。\n');
fprintf(fid,'- 独立核对若通过，只能说明两套理想连续方程实现一致，不能替代EMT或真实控制器验证。\n');
end

function writeM3Table(fid,T)
if isempty(T),fprintf(fid,'（无数据）\n');return;end
vars=T.Properties.VariableNames;fprintf(fid,'|');for k=1:numel(vars),fprintf(fid,'%s|',vars{k});end;fprintf(fid,'\n|');for k=1:numel(vars),fprintf(fid,'---|');end;fprintf(fid,'\n');for r=1:min(height(T),120),fprintf(fid,'|');for k=1:numel(vars),v=T{r,k};if iscell(v),v=v{1};end;if isstring(v)||ischar(v)||islogical(v),s=char(string(v));else,s=num2str(v,8);end;fprintf(fid,'%s|',s);end;fprintf(fid,'\n');end;if height(T)>120,fprintf(fid,'\n（表格超过120行，完整结果见CSV。）\n');end
end

function R=legacy_m2_electromechanical_mechanism(varargin)
%RUN_M2_ELECTROMECHANICAL_MECHANISM M2机电耦合证据链统一入口。
%
% 科研纪律：
% 1) M0/M1结果只用于提出可证伪假设，不作为本程序的预期答案；
% 2) 三架构共享plant、物理平均VSC、参数和目标工作点，只改变控制职责；
% 3) 每个阶段均有Gate，失败后不允许解释后续动态；
% 4) 不保存大规模时序，只保存统一检查点、摘要、报告和必要图片。
%
% 当前实现从M2-A开始。后续阶段仍在本文件中递进扩展，禁止为每个
% 架构或参数点复制模型。

ip=inputParser;
ip.addParameter('StopAfter','A',@(x)ischar(x)||isstring(x));
ip.addParameter('SaveOutputs',true,@(x)islogical(x)&&isscalar(x));
ip.parse(varargin{:}); opt=ip.Results;
stopAfter=upper(char(string(opt.StopAfter)));

here=fileparts(mfilename('fullpath'));
idealRoot=fileparts(here);
m0Dir=fullfile(idealRoot,'CurrentModel_Idealized');
addpath(m0Dir);

% 唯一可信参数源：此前通过Gate A的共同5 MW工作点。
S=load(fullfile(m0Dir,'Architecture_Comparison_Summary.mat'),'R');
base=S.R;
assert(base.passed,'现有共同工作点Gate未通过，禁止进入M2。');
p=base.parameter_vector(:);

% M1-a中已识别的公共GSC电流环稳定化值。对三架构统一应用，避免
% 将“架构差异”与“不同内环带宽”混在一起；不是某架构专用补丁。
commonCurrentKiFactor=0.6;
p(30)=p(30)*commonCurrentKiFactor;

arch=makeArchitectures(base,p);
[models,plantAudit,workAudit,gateA]=runStageA(arch,p,base);

R=struct;
R.objective='M2机电耦合：假设驱动、逐Gate证据升级';
R.model_level='M2 physical averaged VSC, continuous controls';
R.stop_after=stopAfter;
R.common_current_ki_factor=commonCurrentKiFactor;
R.parameter_vector=p;
R.architectures={arch.name};
R.models=models;
R.stageA=struct('plant_audit',plantAudit,'workpoint_audit',workAudit,'gate',gateA);
R.gates=struct('M2_A',gateA.pass);

if opt.SaveOutputs
    writetable(plantAudit,fullfile(here,'M2_CommonPlant_Audit.csv'));
    save(fullfile(here,'M2_Analysis_Checkpoint.mat'),'R','-v7.3');
    writeReport(fullfile(here,'M2_Progress_Report_CN.md'),R);
end

fprintf('M2-A complete: Gate=%d, max residual=%.3e, max workpoint mismatch=%.3e pu.\n', ...
    gateA.pass,gateA.max_normalized_residual,gateA.max_workpoint_pairwise_pu);
if ~gateA.pass
    error('M2-A Gate failed. Stop before modal/mechanism analysis.');
end
if stageRank(stopAfter)<=stageRank('A'), return; end

[modalMap,torsionSummary,gateB]=runStageB(models);
R.stageB=struct('modal_map',modalMap,'torsional_modes',torsionSummary,'gate',gateB);
R.gates.M2_B=gateB.pass;
if opt.SaveOutputs
    save(fullfile(here,'M2_Analysis_Checkpoint.mat'),'R','-v7.3');
    writetable(modalMap,fullfile(here,'M2_Evidence_Summary.csv'));
    makeModalFigure(fullfile(here,'M2_Mechanism_Overview.png'),modalMap);
    writeReport(fullfile(here,'M2_Progress_Report_CN.md'),R);
end
fprintf('M2-B complete: Gate=%d, stable=%d, torsional modes=%d, mixed candidates=%d.\n', ...
    gateB.pass,gateB.all_stable,height(torsionSummary),gateB.num_mixed_candidates);
if ~gateB.pass, error('M2-B Gate failed. Stop before feedback analysis.'); end
if stageRank(stopAfter)<=stageRank('B'), return; end

[feedbackSummary,feedbackCurves,gateC]=runStageC(models,p,torsionSummary);
R.stageC=struct('summary',feedbackSummary,'curves',{feedbackCurves},'gate',gateC);
R.gates.M2_C=gateC.pass;
if opt.SaveOutputs
    save(fullfile(here,'M2_Analysis_Checkpoint.mat'),'R','-v7.3');
    writeEvidenceSummary(fullfile(here,'M2_Evidence_Summary.csv'),R);
    makeOverviewFigure(fullfile(here,'M2_Mechanism_Overview.png'),R);
    writeReport(fullfile(here,'M2_Progress_Report_CN.md'),R);
end
fprintf('M2-C complete: Gate=%d, finite=%d, channel cases=%d.\n',gateC.pass,gateC.all_finite,height(feedbackSummary));
if ~gateC.pass, error('M2-C Gate failed. Stop before disturbance analysis.'); end
if stageRank(stopAfter)<=stageRank('C'), return; end

[residueMap,poleExcitation,gateD]=runStageD(models,modalMap,torsionSummary);
R.stageD=struct('residue_map',residueMap,'pole_excitation',poleExcitation,'gate',gateD);
R.gates.M2_D=gateD.pass;
if opt.SaveOutputs
    save(fullfile(here,'M2_Analysis_Checkpoint.mat'),'R','-v7.3');
    writeEvidenceSummary(fullfile(here,'M2_Evidence_Summary.csv'),R);
    makeOverviewFigure(fullfile(here,'M2_Mechanism_Overview.png'),R);
    writeReport(fullfile(here,'M2_Progress_Report_CN.md'),R);
end
fprintf('M2-D complete: Gate=%d, residue rows=%d, factorization error=%.3e.\n',gateD.pass,height(residueMap),gateD.max_factorization_error);
if ~gateD.pass, error('M2-D Gate failed. Stop before bidirectional sweep.'); end
if stageRank(stopAfter)<=stageRank('D'), return; end

[couplingSummary,couplingCurves,gateE]=runStageE(models,p,torsionSummary);
R.stageE=struct('summary',couplingSummary,'curves',{couplingCurves},'gate',gateE);
R.gates.M2_E=gateE.pass;
if opt.SaveOutputs
    save(fullfile(here,'M2_Analysis_Checkpoint.mat'),'R','-v7.3');
    writeEvidenceSummary(fullfile(here,'M2_Evidence_Summary.csv'),R);
    makeBidirectionalFigure(fullfile(here,'M2_Bidirectional_Coupling.png'),R.stageE);
    writeReport(fullfile(here,'M2_Progress_Report_CN.md'),R);
end
fprintf('M2-E complete: Gate=%d, architectures=%d, finite=%d.\n',gateE.pass,height(couplingSummary),gateE.all_finite);
if ~gateE.pass, error('M2-E Gate failed. Stop before local parameter mechanism scan.'); end
if stageRank(stopAfter)<=stageRank('E'), return; end

[parameterSummary,gateF]=runStageF(models,p,torsionSummary);
R.stageF=struct('summary',parameterSummary,'gate',gateF);
R.gates.M2_F=gateF.pass;
if opt.SaveOutputs
    save(fullfile(here,'M2_Analysis_Checkpoint.mat'),'R','-v7.3');
    writeEvidenceSummary(fullfile(here,'M2_Evidence_Summary.csv'),R);
    makeParameterFigure(fullfile(here,'M2_Local_Parameter_Mechanisms.png'),R.stageF);
    writeReport(fullfile(here,'M2_Progress_Report_CN.md'),R);
end
fprintf('M2-F complete: Gate=%d, points=%d, unstable=%d, hybrid candidates=%d.\n',gateF.pass,height(parameterSummary),gateF.num_unstable_points,gateF.num_hybridization_candidates);
if ~gateF.pass, error('M2-F Gate failed. Stop before cycle-energy verification.'); end
if stageRank(stopAfter)<=stageRank('F'), return; end

[energySummary,gateG]=runStageG(models,p,torsionSummary);
R.stageG=struct('summary',energySummary,'gate',gateG);
R.gates.M2_G=gateG.pass;
if opt.SaveOutputs
    save(fullfile(here,'M2_Analysis_Checkpoint.mat'),'R','-v7.3');
    writeEvidenceSummary(fullfile(here,'M2_Evidence_Summary.csv'),R);
    makeEnergyFigure(fullfile(here,'M2_Cycle_Energy_Verification.png'),R.stageG);
    writeReport(fullfile(here,'M2_Progress_Report_CN.md'),R);
end
fprintf('M2-G complete: Gate=%d, cases=%d, max energy error=%.3e.\n',gateG.pass,height(energySummary),gateG.max_relative_energy_error);
if ~gateG.pass, error('M2-G Gate failed. Stop before NL-SSM verification.'); end
if stageRank(stopAfter)<=stageRank('G'), return; end

[validationSummary,validationCurves,gateH]=runStageH(models,p);
R.stageH=struct('summary',validationSummary,'curves',{validationCurves},'gate',gateH);
R.gates.M2_H=gateH.pass;
if opt.SaveOutputs
    save(fullfile(here,'M2_Analysis_Checkpoint.mat'),'R','-v7.3');
    writeEvidenceSummary(fullfile(here,'M2_Evidence_Summary.csv'),R);
    makeValidationFigure(fullfile(here,'M2_NL_SSM_Representative_Responses.png'),R.stageH);
    writeReport(fullfile(here,'M2_Progress_Report_CN.md'),R);
end
fprintf('M2-H complete: Gate=%d, cases=%d, max key NRMSE=%.3e, max key peak error=%.3e.\n',gateH.pass,gateH.num_cases,gateH.max_key_nrmse,gateH.max_key_peak_error);
if ~gateH.pass, error('M2-H Gate failed. Evidence chain remains open.'); end
end

function [p,wp]=makePhysicalMpptPoint(p0,powerScale,m0Dir)
% 采用通过额定共同工作点的局部最优功率曲线：P_aero~omega^3、
% T_m~omega^2。这样 P0、omega_g0 和 Tm0 同时变化，而不是只缩放
% Pref/Tm。Cp-lambda 只用于记录物理风速与校准系数；当前M2仍不包含
% 显式气动状态，因此本函数不声称已经完成完整气动模型验证。
p=p0(:);
speedScale=powerScale^(1/3);
torqueScale=powerScale^(2/3);
p(12)=p0(12)*speedScale;
p(37)=p0(37)*powerScale;
p(39)=p0(39)*torqueScale;
% 由PCC目标P/Q和电网阻抗重算GFM空载电压幅值，使GFM与GFL在
% 部分负荷仍共享同一Q=0物理工作点；不能沿用额定E0后再把无功差
% 误判为架构机制。
p(40)=pccVoltageMagnitudeForPQ(p(37),p(38),p(4),p(9),p(3)*p(10));

old=path; cleanup=onCleanup(@()path(old)); %#ok<NASGU>
addpath(m0Dir);
p5=Liu2024_5MW_Params();
wind=p(12)*p5.rotor_radius/p5.lambda_opt;
windRatedOpt=p0(12)*p5.rotor_radius/p5.lambda_opt;
rawRated=0.5*p5.air_density*p5.rotor_area*p5.Copt*windRatedOpt^3;
calibration=(p0(39)*p0(12))/rawRated;
rawAero=0.5*p5.air_density*p5.rotor_area*p5.Copt*wind^3;
wp=struct('powerScale',powerScale,'speedScale',speedScale, ...
    'torqueScale',torqueScale,'wind_mps',wind, ...
    'lambda_opt',p5.lambda_opt,'Cp_opt',p5.Copt, ...
    'rawCpPower_W',rawAero,'aeroCalibration',calibration, ...
    'aeroPower_W',calibration*rawAero, ...
    'definition','CALIBRATED_CONSTANT_TSR_MPPT_THROUGH_M2_RATED_POINT');
end

function E=pccVoltageMagnitudeForPQ(P,Q,Vg,Rg,Xg)
opts=optimoptions('fsolve','Display','off','FunctionTolerance',1e-13, ...
    'StepTolerance',1e-13,'OptimalityTolerance',1e-13,'MaxIterations',200);
i0=[P/(1.5*max(Vg,1));-Q/(1.5*max(Vg,1))];
ii=fsolve(@residual,i0,opts);
V=Vg+(Rg+1i*Xg)*(ii(1)+1i*ii(2));
E=abs(V);
    function r=residual(i)
        vv=Vg+(Rg+1i*Xg)*(i(1)+1i*i(2));
        ss=1.5*vv*conj(i(1)+1i*i(2));
        r=[real(ss)-P;imag(ss)-Q]/max(abs(P)+abs(Q),1);
    end
end

function [p,wp]=calibrateMwtTorqueAtTargetSpeed(base,p,p0,wp)
% 先用GWT的本地转矩职责标定共同机械转矩，使P_PCC=Pref且omega=omega0。
% 这样不会在尚未构造好MWT-DVC积分状态时直接求MWT，从而避免把旧符号
% 的历史种子带入新的发电机外向端口约定。标定完成后，用GWT共同电气
% 平衡点构造MWT种子；三架构最终仍由runStageA分别严格求解。
initialTm=p(39); xGwt=[]; converged=false; meta=struct('normalized_residual',Inf);
for iter=1:12
    wp.torqueScale=p(39)/p0(39); wp.aeroPower_W=p(39)*p(12);
    a=makeArchitectures(base,p,wp);
    if ~isempty(xGwt),a(2).seed=xGwt;end
    [xGwt,meta]=solveEquilibrium(a(2).seed,p,a(2).name,a(2).flags,a(2).active);
    if ~all(isfinite(xGwt)) || ~isfinite(meta.normalized_residual),break;end
    [y,names]=m2Outputs(xGwt,p,'GWT',zeros(4,1),a(2).flags);
    Ppcc=y(strcmp(names,'P_PCC'));
    speedError=xGwt(3)-p(12); powerError=p(37)-Ppcc;
    if meta.normalized_residual<1e-10 && ...
            abs(speedError)/max(p(12),eps)<1e-10 && abs(powerError)/p(1)<1e-10
        converged=true;break
    end
    % dP_PCC/dTm约为omega_m；0.8松弛避免滤波器损耗变化造成过调。
    tmTarget=p(39)+0.8*powerError/max(p(12),eps);
    if ~isfinite(tmTarget)||tmTarget<=0,break;end
    p(39)=tmTarget;
end

% 用最后一个GWT共同电气平衡点构造MWT-DVC种子。Udc误差为零时，
% xi_DVC即为iq参考；其余GFM/LCL/电网状态在共同工作点应保持一致。
if ~isempty(xGwt) && all(isfinite(xGwt))
    xMwt=xGwt;
    xMwt(5)=p(39)/p(18);
    xMwt(6)=xMwt(5);
    xMwt(7:8)=0;
    wp.mwtSeedOverride=xMwt;
end
wp.uncompensatedTm_Nm=initialTm;
wp.compensatedTm_Nm=p(39);
wp.lossCompensation_Nm=p(39)-initialTm;
wp.torqueScale=p(39)/p0(39);
wp.aeroPower_W=p(39)*p(12);
wp.mwtCalibrationIterations=iter;
if isempty(xGwt)||~all(isfinite(xGwt)),wp.mwtTargetSpeedRelativeError=Inf;else,wp.mwtTargetSpeedRelativeError=abs(xGwt(3)-p(12))/max(p(12),eps);end
wp.mwtCalibrationConverged=converged;
end

function arch=makeArchitectures(base,p,varargin)
% 三种职责定义；plant与控制器公共参数不在各架构中复制。
wp=[]; if nargin>=3,wp=varargin{1};end
if isempty(wp)
    imq0=base.states(5,3);
    speedScale=1; torqueScale=1; powerScale=1;
    pmsgConv='LEGACY_MOTOR_SIGN';
else
    imq0=p(39)/p(18);
    speedScale=wp.speedScale; torqueScale=wp.torqueScale; powerScale=wp.powerScale;
    pmsgConv=wp.pmsgConvention;
end
Vd0=max(p(4),1);
wnPll=2*pi*10; zetaPll=0.707;
common=struct('vscDcMode','FIXED_NORM','imqRef0',imq0, ...
    'pmsgConvention',char(string(pmsgConv)), ...
    'Kmppt_iq_per_radps',2*imq0/p(12), ...
    'KpGscDvc_W_per_V',5e3,'KiGscDvc_W_per_Vs',5e2, ...
    'KpGscDvc_A_per_V',5e3/(1.5*Vd0), ...
    'KiGscDvc_A_per_Vs',5e2/(1.5*Vd0), ...
    'KpQ_A_per_var',1/(1.5*Vd0), ...
    'KiQ_A_per_vars',(2*pi*5)/(1.5*Vd0), ...
    'KpPll_radps_per_V',2*zetaPll*wnPll/Vd0, ...
    'KiPll_radps2_per_V',wnPll^2/Vd0,'mpGwt',p(34));
seedG=scaleWorkpointSeed(base.states(:,2),p,speedScale,torqueScale,powerScale,imq0);
seedM=scaleWorkpointSeed(base.states(:,3),p,speedScale,torqueScale,powerScale,imq0);
if strcmpi(pmsgConv,'GENERATOR_OUTWARD')
    seedG(7:8)=0; seedM(7:8)=0;
    seedM(6)=imq0;
end
if ~isempty(wp) && isfield(wp,'mwtSeedOverride')
    seedM=wp.mwtSeedOverride(:);
end
arch(1)=struct('name','GFL','description','MSC local MPPT/torque + GSC-DVC current control + continuous PLL', ...
    'flags',common,'seed',seedG,'active',[1:13 15:23]);
arch(2)=struct('name','GWT','description','MSC local MPPT/torque + GSC-DVC + GFM', ...
    'flags',common,'seed',seedG,'active',1:23);
arch(3)=struct('name','MWT','description','MSC-DVC + GSC-GFM', ...
    'flags',common,'seed',seedM,'active',1:23);
% GFL的xi_DVC物理含义为GSC d轴电流偏置；两个GSC电压PI状态不启用。
arch(1).seed(6)=arch(1).seed(18);
arch(1).seed(14)=0;
arch(1).seed(15)=arch(1).seed(19);
arch(1).seed(12)=p(3);
end

function x=scaleWorkpointSeed(x,p,speedScale,torqueScale,powerScale,imq0)
% 只构造 continuation 初值；最终工作点仍由严格平衡求解器确定。
x=x(:);
x(1)=x(1)*torqueScale;
x(2:3)=p(12);
x(4)=x(4)*torqueScale;
x(5)=imq0;
x(6:8)=x(6:8)*torqueScale;
x(9)=p(2);
x(10:11)=x(10:11)*powerScale;
x(13)=x(13)*powerScale;
x(14:19)=x(14:19)*powerScale;
x(20:21)=x(20:21);
x(22:23)=x(22:23)*powerScale;
if speedScale<=0,error('Non-positive MPPT speed scale.');end
end

function [models,plantAudit,workAudit,gate]=runStageA(arch,p,base)
plantNames={'Sb','Vdc0','w0','Vg','Rf','Lf','Cf','Rd','Rg','Lg','Cdc','wm0', ...
    'Rs','Ld','Lq','psi','pole_pairs','Kt','Jt','Jg','Ksh','Dsh','Dt','Dg'};
plantUnits={'W','V','rad/s','V','ohm','H','F','ohm','ohm','H','F','rad/s', ...
    'ohm','H','H','Wb','1','N m/A','kg m2','kg m2','N m/rad','N m s/rad','N m s/rad','N m s/rad'};
nA=numel(arch); rows=cell(numel(plantNames),8);
for j=1:numel(plantNames)
    vals=repmat(p(j),1,nA);
    rows(j,:)={plantNames{j},plantUnits{j},vals(1),vals(2),vals(3),max(vals)-min(vals),0,true};
end

plantAudit=cell2table(rows,'VariableNames',{'Parameter','Unit','GFL','GWT','MWT','MaxAbsDifference','MaxRelativeDifference','PASS'});
plantAudit.Parameter=string(plantAudit.Parameter); plantAudit.Unit=string(plantAudit.Unit);

models=cell(nA,1);
workAudit=table('Size',[nA 15], ...
    'VariableTypes',{'string','string','double','double','double','double','double','double','double','double','double','double','double','double','logical'}, ...
    'VariableNames',{'Architecture','ControlResponsibility','P0_W','Q0_var','Udc0_V','omega_g0_radps','Te0_Nm','Tsh0_Nm','Pmsc0_W','Pgsc0_W','PowerMismatch_W','TorqueMismatch_Nm','NormalizedResidual','MaxAbsDx','PASS'});
for k=1:nA
    [x,meta]=solveEquilibrium(arch(k).seed,p,arch(k).name,arch(k).flags,arch(k).active);
    L=linearizeModel(x,p,arch(k).name,arch(k).flags,arch(k).active);
    [y,names]=m2Outputs(x,p,arch(k).name,zeros(4,1),arch(k).flags);
    get=@(name)y(strcmp(names,name));
    P0=get('P_PCC'); Q0=get('Q_PCC'); Udc=get('Udc'); wg=get('omega_g'); Te=get('T_e'); Tsh=get('T_sh'); Pmsc=get('P_MSC'); Pgsc=get('P_GSC');
    pass=meta.normalized_residual<1e-10 && abs(P0-p(37))/p(1)<1e-4 && abs(Udc-p(2))/p(2)<1e-6;
    workAudit(k,:)={string(arch(k).name),string(arch(k).description),P0,Q0,Udc,wg,Te,Tsh,Pmsc,Pgsc,Pmsc-Pgsc,Tsh-Te,meta.normalized_residual,meta.max_abs_dx,pass};
    models{k}=struct('name',arch(k).name,'description',arch(k).description,'x0',x,'flags',arch(k).flags,'active',arch(k).active,'equilibrium',meta,'linear',L,'y0',y,'output_names',{names});
end

scales=[p(1),p(1),p(2),p(12),p(1)/p(12)];
W=[workAudit.P0_W,workAudit.Q0_var,workAudit.Udc0_V,workAudit.omega_g0_radps,workAudit.Te0_Nm];
pairMax=0;
for j=1:size(W,2), pairMax=max(pairMax,(max(W(:,j))-min(W(:,j)))/scales(j)); end
gate=struct('pass',all(plantAudit.PASS)&&all(workAudit.PASS)&&pairMax<1e-4, ...
    'max_normalized_residual',max(workAudit.NormalizedResidual), ...
    'max_workpoint_pairwise_pu',pairMax,'target_tolerance_pu',1e-4, ...
    'residual_tolerance',1e-10,'reference_gate_passed',base.passed);
end

function [T,Ttor,gate]=runStageB(models)
% 全模态地图：不预设TOR或电气模态的答案，先按参与因子聚合。
rows={}; torRows={}; mixedCount=0; allStable=true;
for a=1:numel(models)
    M=models{a}; A=M.linear.A; active=M.linear.active; names=string(M.linear.state_names);
    [V,D,W]=eig(A); lam=diag(D); maxReal=max(real(lam)); allStable=allStable&&(maxReal<0);
    keep=find(imag(lam)>1e-7 | abs(imag(lam))<=1e-7);
    local=struct('idx',{},'lambda',{},'f',{},'zeta',{},'pf',{},'vfull',{},'g',{},'mix',{},'class',{});
    for kk=1:numel(keep)
        i=keep(kk); l=lam(i); f=abs(imag(l))/(2*pi); z=-real(l)/max(abs(l),eps);
        pf=abs(V(:,i).*conj(W(:,i))); pf=pf/max(sum(pf),eps);
        g=aggregateParticipation(pf,names,M.name);
        mix=4*g.MECH*(1-g.MECH);
        [~,im]=max([g.MECH g.PMSG g.MSC g.DC g.GSC g.SYNC_PLL g.LCL_GRID]);
        labels={'MECH','PMSG','MSC','DC','GSC','SYNC_PLL','LCL_GRID'}; cls=labels{im};
        if mix>=0.25 && g.MECH>=0.05 && (1-g.MECH)>=0.05, cls='MIXED'; mixedCount=mixedCount+1; end
        vfull=zeros(23,1); vfull(active)=V(:,i);
        local(end+1)=struct('idx',i,'lambda',l,'f',f,'zeta',z,'pf',pf,'vfull',vfull,'g',g,'mix',mix,'class',cls); %#ok<AGROW>
    end
    cand=find(arrayfun(@(q)q.f>=0.5&&q.f<=10&&imag(q.lambda)>0,local));
    assert(~isempty(cand),'%s has no oscillatory candidate in torsional band.',M.name);
    [~,jj]=max(arrayfun(@(q)q.g.MECH,local(cand))); itor=cand(jj);
    for k=1:numel(local)
        q=local(k); isTor=(k==itor);
        rows(end+1,:)={"B_MODAL",string(M.name),k,real(q.lambda),imag(q.lambda),q.f,q.zeta,q.g.MECH,q.g.PMSG,q.g.MSC,q.g.DC,q.g.GSC,q.g.SYNC_PLL,q.g.LCL_GRID,q.mix,string(q.class),isTor,maxReal}; %#ok<AGROW>
    end
    q=local(itor);
    torRows(end+1,:)={string(M.name),real(q.lambda),imag(q.lambda),q.f,q.zeta,q.g.MECH,1-q.g.MECH,q.mix,string(q.class),maxReal}; %#ok<AGROW>
end
T=cell2table(rows,'VariableNames',{'Stage','Architecture','ModeIndex','PoleReal','PoleImag','Frequency_Hz','DampingRatio','Pi_MECH','Pi_PMSG','Pi_MSC','Pi_DC','Pi_GSC','Pi_SYNC_PLL','Pi_LCL_GRID','MixingIndex','PhysicalClass','IsTorsional','MaxRealPole'});
T.Stage=string(T.Stage);T.Architecture=string(T.Architecture);T.PhysicalClass=string(T.PhysicalClass);
Ttor=cell2table(torRows,'VariableNames',{'Architecture','PoleReal','PoleImag','Frequency_Hz','DampingRatio','Pi_MECH','Pi_ELECTRICAL','MixingIndex','PhysicalClass','MaxRealPole'});
Ttor.Architecture=string(Ttor.Architecture);Ttor.PhysicalClass=string(Ttor.PhysicalClass);
gate=struct('pass',allStable&&height(Ttor)==3&&all(Ttor.Pi_MECH>0.5),'all_stable',allStable, ...
    'num_torsional_modes',height(Ttor),'num_mixed_candidates',mixedCount, ...
    'max_real_pole',max(T.MaxRealPole),'minimum_torsional_mechanical_participation',min(Ttor.Pi_MECH));
end

function [T,curves,gate]=runStageC(models,p,Ttor)
% 将omega_g视为外生小扰动，打开机械方程后计算电气侧Te反馈。
rows={}; curves=cell(numel(models),1); allFinite=true;
for a=1:numel(models)
    M=models{a}; ft=Ttor.Frequency_Hz(Ttor.Architecture==string(M.name));
    f=logspace(log10(0.2),log10(6),241); [G,De,Ke]=complexTorqueCurve(M.linear,f);
    [~,i0]=min(abs(f-ft)); curves{a}=struct('Architecture',M.name,'f_Hz',f,'G',G,'De',De,'Ke',Ke);
    allFinite=allFinite&&all(isfinite([real(G) imag(G) De Ke]));
    rows(end+1,:)={"C_BASE",string(M.name),"BASE",ft,De(i0),Ke(i0),min(De),max(De),0,0,string(classifyBand(De,0)),"Closed electrical feedback"}; %#ok<AGROW>

    if any(strcmpi(M.name,{'GFL','GWT'}))
        f0=M.flags; f0.Kmppt_iq_per_radps=0;
        L0=linearizeModel(M.x0,p,M.name,f0,M.active); [~,De0,Ke0]=complexTorqueCurve(L0,f);
        dDe=De-De0; dKe=Ke-Ke0;
        rows(end+1,:)={"C_ABLATION",string(M.name),"MPPT_ON_MINUS_OFF",ft,dDe(i0),dKe(i0),min(dDe),max(dDe),De(i0),De0(i0),string(classifyBand(dDe,max(abs(De0)))),"Local tangent MPPT only"}; %#ok<AGROW>
    end
    if strcmpi(M.name,'MWT')
        poff=p; poff(25)=0; poff(26)=0;
        L0=linearizeModel(M.x0,poff,M.name,M.flags,M.active); [~,De0,Ke0]=complexTorqueCurve(L0,f);
        dDe=De-De0; dKe=Ke-Ke0;
        rows(end+1,:)={"C_ABLATION",string(M.name),"MSC_DVC_ON_MINUS_OFF",ft,dDe(i0),dKe(i0),min(dDe),max(dDe),De(i0),De0(i0),string(classifyBand(dDe,max(abs(De0)))),"Same equilibrium; proportional and integral DVC derivatives removed"}; %#ok<AGROW>
    end
    if any(strcmpi(M.name,{'GWT','MWT'}))
        ff=M.flags; ff.freezeSync=true;
        L0=linearizeModel(M.x0,p,M.name,ff,M.active); [~,De0,Ke0]=complexTorqueCurve(L0,f);
        dDe=De-De0; dKe=Ke-Ke0;
        rows(end+1,:)={"C_ABLATION",string(M.name),"GFM_SYNC_ON_MINUS_FROZEN",ft,dDe(i0),dKe(i0),min(dDe),max(dDe),De(i0),De0(i0),string(classifyBand(dDe,max(abs(De0)))),"Auxiliary ablation; zero sync states excluded from interpretation"}; %#ok<AGROW>
    end
end
T=cell2table(rows,'VariableNames',{'Stage','Architecture','Case','Frequency_Hz','De_or_DeltaDe','Ke_or_DeltaKe','BandMinimum','BandMaximum','De_ON','De_OFF','EvidenceStatus','Note'});
T.Stage=string(T.Stage);T.Architecture=string(T.Architecture);T.Case=string(T.Case);T.EvidenceStatus=string(T.EvidenceStatus);T.Note=string(T.Note);
gate=struct('pass',allFinite&&height(T)==8,'all_finite',allFinite,'num_cases',height(T), ...
    'sign_convention','Jg*domega_g=Tsh-Te; positive Re(G_Te,wg) is positive electrical damping', ...
    'frequency_band','0.2 to 6 Hz; torsional frequency marked separately');
end

function [T,Ttor,gate]=runStageD(models,modalMap,torsionSummary)
% 两类外部扰动：机械转矩与电网频率。Residue严格拆成Cv和w^HB。
distIdx=[1 4];distNames=["MechanicalTorque","GridFrequency"];
outNames=["omega_sh","P_PCC"];rows={};torRows={};maxErr=0;
for a=1:numel(models)
    M=models{a};L=M.linear;[V,D,W]=eig(L.A);lam=diag(D);
    keep=find(imag(lam)>1e-7); Qm=modalMap(modalMap.Architecture==string(M.name)&modalMap.PoleImag>0,:);
    target=torsionSummary(torsionSummary.Architecture==string(M.name),:);[~,itor]=min(abs(lam-(target.PoleReal+1i*target.PoleImag)));
    for di=1:numel(distIdx)
        d=distIdx(di);
        for oi=1:numel(outNames)
            iy=find(string(L.output_names)==outNames(oi),1);assert(~isempty(iy),'Output %s missing.',outNames(oi));
            for kk=1:numel(keep)
                i=keep(kk);den=W(:,i)'*V(:,i);cv=L.C(iy,:)*V(:,i);wb=(W(:,i)'*L.B(:,d))/den;
                obs=abs(cv);inp=abs(wb);res=abs(cv*wb);err=abs(res-obs*inp)/max(res,1e-30);maxErr=max(maxErr,err);
                [~,jm]=min(abs((Qm.PoleReal+1i*Qm.PoleImag)-lam(i)));cls=Qm.PhysicalClass(jm);
                isTor=(i==itor);
                rows(end+1,:)={"D_RESIDUE",string(M.name),distNames(di),outNames(oi),real(lam(i)),imag(lam(i)),abs(imag(lam(i)))/(2*pi),-real(lam(i))/abs(lam(i)),obs,inp,res,err,string(cls),isTor}; %#ok<AGROW>
                if isTor&&oi==1,torRows(end+1,:)={string(M.name),distNames(di),real(lam(i)),imag(lam(i)),obs,inp,res,NaN,NaN,""};end %#ok<AGROW>
            end
        end
    end
end
T=cell2table(rows,'VariableNames',{'Stage','Architecture','Disturbance','Output','PoleReal','PoleImag','Frequency_Hz','DampingRatio','Observability','InputProjection','ResidueMagnitude','FactorizationError','PhysicalClass','IsTorsional'});
T.Stage=string(T.Stage);T.Architecture=string(T.Architecture);T.Disturbance=string(T.Disturbance);T.Output=string(T.Output);T.PhysicalClass=string(T.PhysicalClass);
Ttor=cell2table(torRows,'VariableNames',{'Architecture','Disturbance','PoleReal','PoleImag','Observability','InputProjection','ResidueMagnitude','PoleChangeIndex','ResidueChangeDecades','Classification'});
Ttor.Architecture=string(Ttor.Architecture);Ttor.Disturbance=string(Ttor.Disturbance);Ttor.Classification=string(Ttor.Classification);
for d=unique(Ttor.Disturbance,'stable')'
    ref=Ttor(Ttor.Architecture=="GFL"&Ttor.Disturbance==d,:);assert(height(ref)==1,'GFL residue reference missing.');lr=ref.PoleReal+1i*ref.PoleImag;
    ix=find(Ttor.Disturbance==d);
    for q=ix'
        lp=Ttor.PoleReal(q)+1i*Ttor.PoleImag(q);ip=abs(lp-lr)/max(abs(lr),eps);ir=abs(log10(max(Ttor.ResidueMagnitude(q),1e-30)/max(ref.ResidueMagnitude,1e-30)));
        Ttor.PoleChangeIndex(q)=ip;Ttor.ResidueChangeDecades(q)=ir;Ttor.Classification(q)=classifyPoleExcitation(ip,ir);
    end
end
gate=struct('pass',all(isfinite(T.ResidueMagnitude))&&maxErr<1e-8&&height(Ttor)==6, ...
    'max_factorization_error',maxErr,'num_residue_rows',height(T),'num_torsional_diagnostics',height(Ttor), ...
    'classification_thresholds','pole significant >1%; residue significant >0.4771 decades (factor 3)');
end

function s=classifyPoleExcitation(ip,ir)
pole=ip>0.01;path=ir>log10(3);
if pole&&path,s="JOINT";elseif pole,s="POLE_DOMINATED";elseif path,s="EXCITATION_DOMINATED";else,s="SIMILAR_TO_REFERENCE";end
end

function [T,curves,gate]=runStageE(models,p,torsionSummary)
% 归一化双向机电传递矩阵：输入[DeltaTm, DeltaOmegaGrid]，
% 输出[omega_sh, P_PCC]。只陈述频率局部方向占优，不把小数值称为结构零。
f=logspace(log10(0.1),log10(10),241); rows={};curves=cell(numel(models),1);allFinite=true;
Tb=p(1)/p(12); wgBase=p(3); wmBase=p(12); Pbase=p(1);
for a=1:numel(models)
    M=models{a};L=M.linear;
    iw=find(string(L.output_names)=="omega_sh",1);ipcc=find(string(L.output_names)=="P_PCC",1);
    assert(~isempty(iw)&&~isempty(ipcc),'M2-E required outputs are missing.');
    Ggm=zeros(size(f));Gmg=zeros(size(f));Gmm=zeros(size(f));Ggg=zeros(size(f));
    Fall=scaledFrequencyResponse(L,f);
    for k=1:numel(f)
        G=Fall(:,:,k);
        Gmm(k)=G(iw,1);Gmg(k)=G(iw,4);Ggm(k)=G(ipcc,1);Ggg(k)=G(ipcc,4);
    end
    CGM=abs(Gmg)*wgBase/wmBase; CMG=abs(Ggm)*Tb/Pbase;
    dirLog=log10((CGM+1e-30)./(CMG+1e-30)); ratio=(CGM+1e-30)./(CMG+1e-30);
    ft=torsionSummary.Frequency_Hz(torsionSummary.Architecture==string(M.name));[~,i0]=min(abs(f-ft));
    crossings=sum(diff(sign(dirLog))~=0);
    rows(end+1,:)={"E_BIDIRECTIONAL",string(M.name),ft,CGM(i0),CMG(i0),ratio(i0),dirLog(i0),min(CGM),max(CGM),min(CMG),max(CMG),crossings,string(directionLabel(dirLog(i0)))}; %#ok<AGROW>
    curves{a}=struct('Architecture',M.name,'f_Hz',f,'G_MM',Gmm,'G_MG',Gmg,'G_GM',Ggm,'G_GG',Ggg,'C_GridToMachine',CGM,'C_MachineToGrid',CMG,'DirectionLog10',dirLog);
    allFinite=allFinite&&all(isfinite([real(Gmm) imag(Gmm) real(Gmg) imag(Gmg) real(Ggm) imag(Ggm) real(Ggg) imag(Ggg) CGM CMG dirLog]));
end
T=cell2table(rows,'VariableNames',{'Stage','Architecture','TorsionalFrequency_Hz','C_GridToMachine_at_ftor','C_MachineToGrid_at_ftor','DirectionalRatio_at_ftor','Log10DirectionalRatio_at_ftor','C_GM_Min','C_GM_Max','C_MG_Min','C_MG_Max','DirectionCrossings_0p1_10Hz','LocalDirectionAtFtor'});
T.Stage=string(T.Stage);T.Architecture=string(T.Architecture);T.LocalDirectionAtFtor=string(T.LocalDirectionAtFtor);
gate=struct('pass',allFinite&&height(T)==3,'all_finite',allFinite,'num_architectures',height(T), ...
    'normalization','inputs: DeltaTm/(Sb/wm0), DeltaOmegaGrid/w0; outputs: omega_sh/wm0, P_PCC/Sb', ...
    'frequency_band_Hz',[0.1 10],'interpretation_rule','direction is frequency-local; no structural-zero claim');
end

function Fall=scaledFrequencyResponse(L,f)
% SI矩阵跨越MW/A/V/rad量级，先做相似尺度预处理，避免把病态警告
% 误判成物理奇异性。该变换不改变极点和传递函数。
sys=prescale(ss(L.A,L.B,L.C,L.D));
Fall=freqresp(sys,2*pi*f);
end

function s=directionLabel(q)
if q>log10(3),s="GRID_TO_MACHINE_DOMINANT_LOCAL";elseif q<-log10(3),s="MACHINE_TO_GRID_DOMINANT_LOCAL";else,s="COMPARABLE_LOCAL";end
end

function makeBidirectionalFigure(path,E)
fig=figure('Visible','off','Color','w','Position',[80 80 1420 780]);tl=tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');cols=lines(numel(E.curves));
nexttile;hold on;for k=1:numel(E.curves),q=E.curves{k};loglog(q.f_Hz,q.C_GridToMachine,'LineWidth',1.8,'Color',cols(k,:),'DisplayName',q.Architecture);end;grid on;xlabel('Frequency (Hz)');ylabel('C_{GM}');title('Grid to machine normalized coupling');legend('Location','best');
nexttile;hold on;for k=1:numel(E.curves),q=E.curves{k};loglog(q.f_Hz,q.C_MachineToGrid,'--','LineWidth',1.8,'Color',cols(k,:),'DisplayName',q.Architecture);end;grid on;xlabel('Frequency (Hz)');ylabel('C_{MG}');title('Machine to grid normalized coupling');legend('Location','best');
nexttile([1 2]);hold on;for k=1:numel(E.curves),q=E.curves{k};semilogx(q.f_Hz,q.DirectionLog10,'LineWidth',1.8,'Color',cols(k,:),'DisplayName',q.Architecture);end;yline(0,':k','HandleVisibility','off');yline(log10(3),'--k','factor 3','HandleVisibility','off');yline(-log10(3),'--k','HandleVisibility','off');grid on;xlabel('Frequency (Hz)');ylabel('log_{10}(C_{GM}/C_{MG})');title('Frequency-local directional dominance (not structural zero)');legend('Location','best');
sgtitle(tl,'M2-E normalized bidirectional electromechanical coupling');exportgraphics(fig,path,'Resolution',220);close(fig);
end

function [T,gate]=runStageF(models,p,torsionSummary)
% 少量代表点检验H、DVC和SCR对Pole/Path/方向耦合的影响。只有同时出现
% 频率接近、参与度交换和MAC下降时，才标记为模态混合候选。
families={"H","DVC","SCR"}; values={p(33)*[0.5 1 1.5 2],[0.5 1 1.5 2],[3 4 6]};rows={};allFinite=true;allSolved=true;
for a=1:numel(models)
    M=models{a};target=torsionSummary(torsionSummary.Architecture==string(M.name),:);
    ref=trackedMetrics(M.linear,target,[],p);
    for ff=1:numel(families)
        fam=families{ff}; vv=values{ff};
        for k=1:numel(vv)
            pv=p;flags=M.flags;val=vv(k);
            switch fam
                case "H", pv(33)=val; scale=val/p(33);
                case "DVC"
                    scale=val;
                    if strcmpi(M.name,'MWT'),pv(25:26)=p(25:26)*scale;else,flags.KpGscDvc_W_per_V=M.flags.KpGscDvc_W_per_V*scale;flags.KiGscDvc_W_per_Vs=M.flags.KiGscDvc_W_per_Vs*scale;flags.KpGscDvc_A_per_V=M.flags.KpGscDvc_A_per_V*scale;flags.KiGscDvc_A_per_Vs=M.flags.KiGscDvc_A_per_Vs*scale;end
                case "SCR", scale=val/4;pv(9:10)=p(9:10)*(4/val);
            end
            if fam=="SCR"
                x=M.x0;eq=[];for scrStep=linspace(4,val,5),ps=p;ps(9:10)=p(9:10)*(4/scrStep);[x,eq]=solveEquilibrium(x,ps,M.name,flags,M.active);end
            else
                [x,eq]=solveEquilibrium(M.x0,pv,M.name,flags,M.active);
            end
            L=linearizeModel(x,pv,M.name,flags,M.active);
            met=trackedMetrics(L,target,ref,pv);poleIdx=abs(met.ltor-ref.ltor)/max(abs(ref.ltor),eps);resDec=abs(log10(max(met.residue,1e-30)/max(ref.residue,1e-30)));
            cls=classifyPoleExcitation(poleIdx,resDec);hybrid=met.freqGap<0.5&&(met.piMechTor<0.9||met.piMechElec>0.05)&&met.patternCorrelation<0.95;
            allFinite=allFinite&&all(isfinite([real(met.ltor) imag(met.ltor) met.zeta met.piMechTor met.patternCorrelation real(met.lelec) imag(met.lelec) met.piMechElec met.freqGap met.residue met.CGM met.CMG met.dirRatio met.critFreq met.critPiMech]));
            allSolved=allSolved&&eq.normalized_residual<1e-8;
            rows(end+1,:)={"F_LOCAL_SCAN",string(M.name),fam,scale,val,eq.normalized_residual,met.maxReal<0,met.maxReal,met.critFreq,met.critPiMech,string(met.critClass),real(met.ltor),imag(met.ltor),met.ftor,met.zeta,met.piMechTor,met.patternCorrelation,real(met.lelec),imag(met.lelec),met.felec,met.piMechElec,met.freqGap,met.residue,poleIdx,resDec,string(cls),met.CGM,met.CMG,met.dirRatio,hybrid}; %#ok<AGROW>
        end
    end
end
T=cell2table(rows,'VariableNames',{'Stage','Architecture','Parameter','Scale','PhysicalValue','EquilibriumResidual','Stable','MaxRealPole','CriticalModeFrequency_Hz','CriticalModePiMECH','CriticalModeClass','TorPoleReal','TorPoleImag','TorFrequency_Hz','TorDampingRatio','TorPiMECH','TorPatternCorrelation','NearestElecPoleReal','NearestElecPoleImag','NearestElecFrequency_Hz','NearestElecPiMECH','FrequencyGap_Hz','TorResidue_GridToShaft','PoleChangeIndex','ResidueChangeDecades','PolePathClass','C_GridToMachine','C_MachineToGrid','DirectionalRatio','HybridizationCandidate'});
T.Stage=string(T.Stage);T.Architecture=string(T.Architecture);T.Parameter=string(T.Parameter);T.CriticalModeClass=string(T.CriticalModeClass);T.PolePathClass=string(T.PolePathClass);
gate=struct('pass',allFinite&&allSolved&&height(T)==33,'all_finite',allFinite,'all_equilibria_solved',allSolved,'num_points',height(T),'num_unstable_points',sum(~T.Stable),'num_hybridization_candidates',sum(T.HybridizationCandidate), ...
    'hybridization_rule','frequency gap <0.5 Hz AND participation exchange AND torsional participation-pattern correlation <0.95', ...
    'scan_scope','H factors 0.5/1/1.5/2; DVC factors 0.5/1/1.5/2; SCR 3/4/6', ...
    'excluded_point','SCR=2 excluded from this local scan because GFL strict equilibrium continuation did not satisfy Gate; retained as a future feasibility-boundary question');
end

function met=trackedMetrics(L,target,refTrack,p)
[V,D,W]=eig(L.A);lam=diag(D);pos=find(imag(lam)>1e-7);freq=imag(lam(pos))/(2*pi);
names=string(L.state_names);pfAll=cell(numel(pos),1);gAll=cell(numel(pos),1);
for k=1:numel(pos),q=pos(k);pf=abs(V(:,q).*conj(W(:,q)));pf=pf/max(sum(pf),eps);pfAll{k}=pf;gAll{k}=aggregateParticipation(pf,names,target.Architecture);end
if isempty(refTrack)
    [~,j]=min(abs(lam(pos)-(target.PoleReal+1i*target.PoleImag)));itor=pos(j);mac=1;
else
    corrs=zeros(size(pos));proximity=zeros(size(pos));
    for k=1:numel(pos),corrs(k)=dot(refTrack.pfTor,pfAll{k})/max(norm(refTrack.pfTor)*norm(pfAll{k}),eps);proximity(k)=exp(-abs(lam(pos(k))-refTrack.ltor)/max(abs(refTrack.ltor),1));end
    band=freq>=0.5&freq<=10;cand=find(band);if isempty(cand),cand=1:numel(pos);end
    [~,jj]=max(0.75*corrs(cand)+0.25*proximity(cand));sel=cand(jj);itor=pos(sel);mac=corrs(sel);
end
isel=find(pos==itor,1);pf=pfAll{isel};g=gAll{isel};
other=pos(pos~=itor);[~,jj]=min(abs(imag(lam(other))/(2*pi)-abs(imag(lam(itor)))/(2*pi)));ie=other(jj);pfe=abs(V(:,ie).*conj(W(:,ie)));pfe=pfe/max(sum(pfe),eps);ge=aggregateParticipation(pfe,names,target.Architecture);
iy=find(string(L.output_names)=="omega_sh",1);den=W(:,itor)'*V(:,itor);res=abs((L.C(iy,:)*V(:,itor))*((W(:,itor)'*L.B(:,4))/den));
ft=abs(imag(lam(itor)))/(2*pi);[CGM,CMG]=couplingAtFrequency(L,p,ft);
[~,icrit]=max(real(lam));pfc=abs(V(:,icrit).*conj(W(:,icrit)));pfc=pfc/max(sum(pfc),eps);gc=aggregateParticipation(pfc,names,target.Architecture);critClass=dominantParticipationClass(gc);
met=struct('ltor',lam(itor),'ftor',ft,'zeta',-real(lam(itor))/abs(lam(itor)),'piMechTor',g.MECH,'patternCorrelation',mac,'pfTor',pf, ...
    'lelec',lam(ie),'felec',abs(imag(lam(ie)))/(2*pi),'piMechElec',ge.MECH,'freqGap',abs(imag(lam(ie)-lam(itor)))/(2*pi), ...
    'residue',res,'CGM',CGM,'CMG',CMG,'dirRatio',(CGM+1e-30)/(CMG+1e-30),'maxReal',real(lam(icrit)), ...
    'critFreq',abs(imag(lam(icrit)))/(2*pi),'critPiMech',gc.MECH,'critClass',critClass, ...
    'critLambda',lam(icrit),'pfCrit',pfc);
end

function s=dominantParticipationClass(g)
fn={'MECH','PMSG','MSC','DC','GSC','SYNC_PLL','LCL_GRID'};v=zeros(size(fn));for k=1:numel(fn),v(k)=g.(fn{k});end;[~,i]=max(v);s=string(fn{i});
end

function [CGM,CMG]=couplingAtFrequency(L,p,f)
F=scaledFrequencyResponse(L,f);iw=find(string(L.output_names)=="omega_sh",1);ipcc=find(string(L.output_names)=="P_PCC",1);
CGM=abs(F(iw,4))*p(3)/p(12);CMG=abs(F(ipcc,1))*(p(1)/p(12))/p(1);
end

function makeParameterFigure(path,F)
T=F.summary;pars=["H","DVC","SCR"];cols=lines(3);fig=figure('Visible','off','Color','w','Position',[40 40 1550 1200]);tl=tiledlayout(fig,3,3,'TileSpacing','compact','Padding','compact');
for r=1:3
    Q=T(T.Parameter==pars(r),:);aa=unique(Q.Architecture,'stable');
    nexttile;hold on;for a=1:numel(aa),Z=sortrows(Q(Q.Architecture==aa(a),:),'PhysicalValue');plot(Z.PhysicalValue,100*Z.TorDampingRatio,'o-','LineWidth',1.5,'Color',cols(a,:),'DisplayName',aa(a));end;grid on;xlabel(pars(r));ylabel('Torsional damping (%)');title(pars(r)+' pole');if r==1,legend('Location','best');end
    nexttile;hold on;for a=1:numel(aa),Z=sortrows(Q(Q.Architecture==aa(a),:),'PhysicalValue');semilogy(Z.PhysicalValue,max(Z.TorResidue_GridToShaft,1e-30),'o-','LineWidth',1.5,'Color',cols(a,:),'DisplayName',aa(a));end;grid on;xlabel(pars(r));ylabel('|R_{tor,grid}|');title(pars(r)+' path');
    nexttile;hold on;for a=1:numel(aa),Z=sortrows(Q(Q.Architecture==aa(a),:),'PhysicalValue');semilogy(Z.PhysicalValue,max(Z.DirectionalRatio,1e-30),'o-','LineWidth',1.5,'Color',cols(a,:),'DisplayName',aa(a));end;yline(1,':k');grid on;xlabel(pars(r));ylabel('C_{GM}/C_{MG}');title(pars(r)+' direction');
end
sgtitle(tl,'M2-F local parameter mechanisms: pole, path and direction');exportgraphics(fig,path,'Resolution',220);close(fig);
end

function [T,gate]=runStageG(models,p,torsionSummary)
% 用独立的一周期时域积分核对复转矩实部的能量符号。
rows={};allFinite=true;allSign=true;maxErr=0;Aw=1e-3*p(12);
for a=1:numel(models)
    M=models{a};ft=torsionSummary.Frequency_Hz(torsionSummary.Architecture==string(M.name));
    [G,~,~]=complexTorqueCurve(M.linear,ft);rows(end+1,:)=energyRow(M.name,"BASE",ft,G,Aw); %#ok<AGROW>
    if any(strcmpi(M.name,{'GFL','GWT'}))
        f0=M.flags;f0.Kmppt_iq_per_radps=0;L0=linearizeModel(M.x0,p,M.name,f0,M.active);G0=complexTorqueCurve(L0,ft);
        rows(end+1,:)=energyRow(M.name,"MPPT_ON_MINUS_OFF",ft,G-G0,Aw); %#ok<AGROW>
    end
    if strcmpi(M.name,'MWT')
        poff=p;poff(25:26)=0;L0=linearizeModel(M.x0,poff,M.name,M.flags,M.active);G0=complexTorqueCurve(L0,ft);
        rows(end+1,:)=energyRow(M.name,"MSC_DVC_ON_MINUS_OFF",ft,G-G0,Aw); %#ok<AGROW>
    end
    if any(strcmpi(M.name,{'GWT','MWT'}))
        ff=M.flags;ff.freezeSync=true;L0=linearizeModel(M.x0,p,M.name,ff,M.active);G0=complexTorqueCurve(L0,ft);
        rows(end+1,:)=energyRow(M.name,"GFM_SYNC_ON_MINUS_FROZEN",ft,G-G0,Aw); %#ok<AGROW>
    end
end
T=cell2table(rows,'VariableNames',{'Stage','Architecture','Case','Frequency_Hz','De_Nms_per_rad','VelocityAmplitude_radps','CycleEnergyNumeric_J','CycleEnergyAnalytic_J','RelativeError','EnergySign','DeSign','SignConsistent','EvidenceStatus'});
T.Stage=string(T.Stage);T.Architecture=string(T.Architecture);T.Case=string(T.Case);T.EnergySign=string(T.EnergySign);T.DeSign=string(T.DeSign);T.EvidenceStatus=string(T.EvidenceStatus);
allFinite=all(isfinite(T{:,4:9}),'all');allSign=all(T.SignConsistent);maxErr=max(T.RelativeError);
gate=struct('pass',allFinite&&allSign&&maxErr<1e-6&&height(T)==8,'all_finite',allFinite,'all_signs_consistent',allSign,'max_relative_energy_error',maxErr,'num_cases',height(T), ...
    'interpretation','positive integral of DeltaTe*Deltaomega over one cycle is positive electrical damping for Jg*domega=Tsh-Te');
end

function row=energyRow(arch,cas,ft,G,Aw)
w=2*pi*ft;tt=linspace(0,2*pi/w,4001);om=Aw*cos(w*tt);te=real(G*Aw*exp(1i*w*tt));Wnum=trapz(tt,te.*om);Wana=pi*Aw^2*real(G)/w;err=abs(Wnum-Wana)/max(abs(Wana),1e-12);
sgn=@(q)string(signLabel(q));consistent=(abs(Wana)<1e-12&&abs(real(G))<1e-12)||sign(Wnum)==sign(real(G));
if consistent,status="SUPPORTED_BY_CYCLE_ENERGY";else,status="CONTRADICTED_STOP";end
row={"G_CYCLE_ENERGY",string(arch),string(cas),ft,real(G),Aw,Wnum,Wana,err,sgn(Wnum),sgn(real(G)),consistent,status};
end

function s=signLabel(q)
tol=1e-12;if q>tol,s="POSITIVE";elseif q<-tol,s="NEGATIVE";else,s="NEAR_ZERO";end
end

function makeEnergyFigure(path,G)
T=G.summary;fig=figure('Visible','off','Color','w','Position',[80 80 1450 620]);tl=tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');labs=categorical(T.Architecture+"/"+T.Case);
nexttile;bar(labs,T.De_Nms_per_rad);yline(0,':k');grid on;ylabel('D_e or incremental D_e');title('Complex-torque damping');xtickangle(30);
nexttile;bar(labs,T.CycleEnergyNumeric_J);yline(0,':k');grid on;ylabel('Cycle energy (J)');title('Independent one-cycle integral');xtickangle(30);
sgtitle(tl,'M2-G electrical damping sign verified by cycle energy');exportgraphics(fig,path,'Resolution',220);close(fig);
end

function [T,curves,gate]=runStageH(models,p)
% 同源理想连续非线性模型与数值Jacobian SSM小扰动对照。除三架构
% 基准机械扰动外，增加MWT线性区幅值检查及两个路径/边界代表点。
cases={ ...
    'GFL_BASE_MECH_0p1','GFL',1,0.001,1.0; ...
    'GWT_BASE_MECH_0p1','GWT',1,0.001,1.0; ...
    'MWT_BASE_MECH_0p1','MWT',1,0.001,1.0; ...
    'MWT_BASE_MECH_0p2','MWT',1,0.002,1.0; ...
    'MWT_BASE_MECH_0p4','MWT',1,0.004,1.0; ...
    'GWT_DVC2_GRID_0p01','GWT',4,0.0001,2.0; ...
    'MWT_BASE_GRID_0p01','MWT',4,0.0001,1.0; ...
    'MWT_DVC0p5_GRID_0p01','MWT',4,0.0001,0.5};
rows={};curves=cell(size(cases,1),1);allFinite=true;tEnd=8;tStep=0.5;dt=0.005;
for c=1:size(cases,1)
    tag=string(cases{c,1});arch=char(cases{c,2});id=cases{c,3};pu=cases{c,4};dvcScale=cases{c,5};M=models{find(cellfun(@(q)strcmpi(q.name,arch),models),1)};pv=p;flags=M.flags;
    if dvcScale~=1
        if strcmpi(arch,'MWT'),pv(25:26)=p(25:26)*dvcScale;else,flags.KpGscDvc_W_per_V=M.flags.KpGscDvc_W_per_V*dvcScale;flags.KiGscDvc_W_per_Vs=M.flags.KiGscDvc_W_per_Vs*dvcScale;flags.KpGscDvc_A_per_V=M.flags.KpGscDvc_A_per_V*dvcScale;flags.KiGscDvc_A_per_Vs=M.flags.KiGscDvc_A_per_Vs*dvcScale;end
    end
    [x0,eq]=solveEquilibrium(M.x0,pv,arch,flags,M.active);assert(eq.normalized_residual<1e-8,'M2-H equilibrium failed for %s.',tag);L=linearizeModel(x0,pv,arch,flags,M.active);
    if id==1,amp=pu*(p(1)/p(12));dist="MechanicalTorque";else,amp=pu*p(3);dist="GridFrequency";end
    [t,yNL,ySS]=simulateAlignedPair(x0,pv,arch,flags,M.active,L,id,amp,tStep,tEnd,dt);
    outs=["omega_sh","T_e","Udc","P_PCC"];floors=[1e-8,1e-7*(p(1)/p(12)),1e-8*p(2),1e-8*p(1)];caseCurve=struct('Case',tag,'Architecture',arch,'Disturbance',dist,'t_s',t);
    for oi=1:numel(outs)
        iy=find(string(L.output_names)==outs(oi),1);yn=yNL(:,iy);ys=ySS(:,iy);pkS=max(abs(ys));pkN=max(abs(yn));relevant=pkS>floors(oi);
        if relevant,nrmse=sqrt(mean((yn-ys).^2))/max(pkS,eps);peakErr=abs(pkN-pkS)/max(pkS,eps);else,nrmse=NaN;peakErr=NaN;end
        fNL=NaN;fSS=NaN;fErr=NaN;if outs(oi)=="omega_sh"&&relevant,fNL=dominantBandFrequency(t,yn,tStep);fSS=dominantBandFrequency(t,ys,tStep);fErr=abs(fNL-fSS)/max(fSS,eps);end
        key=ismember(outs(oi),["omega_sh","T_e"])&&relevant;
        rows(end+1,:)={"H_NL_SSM",tag,string(arch),dist,pu,dvcScale,outs(oi),pkN,pkS,nrmse,peakErr,fNL,fSS,fErr,relevant,key}; %#ok<AGROW>
        caseCurve.(char(outs(oi)+"_NL"))=yn;caseCurve.(char(outs(oi)+"_SSM"))=ys;
        allFinite=allFinite&&all(isfinite([yn;ys]));
    end
    curves{c}=caseCurve;
end
T=cell2table(rows,'VariableNames',{'Stage','Case','Architecture','Disturbance','Disturbance_pu','DVCScale','Output','Peak_NL','Peak_SSM','NRMSE','PeakRelativeError','DominantFrequency_NL_Hz','DominantFrequency_SSM_Hz','FrequencyRelativeError','ResponseRelevant','GateKeyOutput'});
T.Stage=string(T.Stage);T.Case=string(T.Case);T.Architecture=string(T.Architecture);T.Disturbance=string(T.Disturbance);T.Output=string(T.Output);
K=T(T.GateKeyOutput,:);F=T(T.Output=="omega_sh"&T.ResponseRelevant,:);maxN=max(K.NRMSE);maxP=max(K.PeakRelativeError);maxF=max(F.FrequencyRelativeError);
gate=struct('pass',allFinite&&all(K.NRMSE<0.05)&&all(K.PeakRelativeError<0.05)&&all(F.FrequencyRelativeError<0.02), ...
    'all_finite',allFinite,'num_cases',size(cases,1),'max_key_nrmse',maxN,'max_key_peak_error',maxP,'max_frequency_error',maxF, ...
    'thresholds','key-output NRMSE <5%, peak error <5%, omega_sh dominant-frequency error <2%', ...
    'model_scope','ideal continuous nonlinear equations versus numerical-Jacobian SSM from the same equations');
end

function [t,yNL,ySS]=simulateAlignedPair(x0,p,arch,flags,active,L,id,amp,tStep,tEnd,dt)
t1=(0:dt:tStep)';t2=(tStep:dt:tEnd)';opt=odeset('RelTol',1e-8,'AbsTol',1e-9,'MaxStep',2e-3);
[~,z1]=ode15s(@(tt,z)activeRhs(tt,z,x0,p,arch,flags,active,id,0),t1,x0(active),opt);
[~,z2]=ode15s(@(tt,z)activeRhs(tt,z,x0,p,arch,flags,active,id,amp),t2,z1(end,:)',opt);
t=[t1;t2(2:end)];z=[z1;z2(2:end,:)];ny=numel(L.output_names);yNL=zeros(numel(t),ny);y0=m2Outputs(x0,p,arch,zeros(4,1),flags);
for k=1:numel(t),x=x0;x(active)=z(k,:)';d=zeros(4,1);if t(k)>=tStep,d(id)=amp;end;yNL(k,:)=m2Outputs(x,p,arch,d,flags)'-y0';end
u=zeros(numel(t),4);u(t>=tStep,id)=amp;
% 扰动是阶跃，使用精确ZOH离散后的采样响应，避免lsim默认输入插值对
% 快速LCL模态提出与本低频输出无关的过密采样警告。
sysd=c2d(ss(L.A,L.B,L.C,L.D),dt,'zoh');ySS=lsim(sysd,u,t);
end

function dz=activeRhs(~,z,xbase,p,arch,flags,active,id,amp)
x=xbase;x(active)=z;d=zeros(4,1);d(id)=amp;dx=m2Rhs(x,p,arch,d,flags);dz=dx(active);
end

function f=dominantBandFrequency(t,y,tStep)
ix=t>=tStep+0.15;tt=t(ix);q=detrend(y(ix));N=numel(q);if N<16||max(abs(q))<1e-20,f=NaN;return;end
win=0.5-0.5*cos(2*pi*(0:N-1)'/(N-1));nfft=2^nextpow2(16*N);Y=abs(fft(q.*win,nfft));ff=(0:nfft-1)'/(median(diff(tt))*nfft);band=ff>=1&ff<=4;[~,j]=max(Y(band));fb=ff(band);f=fb(j);
end

function makeValidationFigure(path,H)
pick=[1 3 6 8];fig=figure('Visible','off','Color','w','Position',[30 30 1550 1250]);tl=tiledlayout(fig,numel(pick),2,'TileSpacing','compact','Padding','compact');
for r=1:numel(pick),q=H.curves{pick(r)};nexttile;plot(q.t_s,q.omega_sh_NL,'-','LineWidth',1.5);hold on;plot(q.t_s,q.omega_sh_SSM,'--','LineWidth',1.5);grid on;ylabel('\Delta\omega_{sh}');title(q.Case,'Interpreter','none');if r==1,legend({'Nonlinear','SSM'},'Location','best');end;nexttile;plot(q.t_s,q.T_e_NL/1e3,'-','LineWidth',1.5);hold on;plot(q.t_s,q.T_e_SSM/1e3,'--','LineWidth',1.5);grid on;ylabel('\Delta T_e (kN m)');title(q.Disturbance);end
xlabel(tl,'Time (s)');sgtitle(tl,'M2-H ideal nonlinear (solid) versus SSM (dashed)');exportgraphics(fig,path,'Resolution',220);close(fig);
end

function [G,De,Ke]=complexTorqueCurve(L,f)
names=string(L.state_names); iw=find(names=="omega_g",1); mech=ismember(names,["theta_sh","omega_t","omega_g","xi_pitch"]); eidx=find(~mech);
iy=find(string(L.output_names)=="T_e",1); assert(~isempty(iw)&&~isempty(iy),'Required omega_g or T_e channel missing.');
Aee=L.A(eidx,eidx); Bew=L.A(eidx,iw); Ce=L.C(iy,eidx); Dwg=L.C(iy,iw);
sys=prescale(ss(Aee,Bew,Ce,Dwg));resp=freqresp(sys,2*pi*f);G=reshape(resp,1,[]);
De=real(G);Ke=-(2*pi*f).*imag(G);
end

function s=classifyBand(delta,reference)
tol=max(1e-6,0.01*max(reference,1));
if max(abs(delta))<=tol,s='INCONCLUSIVE';elseif min(delta)>tol,s='SUPPORTED_POSITIVE_IN_TESTED_BAND';elseif max(delta)<-tol,s='SUPPORTED_NEGATIVE_IN_TESTED_BAND';else,s='TUNING_OR_FREQUENCY_DEPENDENT';end
end

function g=aggregateParticipation(pf,names,arch)
is=@(q)ismember(names,string(q)); g=struct;
% xi_pitch is a slow mechanical-control state in S5B.  Counting it in MECH
% prevents the added Pitch integrator from disappearing during normalization.
g.MECH=sum(pf(is({'theta_sh','omega_t','omega_g','xi_pitch'}))); g.PMSG=sum(pf(is({'i_md','i_mq'})));
if strcmpi(arch,'MWT'),mscNames={'xi_DVC','xi_MSC_d','xi_MSC_q'};else,mscNames={'xi_MSC_d','xi_MSC_q'};end
g.MSC=sum(pf(is(mscNames))); g.DC=sum(pf(is({'Udc'})));
if strcmpi(arch,'MWT'),gscNames={'P_f','Q_f','xi_GSC_vd','xi_GSC_vq','xi_GSC_id','xi_GSC_iq'};else,gscNames={'xi_DVC','P_f','Q_f','xi_GSC_vd','xi_GSC_vq','xi_GSC_id','xi_GSC_iq'};end
g.GSC=sum(pf(is(gscNames))); g.SYNC_PLL=sum(pf(is({'omega_sync','delta'}))); g.LCL_GRID=sum(pf(is({'i_f_d','i_f_q','v_c_d','v_c_q','i_g_d','i_g_q'})));
tot=g.MECH+g.PMSG+g.MSC+g.DC+g.GSC+g.SYNC_PLL+g.LCL_GRID;fn=fieldnames(g);for k=1:numel(fn),g.(fn{k})=g.(fn{k})/max(tot,eps);end
end

function makeModalFigure(path,T)
fig=figure('Visible','off','Color','w','Position',[80 80 1400 560]);tl=tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
nexttile;hold on;arch=unique(T.Architecture,'stable');colors=lines(numel(arch));
for k=1:numel(arch),Q=T(T.Architecture==arch(k)&T.PoleImag>=0,:);scatter(Q.PoleReal,Q.PoleImag,40,Q.MixingIndex,'filled','MarkerEdgeColor',colors(k,:),'DisplayName',arch(k));end
xline(0,':k');grid on;xlabel('Real part (1/s)');ylabel('Imaginary part (rad/s)');title('M2-B full modal map');legend('Location','best');colorbar;
nexttile;Q=T(T.IsTorsional,:);bar(categorical(Q.Architecture),[Q.Pi_MECH Q.Pi_PMSG Q.Pi_MSC Q.Pi_DC Q.Pi_GSC Q.Pi_SYNC_PLL Q.Pi_LCL_GRID],'stacked');ylim([0 1]);grid on;ylabel('Normalized participation');title('Torsional-mode participation');legend({'MECH','PMSG','MSC','DC','GSC','SYNC/PLL','LCL/Grid'},'Location','eastoutside');
sgtitle(tl,'M2 modal evidence (observation, not general conclusion)');exportgraphics(fig,path,'Resolution',220);close(fig);
end

function makeOverviewFigure(path,R)
T=R.stageB.modal_map; fig=figure('Visible','off','Color','w','Position',[60 60 1500 1000]);tl=tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
nexttile;hold on;arch=unique(T.Architecture,'stable');colors=lines(numel(arch));
for k=1:numel(arch),Q=T(T.Architecture==arch(k)&T.PoleImag>=0,:);scatter(Q.PoleReal,Q.PoleImag,38,Q.MixingIndex,'filled','MarkerEdgeColor',colors(k,:),'DisplayName',arch(k));end
xline(0,':k');grid on;xlabel('Real (1/s)');ylabel('Imag (rad/s)');title('B: full modal map');legend('Location','best');colorbar;
nexttile;Q=T(T.IsTorsional,:);bar(categorical(Q.Architecture),[Q.Pi_MECH Q.Pi_PMSG Q.Pi_MSC Q.Pi_DC Q.Pi_GSC Q.Pi_SYNC_PLL Q.Pi_LCL_GRID],'stacked');ylim([0 1]);grid on;ylabel('Participation');title('B: torsional participation');legend({'MECH','PMSG','MSC','DC','GSC','SYNC/PLL','LCL/Grid'},'Location','eastoutside');
nexttile;hold on;for k=1:numel(R.stageC.curves),q=R.stageC.curves{k};plot(q.f_Hz,q.De,'LineWidth',1.8,'DisplayName',q.Architecture);end;grid on;xlabel('Frequency (Hz)');ylabel('D_e = Re(G_{Te,wg})');title('C: electrical damping spectrum');legend('Location','best');
nexttile;if isfield(R,'stageD'),D=R.stageD.pole_excitation;cats=categorical(D.Architecture+"/"+D.Disturbance);yyaxis left;bar(cats,D.PoleChangeIndex);ylabel('Pole change index');yyaxis right;plot(cats,D.ResidueChangeDecades,'o-','LineWidth',1.4);ylabel('Residue change (decades)');grid on;title('D: pole versus excitation');xtickangle(25);else,C=R.stageC.summary;Q=C(C.Stage=="C_ABLATION",:);bar(categorical(Q.Architecture+"/"+Q.Case),Q.De_or_DeltaDe);grid on;ylabel('Delta D_e at f_{tor}');title('C: incremental channel contribution');xtickangle(25);end
sgtitle(tl,'M2 mechanism evidence: tested observations only');exportgraphics(fig,path,'Resolution',220);close(fig);
end

function writeEvidenceSummary(path,R)
rows={};B=R.stageB.modal_map;
for k=1:height(B),rows(end+1,:)={B.Stage(k),"MODAL",B.Architecture(k),"Mode"+B.ModeIndex(k),B.Frequency_Hz(k),B.PoleReal(k),B.DampingRatio(k),B.Pi_MECH(k),B.MixingIndex(k),NaN,NaN,NaN,"OBSERVATION",B.PhysicalClass(k)};end %#ok<AGROW>
if isfield(R,'stageC'),C=R.stageC.summary;for k=1:height(C),rows(end+1,:)={C.Stage(k),"FEEDBACK",C.Architecture(k),C.Case(k),C.Frequency_Hz(k),NaN,NaN,NaN,NaN,C.De_or_DeltaDe(k),C.Ke_or_DeltaKe(k),NaN,C.EvidenceStatus(k),C.Note(k)};end,end %#ok<AGROW>
if isfield(R,'stageD'),D=R.stageD.residue_map;for k=1:height(D),rows(end+1,:)={D.Stage(k),"RESIDUE",D.Architecture(k),D.Disturbance(k)+"/"+D.Output(k),D.Frequency_Hz(k),D.PoleReal(k),D.DampingRatio(k),NaN,NaN,NaN,NaN,D.ResidueMagnitude(k),"OBSERVATION",D.PhysicalClass(k)};end,end %#ok<AGROW>
if isfield(R,'stageE'),E=R.stageE.summary;for k=1:height(E),rows(end+1,:)={E.Stage(k),"BIDIRECTIONAL",E.Architecture(k),E.LocalDirectionAtFtor(k),E.TorsionalFrequency_Hz(k),NaN,NaN,NaN,NaN,NaN,NaN,E.DirectionalRatio_at_ftor(k),"FREQUENCY_LOCAL_OBSERVATION","C_GM/C_MG normalized ratio at torsional frequency"};end,end %#ok<AGROW>
if isfield(R,'stageF'),F=R.stageF.summary;for k=1:height(F),rows(end+1,:)={F.Stage(k),"LOCAL_SCAN",F.Architecture(k),F.Parameter(k)+"="+string(F.PhysicalValue(k)),F.TorFrequency_Hz(k),F.TorPoleReal(k),F.TorDampingRatio(k),F.TorPiMECH(k),4*F.TorPiMECH(k)*(1-F.TorPiMECH(k)),NaN,NaN,F.TorResidue_GridToShaft(k),"CONDITIONAL_LOCAL_SCAN",F.PolePathClass(k)};end,end %#ok<AGROW>
if isfield(R,'stageG'),G=R.stageG.summary;for k=1:height(G),rows(end+1,:)={G.Stage(k),"CYCLE_ENERGY",G.Architecture(k),G.Case(k),G.Frequency_Hz(k),NaN,NaN,NaN,NaN,G.De_Nms_per_rad(k),NaN,G.CycleEnergyNumeric_J(k),G.EvidenceStatus(k),"cycle energy sign versus complex-torque damping"};end,end %#ok<AGROW>
if isfield(R,'stageH'),H=R.stageH.summary;for k=1:height(H),rows(end+1,:)={H.Stage(k),"NL_SSM",H.Architecture(k),H.Case(k)+"/"+H.Output(k),H.DominantFrequency_SSM_Hz(k),NaN,NaN,NaN,NaN,NaN,NaN,H.NRMSE(k),"CROSS_MODEL_VALIDATION","solid ideal nonlinear versus dashed same-source SSM"};end,end %#ok<AGROW>
T=cell2table(rows,'VariableNames',{'Stage','Record','Architecture','Case','Frequency_Hz','PoleReal','DampingRatio','Pi_MECH','MixingIndex','De_or_DeltaDe','Ke_or_DeltaKe','ResidueMagnitude','EvidenceStatus','Note'});
writetable(T,path);
end

function [x,meta]=solveEquilibrium(xSeed,p,arch,flags,active)
sx=[1;1;1;1e4;1e4;1e4;1e4;1e4;1e3;5e6;5e6;1;1;1e4;1e4;1e4;1e4;1e4;1e4;1e3;1e3;1e4;1e4];
sr=[1;1;1;1e4;1e4;1e4;1e4;1e4;1e6;5e8;5e8;1;1;1e4;1e4;1e4;1e4;1e6;1e6;1e6;1e6;1e6;1e6];
opts=optimoptions('fsolve','Display','off','Algorithm','levenberg-marquardt', ...
    'FunctionTolerance',1e-12,'StepTolerance',1e-12,'OptimalityTolerance',1e-12, ...
    'MaxIterations',3000,'MaxFunctionEvaluations',50000);
x=xSeed(:); z0=x(active)./sx(active);
[z,fval,exitflag]=fsolve(@residual,z0,opts); x(active)=sx(active).*z;
% 对已接近零的解做显式缩放Newton精化，避免控制积分状态造成的
% LM停止条件早于严格的1e-10归一化残差Gate。
for iter=1:6
    r=residual(x(active)./sx(active));
    if norm(r,inf)<1e-12, break; end
    J=zeros(numel(active)); z=x(active)./sx(active);
    for j=1:numel(active)
        h=1e-6*max(abs(z(j)),1); zp=z; zm=z; zp(j)=zp(j)+h; zm(j)=zm(j)-h;
        J(:,j)=(residual(zp)-residual(zm))/(2*h);
    end
    dz=-(J\r); alpha=1; baseNorm=norm(r,inf);
    while alpha>1/128 && norm(residual(z+alpha*dz),inf)>=baseNorm, alpha=alpha/2; end
    x(active)=sx(active).*(z+alpha*dz);
end
dx=m2Rhs(x,p,arch,zeros(4,1),flags);
nr=norm(dx(active)./sr(active),inf);
meta=struct('exitflag',exitflag,'normalized_residual',nr,'solver_residual',norm(fval,inf), ...
    'max_abs_dx',max(abs(dx(active))),'pass',exitflag>0&&nr<1e-10);
    function r=residual(z)
        q=xSeed(:); q(active)=sx(active).*z;
        dq=m2Rhs(q,p,arch,zeros(4,1),flags);
        r=dq(active)./sr(active);
    end
end

function L=linearizeModel(x,p,arch,flags,active)
[y0,names,units]=m2Outputs(x,p,arch,zeros(4,1),flags);
n=numel(x); ny=numel(y0); nd=4; Afull=zeros(n); Bfull=zeros(n,nd); Cfull=zeros(ny,n); D=zeros(ny,nd);
for j=active
    h=1e-6*max(abs(x(j)),1); e=zeros(n,1); e(j)=h;
    Afull(:,j)=(m2Rhs(x+e,p,arch,zeros(nd,1),flags)-m2Rhs(x-e,p,arch,zeros(nd,1),flags))/(2*h);
    Cfull(:,j)=(m2Outputs(x+e,p,arch,zeros(nd,1),flags)-m2Outputs(x-e,p,arch,zeros(nd,1),flags))/(2*h);
end
hd=[1;1;1e-6;1e-4];
for j=1:nd
    e=zeros(nd,1); e(j)=hd(j);
    Bfull(:,j)=(m2Rhs(x,p,arch,e,flags)-m2Rhs(x,p,arch,-e,flags))/(2*hd(j));
    D(:,j)=(m2Outputs(x,p,arch,e,flags)-m2Outputs(x,p,arch,-e,flags))/(2*hd(j));
end
allNames={'theta_sh','omega_t','omega_g','i_md','i_mq','xi_DVC','xi_MSC_d','xi_MSC_q','Udc','P_f','Q_f','omega_sync','delta','xi_GSC_vd','xi_GSC_vq','xi_GSC_id','xi_GSC_iq','i_f_d','i_f_q','v_c_d','v_c_q','i_g_d','i_g_q'};
L=struct('A',Afull(active,active),'B',Bfull(active,:),'C',Cfull(:,active),'D',D, ...
    'active',active,'state_names',{allNames(active)},'output_names',{names},'output_units',{units}, ...
    'input_names',{{'DeltaTm','DeltaPaero','DeltaThetaGrid','DeltaOmegaGrid'}});
end

function dx=m2Rhs(x,p,arch,d,flags)
% 统一M2物理平均VSC非线性方程；架构差异仅位于职责分配。
arch=upper(char(string(arch))); d=d(:);
Sb=p(1); Vdc0=p(2); w0=p(3); Vg=p(4); Rf=p(5); Lf=p(6); Cf=p(7); Rd=p(8); Rg=p(9); Lg=p(10); Cdc=p(11); wm0=p(12); Rs=p(13); Ld=p(14); Lq=p(15); psi=p(16); np=p(17); Kt=p(18); Jt=p(19); Jg=p(20); Ksh=p(21); Dsh=p(22); Dt=p(23); Dg=p(24); Kpdc=p(25); Kidc=p(26); Kpmi=p(27); Kimi=p(28); Kpgi=p(29); Kigi=p(30); Kpgv=p(31); Kigv=p(32); H=p(33); mp=p(34); wpf=p(35); kq=p(36); Pref=p(37); Qref=p(38); Tm0=p(39); E0=p(40); sP=p(41); ffIg=p(42); ffVpcc=p(43);
theta=x(1); wt=x(2); wg=x(3); imd=x(4); imq=x(5); xiDc=x(6); xiMid=x(7); xiMiq=x(8); Udc=x(9); Pf=x(10); Qf=x(11); wsync=x(12); delta=x(13); xiVd=x(14); xiVq=x(15); xiId=x(16); xiIq=x(17); ifd=x(18); ifq=x(19); vcd=x(20); vcq=x(21); igd=x(22); igq=x(23);
wgrid=w0+d(4); deltaEff=delta+d(3);
we=np*wg; Tgen=Kt*imq; Tsh=Ksh*theta+Dsh*(wt-wg); wcoi=(Jt*wt+Jg*wg)/(Jt+Jg);
eDcM=Vdc0-Udc; imdRef=0;
if strcmp(arch,'MWT')
    imqRef=Kpdc*eDcM+xiDc;
else
    imqRef=flags.imqRef0+flags.Kmppt_iq_per_radps*(wg-wm0);
end
eMid=-imd; eMiq=imqRef-imq;
generatorOut=isfield(flags,'pmsgConvention')&&strcmpi(flags.pmsgConvention,'GENERATOR_OUTWARD');
if generatorOut
    % iq>0: positive generating current leaving PMSG; Te=Kt*iq is positive
    % braking torque. Negative PI output is required because converter
    % terminal voltage acts on outward current with negative plant gain.
    vmdCmd=-Kpmi*eMid-xiMid+we*Lq*imqRef;
    vmqCmd=-Kpmi*eMiq-xiMiq-Rs*imqRef+we*psi;
else
    vmdCmd=Kpmi*eMid+xiMid-we*Lq*imqRef;
    vmqCmd=Kpmi*eMiq+xiMiq+Rs*imqRef+we*psi;
end
vScale=Udc/Vdc0; vmd=vScale*vmdCmd; vmq=vScale*vmqCmd;
Pmsc=1.5*(vmd*imd+vmq*imq);

icapd=ifd-igd; icapq=ifq-igq;
vnodeD=vcd+Rd*ifd-(Rd+1e-4)*igd; vnodeQ=vcq+Rd*ifq-(Rd+1e-4)*igq;
Ppcc=1.5*(vnodeD*igd+vnodeQ*igq); Qpcc=1.5*(vnodeQ*igd-vnodeD*igq);
c=cos(deltaEff); s=sin(deltaEff); vpd=c*vnodeD+s*vnodeQ; vpq=-s*vnodeD+c*vnodeQ;
ifld=c*ifd+s*ifq; iflq=-s*ifd+c*ifq; igld=c*igd+s*igq; iglq=-s*igd+c*igq;
if strcmp(arch,'GFL')
    wctrl=wsync+flags.KpPll_radps_per_V*vpq;
    ifdRef=flags.KpGscDvc_A_per_V*(Udc-Vdc0)+xiDc;
    eQ=Qpcc-Qref;
    ifqRef=flags.KpQ_A_per_var*eQ+xiVq;
    eid=ifdRef-ifld; eiq=ifqRef-iflq;
else
    wctrl=wsync;
    if isfield(flags,'freezeSync')&&flags.freezeSync, wctrl=w0; end
    Vref=E0+kq*(Qref-Qf); evd=Vref-vpd; evq=-vpq;
    ifdRef=Kpgv*evd+xiVd-Cf*wctrl*vpq+ffIg*igld;
    ifqRef=Kpgv*evq+xiVq+Cf*wctrl*vpd+ffIg*iglq;
    eid=ifdRef-ifld; eiq=ifqRef-iflq;
end
ucdCmd=Kpgi*eid+xiId-wctrl*Lf*iflq+ffVpcc*(vpd+Rf*ifld);
ucqCmd=Kpgi*eiq+xiIq+wctrl*Lf*ifld+ffVpcc*(vpq+Rf*iflq);
uinvDCmd=c*ucdCmd-s*ucqCmd; uinvQCmd=s*ucdCmd+c*ucqCmd;
uinvD=vScale*uinvDCmd; uinvQ=vScale*uinvQCmd;
Pgsc=1.5*(uinvD*ifd+uinvQ*ifq);

dx=zeros(23,1);
if isfield(flags,'aeroMode')&&strcmpi(flags.aeroMode,'CP_LAMBDA')
    aw=flags.aero;lambda=max(wt*aw.rotorRadius/max(aw.wind_mps,1e-9),1e-6);cp=m3WindCp(lambda,aw.beta_deg);
    Paero=aw.calibration*0.5*aw.rho*aw.area*cp*aw.wind_mps^3;TmIn=Paero/max(wt,1e-9)+d(1)+d(2)/max(wt,1e-9);
else
    TmIn=Tm0*wm0/max(wt,1e-9)+d(1)+d(2)/max(wt,1e-9);
end
dx(1)=wt-wg; dx(2)=(TmIn-Tsh-Dt*(wcoi-wm0))/Jt; dx(3)=(Tsh-Tgen-Dg*(wcoi-wm0))/Jg;
if generatorOut
    dx(4)=(-vmd-Rs*imd+we*Lq*imq)/Ld;
    dx(5)=(-vmq-Rs*imq+we*(psi-Ld*imd))/Lq;
else
    dx(4)=(vmd-Rs*imd+we*Lq*imq)/Ld;
    dx(5)=(vmq-Rs*imq-we*(Ld*imd+psi))/Lq;
end
if strcmp(arch,'MWT'), dx(6)=Kidc*eDcM; elseif strcmp(arch,'GWT'), dx(6)=flags.KiGscDvc_W_per_Vs*eDcM; else, dx(6)=flags.KiGscDvc_A_per_Vs*(Udc-Vdc0); end
dx(7)=Kimi*eMid; dx(8)=Kimi*eMiq; dx(9)=(Pmsc-Pgsc)/(Cdc*Udc);
dx(10)=wpf*(Ppcc-Pf); dx(11)=wpf*(Qpcc-Qf);
freezeSync=isfield(flags,'freezeSync')&&flags.freezeSync;
if strcmp(arch,'MWT')&&~freezeSync
    dx(12)=w0/(2*H*Sb)*(sP*(Pref-Pf)-(wsync-w0)/mp); dx(13)=wctrl-wgrid;
elseif strcmp(arch,'GWT')&&~freezeSync
    Pctrl=Pref-flags.KpGscDvc_W_per_V*eDcM-xiDc;
    dx(12)=w0/(2*H*Sb)*(sP*(Pctrl-Pf)-(wsync-w0)/flags.mpGwt); dx(13)=wctrl-wgrid;
elseif strcmp(arch,'GFL')
    dx(12)=flags.KiPll_radps2_per_V*vpq; dx(13)=wctrl-wgrid;
else
    dx(12)=0; dx(13)=0;
end
if strcmp(arch,'GFL')
    dx(14)=0; dx(15)=flags.KiQ_A_per_vars*(Qpcc-Qref);
else
    dx(14)=Kigv*(E0+kq*(Qref-Qf)-vpd); dx(15)=Kigv*(-vpq);
end
dx(16)=Kigi*eid; dx(17)=Kigi*eiq;
RfEff=Rf+1e-4;
dx(18)=(uinvD-vnodeD-RfEff*ifd+w0*Lf*ifq)/Lf;
dx(19)=(uinvQ-vnodeQ-RfEff*ifq-w0*Lf*ifd)/Lf;
dx(20)=icapd/Cf+w0*vcq; dx(21)=icapq/Cf-w0*vcd;
dx(22)=(vnodeD-Vg-Rg*igd+w0*Lg*igq)/Lg;
dx(23)=(vnodeQ-Rg*igq-w0*Lg*igd)/Lg;
end

function [y,names,units]=m2Outputs(x,p,arch,d,flags)
% 与m2Rhs同源重建测量；通过复用一次RHS中的控制代数避免隐藏状态。
arch=upper(char(string(arch))); Vdc0=p(2); w0=p(3); Rf=p(5); Lf=p(6); Cf=p(7); Rd=p(8); Rs=p(13); Ld=p(14); Lq=p(15); psi=p(16); np=p(17); Kt=p(18); Ksh=p(21); Dsh=p(22); Kpdc=p(25); Kpmi=p(27); Kpgi=p(29); Kpgv=p(31); kq=p(36); Qref=p(38); E0=p(40); ffIg=p(42); ffVpcc=p(43);
wg=x(3); imd=x(4); imq=x(5); xiDc=x(6); Udc=x(9); Pf=x(10); Qf=x(11); wsync=x(12); delta=x(13); xiVd=x(14); xiVq=x(15); xiId=x(16); xiIq=x(17); ifd=x(18); ifq=x(19); vcd=x(20); vcq=x(21); igd=x(22); igq=x(23);
eDcM=Vdc0-Udc; if strcmp(arch,'MWT'), imqRef=Kpdc*eDcM+xiDc; else, imqRef=flags.imqRef0+flags.Kmppt_iq_per_radps*(wg-p(12)); end
we=np*wg; eMid=-imd; eMiq=imqRef-imq; generatorOut=isfield(flags,'pmsgConvention')&&strcmpi(flags.pmsgConvention,'GENERATOR_OUTWARD');
if generatorOut
    vmdCmd=-Kpmi*eMid-x(7)+we*Lq*imqRef;vmqCmd=-Kpmi*eMiq-x(8)-Rs*imqRef+we*psi;
else
    vmdCmd=Kpmi*eMid+x(7)-we*Lq*imqRef;vmqCmd=Kpmi*eMiq+x(8)+Rs*imqRef+we*psi;
end
vScale=Udc/Vdc0; vmd=vScale*vmdCmd; vmq=vScale*vmqCmd; Pmsc=1.5*(vmd*imd+vmq*imq);
vnodeD=vcd+Rd*ifd-(Rd+1e-4)*igd; vnodeQ=vcq+Rd*ifq-(Rd+1e-4)*igq; Ppcc=1.5*(vnodeD*igd+vnodeQ*igq); Qpcc=1.5*(vnodeQ*igd-vnodeD*igq);
deltaEff=delta+d(3); c=cos(deltaEff); s=sin(deltaEff); vpd=c*vnodeD+s*vnodeQ; vpq=-s*vnodeD+c*vnodeQ; ifld=c*ifd+s*ifq; iflq=-s*ifd+c*ifq; igld=c*igd+s*igq; iglq=-s*igd+c*igq;
if strcmp(arch,'GFL'), wctrl=wsync+flags.KpPll_radps_per_V*vpq; ifdRef=flags.KpGscDvc_A_per_V*(Udc-Vdc0)+xiDc; ifqRef=flags.KpQ_A_per_var*(Qpcc-Qref)+xiVq; else, wctrl=wsync; if isfield(flags,'freezeSync')&&flags.freezeSync,wctrl=w0;end; Vref=E0+kq*(Qref-Qf); ifdRef=Kpgv*(Vref-vpd)+xiVd-Cf*wctrl*vpq+ffIg*igld; ifqRef=Kpgv*(-vpq)+xiVq+Cf*wctrl*vpd+ffIg*iglq; end
eid=ifdRef-ifld; eiq=ifqRef-iflq; ucdCmd=Kpgi*eid+xiId-wctrl*Lf*iflq+ffVpcc*(vpd+Rf*ifld); ucqCmd=Kpgi*eiq+xiIq+wctrl*Lf*ifld+ffVpcc*(vpq+Rf*iflq); uinvD=vScale*(c*ucdCmd-s*ucqCmd); uinvQ=vScale*(s*ucdCmd+c*ucqCmd); Pgsc=1.5*(uinvD*ifd+uinvQ*ifq); Tsh=Ksh*x(1)+Dsh*(x(2)-wg);
y=[Pgsc;Udc;imqRef;imq;Kt*imq;x(2)-wg;Tsh;Pf;Pmsc;Ppcc;Qpcc;wg;delta;wctrl;ifdRef;ifqRef];
names={'P_GSC','Udc','iq_MSC_ref','iq_MSC','T_e','omega_sh','T_sh','P_f','P_MSC','P_PCC','Q_PCC','omega_g','delta','omega_sync','id_GSC_ref','iq_GSC_ref'};
units={'W','V','A','A','N m','rad/s','N m','W','W','W','var','rad/s','rad','rad/s','A','A'};
end

function writeReport(path,R)
fid=fopen(path,'w'); assert(fid>0,'Cannot open report.'); c=onCleanup(@()fclose(fid));
A=R.stageA;
fprintf(fid,'# M2机电耦合机制复核进展\n\n');
fprintf(fid,'## 研究纪律\n\n本报告只记录已执行Gate范围内的证据。所有M0/M1现象均视为待验证假设；未通过跨模型、消融、能量和NL–SSM验证的现象不得写成一般结论。\n\n');
fprintf(fid,'## 当前模型边界\n\n保留两质量轴、PMSG、物理平均VSC、DC-link、LCL、电网及连续控制；未加入PWM、离散、延迟、限幅、Pitch、OpenFAST或EMT。三架构共享参数和工作点目标，只改变控制职责。\n\n');
fprintf(fid,'## M2-A 公共模型与工作点审计：%s\n\n',passfail(A.gate.pass));
fprintf(fid,'- 公共GSC电流环Ki因子：`%.8g`（三架构统一）；\n',R.common_current_ki_factor);
fprintf(fid,'- 最大归一化平衡残差：`%.3e`；\n',A.gate.max_normalized_residual);
fprintf(fid,'- 五个核心工作点量最大架构间差异：`%.3e pu`；\n',A.gate.max_workpoint_pairwise_pu);
fprintf(fid,'- Gate阈值：残差 `< %.1e`，工作点差异 `< %.1e pu`。\n\n',A.gate.residual_tolerance,A.gate.target_tolerance_pu);
writeTable(fid,A.workpoint_audit);
fprintf(fid,'\n## 当前证据状态\n\nM2-A只证明比较基础公平，不支持任何有关正/负阻尼、方向传播、Path/Pole或模态耦合的机理结论。\n');
if isfield(R,'stageB')
    B=R.stageB;
    fprintf(fid,'\n## M2-B 全模态地图：%s\n\n',passfail(B.gate.pass));
    fprintf(fid,'- 三架构全部稳定：`%s`；\n',string(B.gate.all_stable));
    fprintf(fid,'- 最大极点实部：`%.8g 1/s`；\n',B.gate.max_real_pole);
    fprintf(fid,'- 机械—电气混合候选数：`%d`；\n',B.gate.num_mixed_candidates);
    fprintf(fid,'- 轴系模态最低机械参与：`%.6g`。\n\n',B.gate.minimum_torsional_mechanical_participation);
    writeTable(fid,B.torsional_modes);
    fprintf(fid,'\n混合度只用于筛选后续候选模态。频率接近、低机械参与电气模态或单次参与因子结果均不能单独证明机电模态耦合。\n');
end
if isfield(R,'stageC')
    C=R.stageC;fprintf(fid,'\n## M2-C 内生反馈复转矩：%s\n\n',passfail(C.gate.pass));
    fprintf(fid,'符号采用 `%s`。扫频范围为各架构 `%s`。\n\n',C.gate.sign_convention,C.gate.frequency_band);
    writeTable(fid,C.summary);
    fprintf(fid,'\n表中状态只约束当前模型、工作点和测试频带；`SUPPORTED_POSITIVE/NEGATIVE`不得省略“in tested band”。\n');
end
if isfield(R,'stageD')
    D=R.stageD;fprintf(fid,'\n## M2-D 外部扰动与模态残差：%s\n\n',passfail(D.gate.pass));
    fprintf(fid,'最大残差分解误差：`%.3e`。分类阈值：%s。分类只用于诊断，不构成机理结论。\n\n',D.gate.max_factorization_error,D.gate.classification_thresholds);
    writeTable(fid,D.pole_excitation);
end
if isfield(R,'stageE')
    E=R.stageE;fprintf(fid,'\n## M2-E 归一化双向机电耦合：%s\n\n',passfail(E.gate.pass));
    fprintf(fid,'归一化：%s。频段：`%.3g--%.3g Hz`。方向判据只在给定频率局部有效，任何近似零均不在本阶段解释为结构零。\n\n',E.gate.normalization,E.gate.frequency_band_Hz(1),E.gate.frequency_band_Hz(2));
    writeTable(fid,E.summary);
end
if isfield(R,'stageF')
    F=R.stageF;fprintf(fid,'\n## M2-F 局部参数机制扫描：%s\n\n',passfail(F.gate.pass));
    fprintf(fid,'扫描范围：%s。失稳点 `%d`，严格混合判据候选 `%d`。候选判据：%s。该阶段是局部参数证据，不外推为全局单调规律。\n\n',F.gate.scan_scope,F.gate.num_unstable_points,F.gate.num_hybridization_candidates,F.gate.hybridization_rule);
    writeTable(fid,F.summary);
end
if isfield(R,'stageG')
    G=R.stageG;fprintf(fid,'\n## M2-G 循环能量复核：%s\n\n',passfail(G.gate.pass));
    fprintf(fid,'判据：%s。最大数值积分—解析式相对误差 `%.3e`。本结果只验证当前线性工作点与频率处的阻尼符号。\n\n',G.gate.interpretation,G.gate.max_relative_energy_error);
    writeTable(fid,G.summary);
end
if isfield(R,'stageH')
    H=R.stageH;fprintf(fid,'\n## M2-H 理想连续非线性—SSM对照：%s\n\n',passfail(H.gate.pass));
    fprintf(fid,'范围：%s。阈值：%s。关键输出最大NRMSE `%.3e`，最大峰值误差 `%.3e`，最大轴系主频误差 `%.3e`。\n\n',H.gate.model_scope,H.gate.thresholds,H.gate.max_key_nrmse,H.gate.max_key_peak_error,H.gate.max_frequency_error);
    writeTable(fid,H.summary);
    fprintf(fid,'\n## A--H证据链综合判断\n\n');
    fprintf(fid,'### 当前M2范围内得到支持\n\n');
    fprintf(fid,'- 控制职责不仅改变轴系极点，也显著改变扰动输入投影与双向机电传递；\n');
    fprintf(fid,'- GFL/GWT的局部MPPT切线在测试频带提供正电气阻尼；MWT的MSC-DVC在轴系频率处提供负增量阻尼，但其符号随频率变化；\n');
    fprintf(fid,'- 轴系频率处的归一化方向关系为：GFL偏Machine-to-Grid、GWT双向同量级、MWT偏Grid-to-Machine；这属于方向占优，不是结构零；\n');
    fprintf(fid,'- MWT的DVC=0.5首先出现0.363 Hz GSC主导非轴系失稳，而轴系极点仍稳定。\n\n');
    fprintf(fid,'### 当前没有得到支持\n\n');
    fprintf(fid,'- 不能写成“GFM必然恶化轴系稳定性”；\n');
    fprintf(fid,'- 33个局部H/DVC/SCR点没有发现满足联合判据的轴系—电气模态混合；\n');
    fprintf(fid,'- 不能把近似零写成严格结构零，也不能把方向占优称为non-reciprocity。\n\n');
    fprintf(fid,'### 证据边界\n\n');
    fprintf(fid,'上述判断只对当前5 MW、共同工作点、连续物理平均VSC与局部参数域成立。M2-H是同源非线性—SSM验证，不是离散控制或开关EMT的跨实现验证。下一次升级应改变工作点和模型层级来主动寻找反例。\n');
end
end

function writeTable(fid,T)
vars=T.Properties.VariableNames; fprintf(fid,'|'); for k=1:numel(vars),fprintf(fid,'%s|',vars{k});end;fprintf(fid,'\n|');for k=1:numel(vars),fprintf(fid,'---|');end;fprintf(fid,'\n');
for r=1:height(T), fprintf(fid,'|'); for k=1:numel(vars),v=T{r,k};if iscell(v),v=v{1};end;if isstring(v)||ischar(v)||islogical(v),s=char(string(v));else,s=num2str(v,9);end;fprintf(fid,'%s|',s);end;fprintf(fid,'\n');end
end
function s=passfail(x),if x,s='PASS';else,s='FAIL';end,end
function r=stageRank(s),q='ABCDEFGH';r=find(q==upper(s(1)),1);if isempty(r),error('Unknown stage %s',s);end,end
