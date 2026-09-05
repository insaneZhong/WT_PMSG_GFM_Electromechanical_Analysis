function R = run_multimode_grid_disturbance_mechanism(varargin)
%RUN_MULTIMODE_GRID_DISTURBANCE_MECHANISM
% GFM网侧扰动多模态传递机理与参数规律分析主入口。
%
% 执行顺序：Gate A输入公平性 → Gate B传递/多模态机理 →
% SCR/H/DVC摘要扫描 → 少量理想连续非线性验证 → 汇总图和中文报告。
% 所有模型均复用 Architecture_Comparison_Summary.mat 中唯一对齐参数源。
% 不引入EMT、PWM、离散PI、延迟、限幅、启动或保护逻辑。
ip=inputParser; ip.addParameter('SaveFigures',true,@(x)islogical(x)&&isscalar(x)); ip.addParameter('RunRepresentativeNonlinear',true,@(x)islogical(x)&&isscalar(x)); ip.parse(varargin{:}); opt=ip.Results;
here=fileparts(mfilename('fullpath'));

% ---- Gate A：外部 grid-frequency 输入的物理含义与单位审计 ----
[models,base]=prepare_multimode_models(); %#ok<ASGLU>
[frequencyAudit,gateA]=audit_grid_frequency_disturbance_input(models,here);
assert(gateA,'Gate A FAIL：停止全部后续多模态分析。');

% ---- Gate B：路径差异、全模态贡献、最小重构与相位叠加 ----
[chain,firstDivergence]=analyze_grid_to_shaft_transfer_chain(models,here);
gwtMwt=compare_gfm_gwt_vs_mwt_path(models,chain,here);
[fullRanking,~]=analyze_full_modal_contributions(models,here);
[minimal,dominant,details,gateMinimal]=build_minimal_multimode_reconstruction(models,here);
superposition=analyze_modal_superposition(details,here);
freqAngle=analyze_frequency_vs_angle_excitation(models,here);
nonGfl=firstDivergence(~contains(firstDivergence.Architecture,'GFL'),:);
gateDivergence=all(nonGfl.First_Divergence_Node~="NONE");
gatePath=any(gwtMwt.Signal=="iq_MSC_ref" & gwtMwt.MWT_Magnitude>1e-8 & gwtMwt.GWT_Magnitude<1e-10);
gateSuper=height(superposition)>0;
gateB=gateDivergence && gatePath && gateMinimal && gateSuper;
assert(gateB,'Gate B FAIL：未形成可验证的逐级路径或最小多模态闭合，停止参数扫描。');

% ---- Gate B 通过后才允许扫描；所有扫描只保留摘要CSV。 ----
scr=scan_scr_multimode_excitation(here);
H=scan_vsg_H_multimode_excitation(here);
DVC=scan_dvc_bandwidth_multimode_excitation(here);

if opt.RunRepresentativeNonlinear
    validation=validate_multimode_representative_nonlinear();
else
    validation=struct([]);
end
if opt.SaveFigures && ~isempty(validation)
    make_multimode_mechanism_figures(chain,details,scr,H,DVC,validation,here);
end

R=struct('objective','GFM网侧扰动多模态传递机理与参数规律分析','gateA',gateA,'gateB',gateB, ...
    'frequency_input_audit',frequencyAudit,'transfer_chain',chain,'first_divergence',firstDivergence,'gwt_mwt_path',gwtMwt, ...
    'full_modal_ranking',fullRanking,'minimal_reconstruction',minimal,'minimal_dominant',dominant,'modal_superposition',superposition, ...
    'frequency_angle_projection',freqAngle,'scr_scan',scr,'H_scan',H,'DVC_scan',DVC,'representative_validation',validation);
write_multimode_disturbance_report(fullfile(here,'Multimode_Disturbance_Path_Mechanism_Report_CN.md'),R);
latest=fullfile(here,'latest_failed_case.mat'); if exist(latest,'file'), delete(latest); end
fprintf('MULTIMODE_GRID_DISTURBANCE_GATE_A=%d\n',gateA); fprintf('MULTIMODE_GRID_DISTURBANCE_GATE_B=%d\n',gateB);
end
