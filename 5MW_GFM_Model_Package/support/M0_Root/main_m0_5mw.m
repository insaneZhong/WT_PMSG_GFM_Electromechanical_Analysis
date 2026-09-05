function result = main_m0_5mw(action,varargin)
%MAIN_M0_5MW 5 MW理想连续非线性模型与小信号模型的统一主程序。
%
% 推荐用法：
%   main_m0_5mw("构建")      % 生成/更新唯一SLX并编译
%   R = main_m0_5mw("工作点") % 求严格5 MW平衡点，不运行仿真
%   R = main_m0_5mw("线性化") % 从同一非线性RHS生成A/B/C/D
%   R = main_m0_5mw("验证")   % 默认：完成全部门槛并保存精简成果
%
% 本程序只维护一个模型 M0_PMSG_GFM_5MW.slx。非线性仿真与小信号
% 模型共同调用m0_nonlinear_dynamics.m，不再分别维护两套方程。

if nargin<1 || strlength(string(action))==0
    action = "验证";
end
ip = inputParser;
ip.addParameter('ComparisonTime_s',10,@(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('StepTime_s',0.5,@(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('dPref_pu',1e-4,@(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('SaveFinal',true,@(x)islogical(x)&&isscalar(x));
ip.parse(varargin{:});
o = ip.Results;

rootDir = fileparts(mfilename('fullpath'));
addpath(rootDir);
P = init_m0_5mw_parameters('ControllerProfile','AlignedStable');
[OP,P] = solve_m0_equilibrium(P);
lin = linearize_m0_equilibrium(P,OP);
[pvec,parameter_names] = m0_pack_parameters(P,OP);

key = lower(strtrim(string(action)));
switch key
    case {"构建","build"}
        buildInfo = build_m0_model('Parameters',P,'Compile',true);
        result = struct('build',buildInfo,'parameters',P, ...
            'operating_point',OP);

    case {"工作点","operatingpoint","workpoint","op"}
        result = struct('parameters',P,'operating_point',OP);

    case {"线性化","linearize","ssm"}
        result = struct('parameters',P,'operating_point',OP, ...
            'linear_model',lin);

    case {"验证","完整验证","validate"}
        buildInfo = build_m0_model('Parameters',P,'Compile',true);
        validation_results = validate_m0_alignment( ...
            'ComparisonTime_s',o.ComparisonTime_s, ...
            'StepTime_s',o.StepTime_s, ...
            'dPref_pu',o.dPref_pu);
        summary = makeSummary(P,OP,lin,validation_results);
        result = struct('build',buildInfo,'parameters',P, ...
            'operating_point',OP,'linear_model',lin, ...
            'validation',validation_results,'summary',summary);
        if o.SaveFinal
            saveFinalArtifacts(rootDir,P,OP,lin,pvec,parameter_names, ...
                validation_results,summary);
        end
        % 验证器写回同一工作点参数后模型工作区会被标记为Dirty；正式
        % 入口在保存唯一模型后关闭它，避免MATLAB退出时出现保存询问。
        if bdIsLoaded(buildInfo.model_name)
            save_system(buildInfo.model_name);
            close_system(buildInfo.model_name,0);
        end
    otherwise
        error(['未知action：%s。可选："构建"、"工作点"、' ...
            '"线性化"、"验证"。'],action);
end

fprintf('\nM0 5 MW action=%s\n',action);
fprintf('平衡残差 = %.3e, 最大极点实部 = %.6g 1/s\n', ...
    OP.max_normalized_residual,lin.max_real_part);
if isfield(result,'validation')
    fprintf('全部验收门结果 = %s\n',string(result.validation.overall_pass));
end
end

function summary = makeSummary(P,OP,lin,V)
% 从内存结果提取唯一一行、可直接用于论文工作记录的指标。
pos = find(imag(lin.eigenvalues)>0);
[~,q] = min(abs(lin.frequency_Hz(pos)-P.fshaft_openloop_Hz));
kshaft = pos(q);
metrics = V.small_signal_alignment.metrics;
summary = table( ...
    string(P.model_version),string(P.controller_profile), ...
    OP.P_MSC_dc_W/1e6,OP.P_GSC_dc_W/1e6,OP.P_PCC_W/1e6, ...
    OP.max_normalized_residual,V.power_angle.dP_dDelta_W_per_rad, ...
    lin.max_real_part,lin.frequency_Hz(kshaft),100*lin.damping_ratio(kshaft), ...
    V.no_disturbance.max_state_drift_pu, ...
    max([metrics.nrmse]),max([metrics.peak_error]), ...
    max([metrics.final_error]),logical(V.overall_pass), ...
    'VariableNames',{ ...
    'ModelVersion','ControllerProfile','P_MSC_MW','P_GSC_MW','P_PCC_MW', ...
    'MaxNormalizedResidual','dP_dDelta_W_per_rad','MaxPoleReal_per_s', ...
    'ShaftFrequency_Hz','ShaftDamping_pct','NoDisturbanceStateDrift_pu', ...
    'WorstNRMSE','WorstPeakError','WorstFinalError','OverallPASS'});
end

function saveFinalArtifacts(rootDir,P,OP,lin,pvec,parameter_names,V,summary)
% 仅保存可复现最终工作点的必要变量；不保存SimulationOutput和原始时序。
matFile = fullfile(rootDir,'M0_5MW_Aligned_Workpoint_and_SSM.mat');
csvFile = fullfile(rootDir,'M0_5MW_Alignment_Summary.csv');
reportFile = fullfile(rootDir,'M0_5MW_Alignment_Report_CN.md');

x_eq = OP.x0; %#ok<NASGU>
state_names = lin.state_names; %#ok<NASGU>
input_names = lin.input_names; %#ok<NASGU>
output_names = lin.output_names; %#ok<NASGU>
params = P; %#ok<NASGU>
operating_point = OP; %#ok<NASGU>
A = lin.A; B = lin.B; C = lin.C; D = lin.D; %#ok<NASGU>
eigenvalues = lin.eigenvalues; %#ok<NASGU>
validation_results = V; %#ok<NASGU>
save(matFile,'x_eq','state_names','input_names','output_names', ...
    'pvec','parameter_names','params','operating_point', ...
    'A','B','C','D','eigenvalues','validation_results','-v7.3');
writetable(summary,csvFile);
writeChineseReport(reportFile,P,OP,lin,V,summary);
end

function writeChineseReport(file,P,OP,lin,V,S)
fid = fopen(file,'w','n','UTF-8');
assert(fid>0,'无法写入报告：%s。',file);
c = onCleanup(@()fclose(fid)); %#ok<NASGU>
m = V.small_signal_alignment.metrics;
fprintf(fid,'# 5 MW构网型PMSG理想连续模型—小信号严格对齐报告\n\n');
fprintf(fid,'生成时间：%s\n\n',char(string(V.timestamp)));
fprintf(fid,'## 结论\n\n');
fprintf(fid,'总体结果：**%s**。非线性Simulink模型和小信号模型共用同一个连续RHS、参数向量和严格平衡点。\n\n', ...
    passText(V.overall_pass));
fprintf(fid,'## 模型边界\n\n');
fprintf(fid,'- 23个显式连续状态，Type-A MSC-DVC，真正VSG，相对功角，连续P/Q滤波，GSC电压/电流双环和LCL。\n');
fprintf(fid,'- 已删除PWM/SVPWM、采样调度、数字延迟、全部限幅与anti-windup、PLL/预同步、主动阻尼、MPPT/Pitch动态。\n');
fprintf(fid,'- GSC电流环/电压环带宽为 %.3g/%.3g Hz；电压环使用 +i_g 前馈，PCC电压前馈关闭。\n', ...
    P.gsc_current_bw_Hz,P.gsc_voltage_bw_Hz);
fprintf(fid,'- COI阻尼为 %.3g pu，按惯量比例施加到同一COI速度，只锚定公共转速，不进入相对轴系方程。\n\n',P.Dcoi_pu);
fprintf(fid,'## 严格5 MW工作点与能量关系\n\n');
fprintf(fid,'- P_MSC = %.9f MW；P_GSC = %.9f MW；P_PCC = %.9f MW。\n', ...
    OP.P_MSC_dc_W/1e6,OP.P_GSC_dc_W/1e6,OP.P_PCC_W/1e6);
fprintf(fid,'- Udc = %.6f V；delta = %.9f rad；平衡点最大归一化残差 = %.3e。\n', ...
    OP.x0(9),OP.delta_v_rad,OP.max_normalized_residual);
fprintf(fid,'- dP_PCC/ddelta = %.9g W/rad（正），物理VSG功率误差符号为 +1。\n\n', ...
    V.power_angle.dP_dDelta_W_per_rad);
fprintf(fid,'## 稳定性\n\n');
fprintf(fid,'- 最大极点实部 = %.9g 1/s。\n',lin.max_real_part);
fprintf(fid,'- 轴系模态频率 = %.9f Hz；阻尼比 = %.6f%%。\n\n', ...
    S.ShaftFrequency_Hz,S.ShaftDamping_pct);
fprintf(fid,'## 非线性—小信号对齐\n\n');
fprintf(fid,'扰动为 dPref = %.6g pu，比较时间 %.6g s。所有原始时序只在内存中使用。\n\n', ...
    V.small_signal_alignment.dPref_pu,V.small_signal_alignment.comparison_time_s);
fprintf(fid,'|信号|NRMSE|峰值误差|末值误差|通过|\n|---|---:|---:|---:|:---:|\n');
for k=1:numel(m)
    fprintf(fid,'|%s|%.6g|%.6g|%.6g|%s|\n',m(k).signal, ...
        m(k).nrmse,m(k).peak_error,m(k).final_error,passText(m(k).pass));
end
fprintf(fid,'\n无扰动最大状态漂移：%.3e pu。\n\n', ...
    V.no_disturbance.max_state_drift_pu);
fprintf(fid,'## 验收门\n\n');
gn = fieldnames(V.gates);
for k=1:numel(gn)
    fprintf(fid,'- `%s`：%s\n',gn{k},passText(V.gates.(gn{k})));
end
end

function s = passText(tf)
if tf
    s = 'PASS';
else
    s = 'FAIL';
end
end
