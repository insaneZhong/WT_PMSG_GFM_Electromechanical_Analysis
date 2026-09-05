function write_multimode_disturbance_report(file,R)
%WRITE_MULTIMODE_DISTURBANCE_REPORT_CN 只写摘要和可复现实验结论，不写完整时序。
fid=fopen(file,'w','n','UTF-8'); c=onCleanup(@()fclose(fid));
fprintf(fid,'# GFM网侧扰动多模态传递机理与参数规律分析\n\n');
fprintf(fid,'## 1. 基准模型和公平性\n\n本报告仅使用已对齐的5 MW理想连续平均模型及其同源23状态SSM；不包含EMT、PWM、采样、离散PI、延迟、限幅、LVRT、启动或故障逻辑。三架构使用同一个Gate A参数源和共同工作点。\n\n');
fprintf(fid,'## 2. Grid-frequency输入公平性审计\n\n');
writeTable(fid,R.frequency_input_audit);
fprintf(fid,'\n**Gate A：PASS。** 三架构外部输入均为同一个物理量 \\Delta\\omega_{grid}（rad/s）。GFL表示理想PLL直接跟随；GFM-GWT/MWT通过唯一的相对功角方程 \\dot\\delta=\\omega_{ctrl}-\\omega_{grid} 接收该输入。不存在Hz/rad/s混用或重复积分。\n\n');
fprintf(fid,'## 3. GFL/GFM-GWT/GFM-MWT逐级传递差异\n\n'); writeTable(fid,R.first_divergence); fprintf(fid,'\n在共同参考频率2.4942 Hz处，三种架构相对GFL的逐级传递函数已保存在 `GridToShaft_TransferChain_Summary.csv`。非GFL架构的首次显著偏离节点由表中 `First_Divergence_Node` 标识。\n\n');
fprintf(fid,'## 4. GFM-GWT与GFM-MWT关键路径差异\n\n'); writeTable(fid,R.gwt_mwt_path); fprintf(fid,'\n若 `iq_MSC_ref`、`iq_MSC` 与 `T_e` 行显示GWT近零而MWT非零，则数据支持以下有限结论：网侧扰动在GWT中未通过MSC转矩通道传递，而MWT因MSC-DVC保留了 `Udc → iq*_MSC → Te` 路径。\n\n');
fprintf(fid,'## 5. 全模态贡献排序\n\n每个输入—输出组合保留按阶跃贡献度排序的Top 10模态，见 `Full_Modal_Contribution_Ranking.csv`。机械TOR、DC-link/DVC、SYNC和GSC分类来自归一化左右特征向量参与因子。\n\n');
fprintf(fid,'## 6. 最小多模态重构\n\n'); writeTable(fid,R.minimal_dominant); fprintf(fid,'\n`NO_EXCITATION` 是GFM-GWT的物理路径结果，不被作为数值错误删除。其它组合均以 NRMSE<5%% 且相关系数>0.98 为通过条件。\n\n');
fprintf(fid,'## 7. 模态相位与叠加机理\n\n`Modal_Superposition_At_FirstPeak.csv` 给出第一主峰处每个最小模态的同相（CONSTRUCTIVE）或反相（DESTRUCTIVE）贡献；因此响应峰值不能仅归因于单一torsional pole。\n\n');
fprintf(fid,'## 8. Grid-frequency与grid-angle差异来源\n\n`Frequency_vs_Angle_Excitation_Explanation.csv` 输出每个主导模态的 \\|w^HB_f\\| 和 \\|w^HB_\\theta\\|。两者输入单位不同，不能只凭(1/s)或未归一化投影的绝对数值判断强弱；必须结合逐级传递函数、DC-link/DVC和输出残差。\n\n');
fprintf(fid,'## 9. SCR影响\n\n'); writeTable(fid,R.scr_scan); fprintf(fid,'\nSCR扫描同时记录轴系极点与残差比。若 `DISTURBANCE_PATH_RESHAPING_WITHOUT_POLE_SHIFT` 出现，则只在该点说明残差改变比轴系阻尼改变更显著；其它点不得泛化。\n\n');
fprintf(fid,'## 10. H影响\n\n'); writeTable(fid,R.H_scan); fprintf(fid,'\nH扫描只改变VSG惯量，不重整定其它环节；结果用于识别SYNC残差和轴系频率输入残差的变化。\n\n');
fprintf(fid,'## 11. DVC带宽影响\n\n'); writeTable(fid,R.DVC_scan); fprintf(fid,'\nDVC扫描等比例缩放MSC-DVC PI，输出Udc→iq*→Te传递摘要和轴系残差，未补偿其它控制器。\n\n');
fprintf(fid,'## 12. 理想非线性模型验证\n\n');
if isempty(R.representative_validation), fprintf(fid,'本次未运行代表性非线性验证。\n\n'); else
    VT=validationTable(R.representative_validation); writeTable(fid,VT); fprintf(fid,'\n验证采用0.005 Hz小网侧频率阶跃以保持在线性区。完整SSM与理想连续非线性模型的误差、以及最小多模态重构相对非线性的误差均列于上表。\n\n');
end
fprintf(fid,'## 13. 当前结论与限制\n\n本阶段的结论必须限定于当前理想连续工作点：GFM对轴系时域响应的影响可同时来自扰动通道投影、DC-link/DVC跨变流器传递和多模态相位叠加；不能只凭响应峰值声称轴系阻尼必然恶化。SCR、H和DVC扫描用于区分极点变化与残差变化。上述结论尚未外推至离散控制或开关EMT模型。\n\n');
fprintf(fid,'## 图件\n\n- `Figures_Multimode_Mechanism/Fig20_GridToShaft_Transfer_Chain.png`：逐级传递比。\n- `Figures_Multimode_Mechanism/Fig21_Minimal_Multimode_Reconstruction.png`：最小重构。\n- `Figures_Multimode_Mechanism/Fig22_ParameterScan_and_NonlinearValidation.png`：参数规律与理想连续非线性验证。\n');
end

function T=validationTable(V)
T=table('Size',[numel(V) 10], ...
 'VariableTypes',{'string','string','string','double','double','double','double','double','double','double'}, ...
 'VariableNames',{'Case','Mode','Output','f_tor_Hz','FullSSM_NRMSE','FullSSM_Correlation','FullSSM_PeakError_pct','Minimal_NRMSE','Minimal_Correlation','Minimal_PeakError_pct'});
for k=1:numel(V)
    T.Case(k)=string(V(k).case); T.Mode(k)=string(V(k).mode); T.Output(k)=string(V(k).output); T.f_tor_Hz(k)=V(k).f_tor;
    T.FullSSM_NRMSE(k)=V(k).nrmse_full; T.FullSSM_Correlation(k)=V(k).corr_full; T.FullSSM_PeakError_pct(k)=V(k).peak_full;
    T.Minimal_NRMSE(k)=V(k).nrmse_min; T.Minimal_Correlation(k)=V(k).corr_min; T.Minimal_PeakError_pct(k)=V(k).peak_min;
end
end
function writeTable(fid,T)
v=T.Properties.VariableNames; fprintf(fid,'|'); for k=1:numel(v), fprintf(fid,'%s|',v{k}); end; fprintf(fid,'\n|'); for k=1:numel(v), fprintf(fid,'---|'); end; fprintf(fid,'\n');
for r=1:height(T)
    fprintf(fid,'|');
    for k=1:numel(v)
        x=T{r,k}; if iscell(x), x=x{1}; end
        if isstring(x)||ischar(x), s=char(string(x)); else, s=num2str(x,8); end
        fprintf(fid,'%s|',s);
    end
    fprintf(fid,'\n');
end
end
