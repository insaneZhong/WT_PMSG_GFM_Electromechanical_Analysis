# EigenAnalysis 文件夹整理说明

本目录保留小信号模型的分析脚本、核心模型文件和参数文件；过程图表、扫描结果、阶段报告和新导出的模型文件已按用途归档。

## 根目录保留内容

- `*.m` / `*.mlx`：可直接运行或继续修改的分析程序。
- `Parameters.m` / `Parameters.mat`：当前小信号分析共用参数。
- `Unified_WT_PMSG_GFL.mat`、`Unified_WT_PMSG_GFM_GWT.mat`、`Unified_WT_PMSG_VSG.mat`、`Unified_WT_PMSG_VSG_Damping.mat`：四拓扑对照分析的核心模型文件。
- `Unified_VSG.mat`：原始 VSG 模型文件。

## Generated_Models

存放由 Generate_SSM 阶段新导出的扩展模型：

- `Unified_WT_PMSG_VSG_TypeA.mat`
- `Unified_WT_PMSG_VSG_TypeC.mat`
- `Unified_WT_PMSG_VSG_TypeC_Damping.mat`

相关脚本已改为优先从 `Generated_Models` 加载；如果该目录不存在对应文件，才回退到 `EigenAnalysis` 根目录查找。

## Results

存放运行分析程序后生成的图、表和中间结果：

- `Control_Mode_Comparison_Results`：GFL-WT / GFM-GWT / GFM-MWT / GFM-MWT+AD 四拓扑对照。
- `DVC_Type_Comparison_Results`：Type-a / Type-c / Type-c+AD 对照。
- `Control_Parameter_Scan_Results`：控制参数扫描结果。
- `Mode_Trajectory_Results`：模态随参数变化的轨迹图和表。
- `Small_Disturbance_Response_Results`：小扰动时域响应和频谱图。

后续重新运行脚本时，结果会继续写入这些子文件夹。

## Reports

存放阶段性中文说明、论文实验总结和图表索引：

- `Phase1_1MW_Baseline_Summary_20260611`：阶段 1 基准整理文档。
- `Control_Parameter_Scan_Analysis_CN.md`：控制参数扫描解释。
- `Four_Topology_Causal_Analysis_CN.md`：四拓扑因果对照解释。

## 运行建议

1. 修改物理参数或控制参数时，先改 `Parameters.m`。
2. 重新生成模型时，先到 `Generate_SSM` 运行对应导出脚本。
3. 新生成的 Type-a / Type-c 模型建议复制或移动到 `EigenAnalysis\Generated_Models`。
4. 小信号结果图优先运行：
   - `Compare_Control_Mode_Run.m`
   - `Compare_DVC_TypeA_TypeC_Run.m`
   - `Scan_GFM_Control_Parameters_Run.m`
   - `Track_GFM_Mode_Trajectories.m`
   - `Plot_Small_Disturbance_Responses.m`

本次整理没有删除文件，只进行了明确文件和文件夹移动。
