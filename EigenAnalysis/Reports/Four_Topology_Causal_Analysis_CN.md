# 构网型风电机组机电耦合四拓扑因果对照分析

## 对照设计

| 模型 | MSC 控制 | GSC 控制 | 单因素解释目标 |
| --- | --- | --- | --- |
| GFL-WT | MPPT | DVC + PLL | 跟网型参考基线 |
| GFM-GWT | MPPT | DVC + GFM | 在机侧仍为 MPPT 时，仅考察同步方式由 PLL 变为 GFM |
| GFM-MWT | DVC | GFM | 在 GFM 条件下，考察直流电压控制由网侧迁移到机侧的影响 |
| GFM-MWT+AD | DVC | GFM + APCAD | 在耦合结构不变时，考察附加阻尼控制的改善效果 |

`GFM-GWT` 中，GSC-DVC 不用 PLL 电流源结构替代 GFM，而是将直流电压误差生成的有功修正量 `p_dc` 叠加到 VSG 有功参考。因此该模型保持网侧电压源成网特性，同时承担直流链能量平衡。

## 建模与校准

四个模型均保留两质量块传动链、PMSG、电容直流链、LCL 滤波器和电网模型。所跟踪的机电扭振模态由 `theta_tw`、`omega_g` 和 `omega_t` 主导，频率约为 `2 Hz`。

初始设置为 `GFM-GWT` 的 GSC-DVC 采用 `5 Hz` 带宽时，会引入非扭振主导失稳极点，不能用于归因。通过 `Tune_GFM_GWT_DVC.mlx` 扫描后，将暂定带宽设为 `0.1 Hz`；在基准工况下，GFM-GWT 总体稳定且不会人为改变 2 Hz 扭振阻尼。

## 基准工况结果

| 模型 | 扭振频率 (Hz) | 扭振实部 sigma (1/s) | 阻尼比 zeta | 最大实部 (1/s) | 总体稳定 |
| --- | ---: | ---: | ---: | ---: | --- |
| GFL-WT | 1.99727 | -0.15303 | 0.01219 | -0.01013 | 是 |
| GFM-GWT | 1.99727 | -0.15303 | 0.01219 | -0.01013 | 是 |
| GFM-MWT | 1.99729 | -0.09001 | 0.00717 | +0.00507 | 否 |
| GFM-MWT+AD | 1.98597 | -0.19496 | 0.01562 | +0.00509 | 否 |

## 因果解释

1. `GFL-WT -> GFM-GWT`：扭振阻尼变化约为零。当前状态空间结果不支持“只要把 PLL 改为 GFM，就必然增加 2 Hz 机电耦合振荡”的论断。

2. `GFM-GWT -> GFM-MWT`：阻尼比降低 `0.00502`，扭振极点实部右移 `0.06302 1/s`。这一步唯一关键结构变化是 `MSC-MPPT/GSC-DVC` 变为 `MSC-DVC`，因此可归因于直流电压扰动通过机侧电流控制进入 PMSG 电磁转矩，再与两质量块轴系形成闭环耦合。

3. `GFM-MWT -> GFM-MWT+AD`：阻尼比增加 `0.00845`，扭振极点实部左移 `0.10495 1/s`。参与因子中 `x_bp1` 等 APCAD 状态进入扭振模态，说明附加控制确实通过目标频带反馈为轴系振荡提供电气阻尼。

## 扫描结果解读

在 `SCR`、`X/R` 和 `C_dc` 扫描中，`GFM-GWT -> GFM-MWT` 的扭振阻尼下降持续存在；`GFM-MWT -> GFM-MWT+AD` 的扭振阻尼改善也持续存在，证明上述因果结论并非单一基准点偶然结果。

需要严格区分扭振模态稳定性与全系统稳定性：

- `GFM-MWT` 与 `GFM-MWT+AD` 当前仍存在约 `+0.005 1/s` 的非目标慢不稳定模态。APCAD 已改善 2 Hz 扭振，但不能据此宣称全系统稳定。
- `GFL-WT` 在极弱网 `SCR=1.25` 附近存在非扭振高频不稳定；`GFM-GWT` 在极端 `SCR`、低 `X/R` 或较小 `C_dc` 下也出现非扭振失稳。这些现象需要独立的电气环/直流环参数稳定域分析。

## 论文表述建议

在所建立的 WT-PMSG 两质量块小信号模型下，单独将网侧同步控制由 PLL 改为 GFM 并未显著改变约 2 Hz 的轴系扭振模态。当直流电压调节由网侧移至机侧后，直流链能量扰动可经机侧电流环和 PMSG 电磁转矩直接作用于传动链，使扭振模态极点右移且阻尼下降。针对该耦合通道设计的 APCAD 在目标频带内提供附加电气阻尼，使扭振极点明显左移。因此，本文所观察到的机电耦合增强来源于成网控制架构下的能量分配与机侧直流电压闭环，而不能简单归结为 GFM 同步机制本身。

## 对应文件

- 模型：`../Generate_SSM/WT_PMSG_GFM_GWT_Model.mlx`
- 四拓扑比较：`Compare_Control_Mode_Four_Topologies.mlx`
- GFM-GWT 直流环筛选：`Tune_GFM_GWT_DVC.mlx`
- 基准结果：`Results/Control_Mode_Comparison_Results/baseline_torsional_modes.csv`
- 因果增量：`Results/Control_Mode_Comparison_Results/causal_baseline_deltas.csv`
- 工况扫描：`Results/Control_Mode_Comparison_Results/common_condition_sweeps.csv`
