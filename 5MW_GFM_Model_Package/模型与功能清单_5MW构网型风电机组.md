---
title: 5 MW构网型PMSG模型与功能清单
type: model-inventory
status: active
created: 2026-09-05
updated: 2026-09-05
project: 构网型风机机电耦合振荡
evidence_status: source-audited-inventory
tags:
  - gfm-wind
  - pmsg
  - two-mass
  - model-inventory
---

# 5 MW构网型PMSG模型与功能清单

> [!important] 当前主线
> 当前机电耦合、模态、阻尼和扰动通道研究统一使用 **M0理想连续平均模型**及其同源小信号模型。其他模型是对照、反事实检验或工程实现验证，不应直接替代M0作为论文主线。

## 一、当前模型总览

| 模型层级 | 模型/文件 | 主要组成 | 主要用途 | 当前地位 |
| --- | --- | --- | --- | --- |
| M0理想连续平均 | `5MW_Ideal/CurrentModel_Idealized/Grid_Forming_PMSG5MW_TwoMass_Idealized.slx` | 两质量轴系、PMSG、连续平均MSC/GSC控制、DC-link、LCL/电网等连续动态 | 机电耦合、轴系模态、复转矩、Pole/Path/Directional Coupling | **论文主模型** |
| M0同源SSM | `5MW_Ideal/M0_5MW_Aligned_Workpoint_and_SSM.mat` | M0平衡点、状态映射、Jacobian、特征值和参与因子 | 与M0非线性模型对齐，计算模态、阻尼和输入投影 | **论文主线小信号模型** |
| M1物理平均VSC | `5MW_Ideal/M1_Physical_Averaged_VSC/Grid_Forming_PMSG5MW_TwoMass_M1_PhysicalAvg.slx` | 显式体现 `v_conv=(Udc/2)m` 的连续平均变流器 | 检查M0中的结构零/方向性是否由理想交流电压源假设造成 | 机制稳健性验证，在研 |
| GFL对照 | `GFM_1MW _nonlinear/Grid_Following_PMSG5MW_Liu2024_TwoMass.slx` | 两质量PMSG、传统PLL跟网控制 | 与GFM比较轴系阻尼、扰动激励和传播方向 | 对照模型 |
| GFM原始非线性 | `GFM_1MW _nonlinear/Grid_Forming_PMSG5MW_Liu2024_TwoMass.slx` | 原始Legacy/C-S-Function构网控制 | 工程控制器和非线性行为检查 | 辅助；未替代M0 |
| 刚性轴对照 | `GFM_1MW _nonlinear/Grid_Forming_PMSG5MW_Liu2024_RigidShaft.slx` | GFM + 刚性传动链 | 判断振荡是否来自两质量轴系 | 对照模型 |
| MPPT变体 | `GFM_1MW _nonlinear/Grid_Forming_PMSG5MW_Liu2024_TwoMass_MPPT.slx` | 两质量、气动/MPPT和原始非线性控制 | 工况变化和工程真实性补充 | 辅助模型 |
| MPPT+Pitch变体 | `GFM_1MW _nonlinear/Grid_Forming_PMSG5MW_Liu2024_TwoMass_MPPT_Pitch.slx` | MPPT、Pitch执行器和两质量轴系 | 额定以上、风速变化和桨距工况 | 辅助模型；原始Pitch边界仍需审计 |
| S7 Legacy平均 | `5MW_Ideal/M3_Workpoint_Generalization/S7_Legacy_Average_Plant.slx` | Legacy控制器的交流平均接口 | 控制器遗留实现的冷启动/物理闭环认证 | S7验证副本；当前未通过整机闭环认证 |
| S7 Legacy热启动 | `5MW_Ideal/M3_Workpoint_Generalization/S7_Legacy_HotStart_Average_Plant.slx` | Legacy控制器状态注入和热启动 | 热启动恢复与状态连续性调试 | 调试副本；不用于主线结论 |
| S7A同源离散平均 | `5MW_Ideal/M3_Workpoint_Generalization/S7A_DiscreteAvg_5MW.slx` | M0同源参考数字实现、采样/ZOH候选 | 筛查离散化对模态和路径的影响 | 参考数字实现，不等同真实Legacy C |
| EMT/热启动副本 | `GFM_1MW _nonlinear/Grid_Forming_PMSG5MW_Liu2024_TwoMass_EMT_*`、`HotEMT*`、`HotStable*` | PWM/开关器件、采样、延迟、限幅和热启动变体 | 高保真工程验证 | 辅助/历史；未纳入当前理想连续主线 |

## 二、M0主线的控制架构配置

GFL、GFM-GWT和GFM-MWT在当前研究中主要是**同一物理对象上的控制职责配置**，不应误认为三套完全独立的主模型文件。

| 配置 | 控制职责 | 研究问题 |
| --- | --- | --- |
| GFL | PLL同步 + 跟网电流控制 | 传统跟网基准的轴系阻尼和扰动激励 |
| GFM-GWT | 网侧构网/同步，网侧承担直流能量调节，机侧局部MPPT | `Udc -> iq*_MSC` 是否不可达；网侧扰动能否进入轴系 |
| GFM-MWT | 网侧VSG构网，机侧MSC-DVC调节DC-link | `Udc -> iq*_MSC -> Te` 反馈是否改变轴系电气阻尼 |

这些配置由M0/M1的参数化方程、同源SSM和机制分析脚本切换；目前不为每种配置永久复制一个新的主模型文件。

## 三、M2与分析文件不是独立仿真模型

`5MW_Ideal/M2_Electromechanical_Mechanism` 主要包含：

- 复转矩、特征值、参与因子和模态跟踪程序；
- Pole Shaping、Path Shaping和Directional Coupling分析；
- GFL/GWT/MWT的扰动路径、模态残差和参数扫描。

因此M2是**分析层**，不是新的物理plant。分析结果必须回溯到M0/M1模型和对应工作点。

## 四、模型的推荐使用边界

| 研究任务 | 首选模型 | 不应直接使用 |
| --- | --- | --- |
| 轴系固有频率、阻尼和复转矩 | M0 + 同源SSM | EMT、Pitch或旧1 MW模型 |
| GFL/GWT/MWT公平机理对照 | M0参数化配置 + 同源SSM | 启动过程不同的Legacy副本 |
| 检查 `Udc` 到交流端口的可达性 | M1 | 只根据M0理想电压源推断 |
| 工程控制器行为 | Legacy非线性模型 | 把Legacy结果当成M0机理证据 |
| MPPT/Pitch和额定以上工况 | MPPT/Pitch变体 | 用固定运行点结果代替全工况结论 |
| 真实数字/开关实现 | S7A、S7 Legacy和EMT（按Gate开放情况） | 未通过Gate的热启动或EMT副本 |

## 五、当前验证状态与结论边界

- M0理想连续模型与同源SSM已完成当前工作点对齐；代表性轴系模态约为 `2.494 Hz`，阻尼约为 `2.98%`。这些数值只适用于当前模型、工作点和参数域。
- M1用于检验理想平均VSC假设的稳健性，不能自动把M0的“近似零通道”升级为普遍结构零。
- 当前证据不支持“GFM必然恶化轴系稳定性”。应分别报告轴系极点、全系统极点、扰动输入投影和模态残差。
- S7/S7A/EMT属于实现保真度验证路线；Legacy物理闭环和EMT验证尚未替代M0主线。
- 旧1 MW、1.5 MVA和历史小信号包只用于代码、参数和方法参考，不作为当前5 MW论文结论。

## 六、推荐运行入口

### 主线机电耦合研究

1. 打开 `Grid_Forming_PMSG5MW_TwoMass_Idealized.slx`。
2. 使用同目录初始化/平衡点脚本，例如 `initialize_currentmodel_continuous_controller.m` 和 `solve_currentmodel_source_aligned_equilibrium.m`。
3. 使用 `M0_5MW_Aligned_Workpoint_and_SSM.mat` 或 `main_m0_5mw.m` 进行同源小信号分析。

### M1机制稳健性

使用 `M1_Physical_Averaged_VSC/Grid_Forming_PMSG5MW_TwoMass_M1_PhysicalAvg.slx` 及其 `run_m1_physicalavg_validation.m`；结果单独标注为M1，不覆盖M0。

### Legacy/S7验证

使用 `M3_Workpoint_Generalization` 下对应的S7脚本和模型；只有满足相应Gate后，才可把结果写入高保真实现结论。

## 七、文件治理原则

- M0主模型和同源SSM只维护一套；
- 参数扫描保存摘要，不为每个参数永久复制模型；
- 历史EMT、热启动和失败候选保留，但明确标记为辅助或历史；
- 新实验优先写入临时结果目录，只有通过验收的摘要和图进入长期结果目录。

