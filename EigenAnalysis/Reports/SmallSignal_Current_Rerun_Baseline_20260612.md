# 构网型风电机组小信号当前重跑基准（2026-06-12）

本文件记录当前工作树重新运行 `EigenAnalysis/Compare_Control_Mode_Run.m` 后得到的四拓扑小信号基准结果。该结果应作为后续非线性对齐和论文阶段性汇报的当前依据。

## 1. 本次重跑入口

运行脚本：

```matlab
cd('D:/博士工作/论文工作/（1）小信号模型/WT_PMSG_GFM_Electromechanical_Validation/EigenAnalysis')
Compare_Control_Mode_Run
```

主要输出：

- `EigenAnalysis/Results/Control_Mode_Comparison_Results/baseline_torsional_modes.csv`
- `EigenAnalysis/Results/Control_Mode_Comparison_Results/baseline_torsional_participation_top10.csv`
- `EigenAnalysis/Results/Control_Mode_Comparison_Results/causal_baseline_deltas.csv`
- `EigenAnalysis/Results/Control_Mode_Comparison_Results/common_condition_sweeps.csv`
- `EigenAnalysis/Results/Control_Mode_Comparison_Results/control_mode_comparison_results.mat`

## 2. 当前四拓扑扭振模态结果

| 模型 | 频率/Hz | 阻尼比 | Sigma | MaxReal | 全系统稳定 |
|---|---:|---:|---:|---:|---|
| GFL-WT | 1.9976 | 0.04944 | -0.62131 | 1373.08 | 否 |
| GFM-GWT | 1.9976 | 0.04944 | -0.62131 | 1298.83 | 否 |
| GFM-MWT | 2.0011 | -0.01374 | 0.17274 | 1298.83 | 否 |
| GFM-MWT+AD | 1.9534 | 0.01179 | -0.14472 | 1298.83 | 否 |

注意：这里的 `Stable=false` 指全系统仍存在高频正实部模态；论文中目前只能围绕低频轴系扭振模态讨论，不应宣称全系统小信号稳定。

## 3. 当前可支撑的因果解释

1. `GFL-WT -> GFM-GWT`：仅将同步方式从 PLL/GFL 改为 GFM，并未改变该 2 Hz 附近轴系模态的阻尼，二者阻尼比均为 `0.04944`。

2. `GFM-GWT -> GFM-MWT`：将 DC 电压控制从网侧转移到机侧后，扭振模态阻尼从 `0.04944` 降为 `-0.01374`，实部从 `-0.62131` 变为 `+0.17274`。这说明机侧 DC 控制会让发电机电磁转矩更直接参与 DC 能量平衡，从而增强轴系机械状态、机侧功率和直流电压之间的反馈耦合。

3. `GFM-MWT -> GFM-MWT+AD`：加入 APCAD 后，扭振阻尼从 `-0.01374` 提升到 `0.01179`，实部从 `+0.17274` 左移至 `-0.14472`。这说明附加功率阻尼环节对当前低频轴系模态具有改善作用。

## 4. 参与因子判断

当前 2 Hz 附近模态的主要参与状态仍集中在：

- `theta_tw`
- `omega_g`
- `omega_t`

在 `GFM-MWT` 中，`delta/w/v_dc/gamma_dc` 等电气与控制状态也开始参与该模态。这一点与机电耦合机理一致：轴系状态不再只是机械自由振荡，而是通过机侧 DC 控制和构网功率同步环节进入电气控制反馈。

## 5. DC 参数口径对齐检查

新增检查脚本：

```matlab
Check_DC_Link_Alignment_With_Nonlinear
```

输出：

- `EigenAnalysis/Reports/DC_Link_Alignment_With_Nonlinear_20260612.md`
- `EigenAnalysis/Reports/DC_Link_Alignment_With_Nonlinear_20260612.csv`

当前发现：

| 参数 | 小信号 | 非线性当前对象 | 状态 |
|---|---:|---:|---|
| `Vdc` | 1500 V | 1200 V 物理初始电压 | 不一致 |
| `C_dc` | 1.5e-3 F | 0.03 F | 不一致 |
| `L_d/L_q` | 1.05e-3 H | 1.02e-3 H | 小差异 |
| `R_s/n_p/psi_f/J_g` | 与非线性一致 | 与小信号一致 | 一致 |

因此，当前小信号基准可以作为“可复现分析基准”，但还不能称为与非线性模型完全同对象对齐。

## 6. DC 参数场景对比

新增脚本：

```matlab
Compare_DC_Link_Parameter_Scenarios
```

输出：

- `EigenAnalysis/Results/DC_Link_Parameter_Scenarios/dc_link_scenario_torsional_modes.csv`
- `EigenAnalysis/Results/DC_Link_Parameter_Scenarios/DC_Link_Parameter_Scenario_Comparison_20260612.md`

关键结果：

| 场景 | Vdc/V | Cdc/F | GFM-MWT 阻尼比 | GFM-MWT Sigma | GFM-MWT+AD 阻尼比 | GFM-MWT+AD Sigma |
|---|---:|---:|---:|---:|---:|---:|
| FrozenSmallSignal | 1500 | 0.0015 | -0.01374 | 0.17274 | 0.01179 | -0.14472 |
| NonlinearPhysicalInit | 1200 | 0.03 | -0.01732 | 0.21764 | 0.01183 | -0.14463 |
| NonlinearValidationRef | 1000 | 0.03 | -0.01661 | 0.20876 | 0.01183 | -0.14473 |

结论：将 DC 参数改为更接近非线性对象后，`GFM-MWT` 仍表现为负阻尼，`GFM-MWT+AD` 仍恢复正阻尼。因此，“机侧 DC 控制增强机电耦合、附加阻尼可改善该模态”的定性结论没有被 DC 参数口径改变推翻。

## 7. 后续使用原则

1. 当前重跑结果优先于 2026-06-11 旧清单中的四拓扑数值。
2. 后续论文图表应从 `Control_Mode_Comparison_Results` 当前输出重新生成。
3. 非线性模型完成严格无扰动稳态前，不应宣称已经完成小信号-非线性闭环验证。
4. 如果最终决定以非线性模型 `Vdc=1000 V, Cdc=0.03 F` 为主对象，应重新生成一套正式小信号基准，并把旧 `FrozenSmallSignal` 作为历史基准或敏感性对照。
