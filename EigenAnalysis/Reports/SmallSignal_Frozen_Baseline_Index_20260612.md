# 构网型风电机组小信号冻结基准索引（2026-06-12）

本文档用于冻结当前小信号阶段的“可复现实验基准”。冻结的含义是：当前脚本、参数口径、结果文件和可引用结论已经明确；但它不表示所有风机物理参数均已最终文献化，也不表示非线性 Simulink 模型已经完成稳态验证。

## 1. 当前应引用的结果版本

当前四拓扑主结果应以 2026-06-12 重跑结果为准：

- 主报告：`EigenAnalysis/Reports/SmallSignal_Current_Rerun_Baseline_20260612.md`
- 四拓扑结果：`EigenAnalysis/Results/Control_Mode_Comparison_Results/baseline_torsional_modes.csv`
- 参与因子：`EigenAnalysis/Results/Control_Mode_Comparison_Results/baseline_torsional_participation_top10.csv`
- 因果差分：`EigenAnalysis/Results/Control_Mode_Comparison_Results/causal_baseline_deltas.csv`
- DC 参数对齐检查：`EigenAnalysis/Reports/DC_Link_Alignment_With_Nonlinear_20260612.md`
- DC 参数场景对比：`EigenAnalysis/Results/DC_Link_Parameter_Scenarios/DC_Link_Parameter_Scenario_Comparison_20260612.md`

`SmallSignal_Reproducible_Baseline_Manifest_20260611.md` 和 `Frozen_1MW_SmallSignal_Baseline_20260611.md` 保留为历史冻结记录。后续写论文或汇报时，低频轴系模态数值优先采用 2026-06-12 当前重跑结果。

## 2. 四个小信号模型

| 模型 | 机侧控制 | 网侧控制 | 研究作用 |
|---|---|---|---|
| GFL-WT | MSC-MPPT | GSC-DVC + PLL | 传统跟网型风机基准 |
| GFM-GWT | MSC-MPPT | GSC-DVC + GFM | 只替换网侧同步方式，隔离 PLL/GFM 差异 |
| GFM-MWT | MSC-DVC | GSC-GFM | 将 DC 电压控制转移到机侧，观察机电耦合增强风险 |
| GFM-MWT+AD | MSC-DVC | GSC-GFM + APCAD | 在 GFM-MWT 基础上加入附加功率阻尼 |

当前四拓扑结果回答的问题不是“所有 GFM 风机都会振荡”，而是：当 DC 电压控制放在机侧后，发电机电磁转矩、DC 能量平衡和两质量块轴系之间是否形成更强反馈，从而降低低频轴系模态阻尼。

## 3. 当前四拓扑低频轴系模态结果

| 模型 | 频率/Hz | 阻尼比 | Sigma | 说明 |
|---|---:|---:|---:|---|
| GFL-WT | 1.9976 | 0.04944 | -0.62131 | 低频轴系模态阻尼较高 |
| GFM-GWT | 1.9976 | 0.04944 | -0.62131 | 与 GFL-WT 几乎相同，说明仅换 GFM 同步不是主要原因 |
| GFM-MWT | 2.0011 | -0.01374 | 0.17274 | 机侧 DC 控制后轴系模态变为负阻尼 |
| GFM-MWT+AD | 1.9534 | 0.01179 | -0.14472 | APCAD 使轴系模态回到正阻尼 |

注意：`Stable=false` 来自全系统中仍存在高频正实部模态。因此当前可引用的结论是“低频轴系模态的阻尼变化规律”，不能写成“全系统小信号稳定性已经完全成立”。

## 4. 因果差分结论

当前 `causal_baseline_deltas.csv` 支持以下判断：

- `GFL-WT -> GFM-GWT`：阻尼比变化约为 `2.19e-15`，基本为零。说明只把 PLL/GFL 同步替换为 GFM 同步，不会显著改变当前 2 Hz 附近轴系模态。
- `GFM-GWT -> GFM-MWT`：阻尼比变化为 `-0.06318`，Sigma 右移 `+0.79406`。说明 DC 控制从网侧转移到机侧后，机侧转矩控制和 DC 能量平衡进入轴系反馈通道，使机电耦合增强。
- `GFM-MWT -> GFM-MWT+AD`：阻尼比增加 `+0.02553`，Sigma 左移 `-0.31746`。说明 APCAD 对该低频轴系模态有定向阻尼改善作用。

## 5. Type-a / Type-c 结果

当前 Type-a / Type-c 对比结果来自：

- `EigenAnalysis/Results/DVC_Type_Comparison_Results/baseline_torsional_modes.csv`

| 模型 | 频率/Hz | 阻尼比 | Sigma | 稳定 |
|---|---:|---:|---:|---|
| GFM-MWT-TypeA | 2.0011 | -0.01374 | 0.17275 | 否 |
| GFM-MWT-TypeC | 2.0011 | -0.01370 | 0.17226 | 否 |
| GFM-MWT-TypeC+AD | 1.9538 | 0.01160 | -0.14236 | 是 |

当前结果说明：Type-c 相比 Type-a 对低频轴系阻尼只有很小改善，真正明显改变轴系模态的是附加阻尼环节。Type-a / Type-c 的价值主要在于明确 DC 控制和功率前馈路径的位置，为后续阻尼环节放置提供结构依据。

## 6. 2 Hz 模态来源

当前 2 Hz 左右模态来自两质量块轴系参数，不是非线性实验中预设的滤波目标。核心关系为：

```text
J_eq = J_t * J_g / (J_t + J_g)
f_sh = 1/(2*pi) * sqrt(K_sh / J_eq)
zeta_sh = D_sh / (2*sqrt(K_sh*J_eq))
```

当前参数口径：

| 参数 | 当前值 | 来源属性 |
|---|---:|---|
| `J_g` | `1.8375e5 kg*m^2` | 与非线性 PMSG/机械侧设置对齐 |
| `J_t` | `8*J_g = 1.47e6 kg*m^2` | 当前 1 MW 基准假设 |
| `K_sh` | `2.5793e7 N*m/rad` | 由 `f_sh_init_guess=2 Hz` 和 `J_eq` 反推 |
| `D_sh` | `4.1050e4 N*m*s/rad` | 由 `zeta_sh_init_guess=0.01` 反推 |

因此，当前 2 Hz 是轴系等效参数给出的模态频率。后续若采用 2 MW/5 MW 文献参数或厂家参数，应由新的 `J_t/J_g/K_sh/D_sh` 自然计算模态频率，不能在非线性实验中强行固定为 2 Hz。

## 7. 参数来源边界

当前参数分为四类：

| 类型 | 含义 | 当前代表参数 |
|---|---|---|
| 已对齐参数 | 与当前非线性模型或用户设定一致 | `S_base=1e6`、`V_LL=690 V`、`omega_g0=pi rad/s`、`R_s`、`n_p`、`psi_f`、`J_g` |
| 推算参数 | 由运行点或线性化关系计算 | `T_e0`、`theta_tw0`、`J_eq`、部分控制器基准增益 |
| 设计参数 | 为控制设计或扫描设置 | `h`、`mp`、`k_pdc/k_idc`、`k_pq/k_iq`、`K_damp` |
| 暂定参数 | 为 1 MW 基准跑通而设，后续需文献化 | `J_t=8*J_g`、由 2 Hz 反推的 `K_sh`、由 1% 阻尼反推的 `D_sh` |

写论文时应明确：当前小信号基准可用于机制分析和方法验证，但轴系刚度、阻尼和风轮惯量仍需在最终稿中替换为明确文献参数或厂家参数，并重新生成结果。

## 8. 与非线性模型的 DC 参数差异

当前 DC 对齐检查显示：

| 参数 | 小信号冻结默认 | 非线性当前对象 | 状态 |
|---|---:|---:|---|
| `Vdc` | `1500 V` | `1200 V` 物理初值 / `1000 V` 当前验证参考 | 不一致 |
| `C_dc` | `1.5e-3 F` | `0.03 F` | 不一致 |
| `L_d/L_q` | `1.05e-3 H` | `1.02e-3 H` | 小差异 |
| `R_s/n_p/psi_f/J_g` | 与非线性一致 | 与小信号一致 | 一致 |

已有 DC 场景对比表明，把 `Vdc/Cdc` 改为更接近非线性对象后，`GFM-MWT` 仍表现为负阻尼，`GFM-MWT+AD` 仍恢复正阻尼。因此当前定性机制没有被 DC 参数口径推翻。但最终跨域验证前，应以非线性最终对象重新生成一套正式小信号基准。

## 9. 现有图表基础

当前已经具备以下小信号图表基础：

- 四拓扑阻尼柱状图、实部对比图、整体稳定性对比图；
- 四拓扑参与因子表；
- Type-a / Type-c 对比结果；
- `h`、`mp`、`k_pdc/k_idc`、`k_pq/k_iq`、`K_damp` 参数扫描结果；
- 模态轨迹图；
- 小扰动响应及 FFT 指标。

需要注意：参数扫描和模态轨迹文件时间戳较早，属于已有分析类型和图表基础。若最终采用 `Vdc=1000 V, Cdc=0.03 F` 作为统一对象，应重跑这些扫描后再作为论文最终数值。

## 10. 当前可写入论文的阶段性表述

可以写：

1. 已建立包含两质量块轴系、PMSG、MSC/GSC、DC 电容、GFL/GFM 控制差异和 APCAD 的统一小信号分析框架。
2. 当前低频主关注模态约为 2 Hz，参与因子集中在 `theta_tw/omega_g/omega_t`，可判定为轴系扭振模态。
3. 仅将网侧同步方式从 PLL/GFL 改为 GFM，并未显著降低该轴系模态阻尼。
4. 将 DC 电压控制从网侧转移到机侧后，轴系模态阻尼明显下降并可变为负阻尼，说明机侧 DC 控制会增强电磁转矩、DC 能量平衡与轴系扭振之间的耦合。
5. APCAD 可将该模态向左半平面移动，提高低频轴系模态阻尼。

暂时不能写：

1. 当前所有物理参数均已经严格来自厂家或文献。
2. 当前非线性模型已经完成稳态运行验证。
3. 当前小信号模型全系统所有模态均稳定。
4. 非线性实验已经观测到与小信号完全一致的轴系模态。

## 11. 下一步进入非线性的门槛

非线性验证必须先完成无扰动基准：

- 三相电压、电流稳定且不过饱和；
- 有功功率维持在 1 MW 附近；
- DC 电压有界，不持续漂移；
- `omega_t/omega_g/theta_tw/T_sh` 不发散；
- 生成与当前参数一致的热启动文件。

完成该门槛后，再进行小扰动实验。小扰动实验中不应预设 2 Hz 滤波目标，而应先从 `T_sh/omega_g/omega_t/P/iabc` 的时域响应和 FFT 中识别主频，再与小信号预测模态对比。
