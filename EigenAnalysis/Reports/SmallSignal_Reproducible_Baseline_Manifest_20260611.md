# 构网型风电机组小信号可复现实验基准清单（2026-06-11）

本文档用于冻结当前 1 MW 构网型风电机组机电耦合小信号分析基准。它不是新的结果解释报告，而是后续复现实验、论文出图和非线性验证对照时使用的工程清单。

## 1. 基准范围

当前冻结的小信号基准包含四类主模型：

| 模型 | 机侧控制 | 网侧控制 | 用途 |
|---|---|---|---|
| GFL-WT | MSC-MPPT | GSC-DVC + PLL | 传统跟网型风机对照 |
| GFM-GWT | MSC-MPPT | GSC-DVC + GFM | 仅替换网侧同步方式，分离 PLL/GFM 影响 |
| GFM-MWT | MSC-DVC | GSC-GFM | DC 电压控制移至机侧，分析机电耦合增强机制 |
| GFM-MWT+AD | MSC-DVC | GSC-GFM + APCAD | 在 GFM-MWT 基础上验证附加阻尼作用 |

同时冻结 Type-a / Type-c 对照：

| 模型 | DC 控制结构含义 | 用途 |
|---|---|---|
| GFM-MWT-TypeA | 机侧 DVC 主要由 DC 电压反馈生成转矩/电流指令 | 基础机侧 DC 控制结构 |
| GFM-MWT-TypeC | 在 DC 电压反馈外增加功率前馈路径 | 分析功率前馈是否改善低频扭振阻尼 |
| GFM-MWT-TypeC+AD | TypeC 基础上加入附加功率耦合阻尼 | 验证主动阻尼对轴系模态的改善 |

## 2. 复现入口

推荐按以下顺序运行：

1. `EigenAnalysis/Parameters.m`
   - 生成或刷新 `Parameters.mat`。
   - 所有后续小信号脚本均应基于同一份参数文件。

2. `EigenAnalysis/Compare_Control_Mode_Run.m`
   - 复现 GFL-WT、GFM-GWT、GFM-MWT、GFM-MWT+AD 四拓扑对照。

3. `EigenAnalysis/Compare_DVC_TypeA_TypeC_Run.m`
   - 复现 Type-a、Type-c、Type-c+AD 对照。

4. `EigenAnalysis/Scan_GFM_Control_Parameters_Run.m`
   - 复现控制参数扫描。

5. `EigenAnalysis/Track_GFM_Mode_Trajectories.m`
   - 复现特征值随参数变化的轨迹。

6. `EigenAnalysis/Plot_Small_Disturbance_Responses.m`
   - 复现线性小扰动响应和频谱结果。

主要输出目录：

- `EigenAnalysis/Results/Control_Mode_Comparison_Results`
- `EigenAnalysis/Results/DVC_Type_Comparison_Results`
- `EigenAnalysis/Results/Control_Parameter_Scan_Results`
- `EigenAnalysis/Results/Mode_Trajectory_Results`
- `EigenAnalysis/Results/Small_Disturbance_Response_Results`
- `EigenAnalysis/Reports`

## 3. 参数来源分级

当前参数必须按来源分级使用，不能在论文中统一表述为“文献参数”。

| 来源类型 | 含义 | 当前代表参数 |
|---|---|---|
| 用户/模型给定 | 已按当前 1 MW 研究对象固定或与现有模型对齐 | `S_base=1e6 VA`、`V_LL=690 V`、`f_base=50 Hz` |
| 公式推导 | 由运行点、基准值或线性化关系计算得到 | `T_e0`、`theta_tw0`、气动线性化系数 |
| 控制设计 | 用于控制器设计、对比或扫描 | `h`、`mp`、`k_pdc/k_idc`、`k_pq/k_iq`、`K_damp` |
| 暂定等效参数 | 用于构造当前 1 MW 轴系基准，后续需由 2 MW/5 MW 文献或厂家数据替换 | `J_t=8*J_g`、由目标扭振频率反推的 `K_sh`、由阻尼比反推的 `D_sh` |

当前机械侧关键参数：

| 参数 | 当前值 | 用法说明 |
|---|---:|---|
| `J_g` | `1.8375e5 kg*m^2` | 发电机侧惯量 |
| `J_t` | `1.47e6 kg*m^2` | 当前按 `8*J_g` 暂定 |
| `J_eq` | `1.6333e5 kg*m^2` | 两质量块等效惯量 |
| `K_sh` | `2.5793e7 N*m/rad` | 当前为得到 2 Hz 基准扭振模态而反推 |
| `D_sh` | `4.1050e4 N*m*s/rad` | 当前由 1% 轴系阻尼比反推 |

后续如果切换为 2 MW 或 5 MW 风机，必须重新给定或换算 `J_t/J_g/K_sh/D_sh`，并重新运行全部小信号脚本；不能沿用当前 1 MW 基准结果。

## 4. 2 Hz 模态的来源

当前 2 Hz 左右扭振模态来自两质量块轴系固有频率：

```text
J_eq = J_t * J_g / (J_t + J_g)
f_sh = 1/(2*pi) * sqrt(K_sh / J_eq)
```

代入当前参数：

```text
J_t  = 1.47e6 kg*m^2
J_g  = 1.8375e5 kg*m^2
J_eq = 1.6333e5 kg*m^2
K_sh = 2.5793e7 N*m/rad
f_sh = 2.0000 Hz
```

因此，2 Hz 不是非线性实验中的先验滤波目标，而是当前小信号轴系参数计算出的理论模态。非线性验证时应先从时域信号中提取 FFT/PSD 主峰，再与该小信号预测频率比较。

轴系阻尼比按下式表征：

```text
zeta_sh = D_sh / (2*sqrt(K_sh*J_eq))
```

当前 `zeta_sh = 0.01`。这表示基础轴系阻尼较弱，适合作为机电耦合振荡研究对象，但不等价于真实机组阻尼已经完成辨识。

## 5. 当前可引用的小信号结论

四拓扑对照的低频扭振模态结果：

| 模型 | 扭振频率/Hz | 扭振阻尼比 | 扭振实部 Sigma |
|---|---:|---:|---:|
| GFL-WT | 1.9999 | 0.01148 | -0.1442 |
| GFM-GWT | 1.9999 | 0.01148 | -0.1442 |
| GFM-MWT | 2.0000 | 0.00920 | -0.1156 |
| GFM-MWT+AD | 1.9911 | 0.01501 | -0.1878 |

可以支撑的阶段性表述：

1. 仅将网侧 PLL 同步替换为 GFM 同步，并不会必然降低轴系扭振阻尼。
2. 将 DC 电压控制从网侧转移到机侧后，机侧电磁转矩更直接参与 DC 能量平衡，使轴系机械状态、机侧功率和 DC 电压之间形成更强反馈通道，当前参数下表现为扭振阻尼降低。
3. APCAD 附加阻尼能使扭振模态左移，提高该低频机械模态阻尼。
4. 参与因子集中在 `theta_tw/omega_g/omega_t`，说明被跟踪的 2 Hz 模态主要是轴系扭振模态，而不是纯电流环、PLL 或 DC 电压环模态。

Type-a / Type-c 对照的当前结果：

| 模型 | 扭振频率/Hz | 扭振阻尼比 | 扭振实部 Sigma | 低频扭振稳定性 |
|---|---:|---:|---:|---|
| GFM-MWT-TypeA | 2.0011 | -0.01374 | 0.1728 | 不稳定 |
| GFM-MWT-TypeC | 2.0011 | -0.01370 | 0.1723 | 不稳定 |
| GFM-MWT-TypeC+AD | 1.9538 | 0.01160 | -0.1424 | 稳定 |

可以支撑的阶段性表述：

1. 只从 Type-a 改为 Type-c，对当前低频扭振阻尼改善有限。
2. Type-c 的意义主要在于把 DC 电压控制中的功率平衡路径表达得更清楚，方便后续布置 APCAD 或其他功率阻尼环节。
3. 当前真正显著改善扭振阻尼的是附加阻尼环节，而不是 Type-c 结构本身。

## 6. 当前不能宣称的结论

以下表述暂时不能作为论文结论：

1. 不能宣称所有物理参数均为文献或厂家严格给定；当前 `K_sh/D_sh/J_t` 中仍有等效和反推成分。
2. 不能宣称四拓扑全系统均已小信号稳定；当前重点冻结的是低频扭振模态规律，系统中仍存在非扭振正实部模态需要后续处理。
3. 不能宣称非线性 Simulink 模型已经完成稳定运行验证；非线性部分仍处于无扰动稳态整定阶段。
4. 不能在非线性实验中预设 2 Hz 作为固定结论；必须先观测信号，再通过 FFT/PSD/Bode 等方式识别主频。
5. 不能用小信号 `dx=A*x+B*u` 响应替代非线性模型的小扰动验证。

## 7. 与非线性验证的接口要求

非线性模型完成前，应优先满足以下条件：

1. 无扰动下三相电压、电流稳定且有正常波形。
2. 有功功率维持在 1 MW 附近。
3. DC 电压有界，不持续漂移；允许存在与模型结构相关的合理稳态偏差。
4. `omega_t/omega_g/T_sh` 不发散，机械侧状态不持续漂移。
5. 每一组控制参数都生成对应的 `ModelOperatingPoint` 热启动文件，避免跨参数复用旧状态。
6. 小扰动实验中对 `T_sh/omega_g/omega_t/P/iabc` 做 FFT/PSD，并与小信号预测模态比较。

## 8. 后续改动原则

1. 改风机容量或轴系参数时，先修改 `Parameters.m`，再完整重跑小信号基准。
2. 改控制结构时，应新增模型标签和结果目录，不覆盖当前四拓扑基准。
3. 非线性调参结果不得反向修改小信号冻结结论，除非发现小信号参数或模型结构存在明确错误。
4. 论文写作中应区分三类证据：小信号特征值证据、线性小扰动响应证据、非线性 Simulink 时域/频域验证证据。
# 2026-06-12 更新提示

当前工作树已重新运行 `EigenAnalysis/Compare_Control_Mode_Run.m`，四拓扑扭振模态数值以 `SmallSignal_Current_Rerun_Baseline_20260612.md` 和 `Results/Control_Mode_Comparison_Results/baseline_torsional_modes.csv` 为准。本 2026-06-11 清单保留为历史冻结说明，不再作为最新数值来源。
