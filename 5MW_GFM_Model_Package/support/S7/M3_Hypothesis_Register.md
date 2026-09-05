# M3 可证伪假设登记表

## 最高层科学问题

构网控制如何改变风电机组电气控制系统与机械传动链之间的动态能量耦合；这种耦合在什么条件下会激励、放大或抑制轴系扭振，并进一步形成机械—电气多模态振荡与稳定边界？

本表中的内容均为待验证假设，不是论文结论。任何反例优先于既有叙事；只有通过跨运行点、跨参数及跨模型 Gate 的现象，才允许升级证据等级。

## H0：模型与符号前提

- 假设：M2 的公共 plant、状态顺序、端口功率面、扰动归一化和模态跟踪可以作为 M3 唯一冻结基准。
- 反例：任一架构在同一物理目标点不能达到严格共同平衡，或端口功率/转矩定义不能形成可审计的能量关系。
- 当前状态：`S1_GATE_PASS_WITH_M3_CORRECTION`。
- 审计结果：冻结 M2 仍满足旧的加铜耗关系，仅作为历史反事实；M3 已切换为 `GENERATOR_OUTWARD`，四个共同工作点均满足 `P_MSC = T_e*omega_g - P_Cu`，最大共同工作点差异为 `1.745e-7 pu`，最大平衡残差为 `4.954e-13`。该修正尚未经过更高保真模型验证，因此不能外推为 EMT 结论。

## H1：MSC-DVC 的轴系阻尼作用

- 假设：MSC-DVC 在部分运行域内降低轴系频率处的电气正阻尼。
- 可证伪条件：在物理一致运行点或合理控制参数域内，DVC 增量阻尼为正、跨越零点，或其作用远小于其他通道。
- 必查指标：`DeltaDe_DVC(f,P0)`、`lambda_tor`、`zeta_tor`、工作点与端口功率审计。
- 当前证据等级：`CONDITIONAL_CROSS_WORKPOINT`。0.3--0.9 pu 内增量阻尼保持为负，但尚未跨控制参数域检验。

## H2：局部 MPPT 的轴系阻尼作用

- 假设：局部 MPPT 切线在当前额定附近提供正电气阻尼。
- 可证伪条件：随功率/转速变化，`DeltaDe_MPPT` 变负、跨零或对轴系极点影响消失。
- 必查指标：物理一致 `P0-omega0-Tm0` 曲线、`DeltaDe_MPPT(f,P0)`、气动阻尼与电气阻尼的相对大小。
- 当前证据等级：`CONDITIONAL_CROSS_WORKPOINT`。0.3--0.9 pu 常 TSR 切线内保持为正，但尚未加入完整 Cp-lambda 与气动状态。

## H3：控制职责与双向扰动传播

- 假设：GFL、GFM-GWT、GFM-MWT 的控制职责会形成频率局部的传播方向占优。
- 可证伪条件：`L_dir=log10(C_GM/C_MG)` 随运行点或频率跨零，或主、反向通道成为同量级。
- 解释限制：禁止将近似零直接称为结构零，禁止将方向依赖称为非互易性。
- 当前证据等级：`COUNTEREXAMPLE_FOUND`。GWT 的 `L_dir` 在测试运行点内跨越零点，故“传播方向始终固定”已被否证；MWT/GFL 的排序暂时保持，只能视为条件性现象。

## H4：Pole Shaping 与 Excitation Shaping 可分解

- 假设：不同工况下响应差异可以由极点变化、模态残差/输入投影变化及其交互项解释。
- 可证伪条件：反事实重构 `y00/y10/y01/y11` 不能复现总响应变化，或交互项长期占主导而二分框架失效。
- 必查指标：`Delta_y_pole`、`Delta_y_excitation`、`Delta_y_interaction`、`w^H B` 和输出可观测性。
- 当前证据等级：`CONDITIONAL_METHOD_EXECUTED`。已完成 `y00/y10/y01/y11` 分解；GWT机械扰动的分类随运行点迁移，说明 Pole/Excitation 二分不能固定套用，必须逐工况报告交互项。

## H5：真实机电模态混合

- 假设：在合理运行域内，某些电气控制模态可能接近轴系模态并发生真正混合。
- 必须同时满足：特征值 veering、机械/电气参与度交换、MAC 身份交换、双向能量或响应混合。
- 可证伪结果：在工程可接受域内始终只有频率接近而无参与度/MAC交换。该结果同样具有研究价值。
- 当前证据等级：`NOT_DEMONSTRATED`。

## H6：柔性机械跨模型互连的稳定性与 DVC 慢分支

- 假设：将来源可追溯的柔性传动链与 M3 电气模型通过低速轴能量共轭接口互连后，M2/M3 中的机电耦合机制至少在局部 OpenFAST 机械模型上保持；若不保持，应能定位到新增机械阻抗、DC-link 能量动态或 MSC-DVC 闭环。
- 可证伪条件：OpenFAST 柔性机械开环基线不稳定；转矩反馈断开仍不稳定；或三方位角/一周 Floquet 结果不能保持同一分支身份。
- 当前状态：`S6_GATE_FAIL_LOCAL_COUNTEREXAMPLE`。
- 审计结果：Frozen-Wake 直接 ElastoDyn 柔性机械基线及转矩反馈断开反事实均通过稳定性审计；但 MWT 的 MSC-DVC—电磁转矩闭合产生跨三方位角保留的近零频慢速不稳定分支。PI、P-only、I-only 与 DC-link 电容反事实表明，当前失稳不能简单归因于“柔性轴系固有模态”，也不能用单一增益放大解释。
- 最新复核：重新执行 S6 Frozen-Wake（NREL5MW_SCHEDULED_LSS，风速因子 1.15）后，MWT 全坐标最大实部为 `+3.6708714e-2 s^-1`，转矩反馈断开为 `-3.7386747e-3 s^-1`；三方位角全坐标分别为 `+3.6738285e-2`、`+3.6576821e-2`、`+3.6810259e-2 s^-1`，开环分别为 `-3.7385864e-3`、`-3.7375120e-3`、`-3.7399289e-3 s^-1`。PI-DVC 因子 `1e-12` 仍稳定、`3e-12` 已失稳，而其转矩反馈断开反事实仍稳定，进一步支持“电磁转矩闭环触发的低频分支”这一局部假设；这不是轴系 2.5 Hz 扭振证据。
- 低频符号复核：在 1 mHz，MWT 电气增益 `Delta Te/Delta omega_g = -2.0066e6 - j592.98`，OpenFAST 机械增益 `Delta omega_g/Delta Te = -2.7671e-6 + j7.803e-7`，乘积约为 `5.5529-j1.5641`；GFL/GWT 的电气增益为正而机械增益仍为负。结合 `eDc=Vdc0-Udc`、`iq_ref=Kpdc*eDc+xi_DVC` 和正发电制动转矩约定，这说明当前 MWT 低频支路具有“速度上升→DVC减小制动转矩”的反向反馈候选，但仍需在物理可实现的完整控制互连中验证，不能直接命名为普适负阻尼定理。
- 证据边界：S6 只是 OpenFAST 周期线性化柔性机械体与 M3 电气 SSM 的局部混合分析，不是 OpenFAST—Simulink 非线性联合仿真；未验证直驱 OpenFAST、动态 AeroDyn、离散控制或 EMT。该反例只降低“跨模型稳健性”的证据等级，不构成“GFM 必然恶化轴系稳定性”的结论。
- 后续 Gate：在获得严格稳定且无需静态 AeroDyn 状态消元的柔性机械基线，或完成可物理实现的 MWT-DVC 低频互连解释前，`S7` 保持阻塞。

### S6 候选资产追加审计（2026-09-02）

- 已使用统一工具 `tools/OpenFAST/v5.0.0/OpenFAST_Double_Release.exe` 对临时直驱参数化候选执行预审；该工具为官方 v5.0.0 Windows 双精度非 OpenMP 发行版，候选输入仍不是来源可追溯的直驱 5 MW 基准。
- 原候选因 `TimGenOn=9999.9` 未形成有效发电机工作点；保留原文件作为失败证据，未覆盖。
- 新建的 `temp/OpenFAST_S6/runtime/5MW_DirectDrive_Parameterized_GenOn` 将 `TimGenOn=0 s`，在 `TMax=120 s` 下正常完成 3 个线性化点，无强制线性化警告；输出尾段 `GenPwr≈1.807 MW`、`GenTq≈1.942 MN·m`，说明该候选至少形成了非零发电输出。
- 但 MBC 平均机械矩阵仍出现最大实部 `+5.0244e-4 s^-1` 的零频分支（30 个 ElastoDyn 状态）；因此该候选未通过“柔性机械开环稳定基线”预审，不能接入 S6 闭环、不能解除 `S7` 阻塞，也不能作为直驱机组物理证据。
- 下一步只有两条可追溯路线：`ROUTE-A` 获取用户/公开来源的直驱柔性 OpenFAST 输入并做同一审计；或 `ROUTE-B` 明确构造带外部恒转矩/配平气动的最小直接驱动柔性基准，单独标注为条件性方法验证。两条路线均须先通过开环稳定、周期工作点、能量共轭和三方位角一致性 Gate，才允许重新执行 S6。

### S6 Route-B 文献参数候选复核（2026-09-02）

- 在不改变主线模型的前提下，建立临时副本 `temp/OpenFAST_S6/runtime/5MW_DirectDrive_EMD2018_ParamCandidate`。该副本采用 Slot 等人在 WindEurope 2018 报道的 NREL 5 MW 直驱转换惯量/阻尼参数：`GBRatio=1`、`GenIner=250000 kg·m^2`、`DTTorDmp=6.215e6 N·m/(rad/s)`；其余气动、结构和控制输入仍来自本地 NREL 5 MW 案例，因此不是官方直驱基准。
- OpenFAST v5.0.0 正常完成 120 s 运行和 3 个周期线性化点；尾段 `GenPwr=1809.148 kW`、`GenTq=1941.876 kN·m`，说明该候选形成了非零发电工作点。
- MBC 平均 30 状态矩阵的全坐标最大实部为 `+4.27211e-4 s^-1`，但去除发电机角度规范后的最大实部为 `-1.13088e-2 s^-1`。因此机械本体在去规范坐标下稳定，但严格全坐标 Gate 仍失败。
- 将该候选接入 M3 S6 条件审计后：`all_angle_removed_floquet_stable=1`，但 `source_traceable=0`、`pass=0`，状态为 `CONDITIONAL_CUSTOM_FROZEN_WAKE_HYBRID_STABLE_SOURCE_GATE_FAIL`。该结果只能作为 Route-B 方法反例/敏感性证据，不能解除 S7，也不能升级为跨模型稳健性结论。

## 证据升级规则

1. `OBSERVATION`：单一工作点或单一模型的数值现象。
2. `CONDITIONAL_MECHANISM`：通过工作点审计、通道消融和频域/模态一致性验证。
3. `VALIDITY_DOMAIN`：给出保持、弱化、反转和失效边界。
4. `CROSS_MODEL_ROBUST`：在气动/柔性机械/离散平均模型中保持机制身份。
5. `EMT_SUPPORTED`：代表点开关 EMT 与前述趋势和模态身份一致。

在低一级证据未通过前，不进入下一等级，也不把假设写成一般结论。

## S7 双验证路线状态（2026-09-03）

为避免把 S6 的柔性机械证据缺口误当成电气实现缺口，S7 已独立开放；但 S6 状态仍永久保持 `INCONCLUSIVE`。

### S7-0 基准冻结：COMPLETE

- 连续参考模型：`M0_PMSG_GFM_5MW.slx`，23 状态、5 MW、`Ts0=0.0001 s`、`H=3 s`、`SCR=4`、DVC比例为1。
- 已生成 `S7_Reference_Manifest.csv`、`S7_Digital_Controller_Manifest.csv` 和 `Controller_Discretization_Manifest.csv`，记录模型/方程/工作点哈希、控制器来源及本轮筛查使用的离散化假设。
- S7期间不修改公共 plant、工作点、PI、H、DVC 或 MPPT。

### S7A 同源采样平均筛查：V1 PASS；A2/A3 SCREENED

- V1：`Ts/Ts0=0.01、0.05、0.1`，无延迟；映射回连续域后轴系频率误差最大约 `1.11e-4 %`，阻尼代理误差最大约 `6.04e-6`，趋势一致且离散稳定。
- A2：固定 `Ts0`，`Td/Ts=0、0.5、1、1.5`；当前同源采样筛查均稳定，轴系频率约 `2.4827465--2.4827490 Hz`，阻尼代理约 `0.422993--0.423010 /s`。
- A3：典型一拍延迟，`Ts/Ts0=0.5、1、2、4`；当前筛查均稳定；`4Ts` 仅作为 `STRESS` 点，不进入工程主域结论。
- 上述 A2/A3 只说明同源采样映射没有立即产生反例，不等同于真实数字控制器验证，也不等同于 EMT 稳健性。

### Gate V2：BLOCKED；S7B：NOT STARTED

- 当前仓库没有可追溯、与 M0 状态和工作点严格对齐的真实离散平均 Simulink 模型，因此尚不能比较离散 SSM 与“非线性离散平均模型”的小扰动 NRMSE。
- 在 V2 通过前，不启动开关 EMT；不把 S7A 的 `TOR_DampingProxy=-Re(s_TOR)` 写成复转矩 `D_e(jw)`，也不计算虚构的离散模态残差。
- 后续解除 V2 的唯一前提：补齐真实离散平均模型及其状态/输入输出映射，并完成 D1（基准）、D2（实现差异最大但稳定）、D3（稳定边界稳定侧）三点小扰动对照。

### 当前证据等级

`S6=INCONCLUSIVE`；`S7-0=COMPLETE`；`S7A-V1=CONDITIONAL_MECHANISM_SCREENING`；`S7A-A2/A3=IMPLEMENTATION_SCREENING`；`S7A-V2=NOT_ESTABLISHED`；`S7B=NOT_ESTABLISHED`。本节不升级任何 M3 假设为跨模型或普适结论。
