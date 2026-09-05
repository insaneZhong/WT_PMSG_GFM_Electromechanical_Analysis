# Zotero 文献—当前 M0 模型对照与下一步分析计划

## 1. 文献核对范围与证据等级

Zotero Desktop 的本地 API 本轮不可用（`127.0.0.1:23119` 超时，且本机未找到可自动重启的 `prefs.js`）。因此本报告只使用已经同步到本机的 Zotero 全文缓存和 BibTeX 条目，不把仅有题名的记录当作“已阅读全文”。

本轮核对了以下与当前课题直接相关的全文：

1. `liu_impact_2024`：S. Liu 等，*Impact of DC-Link Voltage Control on Torsional Vibrations in Grid-Forming PMSG Wind Turbines*, IEEE Transactions on Energy Conversion, 2024，Zotero key `VDD2CEJF`。
2. *Comparative Evaluation of Converter Control Impact on Torsional Dynamics of Type-IV Grid-Forming Wind Turbines*, IEEE Transactions on Sustainable Energy, 2024，Zotero key `TRPVMYJM`。
3. `chen_drivetrain_2022`：L. Chen 等，*Drivetrain Oscillation Analysis of Grid Forming Type-IV Wind Turbine*, IEEE Transactions on Energy Conversion, 2022，Zotero key `ULY3X6R9`。
4. Y. Ma 等，*Low-Frequency Oscillations and Resonance Analysis of VSG-controlled PMSG-based Wind Generation Systems*, Journal of Modern Power Systems and Clean Energy, 2025，Zotero key `8BWIV6ZB`。
5. Y. Ma 等，*Stability and Dynamic Analysis of PMSG-based Wind Generation System Considering Torsional Oscillation and Virtual Inertia Control*, Journal of Modern Power Systems and Clean Energy, 2025，Zotero key `8GF3MNSX`。
6. `udawatte_active_2026`：H. E. Udawatte 等，*Active Power Control-Based Damping for Torsional Oscillations in Grid-Forming Type-IV Wind Turbines*, IEEE Transactions on Power Electronics, 2026，Zotero key `2RTXN29Z`。

## 2. 文献结论与当前模型的逐项对应

| 文献中的结论 | 当前 M0 证据 | 对应判断 | 还缺什么 |
|---|---|---|---|
| 两质量轴系的扭振频率主要由机械参数决定，通常约 2.5 Hz；控制器对频率的影响小于对阻尼的影响 | 轴系主导极点 `-0.4673857+j15.6715818`，`f_tor=2.49421 Hz`，机械参与度 `0.996384`；GFL、Droop-GFM、VSG-GFM 频率完全一致 | **高度一致** | 只需在论文中给出理论两质量频率与 M0 特征值的并列表 |
| MSC 负责 DVC 的 GFM-MWT 中存在 `GSC/grid → DC-link → MSC-DVC → Te` 反馈，可能产生负电气阻尼 | `Disturbance_Path_Summary.csv`、`Disturbance_Path_Ablation_Summary.csv` 已确认 `P_GSC → Udc → iq_MSC_ref → Te → omega_sh`，C0–C4 消融通过 | **结构一致** | 需要用“完整闭环复转矩”而不是当前冻结机械状态的局部反馈对象重新计算 `G_Te_omega_g` |
| GFM-MWT 比 GFM-GWT/GFL 更容易被网侧相角扰动激发扭振；不同 DVC 类型会改变阻尼 | 当前三控制同工作点的轴系极点和 (G_{T_e,omega_g}) 数值完全相同，但网侧角度/频率残差显著改变；VSG 角度激励比基准约 2.744，频率激励比约 334.410 | **部分一致，且给出更精确的解释** | 必须区分“本征阻尼改变”和“外部扰动激励增强”；当前结果支持后者，尚不足以复现文献的 DVC 型负阻尼结论 |
| 复杂转矩系数法应在扭振频率处分解同步系数与阻尼系数 | 已有 `Feedback_Torque_At_Torsional_Frequency.csv` 和 `Fig05_Feedback_Torque_Bode` | **方法一致，闭环对象尚需核对** | 当前对象由 `A(4:23,4:23)` 和 `B=A(4:23,3)` 构成，即冻结机械状态后从 `omega_g` 到 `Te` 的电气反馈；它不等于网侧角度/频率扰动到 `Te` 的完整闭环响应，需补充后者 |
| DVC 增益、(C_{dc})、SCR、风速和 VSG 惯量会改变轴系阻尼，且规律可能非单调 | M0 第一轮扫描显示 DVC 缩放 0.5→2 时 (zeta_{tor}=0.03033→0.02872)；H 对轴系阻尼基本不变但显著改变网侧激励；当前 SCR 扫描尚未形成临界边界 | **定性一致，定量未闭合** | 做 DVC×SCR 和 DVC×H 的二维扫描，并跟踪轴系模态和邻近电气模态，而不是只做一维点表 |
| 低频电气模态与轴系模态接近时会发生模态耦合、参与因子交换或共振 | `Pole_Excitation_Classification.csv` 已证明极点基本不变而激励变化；`Single_Mode_Reconstruction_Summary.csv` 证明完整 SSM 需要多个模态 | **尚未直接验证** | 计算 GFM/VSG 低频模态与 2.494 Hz 轴系模态的频率距离、MAC/参与因子和根轨迹 |
| 风速阶跃可以激励轴系扭振，且应同时比较非线性时域与小信号 | `Fig08_WindStep_NL_SSM_Alignment` 已完成：0.01 m/s 局部等效风速阶跃，(Delta\omega_{sh}) 峰值误差 0.016%，(T_{sh}) 0.535%，(T_e) 0.488%，(U_{dc}) 0.248%，最大 NRMSE 0.0203 | **严格对应** | 当前是 M0 的局部气动功率等效扰动；后续可增加 0.02/0.05 m/s 并保持线性区，暂不引入 Pitch/EMT |
| 直接在 MSC 转矩参考上加阻尼可能与 DVC 竞争；通过 GSC 有功侧间接注入阻尼更容易保持 DC-link 调节 | 当前 M0 已定位 `P_GSC → Udc → DVC → Te`，但尚未加入阻尼控制器 | **机理上相容，尚未验证** | 只有在闭环复转矩和稳定边界完成后，才比较 MSC 转矩、GSC 有功、VSG 相角三个注入点 |

## 3. 目前可以写入论文的“对应结论”

### 3.1 可以直接写的部分

1. **扭振频率对应上了。** 当前 M0 的 `2.49421 Hz` 与相关文献约 `2.5 Hz` 的两质量轴系结果一致，且轴系状态参与度约 `99.64%`，因此当前峰值确实是轴系主导模态，而不是任意功率环纹波。
2. **MSC-DVC 反馈链对应上了。** 当前同工作点消融已验证网侧功率扰动必须经过 DC-link 和 MSC-DVC 才能形成电磁转矩响应，这与 Liu 2024、Chen 2022 和 Udawatte 2026 的控制结构判断一致。
3. **GFM 可能主要改变扰动激励而不是轴系本征阻尼。** 当前 GFL、Droop-GFM、VSG-GFM 的轴系极点完全重合，但网侧角度/频率扰动残差明显不同。这与文献中“网侧事件激发扭振”的现象一致，同时避免把响应振幅直接误写成阻尼下降。
4. **连续非线性—小信号对齐已达到论文图表可用程度。** `Fig08_WindStep_NL_SSM_Alignment` 的四个信号均达到亚百分比峰值误差或很小 NRMSE，可作为“模型验证图”，但它证明的是同源 M0 对齐，不等于已经证明 GFM 比 GFL 具有更低本征阻尼。

### 3.2 当前不能直接写的部分

不能根据当前结果直接写“VSG 必然恶化轴系阻尼”或“GFM 一定使轴系极点右移”。原因是：当前三种外环在同一 M0 工作点下，`Feedback_Torque_At_Torsional_Frequency.csv` 中的 (G_{T_e,\omega_g})、轴系极点和 (zeta_{tor}) 完全相同。当前可支持的表述是：

> 在本研究的共同工作点和理想连续平均控制结构下，GFM 主要改变网侧扰动到轴系模态的激励残差；轴系本征极点未发生显著迁移。DVC/DC-link 通道是网侧扰动形成电磁转矩响应的必要路径。是否存在文献所述的 GFM-MWT 负电气阻尼，需要进一步以包含完整 GSC—DC-link—MSC 闭环的复转矩对象重新计算。

这一区分很重要：Liu 2024 和 Chen 2022 的结论针对的是完整电气子系统的 (W_d(\omega)) 或 (T_e/\omega_g) 闭环响应；当前 `feedbackTorqueScan` 使用 `A(4:23,4:23)`、`B=A(4:23,3)`，本质上是“冻结机械状态后的电气反馈对象”。它可以作为局部反馈诊断，但不能单独代表网侧角度/频率扰动到电磁转矩的端到端响应，因此二者还不能直接一一比较。

## 4. 下一步应该优先分析什么

### 优先级 0：先补齐“比较层级”

当前 `GFL–Droop-GFM–VSG-GFM` 公平对比只替换了同步/有功外环，三者共同保留 MSC-DVC、DC-link、PMSG 电流环和 GSC 内环。因此它回答的是：

> 在同一 MSC-DVC 架构下，同步控制类型是否改变轴系极点和网侧扰动激励？

而 Liu 2024 和 TSTE 2024 论文的主要架构对照是：

- **GFM-GWT**：GSC 负责 DVC，MSC 负责 MPPT/转矩；
- **GFM-MWT**：MSC 负责 DVC，GSC 负责 GFM 有功—相角控制。

因此在比较文献中的“GFM-MWT 负阻尼”之前，必须增加一个最小架构对照：

1. 当前 M0：MSC-DVC + GSC-GFM（GFM-MWT）；
2. 同一工作点的 GSC-DVC + MSC-MPPT/转矩（GFM-GWT）；
3. GFL 基准。

三者仍共用两质量轴系、PMSG、LCL、电网和功率测量面。只改变 DC-link 调节位置及相应的 MSC/GSC 外环，不恢复离散、PWM、限幅或 EMT。验收量为：

\[
f_{tor},\quad \zeta_{tor},\quad W_s(\omega_{tor}),\quad W_d(\omega_{tor}),\quad R_{tor,grid}
\]

如果只有 GFM-MWT 出现 (W_d<0) 或轴系阻尼下降，而 GFM-GWT/GFL 不出现，就与 Liu 2024、TSTE 2024 的结论直接对应；如果三者仍只表现为激励残差不同，则应把论文结论限定为“同步外环的扰动通道塑形”，不能扩大为“GFM 架构必然产生负阻尼”。

### 优先级 1：重定义并验证“完整闭环复转矩”

在固定 M0 工作点，统一计算以下两个对象，而不是只保留一个：

1. **外部扰动到轴系的闭环转矩响应**
   \[
   G_{T_e,\theta_g}(s)=\frac{\Delta T_e(s)}{\Delta\theta_g(s)},
   \qquad
   G_{T_e,\omega_g}(s)=\frac{\Delta T_e(s)}{\Delta\omega_g(s)}.
   \]
2. **轴系自身反馈转矩响应**
   \[
   G_{T_e,\omega_g}^{\rm fb}(s)
   \]
   该对象需要明确是否包含 GSC、LCL、电网、P/Q 滤波、VSG 和 DC-link 闭环。

验收要求：

- 明确输入、输出和切断点；
- 在 `0.2–10 Hz` 扫频，标出 `2.49421 Hz`；
- 同时输出同步系数、阻尼系数、幅值和相位；
- GFL、Droop-GFM、VSG-GFM 三者分别给出曲线；
- 用同一符号约定与 Liu 2024/Chen 2022 的 (W_s,W_d) 对照。

如果完整闭环复转矩仍然三者一致，结论将变成：当前模型的 GFM 差异确实只在外部网侧激励通道；这本身是一个有价值、且与“振幅不等于阻尼”的文献讨论相区分的结果。如果完整闭环复转矩出现差异，则进一步判断差异来自 DVC、GFM 功角还是网侧内环。

### 优先级 2：DVC—DC-link—GFM 的二维稳定边界

文献普遍不只给单点，而是给参数影响和临界区域。建议先做：

- 横轴：DVC 比例/积分带宽缩放 `0.25–3`；
- 纵轴：SCR `2–8`；
- 每个点跟踪：`Re(lambda_tor)`、`f_tor`、`ζ_tor`、最近电气模态频率和参与因子；
- 输出 `alpha_tor=-Re(lambda_tor)` 与 `alpha_ele=-max Re(lambda_non-tor)` 两张稳定边界图。

先不把 H、虚拟阻抗、主动阻尼和 EMT 一起加入。这样能直接检验 Liu 2024 所强调的 DVC/SCR 耦合，并避免把多个参数效应混在一起。

### 优先级 3：模态接近和残差变化

针对 Ma 2025 的“扭振—低频电气模态共振”结论，增加以下诊断：

- 对 H、SCR 和 DVC 带宽扫描记录最近低频电气模态；
- 计算与轴系模态的频率距离；
- 使用特征向量 MAC 或参与因子连续性跟踪模态，而不是按频率最近匹配；
- 记录网侧角度、网侧频率、轴系状态对该模态的参与度交换；
- 只在出现频率接近和参与度交换时，使用“模态耦合/共振”表述。

### 优先级 4：风速阶跃扩大但保持 M0 线性区

当前 `0.01 m/s` 风速等效阶跃已经完成对齐图。可再做 `0.02` 和 `0.05 m/s` 两个点，检查：

- 频率误差是否仍小于 `1%`；
- `T_sh`、`T_e`、`Udc` 峰值误差是否仍小于 `2%`；
- 是否出现幅值随扰动增大而偏离的非线性。

这一步可以把文献中的“风速扰动验证”与当前 M0 的连续非线性—SSM 对齐正式接上，但仍不需要恢复 EMT 或 Pitch。

## 5. 与文献结论的最终判断

### 已经对应

- 两质量轴系约 `2.5 Hz` 的主模态；
- MSC-DVC、DC-link 和电磁转矩之间的关键耦合结构；
- 网侧扰动可以激发 GFM 风机轴系扭振；
- 连续非线性模型与小信号模型在小扰动下可重合；
- 频率主要由机械轴系决定，控制器更容易改变阻尼或激励。

### 部分对应

- DVC 增益使阻尼变化：当前已经看到方向，但还没有得到完整闭环复转矩和临界边界；
- H、SCR 对稳定性的影响：已有一维扫描，但还没有模态接近和二维稳定区域；
- VSG 比 Droop-GFM 更容易放大网侧扰动：当前残差证据支持，但尚未证明其本征阻尼一定更低。

### 尚未对应

- 文献中的完整 (W_d(\omega)) 负阻尼曲线；
- 扭振模态与 VSG/LFO 模态的根轨迹共振；
- DVC 类型 a/b/c 的公平对照；
- GSC 有功侧阻尼注入与 MSC 转矩侧阻尼注入的优劣；
- EMT、硬件或风电场级验证。本轮按当前研究边界暂不处理 EMT。

## 6. 当前最合理的论文主线

下一阶段不应再扩展模型，而应完成：

\[
\text{完整闭环复转矩}
\rightarrow
\text{DVC/SCR 稳定边界}
\rightarrow
\text{模态接近与残差交换}
\rightarrow
\text{风速阶跃 NL--SSM 对齐}
\]

完成后，论文的贡献可以严谨地分成两种可能：

1. **若完整复转矩随控制结构变化：** 证明 GFM 通过 DVC—DC-link—GSC 闭环改变轴系电气阻尼，并给出边界。
2. **若完整复转矩仍不变：** 证明在当前共同工作点，GFM 不改变轴系本征阻尼，而是通过外部网侧扰动残差塑形放大轴系响应；这将成为对“振荡幅值增大=阻尼下降”这一常见混淆的精确修正。
