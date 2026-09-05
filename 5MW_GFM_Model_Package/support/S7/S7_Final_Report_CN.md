# S7 电气实现保真度验证阶段报告

生成时间：2026-09-04 15:25:00（S7-5B/C 执行后更新）

## 总体状态

- S6：**INCONCLUSIVE**，不因本轮结果升级。
- S7-0参考冻结：**COMPLETE**。
- S7A同源采样平均筛查：**EXECUTED**。
- Gate V1连续极限：**PASS**。
- S7-2/V2：**CONDITIONAL_REFERENCE_PASS**，已建立同源参考离散平均Simulink副本并完成固定点、一步映射和D1/D2/D3小扰动对照；尚未证明遗留C控制器的真实数字实现。
- S7-3：**CONDITIONAL_REFERENCE_SCREENING**，已完成Ts/Ts0=[0.5,1,2]与tau/Ts=[0,0.5,1]九点筛查。
- S7-4：**CONDITIONAL_REFERENCE_SCREENING**，已完成Pole/Excitation模态指标增量复核；跨阶时域反事实未虚构。
- S7-5 L0--L3：**COMPLETE（静态审计）**，已完成遗留 C/S-Function 调用链、隐藏状态、角度/坐标/功率符号和 Reference--Legacy 差异矩阵；生产默认宏与理想化编译宏已分开记录。
- S7-5 LC1/LC2：**CONTROLLER-LAYER PASS**。B1–B6 模块级 C↔Replica 与 LC2 三工况/100事件完整映射均已通过；这不是整机物理固定点或 `Ad/Bd` 认证。
- S7-5C：**BLOCKED（缺少物理闭环）**。控制器探针已运行，但固定 PCC 输入下 GSC 电压 PI 无法通过 LCL/电网反馈闭合。
- Gate V3：**BLOCKED/CONDITIONAL**，在 Legacy 控制器接回连续物理 plant 并通过固定点、离散 SSM 和 NL–SSM 验证前，不进入 EMT。
- S7B开关EMT：**NOT STARTED**。

## 参考边界

模型：`M0_PMSG_GFM_5MW.slx`；状态数：23；额定功率：5 MW；标称采样：0.0001 s。
工作点：P_MSC=5.025979149 MW，P_GSC=5.025979149 MW，P_PCC=5.000000000 MW，Udc=1500.000000 V。

## 方法说明

采样平均筛查器从同一23状态RHS计算采样时刻控制命令，控制器积分状态按前向Euler更新，变流器命令经ZOH保持，plant使用连续ODE积分；延迟通过一个采样区间内旧/新命令分段保持实现。该实现只用于S7A筛查，不能替代真实C/S-Function数字控制器或开关EMT。

## M0离散接口审计

当前保存的 M0 使用 `ode15s` 变步长求解器和 23 个连续 Integrator；`M0_RHS` 在保存模型中为隐藏的 `sf_sfun` S-Function。模型内未发现可追溯的 Unit Delay、Zero-Order Hold 或离散 PI 更新链，因此不能仅通过修改求解器设置得到计划中的真实离散平均模型。

旧 C/S-Function 控制器已单独列入 `S7_Digital_Controller_Manifest.csv`，但其 PLL/模式切换/限幅/PWM 调度与 M0 连续方程不对齐，只能作为实现审计来源，不能用于 Gate V2。

## S7-1状态映射审计

`S7_Controller_State_Audit.csv` 已覆盖全部23个状态：12个物理连续状态采用ODE，11个软件/控制状态给出Forward-Euler候选差分式。该表满足候选映射可追溯性，但由于当前保存模型没有真实离散控制块，Gate G7-1仍标记为 CONDITIONAL，不能据此声称已复刻实际C控制器。

状态审计文件：`S7_Controller_State_Audit.csv`；控制器离散化假设文件：`Controller_Discretization_Manifest.csv`。

## S7A数值摘要

连续极限三点的最大轴系频率误差为 0.000111349%，最大轴系阻尼代理误差为 6.03651e-06；三点均保持离散稳定。
A2延迟筛查：Td/Ts=0、0.5、1、1.5 均未出现离散不稳定，轴系频率范围 2.482747--2.482749 Hz，阻尼代理范围 0.422993--0.42301 /s。
A3采样筛查：Ts/Ts0=0.5、1、2、4 均未出现离散不稳定；Ts/Ts0=4 被标为 STRESS，不作为工程主结论。

## S7-2/V2数值摘要

参考离散平均模型 `S7A_DiscreteAvg_5MW.slx` 为31状态、29输出，固定点最大残差为 `1.4753e-7`（按工作点状态归一化为 `2.9788e-14`）；D1/D2/D3全部极点位于单位圆内，TOR模态约 `2.482747 Hz`、阻尼比约 `0.027106`。采用0.05%物理基值小扰动时，机械转矩扰动的关键输出峰值误差低于0.01%，电网频率扰动的关键输出最大NRMSE约1.52%、峰值误差约3.42%；D1幅值线性验收（0.025/0.05/0.1% pu）最大斜率偏差约0.0012%，通过5%门槛。这些数值证明的是“参考离散映射与其一步线性化的一致性”，不是旧C控制器复现。

## S7-3/S7-4数值摘要

九个Reference数字点均保持稳定，TOR频率变化小于 `1.1e-6 Hz`、阻尼比变化小于 `1.0e-6` 量级；在统一Ts尺度换算后，机械扰动残差变化约 `1.2e-4` 以内，网侧残差在半采样延迟至一拍延迟的多数点变化约几个百分点，但零延迟点存在约48%的残差变化。该结果只能作为“参考数字实现下的Pole/Excitation筛查”，不能直接推出真实数字控制器的实现鲁棒性。

## S7-5 遗留控制器重构进展（2026-09-04）

### L0--L3 审计：完成

已对 `CurrentModel_Idealized` 中的 `main_legacy_ad_base.c`、`motorcontrol_legacy_ad_base.c`、`grid_forming_control.c`、`svpwm.c` 及相关头文件完成：

- 调用链和多速率调度审计；
- C 全局/结构体隐藏状态与更新顺序审计；
- PLL、预同步、GFL/GFM 接管及 VSG 角度坐标审计；
- Clarke/Park、P/Q 正负号和参考 Reference--Legacy 差异矩阵。

对应文件：

- `S7_Legacy_Controller_CallGraph.md`
- `S7_Legacy_Controller_State_Audit.csv`
- `S7_Legacy_Angle_Frame_Audit.md`
- `S7_Reference_vs_Legacy_Controller_Matrix.csv`

### 生产 MEX 烟雾检查：通过，但不等于对齐

使用未加理想化宏的源码配置，在临时目录生成并加载了：

`temp/S7_5_LegacyProduction/main_legacy_ad.mexw64`

该 MEX 通过 1 us、0.15 ms 的最小 S-Function 仿真；在 `Udc=900 V`、`Vdc_ref=1000 V`、DVC 已使能的条件下，第一次 DVC 更新输出为 `5.00250483`，与源码 `motor_PI2_calc` 的预期 `5.00250500` 相差 `1.74e-7`，摘要见：

`temp/S7_5_LegacyProduction/S7_5_LC2_DVC_FirstStep_Summary.csv`

这一步是生产 MEX 可调用和 DVC 单步公式的局部证据；随后 B1–B6/LC2 已完成整个控制器事件层的 C↔Replica 映射（详见 `temp/S7_5_LegacyCertification/`）。它仍不等同于 Legacy 与物理 plant 的联合固定点。

### 当前 Gate

`S7-5 L0--L3 = COMPLETE（静态审计）`；`B1--B6 LC1 = PASS`；`LC2 = PASS（控制器事件层）`。对应摘要位于 `temp/S7_5_LegacyCertification/`：

- `S7_Legacy_LC1_B1_Summary.csv` 至 `S7_Legacy_LC1_B6_Summary.csv`：各模块逐步 C↔Replica 通过；
- `S7_Legacy_LC2_FullMap_Validation.csv`：nominal、Udc 小扰动、组合扰动均通过；
- `S7_Legacy_FixedPoint.csv`：S7-5C 控制器探针记录为 `CONDITIONAL_CONTROLLER_PROBE`，`plant_fixed_point=0`，`physical_closure=0`。

因此仍不能进入 S7-5D（Legacy 离散 SSM）：计划要求的 `Phi_Legacy(X0)=X0` 必须在连续物理 plant 闭环上成立，不能用固定 PCC 序列替代。`Gate V3` 仍为 **BLOCKED/CONDITIONAL**，S7B 开关 EMT 仍不开放。

## 文件

- `D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\5MW_Ideal\M3_Workpoint_Generalization\S7_Reference_Manifest.csv`
- `D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\5MW_Ideal\M3_Workpoint_Generalization\S7_Digital_Controller_Manifest.csv`
- `Controller_Discretization_Manifest.csv`
- `S7A_Continuous_Limit_Gate.csv`
- `S7A_Delay_Scan_Summary.csv`
- `S7A_Sampling_Scan_Summary.csv`
- `S7A_Discrete_Mode_Tracking.csv`
- `S7A_Feedback_Damping.csv`
- `S7A_Excitation_Comparison.csv`
- `S7A_Boundary_Summary.csv`
- `S7_V2_Discrete_Modes.csv`
- `S7_V2_NL_SSM_Validation.csv`
- `S7_V2_Amplitude_Linearity.csv`
- `S7_V2_FixedPoint_Report_CN.md`
- `S7_S3_Reference_Digital_9Point.csv`
- `S7_S3_Modal_Identity_Summary.csv`
- `S7_S4_Pole_Excitation_Decomposition.csv`
- `S7_S3_S4_Report_CN.md`
- `S7_GateV3_Audit_CN.md`
- `S7_Final_Mechanism_Evidence_Matrix.csv`
- `run_s7_discrete_validation.m`
- `build_s7a_discrete_average_model.m`
- `s7a_discrete_average_core.m`
- `s7a_discrete_average_sfun.m`
- `run_s7_v2_validation.m`
- `run_s7_s3_s4_analysis.m`

## 解释纪律

1. 任何A2/A3极点移动只能作为实现依赖性筛查，不称为物理机制。
2. 若V1失败，必须先修正采样映射，不得讨论负阻尼或方向性。
3. 当前V2是 `CONDITIONAL_REFERENCE_PASS`：只能说明同源参考离散映射与其线性化相符，不能写成“遗留数字控制已验证”。
4. 在S7B通过前，不能写成“开关实现稳健”。
5. S7-5 的 B1–B6/LC2 通过证明了生产 C 与显式 Replica 的控制器事件映射；S7-5C 尚未证明 Legacy 与物理 plant 的联合固定点，因此不得进入 EMT 或把 Reference 结果归因于 Legacy。
