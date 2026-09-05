# Gate V3：真实数字控制器开放审计

生成时间：2026-09-04（S7-5B/C 执行后更新）

## 判定

**BLOCKED/CONDITIONAL**。本轮 S7-2/S7-3/S7-4 已在同源 Reference Digital Implementation 上完成；S7-5 B1–B6/LC2 已完成生产 C 与显式 Replica 的控制器事件映射，但 S7-5C 尚未在物理 plant 闭环上形成 Legacy 固定点，因此不能开放 S7B 开关 EMT。当前阻塞点是 plant 闭环固定点，不是 LC1/LC2 的公式或更新顺序不一致。

## S7-5 当前证据

|阶段|状态|范围与边界|
|---|---|---|
|B1–B6 LC1|PASS|MSC、GSC、P/Q 滤波、PLL/GFL、VSG、调度器逐模块 C↔Replica；不含物理 plant|
|LC2|PASS|nominal、Udc 小扰动、组合扰动各 100 个控制事件；最大逐项误差见 `temp/S7_5_LegacyCertification/S7_Legacy_LC2_FullMap_Validation.csv`|
|S7-5C 控制器探针|CONDITIONAL|旋转 abc 固定输入下 C 与 Replica 可重放；不是 PMSG/两质量/DC-link/电网固定点|
|S7-5C 物理固定点|BLOCKED|GSC 电压 PI 命令没有经 LCL/电网反馈，不能满足 `Phi_Legacy(X0)=X0`|
|S7-5D/E/F|NOT STARTED|按停止规则等待物理 Legacy 固定点|

## 已核对的遗留实现特征

|对象|来源|观察到的实现|对 Gate V3 的影响|
|---|---|---|---|
|MSC 电流/DVC|`CurrentModel_Idealized/motorcontrol_legacy_ad_base.c`|`motor_PI2_calc`、斜率限制、电压限幅、抗饱和状态|不能直接映射为当前 M0 的连续 PI 状态|
|GSC 电压/电流|`CurrentModel_Idealized/grid_forming_control.c`|采样控制调度、矢量/电压限幅、PI状态|更新前/更新后输出顺序仍需确认|
|GFM/VSG|同上|PLL、预同步、模式切换、`w_vsg_state` 状态更新|与 M0 的直接 GFM 工作点不等价|
|PWM/调制|同目录控制实现及 `svpwm.c`|PWM 调度、占空时间裁剪和调制限幅|不能在 V2 未对齐前作为实现证据|

## 与 M0 的关键差异

1. M0 保存模型为 `ode15s` 变步长、23个连续 Integrator，`M0_RHS` 为隐藏 `sf_sfun`；未提供可追溯的 Unit Delay/ZOH/离散 PI 链。
2. 遗留控制器包含 PLL/预同步/接管、限幅和PWM调度；M0 参考工作点直接处于连续 GFM 状态。
3. 旧 C 控制器中的采样时刻、PI 更新顺序和内部全局状态尚未能由当前 M0 工作点唯一反算。

## 因此当前能说什么

- `S7A_DiscreteAvg_5MW.slx` 证明了一个明确写出的离散平均映射可以在 31 状态下形成稳定固定点，并与其一步 Jacobian 小扰动响应一致。
- 九点参考筛查显示该候选数字实现的 TOR pole 近似不变；部分延迟点的网侧激励残差变化明显，属于待证伪的实现依赖性假设。
- 以上不能外推为“遗留 C 数字控制器已验证”，也不能外推为“EMT 下机制稳健”。

## 重新开放 Gate V3 的必要条件

1. ~~明确实际控制器每个状态的采样、更新和输出时序~~（已由 B1–B6/LC2 完成）；
2. 在连续物理 plant 闭环中求解 Legacy 数字固定点，并保存 `physical_closure` 与功率/转矩/直流能量残差；
3. 从该固定点建立 `Ad/Bd`，重复 V2 的固定点、D1/D2/D3 小扰动、Pole/Excitation 指标；
4. 通过后才建立最小开关 EMT，且第一版仍关闭死区、限流、LVRT、噪声和保护。

在这些条件满足前，S7B 保持 `NOT_STARTED`，不把参考模型结果写成最终论文的数字/开关实现结论。
