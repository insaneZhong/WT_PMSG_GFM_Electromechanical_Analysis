# 5 MW构网型PMSG理想连续模型—小信号严格对齐报告

生成时间：2026-08-10 22:20:58

## 结论

总体结果：**PASS**。非线性Simulink模型和小信号模型共用同一个连续RHS、参数向量和严格平衡点。

## 模型边界

- 23个显式连续状态，Type-A MSC-DVC，真正VSG，相对功角，连续P/Q滤波，GSC电压/电流双环和LCL。
- 已删除PWM/SVPWM、采样调度、数字延迟、全部限幅与anti-windup、PLL/预同步、主动阻尼、MPPT/Pitch动态。
- GSC电流环/电压环带宽为 300/30 Hz；电压环使用 +i_g 前馈，PCC电压前馈关闭。
- COI阻尼为 0.12 pu，按惯量比例施加到同一COI速度，只锚定公共转速，不进入相对轴系方程。

## 严格5 MW工作点与能量关系

- P_MSC = 5.025979149 MW；P_GSC = 5.025979149 MW；P_PCC = 5.000000000 MW。
- Udc = 1500.000000 V；delta = 0.255900963 rad；平衡点最大归一化残差 = 2.951e-10。
- dP_PCC/ddelta = 18220867.1 W/rad（正），物理VSG功率误差符号为 +1。

## 稳定性

- 最大极点实部 = -0.00724970187 1/s。
- 轴系模态频率 = 2.482747520 Hz；阻尼比 = 2.710608%。

## 非线性—小信号对齐

扰动为 dPref = 0.0001 pu，比较时间 10 s。所有原始时序只在内存中使用。

|信号|NRMSE|峰值误差|末值误差|通过|
|---|---:|---:|---:|:---:|
|P_PCC_W|2.38264e-06|9.60641e-07|8.95767e-08|PASS|
|Udc_V|7.01017e-05|0.000102403|0.000116676|PASS|
|Tgen_Nm|0.000148885|0.000316279|0.000316279|PASS|
|Tshaft_Nm|0.000150626|0.000318447|0.000318447|PASS|
|omega_rel_radps|8.5928e-05|0.000276816|1.35875e-05|PASS|
|omega_vsg_radps|5.82086e-06|5.18977e-07|2.59201e-07|PASS|

无扰动最大状态漂移：2.348e-12 pu。

## 验收门

- `structure`：PASS
- `equilibrium_and_energy`：PASS
- `positive_power_angle_slope`：PASS
- `all_poles_stable`：PASS
- `no_disturbance`：PASS
- `nonlinear_linear_alignment`：PASS
