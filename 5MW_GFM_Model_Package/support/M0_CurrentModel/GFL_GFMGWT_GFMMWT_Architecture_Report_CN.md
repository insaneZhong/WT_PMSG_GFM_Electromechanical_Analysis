# GFL–GFM-GWT–GFM-MWT 公平对比报告

## 目标与结构

本阶段仅使用同一个5 MW理想连续平均M0状态方程，不恢复PWM、离散、数字延迟、限幅或EMT。三种架构保持机械、PMSG、MSC/GSC内环、DC-link、LCL和电网参数一致，仅改变同步与有功控制分配。

## Gate A 工作点

|Architecture|P_PCC_MW|Q_PCC_Mvar|Udc_V|omega_g_radps|Te_MNm|Tsh_MNm|P_MSC_MW|P_GSC_MW|P_Mismatch_W|Torque_Mismatch_Nm|Residual_norm|Max_abs_dx|GateA|
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|GFL|5|2.8691e-13|1500|1.27|3.8326|3.8326|5.0314|5.0314|0|0.103|1.2503e-09|1.2503e-09|true|
|GFM-GWT (GSC-DVC + MSC-MPPT/转矩)|5|-1.265e-07|1500|1.27|3.8326|3.8326|5.0314|5.0314|-5.7742e-08|4.6566e-10|4.9299e-08|4.9299e-08|true|
|GFM-MWT (MSC-DVC + GSC-VSG)|5|2.8691e-13|1500|1.27|3.8326|3.8326|5.0314|5.0314|0|0.103|1.2503e-09|1.2503e-09|true|

## 轴系极点与复转矩

|Architecture|ShaftEigenvalueReal|ShaftEigenvalueImag|f_tor_Hz|zeta_tor|MechanicalParticipation|MaxRealPole|GTeOmegaReal|GTeOmegaImag|Ke_at_ftor|TeThetaGain_at_ftor|TeFreqGain_at_ftor|dP_dDelta_W_per_rad|
|---|---|---|---|---|---|---|---|---|---|---|---|---|
|GFL|-0.46739|15.6716|2.4942|0.029811|0.99638|0|-168723.3336|755730.2193|-11843487.9568|-11843487.9568|774335.6686|40391.7409|
|GFM-GWT (GSC-DVC + MSC-MPPT/转矩)|-0.47898|15.7257|2.5028|0.030444|1|-0.10087|7.3554e-08|9.1619e-09|-1.4408e-07|-1.4408e-07|7.4123e-08|40391.867|
|GFM-MWT (MSC-DVC + GSC-VSG)|-0.46739|15.6716|2.4942|0.029811|0.99638|-0.032914|-168723.3336|755730.2193|-11843487.9568|-11843487.9568|774335.6686|40391.7409|

## 网侧相角小扰动

|Architecture|GridAngle_deg|omegaRelPeak|TshPeak|TePeak|PpccPeak_MW|f_est_Hz|zeta_est|peak_ratio_to_GFL|Status|
|---|---|---|---|---|---|---|---|---|---|
|GFL|0.2|0.00024801|109284.0486|122647.2614|0.068093|2.4868|0.035074|1|PASS|
|GFM-GWT (GSC-DVC + MSC-MPPT/转矩)|0.2|3.0756e-13|1.6218e-05|9.7869e-05|0.116|2.5028||1.2401e-09|NO_TORSIONAL_EXCITATION|
|GFM-MWT (MSC-DVC + GSC-VSG)|0.2|0.00034666|20151.5114|14659.309|0.11601|2.4935|0.014675|1.3978|PASS|

## 文献对应关系

Liu等人的对照将GFM-GWT（GSC-DVC、MSC-MPPT/转矩）与GFM-MWT（MSC-DVC、GSC-GFM）区分开来；本报告正是对这两个架构在同一连续M0中的复现层。若两者轴系阻尼不同，应归因于DVC位置和DC-link能量通道，而不能只归因于“GFM”三个字。

## 当前结论

工作点Gate A=1。GFM-GWT与GFM-MWT的极点、轴系频率和电气转矩反馈已被分开记录；最终结论以表格和图10的网侧扰动响应为准。当前结果属于理想连续模型证据，不等同于开关EMT结论。
