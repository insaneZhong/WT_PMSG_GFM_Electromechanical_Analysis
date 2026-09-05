# 5 MW 理想连续平均模型：等效风速阶跃与模态残差分析

## 本阶段目标

在已通过 Gate A 的 GFL、GFM-GWT、GFM-MWT 三架构共同工作点上，比较等效风速阶跃的连续非线性模型与同源小信号模型，并用模态残差区分“极点阻尼”和“扰动激励强度”。

## 输入定义

- 工作点风速：12.2 m/s；
- 等效风速阶跃：0.01 m/s；
- 等效气动功率阶跃：11969.1 W；
- 阶跃时刻：0.1 s；
- 终止时间：10 s。

当前 M0 没有显式 Cp–lambda 风速状态，因此本阶段的“风速阶跃”是固定局部运行点下的等效气动功率输入，不代表重新运行 MPPT/Pitch 动态。

## 非线性—小信号响应对照

验收门槛为：普通输出峰值误差不超过 2%、NRMSE 不超过 5%；轴系主频误差用于辅助判断。三架构共 12 条输出记录中，11 条正常响应通过该门槛，GFM-GWT 的 `Delta T_e` 为数值零激励，单独标记为 `NO_EXCITATION`，不作为失败。

|Architecture|Signal|DeltaWind_mps|EquivalentDeltaPaero_W|StepTime_s|Peak_NL|Peak_SSM|PeakError_pct|NRMSE|Correlation|f_NL_Hz|zeta_NL|f_SSM_Hz|zeta_SSM|Final_NL|Final_SSM|Status|InputDefinition|
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|GFL|Delta omega_sh|0.01|11969.093|0.1|4.7160903e-05|4.7165666e-05|0.010100254|0.0014805|0.99999914|2.5|0.029977113|2.4942097|0.029810519|-1.7483359e-06|-1.7909681e-06|PASS|DeltaPaero=(dPaero/dV)atV0 times DeltaVwind|
|GFL|Delta T_sh|0.01|11969.093|0.1|0.010528497|0.01058515|0.53809325|0.0064380215|0.99999886|NaN|NaN|NaN|NaN|-0.010528497|-0.01058515|PASS|DeltaPaero=(dPaero/dV)atV0 times DeltaVwind|
|GFL|Delta T_e|0.01|11969.093|0.1|0.0130131|0.013076631|0.48821284|0.0076855495|0.99999914|NaN|NaN|NaN|NaN|-0.0130131|-0.013076631|PASS|DeltaPaero=(dPaero/dV)atV0 times DeltaVwind|
|GFL|Delta Udc|0.01|11969.093|0.1|0.63793897|0.63951949|0.24775396|0.020270263|0.99995344|NaN|NaN|NaN|NaN|0.51706349|0.52169784|PASS|DeltaPaero=(dPaero/dV)atV0 times DeltaVwind|
|GFM-GWT (GSC-DVC + MSC-MPPT/转矩)|Delta omega_sh|0.01|11969.093|0.1|4.7177032e-05|4.7186499e-05|0.020067173|0.00031956113|0.99999997|2.5|0.03050916|2.5028159|0.030444421|-5.0762155e-07|-5.0866633e-07|PASS|DeltaPaero=(dPaero/dV)atV0 times DeltaVwind|
|GFM-GWT (GSC-DVC + MSC-MPPT/转矩)|Delta T_sh|0.01|11969.093|0.1|0.0051811527|0.0051818222|0.012921263|0.00092030271|0.99999992|NaN|NaN|NaN|NaN|0.00063656805|0.00063647581|PASS|DeltaPaero=(dPaero/dV)atV0 times DeltaVwind|
|GFM-GWT (GSC-DVC + MSC-MPPT/转矩)|Delta T_e|0.01|11969.093|0.1|3.7252903e-15|1.3766071e-17|99.63047|1.7594024|5.6569993e-18|NaN|NaN|NaN|NaN|-9.3132257e-16|-1.2236044e-18|NO_EXCITATION|DeltaPaero=(dPaero/dV)atV0 times DeltaVwind|
|GFM-GWT (GSC-DVC + MSC-MPPT/转矩)|Delta Udc|0.01|11969.093|0.1|1.059708|1.0604182|0.067021505|0.0018776599|0.99999998|NaN|NaN|NaN|NaN|0.98629804|0.9869491|PASS|DeltaPaero=(dPaero/dV)atV0 times DeltaVwind|
|GFM-MWT (MSC-DVC + GSC-VSG)|Delta omega_sh|0.01|11969.093|0.1|4.7158071e-05|4.7165666e-05|0.016106369|0.0017550652|0.99999865|2.5|0.029977113|2.4942097|0.029810519|-1.7327254e-06|-1.7909681e-06|PASS|DeltaPaero=(dPaero/dV)atV0 times DeltaVwind|
|GFM-MWT (MSC-DVC + GSC-VSG)|Delta T_sh|0.01|11969.093|0.1|0.010528747|0.01058515|0.53569826|0.0064335514|0.99999886|NaN|NaN|NaN|NaN|-0.010528747|-0.01058515|PASS|DeltaPaero=(dPaero/dV)atV0 times DeltaVwind|
|GFM-MWT (MSC-DVC + GSC-VSG)|Delta T_e|0.01|11969.093|0.1|0.013013096|0.013076631|0.48824462|0.0076857484|0.99999914|NaN|NaN|NaN|NaN|-0.013013096|-0.013076631|PASS|DeltaPaero=(dPaero/dV)atV0 times DeltaVwind|
|GFM-MWT (MSC-DVC + GSC-VSG)|Delta Udc|0.01|11969.093|0.1|0.6379394|0.63951949|0.24768596|0.020269327|0.99995345|NaN|NaN|NaN|NaN|0.51706379|0.52169784|PASS|DeltaPaero=(dPaero/dV)atV0 times DeltaVwind|

## 模态残差对照

|Architecture|Disturbance|f_tor_Hz|zeta_tor|MechanicalParticipation|O_omega_abs|O_Tsh_abs|K_abs|Residue_omega_abs|Residue_Tsh_abs|StepResidue_omega_abs|StepResidue_Tsh_abs|Gamma_omega_vs_GFL|Gamma_Tsh_vs_GFL|PoleReal|PoleImag|InputAmplitude|InputUnit|Classification|
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|GFL|Mechanical torque|2.4942097|0.029810519|0.99638401|0.00046699873|25851.533|9.0136659e-05|4.2093705e-08|2.3301708|2.6847958e-09|0.14862158|1|1|-0.46738571|15.671582|12865|N m|GFL基准|
|GFL|Aerodynamic power|2.4942097|0.029810519|0.99638401|0.00046699873|25851.533|7.09738e-05|3.3144674e-08|1.8347815|2.114014e-09|0.11702495|1|1|-0.46738571|15.671582|25000|W|GFL基准|
|GFL|Grid angle|2.4942097|0.029810519|0.99638401|0.00046699873|25851.533|860.48161|0.40184381|22244769|0.025630165|1418802.7|1|1|-0.46738571|15.671582|0.0034906585|rad|GFL基准|
|GFL|Grid frequency|2.4942097|0.029810519|0.99638401|0.00046699873|25851.533|0.45037225|0.00021032327|11642.813|1.3414714e-05|742.59502|1|1|-0.46738571|15.671582|0.31415927|rad/s|GFL基准|
|GFM-GWT (GSC-DVC + MSC-MPPT/转矩)|Mechanical torque|2.5028159|0.030444421|1|3.9184086e-07|21.615378|0.10671535|4.1815435e-08|2.3066927|2.6578257e-09|0.14661541|0.98995449|0.98650147|-0.47898051|15.725656|12865|N m|轴系激励近似不变|
|GFM-GWT (GSC-DVC + MSC-MPPT/转矩)|Aerodynamic power|2.5028159|0.030444421|1|3.9184086e-07|21.615378|0.084027835|3.2925539e-08|1.8162934|2.0927761e-09|0.1154452|0.98995375|0.98650074|-0.47898051|15.725656|25000|W|轴系激励近似不变|
|GFM-GWT (GSC-DVC + MSC-MPPT/转矩)|Grid angle|2.5028159|0.030444421|1|3.9184086e-07|21.615378|0|0|0|0|0|0|0|-0.47898051|15.725656|0.0034906585|rad|轴系激励减弱|
|GFM-GWT (GSC-DVC + MSC-MPPT/转矩)|Grid frequency|2.5028159|0.030444421|1|3.9184086e-07|21.615378|0|0|0|0|0|0|0|-0.47898051|15.725656|0.31415927|rad/s|轴系激励减弱|
|GFM-MWT (MSC-DVC + GSC-VSG)|Mechanical torque|2.4942097|0.029810519|0.99638401|0.00046699873|25851.533|9.0136659e-05|4.2093705e-08|2.3301708|2.6847958e-09|0.14862158|1|1|-0.46738571|15.671582|12865|N m|轴系激励近似不变|
|GFM-MWT (MSC-DVC + GSC-VSG)|Aerodynamic power|2.4942097|0.029810519|0.99638401|0.00046699873|25851.533|7.09738e-05|3.3144674e-08|1.8347815|2.114014e-09|0.11702495|1|1|-0.46738571|15.671582|25000|W|轴系激励近似不变|
|GFM-MWT (MSC-DVC + GSC-VSG)|Grid angle|2.4942097|0.029810519|0.99638401|0.00046699873|25851.533|2361.333|1.1027395|61044077|0.070334278|3893477.2|2.7441992|2.7441992|-0.46738571|15.671582|0.0034906585|rad|轴系激励增强|
|GFM-MWT (MSC-DVC + GSC-VSG)|Grid frequency|2.4942097|0.029810519|0.99638401|0.00046699873|25851.533|150.60914|0.070334278|3893477.2|0.0044860193|248331.46|334.41035|334.41035|-0.46738571|15.671582|0.31415927|rad/s|轴系激励增强|

## 电磁转矩新稳态核验

对 GFL 与 GFM-MWT 将同一等效风速阶跃延长至 180 s。判定依据为：末 20 s 均值相对小信号新稳态误差不超过 0.5%、尾段半幅不超过 0.5%、尾段线性斜率绝对值不超过 0.01 kNm/s。

|Architecture|SettlingStop_s|DeltaTe_ss_SSM_MNm|DeltaTe_eq_NL_MNm|EqResidual_norm|Eq_vs_SSM_pct|DeltaTe_tailMean_NL_MNm|DeltaTe_tailRange_NL_kNm|TailSlope_NL_kNm_per_s|TailMean_vsEq_pct|TailOscillation_pct|DeltaTe_final_NL_MNm|DeltaTe_final_SSM_MNm|TailStart_s|TailEnd_s|MaxStablePoleReal|Conclusion|
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|GFL|180|-0.047343254|-0.046687387|1.2502896e-15|1.4048067|-0.046506535|0.11807672|-0.0058684417|0.38736669|0.12645462|-0.046559208|-0.047243571|160|180|-0.032677594|收敛至新稳态|
|GFM-MWT (MSC-DVC + GSC-VSG)|180|-0.04737093|-0.046687387|5.391874e-15|1.4640864|-0.046506535|0.1180767|-0.0058684407|0.38736672|0.1264546|-0.046559208|-0.047243571|160|180|-0.032677594|收敛至新稳态|

## 结果解释

1. 风速等效阶跃采用同一个物理输入同时作用于连续非线性方程和小信号方程，因此响应曲线可以直接用实线/虚线对照。
2. 模态残差为左/右特征向量形成的轴系模态输入—输出残差，并使用四类统一小扰动幅值归一化；它不是新的极点，而是该扰动对轴系模态的激励能力。
3. 因此，GFM 与 GFL 的差异必须分成两部分报告：轴系极点阻尼变化，以及同一轴系极点被不同扰动激励时的残差变化。不能仅用某一张时域响应图声称“GFM必然恶化轴系稳定性”。
4. 本阶段仍限于 M0 理想连续平均模型，不包含采样、PWM、数字延迟、限幅和 EMT 开关纹波。

## 本阶段结论

- 风速等效阶跃下，三种架构的非线性连续平均模型与同源小信号模型逐条对应，主要轴系频率约为 2.49–2.50 Hz，阻尼比约为 2.98%–3.05%。
- GFM-GWT 的机械/气动扰动残差约为 GFL 的 0.99，电网角度/频率扰动残差接近零；其主要作用表现为削弱电网侧扰动对轴系模态的激励。
- GFM-MWT 的机械/气动扰动残差基本不变，但电网角度和频率扰动残差分别约增大 2.74 倍和 334 倍。当前结果表明，GFM 对轴系响应的影响主要体现为扰动通道塑形，而不是在本工作点上必然改变轴系极点阻尼。
- 电磁转矩收敛性由上述表给出：若结论为“收敛至新稳态”，则图中的阶跃后永久偏置是新功率平衡，而非持续下降。

## 生成图片

- `Figures_Disturbance_Path/Fig11_WindStep_ThreeArchitecture_NL_SSM.png`：三架构风速等效阶跃，实线为连续非线性，虚线为小信号。
- `Figures_Disturbance_Path/Fig12_ModalResidue_ThreeArchitecture.png`：四类扰动的轴系模态残差相对 GFL 比值。
- `Figures_Disturbance_Path/Fig13_WindStep_ThreeArchitecture_Overlay_NL_SSM.png`：三架构同图叠加；右列仅添加视觉分离偏置，保证重合曲线可辨。
- `Figures_Disturbance_Path/Fig14_WindStep_Te_Settling_GFL_GFMMWT.png`：GFL 与 GFM-MWT 的电磁转矩向新稳态收敛情况。
