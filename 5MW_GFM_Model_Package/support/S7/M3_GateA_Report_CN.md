# M3 Gate A：物理一致跨运行点与反例搜索报告

## 执行边界

本轮只完成 S0 与 S1。未执行 H/DVC/SCR 控制扫参、目标模态混合搜索、Pitch/OpenFAST、离散平均模型或 EMT。三个架构共享 plant、目标P/Q、Udc、转速和机械转矩，只改变控制职责。

## Gate A 工作点

- 工况：P0/PN = 0.3, 0.5, 0.7, 0.9；
- 最大平衡残差：4.953e-13；
- 最大架构间工作点差异：1.745e-07 pu；
- 严格共同工作点 Gate：PASS。

运行点采用通过额定M2点校准的常TSR曲线：P~omega^3、Tm~omega^2；同时按目标P/Q重算GFM电压幅值，并统一标定机械转矩以补偿PMSG与滤波器损耗。

## 功率与符号审计

- M3采用 `GENERATOR_OUTWARD`：正iq表示从PMSG流向DC-link的正发电电流，正Te表示发电制动转矩。
- 物理恒等式 `P_MSC = T_e*omega_g - P_Cu`：PASS；最大残差 1.146e-07 W。
- 冻结旧M2的加铜耗关系残差最大为 3.264e+05 W，仅保留作反事实参考。

机器侧电流、转矩及DC-link输入功率方向已经在同一发电机外向端口约定下闭合；旧M2数值只作为冻结历史基准，不再作为M3物理结论。

## 主要结果

- GFL：f_tor 2.4991--2.5010 Hz，zeta_tor 4.907%--5.881%，Ldir -3.490---3.187。
- GWT：f_tor 2.4992--2.5009 Hz，zeta_tor 4.914%--5.884%，Ldir -0.108--0.838。
- MWT：f_tor 2.4994--2.5002 Hz，zeta_tor 2.940%--2.963%，Ldir 2.066--2.690。

- GFL/GWT局部MPPT增量阻尼在4个工况均为正；本轮未找到符号反例。
- MWT的MSC-DVC增量阻尼在4个工况均为负；本轮未找到符号反例。
- GWT的Ldir由低功率正值降至0.9 pu附近负值，出现A-C2方向跨零反例；“GWT始终Grid-to-Machine占优”不成立。
- 模态身份由MAC和机械参与度联合检查；任何失败行必须先人工复核，不能按频率最近强制续接。

## 反例登记

|CounterexampleID|Architecture|Test|Detected|Evidence|Interpretation|
|---|---|---|---|---|---|
|A-C1|MWT|MSC-DVC damping sign reversal|false|DeltaDe range [-202649, -80247.3]|No sign reversal in tested workpoints|
|A-C1|GFL|Local MPPT damping sign reversal|false|DeltaDe range [4.26231e+06, 6.24266e+06]|No sign reversal in tested workpoints|
|A-C1|GWT|Local MPPT damping sign reversal|false|DeltaDe range [4.27163e+06, 6.24812e+06]|No sign reversal in tested workpoints|
|A-C2|GFL|Directional dominance reversal|false|Ldir range [-3.48976, -3.18683]|Direction ordering retained in tested workpoints|
|A-C2|GWT|Directional dominance reversal|true|Ldir range [-0.107952, 0.83805]|Direction crosses equality boundary|
|A-C2|MWT|Directional dominance reversal|false|Ldir range [2.06563, 2.68953]|Direction ordering retained in tested workpoints|
|A-C3|GWT|Pole/excitation class migration: MechanicalTorque|true|EXCITATION_DOMINATED;JOINT|Response-difference mechanism migrates|
|A-C3|GWT|Pole/excitation class migration: GridFrequency|false|EXCITATION_DOMINATED|Classification retained|
|A-C3|MWT|Pole/excitation class migration: MechanicalTorque|false|POLE_DOMINATED|Classification retained|
|A-C3|MWT|Pole/excitation class migration: GridFrequency|false|EXCITATION_DOMINATED|Classification retained|
|A-C4|GFL|Torsional mode identity change|false|minimum MAC 0.987443; minimum PiMECH 0.999608|Torsional identity retained|
|A-C4|GWT|Torsional mode identity change|false|minimum MAC 0.858632; minimum PiMECH 0.999143|Torsional identity retained|
|A-C4|MWT|Torsional mode identity change|false|minimum MAC 0.952702; minimum PiMECH 0.993389|Torsional identity retained|

## 反事实 Pole–Excitation 分解

|WorkpointScale|Architecture|Disturbance|Lambda0Real|Lambda0Imag|Lambda1Real|Lambda1Imag|Residue0|Residue1|InputProjection0|InputProjection1|DeltaPoleNorm|DeltaExcitationNorm|DeltaTotalNorm|DeltaInteractionNorm|PoleFraction|ExcitationFraction|InteractionFraction|Classification|
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|0.3|GWT|MechanicalTorque|-0.77203876|15.714214|-0.77307903|15.713756|4.1826157e-08|4.1840901e-08|0.13149021|0.43199688|2.765366e-07|6.6246683e-07|7.8752127e-07|1.674505e-10|0.35114811|0.84120501|0.00021262982|EXCITATION_DOMINATED|
|0.3|MWT|MechanicalTorque|-0.77203876|15.714214|-0.46198402|15.709095|4.1826157e-08|4.1862945e-08|0.13149021|0.060680654|0.00010637278|2.1905992e-05|0.00011383939|1.7825285e-06|0.93441103|0.19242893|0.015658276|POLE_DOMINATED|
|0.3|GWT|GridFrequency|-0.77203876|15.714214|-0.77307903|15.713756|2.876872e-07|0.0011879586|0.90441132|12265.377|1.008054e-10|0.00076856242|0.00076834959|4.1196611e-07|1.3119731e-07|1.000277|0.00053617014|EXCITATION_DOMINATED|
|0.3|MWT|GridFrequency|-0.77203876|15.714214|-0.46198402|15.709095|2.876872e-07|0.051466226|0.90441132|74600.683|3.8398417e-08|0.067207515|0.06957796|0.0068810702|5.5187616e-07|0.9659311|0.098897269|EXCITATION_DOMINATED|
|0.5|GWT|MechanicalTorque|-0.83552983|15.709873|-0.83640939|15.709311|4.1839216e-08|4.1853261e-08|0.19430753|0.47386592|1.916408e-07|2.3522174e-07|3.3524643e-07|7.546499e-11|0.57164159|0.70163832|0.00022510304|JOINT|
|0.5|MWT|MechanicalTorque|-0.83552983|15.709873|-0.46351218|15.706081|4.1839216e-08|4.188322e-08|0.19430753|0.082367371|0.00010118177|2.1500358e-05|0.00010890118|1.9689223e-06|0.92911546|0.19742998|0.018079898|POLE_DOMINATED|
|0.5|GWT|GridFrequency|-0.83552983|15.709873|-0.83640939|15.709311|3.1294003e-07|0.001050823|1.4533399|11897.501|9.1737911e-11|0.0011893393|0.0011892625|2.9014126e-07|7.7138485e-08|1.0000645|0.00024396737|EXCITATION_DOMINATED|
|0.5|MWT|GridFrequency|-0.83552983|15.709873|-0.46351218|15.706081|3.1294003e-07|0.04420458|1.4533399|86932.547|4.7221363e-08|0.061469761|0.063595418|0.0066277195|7.4252776e-07|0.96657532|0.10421693|EXCITATION_DOMINATED|
|0.7|GWT|MechanicalTorque|-0.88433136|15.706067|-0.88500221|15.705813|4.1851247e-08|4.1855841e-08|0.25229752|0.48769437|1.104866e-07|1.459174e-07|2.1106593e-07|1.9660657e-11|0.5234696|0.69133565|9.3149364e-05|JOINT|
|0.7|MWT|MechanicalTorque|-0.88433136|15.706067|-0.46463645|15.704829|4.1851247e-08|4.1901078e-08|0.25229752|0.10133062|9.7587877e-05|2.1401338e-05|0.0001055868|2.1103593e-06|0.92424311|0.20268951|0.01998696|POLE_DOMINATED|
|0.7|GWT|GridFrequency|-0.88433136|15.706067|-0.88500221|15.705813|3.1726815e-07|0.00085796536|1.9126304|9996.8098|5.9506843e-11|0.0011541252|0.0011540856|1.4981449e-07|5.1561897e-08|1.0000343|0.00012981228|EXCITATION_DOMINATED|
|0.7|MWT|GridFrequency|-0.88433136|15.706067|-0.46463645|15.704829|3.1726815e-07|0.03819942|1.9126304|92378.786|5.2018162e-08|0.055031692|0.056901884|0.006150799|9.1417293e-07|0.96713305|0.10809482|EXCITATION_DOMINATED|
|0.9|GWT|MechanicalTorque|-0.92504203|15.702554|-0.92556928|15.70276|4.1862706e-08|4.1860667e-08|0.30639641|0.52083571|7.4754578e-08|1.7514222e-07|2.1376507e-07|1.4983643e-11|0.34970436|0.81932104|7.0093971e-05|EXCITATION_DOMINATED|
|0.9|MWT|MechanicalTorque|-0.92504203|15.702554|-0.46555758|15.704384|4.1862706e-08|4.1915405e-08|0.30639641|0.11926395|9.4840467e-05|2.1500039e-05|0.00010314265|2.2356426e-06|0.91950779|0.20844956|0.02167525|POLE_DOMINATED|
|0.9|GWT|GridFrequency|-0.92504203|15.702554|-0.92556928|15.70276|3.3763312e-07|0.00070583419|2.4711632|8782.0782|4.6419725e-11|0.0010155353|0.0010154908|9.5868514e-08|4.5711616e-08|1.0000438|9.440609e-05|EXCITATION_DOMINATED|
|0.9|MWT|GridFrequency|-0.92504203|15.702554|-0.46555758|15.704384|3.3763312e-07|0.033246938|2.4711632|94599.141|5.8927597e-08|0.048949725|0.050594001|0.0056349915|1.1647151e-06|0.96750058|0.11137667|EXCITATION_DOMINATED|

## Gate 决策

S1 PASS：严格共同工作点、全部极点稳定、模态身份连续以及PMSG发电端口功率恒等式均通过。允许进入S2稀疏控制参数边界扫描；所有本轮机制判断仍保持 `CONDITIONAL_CROSS_WORKPOINT`，不得外推到未测试参数或更高保真模型。
