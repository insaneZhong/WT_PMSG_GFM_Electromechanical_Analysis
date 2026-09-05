# M3 S5B 候选整定物理/数值审计

## 审计目标

本轮不覆盖原始 Kp=10 的失败证据，只判断 Kp=3（30 deg/(rad/s)）是否足以冻结为 S6 公共额定区基准。连续风速加密扫描、三架构公平性、Pitch断环重构和Kp稳定区间均由同一M3方程生成；没有保存时序。

## Gate结果

- 加密数值稳定性：PASS（183案例，最弱稳定极点 -0.000237168 1/s）；
- 三架构共同工作点公平性：PASS；
- Pitch断环重构：PASS；
- Kp稳定区间距离：PASS；
- 参数来源与执行器物理性：FAIL；
- **总体：FAIL**。

## 控制器尺度审计

- Ki=0.01 deg/rad，PI零点 5.30516e-05 Hz，对应时间常数 3000 s；
- NREL 5 MW基准控制器折算到低速轴的量级参考：Kp约 104.634 deg/(rad/s)，Ki约 44.843 deg/rad；该参考只做量级核对，不直接替代当前直驱PMSG重新设计；
- 当前候选未包含Pitch执行器滞后、8 deg/s速率限制、角度限位和增益调度。

官方参考：[NREL 5-MW Baseline Wind Turbine](https://www.nrel.gov/docs/fy09osti/38060.pdf)；[OpenFAST DISCON baseline controller](https://github.com/OpenFAST/openfast/blob/main/share/discon/DISCON.F90)。

## 代表点断环诊断与Kp区间

|WindFactor|Architecture|MarginEquilibriumResidual|OpenLoopPitchPlantMaxReal|ClassicalMarginApplicable|PitchLoopGainMarginFactor|PitchLoopPhaseMargin_deg|GainMarginFrequency_Hz|PhaseCrossover_Hz|PitchClosedLoopBandwidth_Hz|PitchLoopClosurePoleMismatch|LoopClosure_PASS|KpLowerCriticalScale|KpUpperCriticalScale|KpLowerMarginFactor|KpUpperMarginFactor|KpIntervalStatus|KpIntervalMargin_PASS|
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|1.05|GFL|1.9929736e-13|-0.05514372|true|7.6335204e-05|132.13629|0|4.8020258e-05|2.5807067e-05|6.6246957e-10|true|NaN|NaN|Inf|Inf|NOT_LIMITING_ARCHITECTURE|true|
|1.05|GWT|7.4235946e-15|-0.055147773|true|0.75005826|-32.132181|1.3401075|1.275578|10.259611|6.2818094e-10|true|NaN|NaN|Inf|Inf|NOT_LIMITING_ARCHITECTURE|true|
|1.05|MWT|1.687891e-15|0.15282248|false|0.70075121|-36.485042|1.3416263|1.269953|0.033629559|6.6626346e-08|true|0.4231676|NaN|7.0893896|Inf|LOWER_BOUNDARY_ONLY_NO_UPPER_TO_12|true|
|1.15|GFL|1.9929736e-13|-0.025155911|true|0.00036268175|98.25175|0|7.7148664e-06|6.7635811e-06|5.3871296e-11|true|NaN|NaN|Inf|Inf|NOT_LIMITING_ARCHITECTURE|true|
|1.15|GWT|7.4235946e-15|-0.025156467|true|3.698459|Inf|1.3401076|NaN|10.536722|2.1664226e-11|true|NaN|NaN|Inf|Inf|NOT_LIMITING_ARCHITECTURE|true|
|1.15|MWT|4.032184e-15|0.18310827|false|3.4624058|Inf|1.3416319|NaN|0.028311816|8.974391e-09|true|2.4117427|NaN|1.2439138|Inf|LOWER_BOUNDARY_ONLY_NO_UPPER_TO_12|true|
|1.25|GFL|1.9929736e-13|-0.092884983|true|0.00027160328|100.65415|0|9.9969889e-06|8.4762923e-06|1.1905168e-11|true|NaN|NaN|Inf|Inf|NOT_LIMITING_ARCHITECTURE|true|
|1.25|GWT|7.4235946e-15|-0.092817105|true|2.5251149|Inf|1.3401072|NaN|10.490585|1.1504299e-11|true|NaN|NaN|Inf|Inf|NOT_LIMITING_ARCHITECTURE|true|
|1.25|MWT|5.4136589e-15|0.11362538|false|2.3504888|Inf|1.3416102|NaN|0.02919385|1.0911295e-08|true|1.1117246|NaN|2.6985101|Inf|LOWER_BOUNDARY_ONLY_NO_UPPER_TO_12|true|
|1.35|GFL|1.9929736e-13|-0.10207122|true|0.00020445126|103.56616|0|1.2815148e-05|1.044594e-05|9.3569819e-11|true|NaN|NaN|Inf|Inf|NOT_LIMITING_ARCHITECTURE|true|
|1.35|GWT|7.4235946e-15|-0.10074601|true|1.7411255|Inf|1.3401066|NaN|10.438191|4.5072168e-11|true|NaN|NaN|Inf|Inf|NOT_LIMITING_ARCHITECTURE|true|
|1.35|MWT|1.0705605e-14|0.04463653|false|1.6081736|Inf|1.34165|NaN|0.030252619|1.0399302e-08|true|0.32992432|NaN|9.0929944|Inf|LOWER_BOUNDARY_ONLY_NO_UPPER_TO_12|true|

传统单环 margin 只作为诊断：断开的Pitch plant包含公共转速积分/既有多环动态；当 `ClassicalMarginApplicable=false` 时，不得用传统相位裕度或增益裕度作Gate。正式数值Gate采用闭环极点重构误差和Kp稳定区间距离。

## 结论边界

Kp=30候选若通过加密稳定性与Kp区间Gate，只能证明当前**无执行器的连续代数Pitch模型**在测试域内数值稳定。因为候选增益不是冻结参数源，且省略了实际Pitch执行器动态，不能据此把它升级成物理可信的额定区公共基准。原Kp=10反例与本候选必须并列保留。

## 决策

S5B候选未通过物理基准Gate，S6继续阻塞。下一步应使用可追溯的增益调度Pitch控制器和显式执行器一阶动态重建额定区基准，而不是继续单独放大Kp。
