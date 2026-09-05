# M3 S5B 候选整定物理/数值审计

## 审计目标

本轮不覆盖原始 Kp=10 的失败证据，只判断 `NREL5MW_SCHEDULED_LSS`、增益比例 1（Kp 59.781--59.781 deg/(rad/s)，Ki 25.6204--25.6204 deg/rad）是否足以冻结为 S6 公共额定区基准。连续风速加密扫描、三架构公平性、Pitch断环重构和增益稳定区间均由同一M3方程生成；没有保存时序。

## Gate结果

- 加密数值稳定性：PASS（3案例，最弱稳定极点 -0.080316 1/s）；
- 三架构共同工作点公平性：PASS；
- Pitch断环重构：PASS；
- 增益稳定区间距离：PASS；
- 参数来源可追溯且适用于S6命令级分析：PASS；
- Pitch执行器/速率/角度限制已建模：NOT_YET（本项不是S6 Gate，S7前必须补齐）；
- **总体：PASS**。

## 控制器尺度审计

- 控制器模式：`NREL5MW_SCHEDULED_LSS`；Kp范围 59.781--59.781 deg/(rad/s)，Ki范围 25.6204--25.6204 deg/rad；
- PI零点 0.0682093 Hz，对应时间常数 2.33333 s；
- NREL 5 MW基准控制器折算到低速轴、未施加桨距增益调度前：Kp约 104.634 deg/(rad/s)，Ki约 44.843 deg/rad；
- 当前模型保留官方形式的增益调度时，`PitchGainScheduleFactor` 随平衡桨距变化；
- 当前仍是连续命令级模型，未包含Pitch执行器滞后、8 deg/s速率限制和角度限位。这些必须在S7恢复，本轮不能据此宣称真实执行器已验证。

官方参考：[NREL 5-MW Baseline Wind Turbine](https://www.nrel.gov/docs/fy09osti/38060.pdf)；[OpenFAST DISCON baseline controller](https://github.com/OpenFAST/openfast/blob/main/share/discon/DISCON.F90)。

## 代表点断环诊断与增益区间

|WindFactor|Architecture|MarginEquilibriumResidual|OpenLoopPitchPlantMaxReal|ClassicalMarginApplicable|PitchLoopGainMarginFactor|PitchLoopPhaseMargin_deg|GainMarginFrequency_Hz|PhaseCrossover_Hz|PitchClosedLoopBandwidth_Hz|PitchLoopClosurePoleMismatch|LoopClosure_PASS|KpLowerCriticalScale|KpUpperCriticalScale|KpLowerMarginFactor|KpUpperMarginFactor|KpIntervalStatus|KpIntervalMargin_PASS|
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|1.15|GFL|1.9929736e-13|-0.025155911|true|0.00014346956|102.65995|0|0.020677948|0.016999525|1.5539251e-10|true|NaN|NaN|Inf|Inf|NOT_LIMITING_ARCHITECTURE|true|
|1.15|GWT|7.4235946e-15|-0.025156467|true|3.0492335|-91.038324|1.3215094|0.00094058518|0.00092181097|1.5572164e-10|true|NaN|NaN|Inf|Inf|NOT_LIMITING_ARCHITECTURE|true|
|1.15|MWT|5.0011585e-15|0.18310826|false|3.4508169|70.731038|1.3111419|0.0081912141|0.0088338268|5.755884e-09|true|0.40953738|NaN|2.4417795|Inf|LOWER_BOUNDARY_ONLY_NO_UPPER_TO_12|true|

传统单环 margin 只作为诊断：断开的Pitch plant包含公共转速积分/既有多环动态；当 `ClassicalMarginApplicable=false` 时，不得用传统相位裕度或增益裕度作Gate。正式数值Gate采用闭环极点重构误差和增益比例稳定区间距离。

## 结论边界

本轮只证明当前**无执行器的连续命令级Pitch模型**在测试域内是否稳定、是否可追溯。原Kp=10反例、Kp=30数值候选与本次可追溯控制器结果必须并列保留；任何一个结果都不能外推到离散、限幅、OpenFAST或EMT模型。

## 决策

S5B可追溯连续命令级基准通过本轮全部Gate，可进入S6的局部柔性机械扩展；S7前必须补回执行器、速率和角度限制。
