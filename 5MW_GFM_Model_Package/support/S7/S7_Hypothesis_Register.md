# S7 电气实现保真度假设寄存器

更新时间：2026-09-03 18:05:00

## 证据边界

- S6：**INCONCLUSIVE**。S6机械模型保真度未完成同机组、可追溯柔性闭环验证；S7独立开放，不将S7结果升级为完整柔性风机结论。
- S7仅检验连续平均候选机制在采样、ZOH、控制器离散和开关实现中的保持、迁移或失效。
- S7A-V1/A2/A3为同源RHS筛查，不等同于真实C/S-Function离散控制器。
- S7-2已建立 `S7A_DiscreteAvg_5MW.slx` 参考离散平均副本；V2为 `CONDITIONAL_REFERENCE_PASS`，S7-3/S7-4为 `CONDITIONAL_REFERENCE_SCREENING`，S7B EMT仍未启动。

## 冻结项

|项目|冻结值|
|---|---:|
|连续参考模型|`M0_PMSG_GFM_5MW.slx`|
|状态数|23|
|额定功率|5 MW|
|标称控制采样|0.0001 s|
|H|3 s|
|SCR|4|
|DVC比例|1|

## 候选假设

|编号|候选机制|可证伪条件|当前状态|
|---|---|---|---|
|H7-1|数字延迟改变 `G_Te,omega_g` 的幅相|轴系频率处阻尼代理由正变负或显著迁移|参考九点中未见TOR pole显著迁移；残差仍需真实数字实现复核|
|H7-2|Pole/Excitation分类随实现迁移|TOR极点或模态残差排序发生反转|参考九点显示零延迟点残差变化较大，其余多数点较小；暂定待证伪|
|H7-3|电气模态进入轴系时间尺度|MAC/参与因子连续性支持模态接近或交换|参考模型未发现显著组参与度；跨阶MAC尚未计算|
|H7-4|低频输入输出响应依赖实现|相同扰动下频率/阻尼/排序不一致|参考V2关键输出对照通过；遗留控制器结论未建立|

## Gate规则

- Gate V1：连续极限 `Ts/Ts0 -> 0`，频率误差 <0.5%，TOR趋势一致；否则停止机制解释。
- Gate V2：参考离散平均模型固定点归一化残差 `2.98e-14`，D1/D2/D3全部稳定；机械扰动峰值误差<0.01%，电网频率扰动关键输出最大峰值误差约3.42%，因此为 `CONDITIONAL_REFERENCE_PASS`，不等同于遗留数字控制器通过。
- Gate V3：实际控制器状态、更新顺序和延迟尚未对齐，保持 `BLOCKED/CONDITIONAL`。
- Gate S7B：只有真实数字模型完成V2和S7-3/S7-4后才选择少量开关EMT工况。

## 当前文件

- `D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\5MW_Ideal\M3_Workpoint_Generalization\S7_Reference_Manifest.csv`
- `D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\5MW_Ideal\M3_Workpoint_Generalization\S7_Digital_Controller_Manifest.csv`
- `D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\5MW_Ideal\M3_Workpoint_Generalization\Controller_Discretization_Manifest.csv`
- `D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\5MW_Ideal\M3_Workpoint_Generalization\S7_Controller_State_Audit.csv`
- 同目录下的S7A摘要CSV。
- `S7_V2_FixedPoint_Report_CN.md`、`S7_V2_NL_SSM_Validation.csv`、`S7_V2_Discrete_Modes.csv`
- `S7_S3_S4_Report_CN.md`、`S7_S3_Reference_Digital_9Point.csv`、`S7_S4_Pole_Excitation_Decomposition.csv`
