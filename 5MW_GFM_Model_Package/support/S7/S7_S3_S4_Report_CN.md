# S7-3/S7-4 真实数字参考模型九点筛查与 Pole–Excitation 复核

生成时间：2026-09-03 18:04:10

## 证据等级

本轮为 **CONDITIONAL_REFERENCE_DIGITAL_SCREENING**。使用的是已通过V2条件验证的 `S7A_DiscreteAvg_5MW.slx` 同源参考离散平均映射，不是遗留C控制器的最终实现，也不是EMT证据。

## S7-3九点结果

连续M0基线轴系模态：2.482748 Hz，阻尼比 0.027106；连续模态机械扰动残差幅值 4.23743e-08，电网频率扰动残差幅值 0.148352。

九点全部计算完成，固定点残差、全部极点、TOR/DC/SYNC/GSC模态摘要见 `D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\5MW_Ideal\M3_Workpoint_Generalization\S7_S3_Reference_Digital_9Point.csv` 和 `D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\5MW_Ideal\M3_Workpoint_Generalization\S7_S3_Modal_Identity_Summary.csv`。

|Ts/Ts0|tau/Ts|TOR Hz|zeta|De proxy|Pole/Excitation分类|
|---:|---:|---:|---:|---:|---|
|0.5|0|2.482748|0.027106|-655901|EXCITATION_IMPLEMENTATION_DEPENDENT|
|0.5|0.5|2.482748|0.027106|-655920|DIGITAL_INSENSITIVE|
|0.5|1|2.482747|0.027106|-655940|DIGITAL_INSENSITIVE|
|1|0|2.482748|0.027107|-654713|EXCITATION_IMPLEMENTATION_DEPENDENT|
|1|0.5|2.482747|0.027106|-654753|DIGITAL_INSENSITIVE|
|1|1|2.482747|0.027106|-654791|DIGITAL_INSENSITIVE|
|2|0|2.482748|0.027107|-652336|EXCITATION_IMPLEMENTATION_DEPENDENT|
|2|0.5|2.482747|0.027107|-652417|DIGITAL_INSENSITIVE|
|2|1|2.482746|0.027106|-652494|DIGITAL_INSENSITIVE|

## S7-4解释

本轮以连续M0的TOR pole/residue与数字映射的TOR pole/residue做增量对照：Pole变化记录在 `delta_pole_*`，激励变化记录在 `delta_excitation_*`。由于连续与数字模型状态阶数不同，本程序没有虚构跨阶的 y_cc/y_dc 时域反事实；`S7_S4_Pole_Excitation_Decomposition.csv` 中的 `counterfactual_note` 明确标记为 modal-metric decomposition。

当前筛查中，若分类为 DIGITAL_INSENSITIVE，只说明在这9个Reference数字点上TOR pole与残差变化均小；若为 POLE_IMPLEMENTATION_DEPENDENT 或 EXCITATION_IMPLEMENTATION_DEPENDENT，只能作为待复核假设，不能直接升级为论文结论。

## Gate V3建议

在真实遗留数字控制器映射尚未完成前，Gate V3保持 **BLOCKED/CONDITIONAL**，不进入EMT。只有将实际采样顺序、PI更新顺序、延迟和控制状态与C代码逐一对齐，并重复V2和S7-3/S7-4后，才判断EMT工作量。

## 产物

- `D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\5MW_Ideal\M3_Workpoint_Generalization\S7_S3_Reference_Digital_9Point.csv`
- `D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\5MW_Ideal\M3_Workpoint_Generalization\S7_S3_Modal_Identity_Summary.csv`
- `D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\5MW_Ideal\M3_Workpoint_Generalization\S7_S4_Pole_Excitation_Decomposition.csv`
- `D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\5MW_Ideal\M3_Workpoint_Generalization\S7_S3_S4_Pole_Excitation.png`
