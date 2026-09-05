# M1物理平均VSC跨模型验证报告

M0保持不变；M1只有一个可切换模型。M1-a使用固定直流基准归一化，M1-b使用实时Udc前馈。未加入PWM、采样、延迟、限幅、LVRT或Pitch。

## Gate M1

- 平衡点：PASS
- 全部代表架构稳定：FAIL
- M1-b回归M0：PASS（A矩阵最大相对误差 1.413e-10）
- M1-a物理直接通道：PASS

|架构|C_GM|C_MG|Gamma_dir|结论|
|---|---:|---:|---:|---|
|GFM-MWT|12.348|0.12134|101.76|B: reverse path opens but directional dominance remains|
|GFM-GWT|0.20826|0.1128|1.8463|C: forward and reverse paths are comparable|

若M1-a把反向通道打开但Gamma_dir仍远大于1，原结论应改写为“方向占优”，不再宣称严格结构零。若两向同量级，本报告已停止后续强化。

**条件性停止：** M1-a至少一个架构不稳定或两向耦合已经同量级。最不稳定代表点为 `GFM-GWT (GSC-DVC + MSC-MPPT/转矩)`，最大实部 0.131385 1/s，频率 0.468232 Hz，归属 `GSC-SYNC electrical mode`；MECH参与 6.541e-05，GSC参与 0.7714，SYNC/DC/GSC-DVC参与合计 0.2286。因此未在该未整定基准上继续计算M2/M3并声称跨模型稳健。

## Gate M2：Pole–Path稳健性

|对象|Robust|Pole|Path|Modal set|说明|
|---|---|---|---|---|---|
|Stage M2|FAIL|FAIL|FAIL|FAIL|Gate M1 conditionally stopped: M1-a is unstable or a reverse path became comparable|

## Gate M3：SYNC–DC边界

未通过或因Gate M1情况C而停止。

## 文件

- `M1_PhysicalAvg_Summary.csv`：全部摘要；
- `M0_M1_PhysicalAvg_Comparison.png`：唯一综合图；
- `Grid_Forming_PMSG5MW_TwoMass_M1_PhysicalAvg.slx`：唯一M1模型。
