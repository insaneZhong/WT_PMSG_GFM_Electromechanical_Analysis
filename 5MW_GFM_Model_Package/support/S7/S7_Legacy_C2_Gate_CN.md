# S7-5C2 Legacy Controller–Average Plant Gate 结果

## 结论

**C2 = BLOCKED（Legacy 输出尚未与物理 plant 形成共同工作点）**。

本轮只运行了隔离副本 `S7_Legacy_Average_Plant.slx`，原 M0 模型和原始 C 控制器文件未修改。C0/C1 接口检查仍保持 PASS，但接口可解析不等于物理闭合。

## 分段诊断证据

|仿真时长|状态|Udc 末值|PCC 有功末值|Te 末值|Tsh 末值|说明|
|---:|---|---:|---:|---:|---:|---|
|0.001 s|PASS（仅短时可运行）|1500 V|0 W|3.8286 MN·m|3.8286 MN·m|与 M0 初始转矩接近，尚未暴露漂移|
|0.005 s|PASS（仅求解完成）|1218.9 V|0 W|1.8668 MN·m|3.8215 MN·m|Udc、Te 已明显偏离工作点；Udc 漂移约 −562.2 V|
|0.200 s|FAIL|—|—|—|求解器在 0.0079202 s 处停止，无法继续|

原始诊断摘要保存在 `S7_Legacy_Progressive_Closure.csv` 的最后一行；未保存长时序或完整 SimulationOutput。

## 当前可以确认的事实

1. Legacy C 平均输出包装器、20 路输入、18 路诊断端口和四路 alpha-beta 电压指令在 Simulink 接口层可解析。
2. 将 Legacy 输出直接驱动 M0 的平均 VSC 后，物理 plant 没有保持 M0 的共同固定点。
3. 5 ms 诊断中的 `PCC 有功=0 W` 与 `Udc` 快速下降说明网侧功率路径、初始 LCL/电网状态或控制输出工作点至少有一项尚未对齐。
4. Powergui 报告 `Il_L1`、`Il_L3` 初始电流被忽略；这意味着当前副本的 LCL 初始状态也不能直接视为 M0 的完整固定点。

## 尚不能直接断言的内容

- 不能仅凭本次失败断言是 alpha-beta 符号错误、Legacy 电压指令尺度错误、PCC 功率测量面错误，还是 LCL 初始状态缺失。
- 不能把 0.001 s 的短时 PASS 当成闭环稳定或共同周期固定点。
- C3 逐层闭合、C4/C5 固定点与能量 Gate、C6 小扰动验证均未开始。

## 下一步唯一允许的 C2 动作

先用 M0 工作点冻结 Legacy 的四路平均电压输出，旁路 Legacy 动态控制器，分别测试：

1. MSC 电压指令正/负号；
2. GSC 电压指令正/负号；
3. MSC/GSC alpha-beta 交换与缩放；
4. PCC 功率方向和 DC-link 能量方向。

只有冻结输出能够维持物理 plant 的功率、转矩和 DC-link 平衡，才恢复 C2 的逐类方向测试并进入 C3。否则应先修正接口合同或初始条件，不得继续做参数扫描或 S7-5D。

