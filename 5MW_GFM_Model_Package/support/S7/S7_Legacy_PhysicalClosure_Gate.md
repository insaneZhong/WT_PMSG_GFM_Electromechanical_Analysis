# S7-5C2 Legacy Controller–Average Plant Gate

- 模型副本：`D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\5MW_Ideal\M3_Workpoint_Generalization\S7_Legacy_Average_Plant.slx`
- 隔离 MEX：`D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\5MW_Ideal\M3_Workpoint_Generalization\temp\S7_5_LegacyPlant\main_s7_legacy_avg.mexw64`
- 当前总状态：**C2 BLOCKED（接口通过，但物理闭合失败）**

## 审计项目

|项目|状态|观测|判据|
|---|---|---|---|
|C0_model_copy|PASS|D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\5MW_Ideal\M3_Workpoint_Generalization\S7_Legacy_Average_Plant.slx|file exists|
|C1_mex|PASS|D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\5MW_Ideal\M3_Workpoint_Generalization\temp\S7_5_LegacyPlant\main_s7_legacy_avg.mexw64|file exists|
|C1_input_port_count|PASS|20|20|
|C1_output_port_count|PASS|18|18|
|C1_legacy_wrapper|PASS|LegacyC=1;Mux=1;Demux=1|LegacyC + 20路Mux + 41路Demux|
|C1_old_controller_removed|PASS|true|no IdealCtrlRHS inside wrapper|
|C1_sfunction_name|PASS|main_s7_legacy_avg|main_s7_legacy_avg|
|C1_sfunction_parameters|PASS|5e6,0,563,1500|5e6,0,563,1500|
|C1_parent_input_connections|PASS|20 unique ports [1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20]|20 unique ports|
|C1_average_command_tags|PASS|Ideal_GSC_Ualpha,Ideal_GSC_Ubeta,Ideal_MSC_Ualpha,Ideal_MSC_Ubeta,Pulse1,Pulse2|Ideal_MSC_Ualpha,Ideal_MSC_Ubeta,Ideal_GSC_Ualpha,Ideal_GSC_Ubeta|
|C1_model_update|PASS|UPDATE_OK|no update error|
|C1_short_smoke|PASS|status=PASS
stopTime=0.005
message=update + short normal simulation completed
|status=PASS|

## 边界

本报告证明 C0/C1 的副本、接口和极短仿真可解析，但 C2 诊断在 0.0079202 s 处失败，且 0.005 s 已出现 Udc 快速漂移和 PCC 有功为零。它**不**证明共同周期固定点、功率/转矩能量闭合，也不证明 Legacy 闭环稳定。C3 逐环闭合、C4/C5 固定点和 C6 扰动 Gate 必须等 C2 冻结输出/方向测试通过后再执行。

详细失败证据见 `S7_Legacy_C2_Gate_CN.md`；本轮未保存长时序。
