# VSG-MWT 非线性电压裕度诊断（2026-06-12）

本文档记录当前 GFM-MWT 非线性模型在无扰动稳态整定阶段的最新诊断。目标不是证明模型已经稳定，而是定位下一步整定入口。

## 1. 当前基准状态

最近一次从 20 s 热启动继续运行的候选结果来自：

- `Validation_Results/vsg_operating_point_segments_ContinueFrom20s_Vlim280_FF225_Kp0055_Ki00036_20260612.csv`
- 当前诊断文件：`Validation_Results/no_disturbance_diagnosis_Grid_FormingVSG_PMSG.mat`

关键结果：

| 指标 | 当前值 | 判断 |
|---|---:|---|
| `Ppcc_end_mean` | `1006.185 kW` | 有功基本达到 1 MW |
| `Udc_end_mean` | `978.305 V` | DC 有界，但偏离 1000 V |
| `Udc_end_slope` | `-5.554 V/s` | 不满足严格 DC 稳态 |
| `T_sh_end_slope` | `-3.8302e4 N*m/s` | 机械侧尚未严格收敛 |
| `msc_iq_pi_out_end_mean` | `280` | MSC q 轴 PI 输出顶到限幅 |
| `msc_iq_pi_out_end_maxabs` | `280` | MSC q 轴 PI 输出顶到限幅 |

结论：当前模型可以认为是“有界运行候选”，不能认为已经完成无扰动稳态验证。主要问题不是有功不能达到，而是 MSC 控制裕度和机械/DC 慢动态仍未完全收敛。

## 2. 本次扫描设置

新增脚本：

- `run_vsg_margin_tuning_scan_20260612.m`

本次只改变非结构参数：

- MSC 电流环 PI 电压限幅；
- MSC dq 电流环参数；
- MSC-DVC 外环参数；
- MSC 功率前馈系数；
- 显式传入 `DcCapF=0.03`、`MotorLd=1.02e-3`、`MotorLq=1.02e-3`。

每个候选从同一个热启动文件继续：

- `Validation_Results/Initial_State/Grid_FormingVSG_PMSG_Init_ContinueFrom20s_Vlim280_FF225_Kp0055_Ki00036_20260612_Segment_05.mat`

本次每个候选只运行 2 个 1 s 分段，因此结果仅用于判断调参方向，不作为最终长时稳定结论。

## 3. 扫描结果

汇总文件：

- `Validation_Results/vsg_margin_tuning_scan_20260612.csv`

| 候选 | `Ppcc_kW` | `Udc_V` | `UdcSlope` | `TshSlope` | `MscIqPiMax` | `MscIqPiMargin` | 判断 |
|---|---:|---:|---:|---:|---:|---:|---|
| `Margin_Vlim300_SofterDVC` | `1001.5` | `974.0` | `5.3123` | `-1.554e5` | `300` | `0` | 短时评分最好，但仍顶限幅 |
| `Margin_Vlim320_DefaultInner` | `1001.2` | `974.1` | `5.4816` | `-1.562e5` | `320` | `0` | 抬限幅未解除饱和 |
| `Margin_Vlim300_DefaultInner` | `1001.3` | `974.0` | `5.5769` | `-1.539e5` | `300` | `0` | 抬限幅未解除饱和 |
| `Margin_Vlim300_SofterInner` | `1001.1` | `973.1` | `5.8457` | `-1.549e5` | `300` | `0` | 软化内环未明显改善 |

## 4. 诊断结论

1. 只把 MSC 电流环电压限幅从 280 提高到 300/320，不能解决问题。PI 输出仍然贴着新限幅，说明控制器在当前工作点持续要求更大的 q 轴电压。
2. 轻微软化 MSC 内环或 MSC-DVC 外环，短时改善有限，不能使机械侧达到稳态。
3. 当前 DC 电压不是快速发散，而是围绕 970-980 V 有界运行并伴随慢动态；因此后续目标应是降低持续限幅和机械扭矩斜率，而不是强制 DC 精确等于 1000 V。
4. 下一轮整定应优先检查 MSC 控制量尺度和物理约束，包括：q 轴电压限幅是否与 690 V/1000 V DC 物理电压裕度一致、S-Function 输出电压量是否为相电压峰值/线电压/调制量、PMSG 电压方程中的转速反电势是否导致当前 1 MW 工作点天然需要接近限幅。

## 5. 下一步执行建议

下一轮不建议继续简单抬限幅。建议按以下顺序：

1. 对 MSC 电压指令做量纲核查：确认 `MscVcmdMax`、`MOTOR_PI_IQ_OUT_MAX` 与 SVPWM 调制输入使用同一电压定义。
2. 检查 1 MW、`omega_g=pi rad/s`、`psi_f=8.64` 下 PMSG 反电势和 q 轴电流需求，判断当前工作点是否物理上需要过高电压。
3. 若量纲正确，再做更小范围的 MSC-DVC 外环整定，重点降低 `T_sh_end_slope` 和 `UdcSlope`，而不是只追踪 `Udc=1000 V`。
4. 只有当无扰动运行满足有功、DC 有界、三相波形稳定、机械侧不发散后，才进入小扰动 FFT 验证。
