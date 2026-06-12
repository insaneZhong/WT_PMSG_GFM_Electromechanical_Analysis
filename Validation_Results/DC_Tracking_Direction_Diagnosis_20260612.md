# GFM-MWT DC 跟踪方向诊断记录（2026-06-12）

## 1. 本次诊断目的

本次诊断用于回答一个具体问题：当前 `Grid_FormingVSG_PMSG.mdl` 中 DC 电压无法严格跟踪，是否主要由 Type-c 的有功功率前馈项导致。

为避免旧热启动文件与当前模型结构不匹配造成干扰，本次采用冷启动方式，不加载旧 `xInitial`。模型初始 DC 电压临时设为 1150 V，参考值也设为 1150 V，其他控制参数保持当前基线：

- `VSG_H = 250`
- `VSG_MP = 1.5707963e-6`
- `MotorKp = 0.05`
- `MotorKi = 0.00032`
- `MotorKc = 1e-5`
- `MotorVoltLimit = 300 V`
- `MotorCurrentKp = 1.4`
- `MotorCurrentKi = 0.00290476`
- `Cdc = 0.03 F`
- `Pref = 1 MW`
- `VacRef = 563 V`

## 2. 热启动文件状态

旧热启动文件：

```text
Validation_Results/Initial_State/Grid_FormingVSG_PMSG_Init_Vdc1150_Regen.mat
```

已经不能用于当前模型。Simulink 报错：

```text
Simulink 无法加载初始工作点，因为模型 'Grid_FormingVSG_PMSG' 在工作点保存后发生了更改。
```

因此当前模型后续必须重新生成热启动文件。旧热启动结果只能作为历史调参记录，不能作为当前模型的正式验证入口。

## 3. Type-a / Type-c 冷启动 4 s 对照

| 指标 | Type-a | Type-c | 解释 |
|---|---:|---:|---|
| `PreSyn` | 1.000 | 1.000 | 两者均完成并网 |
| `Ppcc_end_mean` | 1034.19 kW | 1027.14 kW | 两者均能送出约 1 MW |
| `Udc_end_mean` | 1006.00 V | 1007.26 V | 并网后均回落到约 1.0 kV |
| `Udc_end_slope` | -37.69 V/s | -27.02 V/s | DC 仍在下滑，未稳态 |
| `msc_iqref_end_mean` | -54.79 A | -264.92 A | Type-c 因功率前馈使 `Iq_ref` 更负 |
| `msc_iq_end_mean` | -1223.94 A | -1135.95 A | 实际 `Iq` 均远比参考更负 |
| `Iq_ref - Iq` | 1169.15 A | 871.03 A | 两者均严重无法跟踪 |
| `msc_iq_pi_out` | 300 V | 300 V | 两者 q 轴电流 PI 均贴正限幅 |
| `msc_mod_depth_mean` | 1.387 | 1.380 | 两者均进入过调制区 |

## 4. 直接结论

当前 DC 无法稳定跟踪不能简单归因于 Type-c 有功功率前馈。

理由：

1. Type-a 去掉有功功率前馈后，`Iq_ref` 确实明显变小，但实际 `Iq` 仍大幅偏负；
2. Type-a 和 Type-c 的 q 轴电流 PI 都贴 `+300 V` 限幅；
3. 两者的 MSC 电压调制深度都约为 `1.38`，均超过线性调制范围；
4. 两者并网后 DC 电压都从 1150 V 降到约 1007 V，并仍有负斜率。

因此，当前根因更接近：

```text
并网送功后 MSC 侧 q 轴电流无法按参考调节
-> q 轴 PI 长期正向饱和
-> MSC 电压指令超过可用 DC 电压裕度
-> 电流/电磁转矩不能按 DC 外环要求调节
-> DC 电压有界但持续漂移
```

## 5. 下一步诊断优先级

下一步不应继续大范围扫 `MotorKp/MotorKi`。应优先做以下检查：

1. **MSC q 轴电流环极性检查**  
   当前 `Iq_ref` 为负，实际 `Iq` 更负，误差 `Iq_ref - Iq` 为正，PI 输出为 `+300 V`。需要确认正 `Uq` 在当前 dq 定义、PMSG 端口方向和 SVPWM 极性下，是否真的会让 `Iq` 往参考方向回调。

2. **PMSG/SVPWM 电压尺度检查**  
   当前 `Uq_fwd` 约 615 V，而 `Udc/1.5` 约 671 V，前馈已经占据大部分线性电压裕度。只要 PI 需要额外电压，就会过调制。需要核对 `Ud1_ref/Uq1_ref` 的单位是否与 Universal Bridge DC 母线利用关系一致。

3. **电流测量方向检查**  
   当前实际 `Iq` 长期比参考更负，需要确认 `abc -> dq` 变换、电流传感器方向、PMSG 发电机约定是否与控制器中 `Iq_ref` 的正负号一致。

4. **重新生成当前模型热启动**  
   旧热启动已经失效。完成上述方向/尺度检查后，应重新生成与当前模型完全匹配的 `xInitial`，再做长时无扰动验证。

## 6. 对论文实验流程的影响

这组结果说明：Type-a/Type-c 的结构差异仍然有研究意义，但当前非线性模型的不稳态问题不是 Type-c 前馈单独造成的。论文层面应将其表述为“机侧 DC 控制结构下的执行器电压裕度与电流方向一致性问题尚需校准”，不能直接把当前非线性 DC 漂移解释为机电耦合负阻尼现象。

在无扰动基准稳定前，不应进入小扰动 FFT、SCR 扫描或 APCAD 非线性验证。

## 7. q 轴电流环符号诊断

为进一步判断 `Iq` 无法跟踪是否来自 q 轴电流环电压作用方向，新增了两个只用于诊断的编译期宏：

```text
MOTOR_IQ_PI_OUTPUT_SIGN
MOTOR_IQ_FEEDBACK_SIGN
```

默认值均为 `+1`，不改变原模型行为。诊断时通过 `compile_vsg_sfunction` 临时传入 `MotorIqPiSign` 或 `MotorIqFeedbackSign`，仿真结束后已重新编译回默认 `+1/+1`。

### 7.1 q 轴 PI 输出反号

结果文件：

```text
Validation_Results/vsg_iq_pi_sign_diagnosis_20260612.csv
```

| `MotorIqPiSign` | `Udc` / V | `UdcSlope` / V/s | `Ppcc` / kW | `Iq_ref` / A | `Iq` / A | `Iq_ref-Iq` / A | `Iq PI` / V | `ModDepth` |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| +1 | 1007.26 | -27.02 | 1027.14 | -264.92 | -1135.95 | 871.03 | +300 | 1.380 |
| -1 | 4330.98 | -160.75 | 1045.76 | 1285.00 | -1387.04 | 2672.04 | -300 | 0.088 |

结论：单独反转 q 轴 PI 输出并不能修复电流跟踪，反而导致 DC 电压严重抬升到约 4.3 kV。虽然调制深度变小，但这是因为 DC 电压异常升高，不是稳定运行点改善。

### 7.2 q 轴电流反馈反号

结果文件：

```text
Validation_Results/vsg_iq_feedback_sign_diagnosis_20260612.csv
```

| `MotorIqFeedbackSign` | `Udc` / V | `UdcSlope` / V/s | `Ppcc` / kW | `Iq_ref` / A | `Iq` / A | `Iq_ref-Iq` / A | `Iq PI` / V | `ModDepth` |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| -1 | 5059.85 | -1581.75 | 1007.80 | 1067.94 | -1159.66 | 2227.61 | -300 | 0.075 |

结论：单独反转 q 轴反馈也不能作为修复方案。它同样导致 DC 电压严重过高，并且 q 轴 PI 仍然贴限幅。

## 8. 更新后的根因判断

当前证据排除了两个过于简单的解释：

1. 不是 Type-c 有功功率前馈单独导致；
2. 不是单独把 q 轴 PI 输出或 q 轴反馈反号即可解决。

更合理的判断是：当前机侧控制的符号和尺度需要作为一个整体重新校准，包括：

- PMSG 发电机模式下 `Iq` 正负号与电磁转矩方向；
- `Udc < VdcRef` 时，DC 外环输出应使机侧向 DC 电容补能还是减小发电功率；
- 当前 `Iq_ref = iq_power_ff - PI(VdcRef-Udc)` 是否与上述能量方向一致；
- q 轴电流环在限幅后的抗饱和是否能恢复；
- `Uq_fwd` 与 Universal Bridge/SVPWM 的电压尺度是否一致。

下一步建议不是继续扫增益，而是输出并核对控制器内部变量：

```text
pwm_speed_pi.Ref/Fdb/Error/Out
iq_pi.Ref/Fdb/Error/Out
```

然后基于这些内部量重写一版“发电机侧 DC 电压控制符号表”，再决定是否修改 `Iq_ref` 生成公式或 q 轴电流环符号。
