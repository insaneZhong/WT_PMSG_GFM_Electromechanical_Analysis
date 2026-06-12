# MSC 电压裕度与 DC 参考合理性检查（2026-06-12）

本文档解释当前 GFM-MWT 非线性模型中 MSC q 轴 PI 输出长期顶限幅的原因。该检查只基于现有控制代码和最近一次诊断结果，不改变 Simulink 结构。

## 1. 代码中的电压量纲

MSC 电流环在 `motorcontrol.c` 中生成：

```c
Ud_fwd = Rs * Id_ref - Polar * We * Lq * Iq_ref;
Uq_fwd = Rs * Iq_ref + Polar * We * (Ld * Id_ref + Fm);
Ud1_ref = id_pi.Out + Ud_fwd;
Uq1_ref = iq_pi.Out + Uq_fwd;
```

随后坐标变换得到 `Us_alfa/Us_beta`，并输入 `svpwm.c`。SVPWM 中的调制深度为：

```c
ModulationDepth = Us_Amplitude / Udc * 1.5;
```

因此，当：

```text
Us_Amplitude = Udc / 1.5
```

时，调制深度约为 1。换句话说，若 `Udc=1000 V`，线性 SVPWM 可用电压矢量幅值约为：

```text
U_limit = 1000 / 1.5 = 666.7 V
```

当前运行中 `Udc` 实际约为 974 V，因此线性电压矢量上限约为：

```text
U_limit = 974 / 1.5 = 649.3 V
```

## 2. 当前工况下 PMSG 反电势量级

最近一次诊断采用的末端运行点：

| 量 | 数值 |
|---|---:|
| `omega_g_end_mean` | `3.4590 rad/s` |
| `n_p` | `20` |
| `psi_f` | `8.64 Wb` |
| `Iq_ref` | `-192.742 A` |
| `Udc_end_mean` | `973.998 V` |

电角速度：

```text
omega_e = n_p * omega_g = 20 * 3.4590 = 69.1807 rad/s
```

只考虑前馈项时：

```text
Ud_fwd = Rs*Id_ref - omega_e*Lq*Iq_ref = 13.6 V
Uq_fwd = Rs*Iq_ref + omega_e*(Ld*Id_ref + psi_f) = 595.4 V
|U_fwd| = 595.5 V
```

对应调制深度：

```text
M_fwd = 1.5 * |U_fwd| / Udc = 0.917
```

这说明：即使电流 PI 输出接近 0，仅 PMSG 反电势前馈已经占用了约 92% 的线性调制裕度。

## 3. 当前实际命令已经进入过调制区

最近诊断中：

| 量 | 数值 |
|---|---:|
| `MscVcmdMean` | `919.3 V` |
| `MscVcmdMax` | `960.2 V` |
| `iq_pi_out_mean` | `300 V` |
| `iq_pi_out_maxabs` | `300 V` |

对应调制深度：

```text
M_mean = 1.5 * 919.3 / 974.0 = 1.416
M_max  = 1.5 * 960.2 / 974.0 = 1.479
```

该值显著大于 1，说明 MSC 电压命令已经超出线性 SVPWM 电压裕度。此前把 `MotorVoltLimit` 从 280 提高到 300/320 后，`iq_pi_out` 仍然贴着新限幅，原因也在这里：系统需要的调节电压已经超过当前 DC 电压下的可用线性电压。

## 4. 对 DC 无法平稳跟踪的解释

当前问题不能简单归因于 DC 外环 PI 参数太弱或太强。更直接的限制是：

1. `VdcRef=1000 V` 时，MSC 线性电压裕度较小；
2. PMSG 反电势前馈本身已经接近可用电压上限；
3. 电流 PI 为了追踪 q 轴电流继续增加电压命令，导致长期顶限幅；
4. 一旦 MSC 进入过调制/限幅，机侧有功和电磁转矩不能严格按 DC 外环期望调节；
5. 因此 DC 电压会表现为有界但难以严格收敛，机械侧 `T_sh/omega_g/omega_t` 也会保留慢动态。

这解释了为什么继续小范围调整 `MotorKp/MotorKi`、`MotorFF`、`MotorVoltLimit` 没有明显改善。

## 5. 对当前研究对象的影响

如果继续坚持 `VdcRef=1000 V`，则必须接受以下风险：

- 当前 PMSG 参数和 1 MW 工况下，MSC 电压裕度偏紧；
- 后续即使有功约为 1 MW，三相波形和机械侧慢动态也可能难以达到理想稳态；
- 小扰动实验可能混入控制限幅/过调制效应，影响与小信号模型的对应关系。

更合理的工程选择是把非线性验证 DC 参考恢复到与 690 V AC、PMSG 反电势和原模型更一致的范围，例如 `1150 V` 或 `1200 V`，再重新整定无扰动稳态。这样做不等于放弃 1000 V 小信号分析，而是说明：非线性模型的物理电压裕度需要先自洽。

## 6. 下一步建议

建议下一轮做两个只改参数的验证：

1. `VdcRef=1150 V`，`MotorVoltLimit=300/320`，从当前热启动或重新生成热启动继续运行；
2. `VdcRef=1200 V`，`MotorVoltLimit=300/320`，与原 Simulink 物理初始电压一致。

判断标准：

- `MscVcmdMax / (Udc/1.5)` 是否回到 1 附近或以下；
- `iq_pi_out` 是否不再长期贴限幅；
- `UdcSlope` 是否下降；
- `T_sh_end_slope` 是否下降；
- 三相电压、电流是否更平稳。

若 1150/1200 V 明显改善，则当前 1000 V 不应作为非线性主基准，而应作为低 DC 电压裕度敏感性工况。

## 7. 短时方向测试结果

新增复现实验脚本：

- `run_vsg_dc_headroom_direction_scan_20260612.m`

输出文件：

- `Validation_Results/vsg_dc_headroom_direction_scan_20260612.csv`

本次从当前 1000 V-class 热启动文件直接切换 `VdcRef=1150/1200 V`，每个工况运行 2 个 1 s 分段。结果如下：

| `VdcRef` | `Ppcc_kW` | `Udc_V` | `UdcSlope` | `TshSlope` | `MscIqPiMax` | `MscVcmdMax` | `MscModDepthMax` | 结论 |
|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 1150 | 1001.3 | 974.03 | 5.4845 | -1.5854e5 | 300 | 961.04 | 1.4800 | 仍过调制 |
| 1200 | 1001.2 | 974.03 | 5.4706 | -1.5404e5 | 300 | 960.88 | 1.4798 | 仍过调制 |

该结果不能证明 1150/1200 V 无效，因为初始状态仍来自 1000 V 附近运行点，短时间内实际 `Udc` 没有升到新的参考值。它只能说明：若要验证较高 DC 参考是否改善 MSC 电压裕度，必须重新生成 1150/1200 V 对应的热启动运行点，而不能直接从 1000 V 热启动短时切换。

下一步应执行：

1. 选择 `VdcRef=1150 V` 或 `1200 V`；
2. 从冷启动或更接近该 DC 电压的初值开始长时运行；
3. 保存新的 `xInitial`；
4. 再检查 `MscModDepthMax` 是否降到 1 附近或以下。
