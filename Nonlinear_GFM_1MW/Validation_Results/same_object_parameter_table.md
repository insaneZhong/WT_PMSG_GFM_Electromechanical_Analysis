# 同一对象参数对照表

- Small-signal: `D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\GFM_1MW _nonlinear\..\..\..\（1）小信号模型\WT_PMSG_GFM_小信号分析_最新整理包_20260526\EigenAnalysis\Parameters.mat`
- Nonlinear init: `GFM_MWT_Nonlinear_Params.m`
- Nonlinear control: `motorcontrol.h` + `grid_forming_control.h`

| 参数 | 小信号 | 非线性当前值 | 比值(非线性/小信号) |
|---|---:|---:|---:|
| S_base/P_wt_rated | 1e+06 | 1e+06 | 1 |
| omega_g0/omega_m0 | 15.708 | 15.708 | 1 |
| v_w0 | 12 | 12 | 1 |
| J_t | 1.47e+06 | 1.47e+06 | 1 |
| J_g | 183750 | 183750 | 1 |
| K_sh | 2.57926e+07 | 2.57926e+07 | 1 |
| D_sh | 41050.1 | 41050.1 | 1 |
| D_aero | 3647.56 | 3647.56 | 1 |
| K_v_aero | 14323.9 | 14323.9 | 1 |
| R_s vs MOTOR_RS | 0.0122 | 0.0122 | 1 |
| L_d vs MOTOR_LD | 0.00102 | 0.00102 | 1 |
| L_q vs MOTOR_LQ | 0.00102 | 0.00102 | 1 |
| n_p vs MOTOR_POLE_PAIR | 20 | 20 | 1 |
| C_dc vs GRID_UDC_C | 0.0015 | 0.0015 | 1 |
| lf1 vs GRID_FILTER_LS | 0.00014 | 0.00012 | 0.857143 |
| cf vs GRID_FILTER_C | 0.000334 | 5.5e-05 | 0.164671 |
| lg vs GRID_LINE_L | 0.000297209 | 0.0005 | 1.68232 |
| k_pm vs MOTOR_ID_KP | 1.7 | 1.4 | 0.823529 |
| k_im vs MOTOR_ID_KI | 20.3333 | 0.00290476 | 0.000142857 |
| k_pdc vs GSC_P_KP | 0.5 | 1e-06 | 2e-06 |
| k_idc vs GSC_P_KI | 50 | 2e-05 | 4e-07 |
| h (VSG inertia) vs VSG_J | 506.606 | 10 | 0.0197392 |
| mp (droop) vs (P-loop only) | 1.5708e-06 | NaN | NaN |

## 控制结构差异（当前）
- 小信号 GFM 以 `h/mp` 的 VSG 结构为核心（含模式扫描）。
- 非线性当前 GSC 为 `P环->w_ref` + `Q下垂->E`，属于下垂/P环驱动，并非完整二阶VSG摆动方程。
- 已可通过参数继承实现“同一对象参数”；控制结构统一还需在 C 侧补充 `2H dw/dt = P_ref-P-D(w-w0)` 状态环节。
