# 同一对象参数对照表

- Small-signal source: `D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\GFM_1MW _nonlinear\..\..\..\（1）小信号模型\WT_PMSG_GFM_Electromechanical_Validation\EigenAnalysis\Parameters.mat`
- Nonlinear init script: `GFM_MWT_Nonlinear_Params.m`
- Nonlinear C control headers: `motorcontrol.h`, `grid_forming_control.h`

| Parameter | Small-signal | Nonlinear | Ratio (non/ss) |
|---|---:|---:|---:|
| S_base/P_wt_rated | 1e+06 | 1e+06 | 1 |
| omega_g0/omega_m0 | 15.708 | 15.708 | 1 |
| v_w0 | 12 | 12 | 1 |
| J_t | 1.47e+06 | 1.47e+06 | 1 |
| J_g | 183750 | 183750 | 1 |
| K_sh | 2.57926e+07 | 2.57926e+07 | 1 |
| D_sh | 41050.1 | 41050.1 | 1 |
| D_aero | 4052.85 | 4052.85 | 1 |
| K_v_aero | 15915.5 | 15915.5 | 1 |
| R_s vs MOTOR_RS | 0.0122 | 0.0122 | 1 |
| L_d vs MOTOR_LD | 0.00105 | 0.00102 | 0.971429 |
| L_q vs MOTOR_LQ | 0.00105 | 0.00102 | 0.971429 |
| n_p vs MOTOR_POLE_PAIR | 20 | 20 | 1 |
| C_dc vs GRID_UDC_C | 0.0015 | 0.0015 | 1 |
| lf1 vs GRID_FILTER_LS | 0.00012 | 0.00012 | 1 |
| cf vs GRID_FILTER_C | 5.5e-05 | 5.5e-05 | 1 |
| lg vs GRID_LINE_L | 0.000297209 | 0.0005 | 1.68232 |
| k_pm vs MOTOR_ID_KP | 1.4 | 1.4 | 1 |
| k_im vs MOTOR_ID_KI | 16.2667 | 0.00290476 | 0.000178571 |
| k_pdc vs GSC_P_KP | 0.5 | 1e-06 | 2e-06 |
| k_idc vs GSC_P_KI | 50 | 2e-05 | 4e-07 |
| h (VSG inertia) vs VSG_J | 506.606 | 10 | 0.0197392 |
| mp (droop) vs 1/VSG_D (equiv) | 1.5708e-06 | 0.0005 | 318.31 |

## 控制结构差异（当前）
- 小信号主模型以 `h/mp` 对应的 VSG/下垂参数进行模态分析。
- 非线性当前为 `P环->w_ref` 与 `Q下垂->E` 的实现，尚未完全等价于二阶 VSG 摆动方程。
- 建议后续在 C 侧补齐等价状态环节：`2H*w0*dw/dt = P_ref - P - D*(w-w0)`，保证与小信号严格一致。
