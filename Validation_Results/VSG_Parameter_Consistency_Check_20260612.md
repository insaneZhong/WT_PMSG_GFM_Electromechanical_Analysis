# VSG Nonlinear Parameter Consistency Check

- Generated at: 2026-06-12 09:59:58
- Nonlinear directory: `D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\GFM_1MW _nonlinear`
- Small-signal parameter file: `D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\GFM_1MW _nonlinear\..\..\..\（1）小信号模型\WT_PMSG_GFM_Electromechanical_Validation\EigenAnalysis\Parameters.mat`

## 1. MATLAB Parameters vs Small-Signal MAT

| Parameter | Nonlinear MATLAB | Small-Signal MAT | Delta | Status |
|---|---:|---:|---:|---|
| `P_wt_rated` | 1000000 | 1000000 | 0 | match |
| `omega_m0` | 3.141592654 | 3.141592654 | 0 | match |
| `v_w0` | 12 | 12 | 0 | match |
| `J_t` | 1470000 | 1470000 | 0 | match |
| `J_g` | 183750 | 183750 | 0 | match |
| `K_sh` | 25792566.17 | 25792566.17 | 0 | match |
| `D_sh` | 41050.14401 | 41050.14401 | 0 | match |
| `D_t` | 506.6059182 | 506.6059182 | 0 | match |
| `D_g` | 506.6059182 | 506.6059182 | 0 | match |
| `T_aero0` | 318309.8862 | 318309.8862 | 0 | match |
| `theta_tw0` | 0.01234114838 | 0.01234114838 | 0 | match |
| `D_aero` | 101321.1836 | 101321.1836 | 0 | match |
| `K_v_aero` | 79577.47155 | 79577.47155 | 0 | match |

## 2. Derived Shaft Quantities

- Equivalent inertia `J_eq = 163333.3333 kg*m^2`
- Shaft natural frequency `f_sh = 2.000000 Hz`
- Shaft damping ratio `zeta_sh = 0.010000`
- Steady torque from power/speed `T_e0 = 318309.8862 N*m`
- Steady twist angle `theta_tw0 = 0.01234114838 rad`

## 3. C Header Macros

| File | Macro | Value |
|---|---|---:|
| `motorcontrol.h` | `GRID_UDC__C` | 0.03 |
| `motorcontrol.h` | `MOTOR_POLE_PAIR` | 20 |
| `motorcontrol.h` | `MOTOR_RS` | 0.0122 |
| `motorcontrol.h` | `MOTOR_LD` | 0.00102 |
| `motorcontrol.h` | `MOTOR_LQ` | 0.00102 |
| `motorcontrol.h` | `MOTOR_FM_25_TEMPERATURE` | 8.64 |
| `motorcontrol.h` | `MOTOR_JM` | 183750 |
| `motorcontrol.h` | `CURRENT_LOOP_BANDWITH_ID` | 220 |
| `grid_forming_control_vsg.h` | `GRID_UDC__C` | 0.03 |
| `grid_forming_control_vsg.h` | `CURRENT_LOOP_BANDWITH` | 220 |

## 4. C Macros vs Small-Signal Parameters

| C Macro | Small-Signal Parameter | Meaning | C Value | Small-Signal Value | Relative Difference | Status |
|---|---|---|---:|---:|---:|---|
| `GRID_UDC__C` | `C_dc` | DC-link capacitance | 0.03 | 0.0015 | 1900% | mismatch |
| `MOTOR_POLE_PAIR` | `n_p` | PMSG pole pairs | 20 | 20 | 0% | match |
| `MOTOR_RS` | `R_s` | PMSG stator resistance | 0.0122 | 0.0122 | 0% | match |
| `MOTOR_LD` | `L_d` | PMSG d-axis inductance | 0.00102 | 0.00105 | -2.857% | mismatch |
| `MOTOR_LQ` | `L_q` | PMSG q-axis inductance | 0.00102 | 0.00105 | -2.857% | mismatch |
| `MOTOR_FM_25_TEMPERATURE` | `psi_f` | PMSG PM flux linkage | 8.64 | 8.64 | 0% | match |
| `MOTOR_JM` | `J_g` | PMSG generator inertia | 183750 | 183750 | 0% | match |

## 5. Diagnosis

- Mechanical MATLAB parameters match the small-signal MAT file.
- If `GRID_UDC__C` differs from small-signal `C_dc`, DC-link time scale and controller tuning are not comparable.
- PMSG resistance, inductance, flux linkage, pole pairs, and inertia should be kept aligned before interpreting torque/DC balance.
- If no-disturbance operation still fails to settle, inspect tail-window `T_aero - T_sh` and `T_sh + T_e` before expanding controller sweeps.
