# 1MW 基准参数状态与后续替换清单

## 1. 小信号与非线性参数同步状态

当前非线性脚本 `GFM_MWT_Nonlinear_Params.m` 默认加载小信号生成的 `Parameters.mat`，并继承主要风机机械/气动参数：

```text
S_base / P_wt_rated
omega_g0 / omega_m0
v_w0
J_t
J_g
K_sh
D_sh
D_t
D_g
T_e0
D_aero
K_v_aero
```

因此机械轴系和气动侧在当前阶段基本是同一对象。

## 2. 当前基准参数

| 参数 | 当前值 | 类型 | 说明 |
|---|---:|---|---|
| `S_base` | 1e6 VA | TEMP/USER | 当前 1MW 算例基准 |
| `V_LL` | 690 V | USER | 交流线电压 |
| `Vdc` | 1500 V | USER | 小信号侧额定 DC 电压；非线性当前工作基线为 1000 V |
| `f_base` | 50 Hz | USER | 电网频率 |
| `fsw` | 4 kHz | DESIGN | PWM/开关频率 |
| `omega_g0` | pi rad/s | USER | 当前发电机机械转速 |
| `n_p` | 20 | USER | PMSG 极对数 |
| `R_s` | 0.0122 Ohm | USER | PMSG 定子电阻 |
| `L_d/L_q` | 1.05e-3 H | USER | PMSG dq 电感 |
| `psi_f` | 8.64 Wb | USER | 永磁体磁链 |
| `J_g` | 1.8375e5 kg*m^2 | USER | 发电机侧惯量 |
| `J_t` | 8 * J_g | TEMP/USER | 风轮侧惯量，当前按比例假设 |
| `K_sh` | 2.579e7 N*m/rad | DERIVED/USER | 由 2 Hz 目标频率反推 |
| `D_sh` | 4.105e4 N*m*s/rad | DERIVED/USER | 由 1% 阻尼比反推 |
| `v_w0` | 12 m/s | USER | 基准风速 |
| `D_aero` | 1.013e5 N*m*s/rad | DERIVED/USER | 简化气动阻尼 |
| `K_v_aero` | 7.958e4 N*m/(m/s) | DERIVED/TEMP | 简化风速-转矩增益 |
| `K_beta_aero` | 0 | TEMP/USER | 当前低于额定/无桨距假设 |
| `D_t/D_g` | 0.005 * D_aero | TEMP/USER | 自阻尼假设 |
| `C_dc` | 1.5e-3 F | USER/TODO | 小信号侧 DC 电容；非线性 C 头文件曾存在不同值，需要继续统一 |

## 3. 后续必须替换或校准的参数

### 3.1 风机物理参数

| 优先级 | 参数 | 当前问题 | 建议 |
|---|---|---|---|
| 高 | `J_t` | 当前按 `8*J_g` 假设 | 用 2MW/5MW 文献或厂家数据替换 |
| 高 | `K_sh` | 当前由 2 Hz 反推 | 用轴系刚度或文献轴系频率重算 |
| 高 | `D_sh` | 当前由假定阻尼比反推 | 用文献阻尼比或辨识结果 |
| 高 | `C_dc` | 小信号与非线性仍需统一 | 统一到实际 DC 电容和工作电压 |
| 中 | `D_aero` | 简化线性化 | 基于 Cp-lambda-beta 曲线求偏导 |
| 中 | `K_v_aero` | 简化三次方关系 | 基于气动模型运行点线性化 |
| 中 | `K_beta_aero` | 当前置零 | 额定以上或桨距控制实验中必须补充 |
| 中 | `D_t/D_g` | 人为比例阻尼 | 与轴系阻尼一起校准 |

### 3.2 控制器参数

这些不是风机物理参数，但换容量等级后必须重新整定：

```text
h / mp
k_pq / k_iq
k_pdc / k_idc
k_pdc_gfl / k_idc_gfl
k_pm / k_im
K_damp
f_damp / zeta_damp / T_lead_damp
```

## 4. 换 2MW/5MW 时的推荐流程

1. 建立新的参数表，不直接覆盖当前 1MW 基准。
2. 先统一额定值：`S_base`、`V_LL`、`Vdc`、`C_dc`。
3. 替换 PMSG 参数：`n_p`、`R_s`、`L_d/L_q`、`psi_f`、额定转速。
4. 替换机械参数：`J_t`、`J_g`、`K_sh`、`D_sh`。
5. 重算运行点：`T_e0`、`i_m_q0`、`theta_tw0`。
6. 重新生成小信号矩阵和特征值结果。
7. 重新整定非线性模型，使无扰动稳定。
8. 重新做小扰动和 FFT 对应验证。

