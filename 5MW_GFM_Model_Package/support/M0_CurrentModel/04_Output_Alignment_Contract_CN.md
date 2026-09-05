# 当前理想化副本与M0小信号模型：输出对齐合同

本合同规定后续所有非线性—小信号比较只使用下列六个输出。

| M0输出 | 当前模型变量 | 单位 | 测量面 | 符号要求 | 已存在 |
|---|---|---|---|---|---|
| `P_PCC_W` | `stage4_Ppcc` | W | PCC export power (not GSC AC-port power) | verify: export convention differs in current model | yes |
| `Udc_V` | `stage4_Udc` | V | unique DC-link state | fixed: positive voltage | yes |
| `Tgen_Nm` | `tm_T_e` | N*m | generator electromagnetic/braking torque | verify: must match Jg*dwg=Tshaft-Tgen | yes |
| `Tshaft_Nm` | `tm_T_sh` | N*m | two-mass shaft torque | verify: must match Ksh*theta+Dsh*(wt-wg) | yes |
| `omega_rel_radps` | `tm_delta_omega_sh` | rad/s | omega_t - omega_g | fixed: measured name and short-run data confirm wt-wg | yes |
| `omega_vsg_radps` | `gfm_omega_vsg` | rad/s | absolute VSG angular frequency | fixed: absolute angular frequency | yes |

## 对齐验收顺序

1. 在稳定平衡点完成端口 abc/dq 功率与 DC-link 能量方向测试；
2. 用两质量方程确认 `Tgen`、`Tshaft` 的正方向；
3. 对同一 dPref 小阶跃比较六个输出的频率、阻尼、峰值与相位；
4. 仅当六项均通过后，才将该副本用于复转矩和论文参数扫描。
