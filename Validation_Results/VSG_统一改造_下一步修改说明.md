# GFM 控制结构统一（下垂 -> VSG）下一步修改说明

## 1) 现状结论
- `Grid_Forming_PMSG.mdl` 的 `MOTOR_CONTROL1` 当前频率环核心是 `P_ref - P_meas -> PI -> w_ref`，属于下垂/P环式。
- 小信号模型的主线是 VSG（`h/mp`），两者控制结构不一致。

## 2) 建议的 C 侧改造（方案 A）
在 `grid_forming_control.c` 的并网后分支（`Pre_syn == 1`）增加模式开关：
- 模式0：保留现有 `power_loop_pi` 路径（便于回归）。
- 模式1：改成 VSG 摆动方程：
  - `w_dot = (P_ref - P_meas - D*(w-w0)) / M`
  - `w = w + Ts * w_dot`
  - `theta = theta + Ts * w`

参数映射建议：
- `M = 2 * h * wn`（来自小信号）
- `D = 1/mp` 或按小信号阻尼目标换算
- `w0 = wn`

## 3) 先后顺序
1. 先实现模式开关，默认仍走模式0，保证可回归。  
2. 打开模式1后，在无扰动下检查 `w_ref`、`P_meas`、`Udc` 是否收敛。  
3. 稳态通过后，再上小扰动（频率/有功微小阶跃）验证 2Hz 机电耦合响应。  

## 4) 本轮已完成的基础
- 已实现“同一对象参数”自动继承入口（`GFM_MWT_Nonlinear_Params.m`）。
- 已生成参数对照表：`same_object_parameter_table.csv/.md`。
- 已验证继承后模型仍可运行。

