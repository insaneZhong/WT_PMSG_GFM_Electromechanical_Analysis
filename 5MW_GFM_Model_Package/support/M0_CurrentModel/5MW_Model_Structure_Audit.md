# 5 MW 两质量块构网型 PMSG 结构与参数审计

审计对象：`Grid_Forming_PMSG5MW_Liu2024_TwoMass.slx`

审计日期：2026-07-20
冻结提交：`cabdbdc86393801c202683620a8d1ee98f8c0bd5`

## 1. 审计结论

- 模型更新编译通过；
- `audit_5mw_parameter_residuals()` 通过；
- 模型不加载旧 `xInitial`，机械初值由当前 5 MW 参数入口生成；
- PMSM、直流电容、额定功率、直流参考和控制器 MEX 均与当前参数文件一致；
- 动态 MPPT 功率参考、两质量块反馈和 VSG 接管链路均实际接通；
- 当前控制器是 VSG 构网控制，不是旧 P-f PI 分支；
- LVRT 通用代码仍在基础源文件中，但当前 MEX 的阈值、限流和增益组合使其禁用；
- 当前 5 MW 稳态基线可以保留，下一阶段应补齐实验入口和诊断汇总，不需要重新迁移额定参数。

## 2. 实际能量与信号链

```text
wind_speed, omega_t, beta
        -> Cp(lambda,beta), Paero, Taero
        -> Drivetrain_TwoMass(Jt,Jg,Ksh,Dsh)
        -> omega_g
        -> PMSM electrical plant
        -> MSC -> DC link -> GSC(VSG) -> PCC/grid

omega_t -> MPPT/Region2.5/rated manager -> P_ref_MPPT
P_ref_MPPT -> MOTOR_CONTROL1 input 18 -> VSG active-power command
omega_t - omega_g -> MOTOR_CONTROL1 input 17 -> MSC active damping iq
PMSM Te -> TeEffective -> Drivetrain_TwoMass input 2
Drivetrain omega_g -> PMSM Speedin
```

### 两质量块接口

| 端口 | 实际连接 | 结论 |
|---|---|---|
| `Taero` | `TaeroEffective -> Drivetrain_TwoMass` | 已接通 |
| `Te` | `TeEffective -> Drivetrain_TwoMass` | 已接通 |
| `omega_t` | 轴系输出至气动/MPPT与日志 | 已接通 |
| `omega_g` | 轴系输出至 PMSM `Speedin`、阻尼与日志 | 已接通 |
| `theta_tw,Tsh` | 轴系输出至日志 | 已接通 |

PMSM 的内部机械输入仍保留兼容常数，但外部 `Speedin` 由两质量块的 `omega_g` 驱动；因此实际转速状态位于外置轴系。

## 3. 构网控制审计

### PLL 与 VSG 接管

- `Pre_syn` 在 1.75 s 置位；
- 预同步前 PLL 估计电网角度与频率；
- 1.75--2.25 s 之间冻结同步角并按额定频率推进；
- 2.25 s 后 `gfm_enabled=1`；
- 构建脚本设置 `ENABLE_VSG_EQUIV_WREF=1`；
- 2.25 s 后频率状态由 VSG 摆动方程更新，相角由该内部频率积分得到。

实际 VSG 方程实现为：

```text
p_err = Pref_ramped - Ppcc_filtered
dw = w0*(p_err - (w_vsg-w_anchor)/mp)/(2*H*Sbase)
w_vsg(k+1) = w_vsg(k) + Ts*dw
theta(k+1) = theta(k) + Ts*w_vsg
```

### Q-V 控制

GFM 接管后：

```text
E_ref = E_nominal + Kqv*(Qref-Qpcc_filtered)
```

随后通过交流电压环生成 d/q 电流参考，再进入 GSC 电流环。Qref 当前是控制器掩膜参数，默认值为 0 var。

### MSC Type-c DVC

- PCC 有功功率低通值作为前馈；
- 功率参考按 `P/omega_g` 进行转矩/电流归一化；
- 直流电压 PI 对前馈电流进行校正；
- `omega_t-omega_g` 经编译增益与限幅形成附加 `iq` 阻尼量；
- DVC 在 1.25 s 使能，早于 GFM 接管，以建立直流侧能量平衡。

## 4. 气动、MPPT 与桨距

`Wind_Turbine_Aero_MPPT_Pitch` 包含：

- `Wind_Aero`：二维 `Cp(lambda,beta)` 查表；
- `MPPT_Power_Manager`：额定以下 `Kopt*omega_t^3`；
- 0.75--0.98 pu Region 2.5 平滑过渡；
- 额定以上 5 MW 恒功率管理；
- `Pitch_Controller`：转速 PI、桨距角与速率限制；
- `Startup_Coordinator`：气动释放与电气功率爬坡协调。

当前审计只证明结构与接口存在；全风速 MPPT/桨距性能不属于本阶段验收。

## 5. 诊断量审计

模型已有控制器 37 路诊断向量和机械/气动日志。已直接获得：

- Pref、Ppcc、Qpcc、VSG `w_ref/theta_ref`；
- GSC `Ud/Uq`、`Id_ref/Id`、`Iq_ref/Iq`、PCC d/q 电压；
- PLL角、预同步状态；
- MSC `Iq_ref/Iq`、DVC 输出、`Ud/Uq` 及 MSC 调制度；
- Udc、Taero、Te、Tsh、omega_t、omega_g、theta_tw；
- Cp、lambda、beta、Paero 和 MPPT 原始/管理后功率。

以下量无需扩大 S-Function 端口，可在验收脚本中计算：

- GSC 调制度：`1.5*hypot(Ud_gsc,Uq_gsc)/Udc`；
- GSC/MSC 电流跟踪误差；
- 电流矢量、调制度和电压指令限幅裕度；
- 机械功率 `Taero*omega_t`、电磁功率 `-Te*omega_g`；
- 轴系储能变化与直流电容能量变化；
- Paero--Ppcc 的能量/损耗残差。

若后续无法从现有量判断 PI 是否持续饱和，再增加显式饱和标志；在此之前不改变 S-Function 宽度。

## 6. LVRT状态

当前构建清单中：

- `LvrtUdcThreshold=1e9 V`；
- `LvrtPccVoltageThreshold=0 V`；
- MSC LVRT 电流/调制度/功率增益均为 0；
- GSC LVRT 电流优先和积分冻结均关闭。

结论：当前模型没有有效低压穿越动作，符合正常稳态与构网属性验证的边界。

## 7. 已发现问题与处理顺序

| 编号 | 问题 | 影响 | 处理 |
|---|---|---|---|
| A1 | 参数文件曾错误注明 VSG 未启用 | 论文和小信号结构会选错 | 已修正注释，不改功能 |
| A2 | 当前统一运行脚本只支持风速、轴阻尼和功率爬坡覆盖 | 无法统一执行 Qref/SCR/P-f 实验 | 增加参数化实验配置 |
| A3 | 电网源支持幅值/频率变化，但默认扰动为关闭状态 | P-f/Q-V 尚无可重复扰动入口 | 通过同一模型的掩膜参数覆盖实现 |
| A4 | SCR 固定写在统一参数函数中 | 弱电网矩阵无法用运行时变量切换 | 增加 `scr_override`，同步计算 Rg/Lg |
| A5 | 额定验收只直接检查 MSC 调制度 | GSC 电压裕度证据不足 | 从现有 Ud/Uq/Udc 计算并加入断言 |
| A6 | Paero 与 Ppcc 的差额尚未分项闭合 | 影响物理合理性说明 | 加入机械、电磁、直流和并网能量表 |
| A7 | 额定转速 1.327 rad/s，相对 1.27 rad/s 偏高约 4.5% | 通过现门槛但论文余量偏小 | 先记录，不在构网审计阶段盲调 |
| A8 | 顶层仍有兼容块、未使用控制输出和旧命名 | 编译提示与可读性问题 | 仅在确认不参与闭环后逐项清理 |
| A9 | 原 `mp=6.28e-8 rad/s/W` 相当于约 0.1% 下垂，0.2 Hz 对应约 20 MW | P-f 扰动立即触发限流和能量失衡 | 改为可解释的 5% 下垂 `mp=3.14e-6 rad/s/W` 后重新验证 |
| A10 | 原摆动方程缺少 `S_base`，使用 `1/(2*H*w0)` 缩放 | 5 MW 频率状态约快 50.7 倍，依赖非物理硬下垂维持启动 | 改为 SI 形式 `w0/(2*H*S_base)` |
| A11 | 直接从 2.25 s 启用物理 VSG 动态会破坏既有冷启动 | 启动过程与正式构网参数互相制约 | 0--15 s 保留 commissioning 动态，15--20 s 平滑过渡，20 s 后仅使用物理运行方程 |
| A12 | 频率短断面曾受直流暂态混杂而误判功角方向；15--21 s 无扰动断面确认 `Ppcc-Pref` 在运行段形成正反馈 | 不应镜像振荡器坐标或反转正常功率误差 | 保持 PWM/Park 角度连续，运行段使用 `Pref-Ppcc`，只对启动/运行动力学做15--20 s连续混合 |

## 8. 本阶段下一项工作

在同一个模型上建立统一实验入口，支持：

1. 额定回归；
2. 网频正负扰动；
3. Qref 与电网电压正负扰动；
4. SCR=8、4、2；
5. 统一提取 GSC/MSC 限幅、能量和轴系指标。

每新增一个入口先执行 6 s 编译/接管检查，再执行额定 60 s 回归；只有异常断面证明需要时才修改控制参数。
