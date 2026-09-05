# S7-5 遗留数字控制器调用链审计（L0）

生成日期：2026-09-04  
审计范围：`CurrentModel_Idealized` 中的遗留 C/S-Function 源码与当前 S7 参考数字实现。  
审计性质：静态源码审计；尚未声称实际遗留 MEX 与参考模型已完成一步映射。

## 1. 审计对象

主要源码：

- `main_legacy_ad_base.c`：S-Function 入口、输入/输出、调度器、工作点快照；
- `motorcontrol_legacy_ad_base.c`：MSC-DVC、MSC 电流环、主动阻尼入口；
- `grid_forming_control.c`：PLL/预同步、GFL/GFM 分支、VSG、Q–V、电压/电流环、功率滤波；
- `svpwm.c`：SVPWM 计算、过调制缩放、脉冲段生成；
- `motorcontrol_legacy_tunable.h`、`grid_forming_control.h`、`svpwm.h`：结构体、宏和默认参数；
- `build_idealized_controller_mex.m`：理想化编译宏，不等于生产 EMT 编译配置。

源码快照（SHA-256，便于复核）：

| 文件 | SHA-256（2026-09-04） |
|---|---|
| `main_legacy_ad_base.c` | `6F71546BBF1A2A1B377A288E5E28F46B41EF71DAF26F23FC5BEEE6CD8DA73422` |
| `motorcontrol_legacy_ad_base.c` | `B69B806EE05FFCBBBC6344D5A56886F3DF61571955B2058AB701152D802F594D` |
| `grid_forming_control.c` | `357FC14D37DA707D9258742E5E32E952E4BC2EEC8182AA94960037ABE4EA8007` |
| `svpwm.c` | `BC36D4B9372D16C19069F5633BF2306248BF5EE7FEDC60D2EB63A8A26D0415D1` |

## 2. 总体调用链

```text
Simulink 1 us sample hit
        |
        v
main_legacy_ad_base.c: mdlOutputs (413--866)
        |
        +-- ControlTimerFlag 分支（443--450）
        |       |-- 参考值、Udc 参考、DC 能量修正、LVRT/可选扰动
        |       |-- motor_control(&motor)
        |       |       |-- MSC-DVC PI（pwm_speed_pi）
        |       |       |-- iq* = -前馈 - DVC输出 + 主动阻尼
        |       |       |-- RotorPos 乘极对数、abc->dq
        |       |       |-- MSC d/q 电流 PI
        |       |       `-- 解耦前馈，输出 Us_alpha/beta
        |       |
        |       |-- !IDEAL_CONTINUOUS_CONTROLLER 时调用 svpwm1.calc
        |       |
        |       |-- grid_side_control(&grid_side)
        |       |       |-- PCC 电压 abc->alpha/beta->dq
        |       |       |-- PLL/预同步/并网后测量 PLL
        |       |       |-- PCC P/Q 计算与 20 Hz 数字滤波
        |       |       |-- GFL、droop-P/f 或严格 VSG 分支
        |       |       |-- Q-V、电压外环、电流内环
        |       |       `-- 输出 GSC Us_alpha/beta
        |       |
        |       `-- !IDEAL_CONTINUOUS_CONTROLLER 时调用 svpwm2.calc
        |
        +-- 每次 mdlOutputs 都推进 ControlTimerCounter（710--723）
        |       `-- 到周期后置 ControlTimerFlag=1，并更新 PWM 周期
        |
        `-- 每个 1 us tick 推进 FPGA 脉冲段（725--764），输出 12 个门极
                +-- OutPut[0:5] MSC 门极
                +-- OutPut[6:11] GSC 门极
                `-- OutPut[12:36] 诊断；IDEAL_AVG_OUTPUTS 时 [37:40] 连续电压指令
```

## 3. 调度与更新顺序

1. S-Function sample time 由 `LEGACY_SFUNCTION_SAMPLE_TIME_S` 决定，生产默认为 `1e-6 s`（`main_legacy_ad_base.c:163--168, 373--376`）。
2. 生产路径只有 `ControlTimerFlag==1` 时才执行控制器（`443--450`）。控制周期来自 `svpwm1.Val.PwmVecterPeriod`，不是固定的 Simulink sample time；计数条件为 `ControlTimerCounter>ControlTimerPeriod`（`710--723`），因此不能简单等同于精确的 100 us。
3. 每次控制中先执行 MSC，再执行 GSC；MSC 命令随后进入 `svpwm1`，GSC 命令随后进入 `svpwm2`（`671--701`）。
4. `mdlUpdate` 未启用，S-Function 没有显式 Simulink 离散状态；PI、滤波器、相角、定时器和控制结构体均为 C 全局/静态状态，并通过 custom operating point 字节快照保存（`main_legacy_ad_base.c:881--1030`）。
5. `IDEAL_CONTINUOUS_CONTROLLER=1` 时强制每个 S-Function 步进入控制且 `Ts_control=LEGACY_SFUNCTION_SAMPLE_TIME_S`，同时跳过两个 SVPWM 计算（`439--450, 677--701`）。这是 `build_idealized_controller_mex.m` 生成的理想化分支，不是生产遗留数字实现。

## 4. 关键分支和边界

### 4.1 启动、PLL 与 GFM 接管

- `grid_side_control` 在 `grid_forming_control.c:155--157` 依据 `system_Time` 置位 `Pre_syn` 和 `gfm_enabled`。
- `Pre_syn==0` 时使用 `grid_phase_angle` 驱动 PLL PI，并以 `p->Ts` 积分 PLL 角（`210--229`）。
- 并网但尚未 GFM 接管时以额定频率推进同步角（`230--244`）。
- GFM 接管后 `grid_pll_phase/freq` 只用于测量，不替换 `p->pf.thet_ref`（`245--266`）。
- GFL 分支使用测量 PLL 角频率；非 GFL 分支再选择严格 VSG 或功率 PI（`325--386`）。

### 4.2 机侧控制

- `motorcontrol_legacy_ad_base.c:64--73`：Udc-DVC PI 在 `MSC_DVC_ENABLE_TIME_S` 之前复位，之后调用 `motor_PI2_calc`。
- `78--87`：主动阻尼入口为 `legacy_omega_rel_ad`，默认增益/比例可为零，但源码路径存在；`Iq_ref` 还包含前馈和 DVC 输出。
- `91--103`：转子位置乘极对数并执行电流 abc/dq 变换。
- `105--144`：MSC d/q 电流 PI、反电势/交叉耦合前馈；`145--166` 仍保留 LVRT 电压限幅路径。

### 4.3 网侧控制

- `grid_forming_control.c:194--206`：线电压恢复相电压并执行 Clarke/Park。
- `300--311`：按 `1.5(ud id+uq iq)` 和 `1.5(ud iq-uq id)` 计算 P/Q；P/Q 滤波器在 `657--665` 内硬编码 `Ts_frequency=0.00025 s`。
- `334--380`：严格 VSG（若编译宏开启）更新 `w_vsg_state`，并积分 `p->pf.thet_ref`；否则 `382--386` 为 P/f PI。
- `451--608`：GFL 电流参考或 GFM 电压外环、电流内环，包含可选电流矢量限幅和调制限幅。

## 5. L0 结论

1. 遗留实现是“1 us S-Function 调度 + PWM 周期控制中断 + C 全局状态”的多速率数字控制器，不是一个拥有显式离散状态向量的普通 Simulink Discrete block。
2. 当前参考 S7 实现的 Forward-Euler 状态更新、固定采样、显式 ZOH/延迟与遗留 C 的真实更新顺序尚未证明相同。
3. `build_idealized_controller_mex.m` 的宏会打开严格 VSG、连续每步控制和连续电压输出；因此必须把它标为理想化编译分支，不能用它替代生产遗留 MEX 的 LC1/LC2 证据。
4. 后续一步必须先完成状态/角度/参数矩阵（L1--L3），再决定是否编译生产配置并做 C↔Replica 单步测试。

## 6. S7-5 执行记录

本轮在临时目录编译了**不带理想化宏**的生产入口：

`temp/S7_5_LegacyProduction/main_legacy_ad.mexw64`  
SHA-256：`5D5B5A8156FF3A2EADAB3D6ED5C8E4A6A8740E3F17CD3184FFA52F84D5CF19EE`

并用 20 输入、37 输出的最小 S-Function 模型完成了加载和 1 us 固定步短仿真。另以 `Udc=900 V`、`Vdc_ref=1000 V`、`system_Time=3 s` 检查了首次 MSC-DVC 更新，C 输出与 `motor_PI2_calc` 公式相差 `1.74e-7`（摘要：`temp/S7_5_LegacyProduction/S7_5_LC2_DVC_FirstStep_Summary.csv`）。

这两项属于接口/局部公式 smoke 证据；完整的 C↔Replica 单步矩阵仍待在同一 5 MW 工作点、同一输入序列和同一控制事件时间戳下完成。
