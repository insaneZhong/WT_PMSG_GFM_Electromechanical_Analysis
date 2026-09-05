# 5 MW 两质量块构网型 PMSG 基线审计与构网属性验证计划

## 1. 目标与边界

唯一主模型为 `Grid_Forming_PMSG5MW_Liu2024_TwoMass.slx`。本阶段完成：

1. 冻结并审计当前 5 MW 额定稳定基线；
2. 在不改变默认闭环行为的前提下补齐必要诊断；
3. 完成 P-f、Q-V 和 SCR 构网属性验证；
4. 形成可重复脚本、验收数据、图表和审计报告。

本阶段不进行低压穿越、单质量块对比、全风速 MPPT 验证或小信号模型重构。所有实验使用同一个 `.slx`，通过 `Simulink.SimulationInput`、参数覆盖和统一运行脚本改变工况，不生成工况模型副本。

## 2. 冻结基线

- Git 分支：`agent/5mw-two-mass-stable`
- Git 提交：`cabdbdc86393801c202683620a8d1ee98f8c0bd5`
- 基线验证窗口：60 s 仿真末端 58--60 s
- 基线结果：`Validation_Results/liu2024_5mw_active_run.mat`

| 文件 | SHA-256 |
|---|---|
| `Grid_Forming_PMSG5MW_Liu2024_TwoMass.slx` | `7A5BC86E7871DF951780DB6517EA7C2DE07078FDEE5EB5C96743D0EB5BE461B4` |
| `Liu2024_5MW_Params.m` | `2ADCB8CEC4CCC3ACE80E36EAE626425ED3B5F8550D0D6025027CD5B40DE0B793` |
| `main_legacy_liu2024_5mw_stable.mexw64` | `FBCB274E66D71386A8D523AE97A4807F139454D0158B542B1E0EFE79A56EBBFC` |
| `main_legacy_liu2024_5mw_stable_build_manifest.csv` | `6C5BF5CDF3AD22AEE716C4C8C087173896D2884A2BAB7774AE9D03A6C9242E94` |
| `run_liu2024_5mw_full_load_validation.m` | `E9D3688C5ECB026A6BBE40BEB40C3053C25A2CD1CE13E24ECAC9530B03B4D6AA` |
| `Validation_Results/liu2024_5mw_active_run.mat` | `2C8744BA4DACB748A0C06427956C8937B6E98530ABA3F979C27B1AC29121B07F` |

任何结构或控制修改后必须重新执行额定稳态回归。若回归失败，先用上述提交、校验值和异常断面定位差异，不通过复制模型规避问题。

## 3. 实施阶段与门槛

### A. 结构与参数审计

核对以下实际信号链和编译宏：

- PLL 预同步、断路器闭合和 VSG 接管时序；
- VSG 摆动方程、P-f 关系和 Q-V 下垂；
- GSC 电压环、电流环、矢量限流和调制度限幅；
- MSC Type-c DVC、PCC 功率前馈、速度归一化和电流环；
- `Te -> 两质量块 -> omega_g -> PMSM` 机械接口；
- `Cp(lambda,beta)`、MPPT、Region 2.5、桨距和启动协调；
- 附加轴系阻尼的输入、增益、限幅和符号；
- LVRT 编译禁用状态；
- 物理 SI 量、控制器内部量和标幺基值。

退出条件：形成逐项“实际启用/未启用/占位参数/论文来源/工程调整”清单；修正参数文件中“VSG 未启用”的过时注释。

### B. 非侵入式诊断扩展

优先复用现有 S-Function 诊断向量和 To Workspace 信号。仅在现有信号不足时增加诊断输出，不改变控制计算顺序或默认增益。至少记录：

- VSG 频率、相角、P/Q 参考和实际值；
- GSC d/q 电流参考、实际电流、电压指令、PI 饱和及调制度；
- MSC d/q 电流参考、实际电流、电压指令、PI 饱和及调制度；
- Udc、Pdc/Ppcc、Paero、PMSG 电磁功率及主要损耗；
- omega_t、omega_g、theta_tw、Te、Tsh、Taero；
- PLL/VSG 接管状态、限流状态和附加阻尼输出。

退出条件：短时编译和 6 s 接管检查无新增错误；新增输出不反馈进入控制链。

### C. 额定稳态回归

保持 12.20 m/s、5 MW、1500 V、SCR=4，重新运行 60 s。

| 指标 | 回归门槛 |
|---|---:|
| `abs(Ppcc-5 MW)` | < 1% |
| `abs(Udc-1500 V)` | < 2% |
| `abs(dP/dt)` | < 5 kW/s |
| `abs(dUdc/dt)` | < 5 V/s |
| `max(abs(domega_t/dt),abs(domega_g/dt))` | < 0.01 rad/s^2 |
| `abs(dtheta_tw/dt)` | < 5e-4 rad/s |
| 转矩平衡残差 | < 1% |
| GSC/MSC 调制度 | < 0.90 |
| PI/电流限幅 | 不允许末端持续饱和 |

新增回归要求：相对冻结基线的 Ppcc、Udc、omega_g 和 theta_tw 指标不得出现无解释退化。

### D. P-f 构网验证

基准为 SCR=4、Qref=0。额定稳态建立后施加网频 `-0.2 Hz` 和 `+0.2 Hz` 扰动，必要时增加 `+/-0.1 Hz` 小扰动用于线性区检查。

观测 VSG 频率/相角、有功功率、Udc、GSC 电流、机械转速和轴转矩。验收包括：响应方向正确、无持续振荡、无长期限流、扰动后回到新平衡或按下垂特性形成可解释偏差。

### E. Q-V 构网验证

分别执行 Qref `+/-0.1 pu` 和电网电压 `+/-5%`。观测 PCC 电压、Q、d/q 电流、调制度、Udc 和轴系变量。

验收包括：Q-V 响应方向正确、Q 跟踪误差和稳态电压偏差可解释、GSC/MSC 均不持续饱和、机械侧不出现被电压环激发的发散扭振。

### F. SCR 构网验证

使用参数覆盖改变同一模型的电网阻抗，验证 `SCR=8、4、2`。每个 SCR 先运行额定无扰动，再选择一个 P-f 和一个 Q-V 代表扰动。

验收包括：所有工况无发散；记录稳定时间、超调、最低阻尼表现、最大电流和最大调制度。若 SCR=2 不通过，必须先做失稳时刻断面分析，定位为同步、外环、内环、限幅、直流能量或轴系耦合问题后再整定。

## 4. 整定规则

1. 不以大范围 PI 扫描作为首个动作；
2. 每次整定前保存异常断面的 P/Q、Udc、电流参考/实际、调制度、VSG 状态和轴系状态；
3. 一次只修改一类参数，并记录物理或控制依据；
4. 每次修改后先运行触发异常的短工况，再运行 SCR=4 额定回归；
5. 不用非物理增大 `D_sh` 掩盖控制负阻尼；
6. 当前附加阻尼默认值保持不变，后续只增加显式使能/记录能力；
7. LVRT 保持禁用，不将故障穿越功能混入本目标。

## 5. 预期产物

- `5MW_GFM_BASELINE_AND_CHARACTERIZATION_PLAN.md`：本执行计划；
- `5MW_Model_Structure_Audit.md`：结构、信号、参数来源和遗留项；
- `Validation_Results/5MW_GFM_Characterization/parameter_dictionary.csv`；
- `Validation_Results/5MW_GFM_Characterization/baseline_regression.csv`；
- `Validation_Results/5MW_GFM_Characterization/pf_characterization.csv`；
- `Validation_Results/5MW_GFM_Characterization/qv_characterization.csv`；
- `Validation_Results/5MW_GFM_Characterization/scr_characterization.csv`；
- `Validation_Results/5MW_GFM_Characterization/figures/` 下的 P-f、Q-V、Udc、电流限幅和轴系响应图；
- 一个统一的参数化验证入口脚本，不创建新的 `.slx`；
- 精确选择本目标文件的 Git 提交。

## 6. 完成判定

只有同时满足以下条件才完成本目标：

1. 当前额定 5 MW 稳态回归通过；
2. VSG、Q-V、Type-c DVC、两质量块和 MPPT 的实际信号链有可核查记录；
3. P-f 与 Q-V 响应方向和稳态特性正确；
4. SCR=8、4、2 的结果已形成验收表，失败工况有明确原因和边界说明；
5. 无新增工况模型副本；
6. 报告、CSV、图表和统一运行说明齐全。
