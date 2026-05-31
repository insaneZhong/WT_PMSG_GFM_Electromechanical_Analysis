# GFM-MWT 无扰动稳态调试纪要（2026-05-29）

## 1) 参数与信号链确认
- 已确认 `main.c` 当前使用参数驱动：
  - `grid_side.ref.P_active_power_ref = para1[0];`
  - `motor.ref.voltage_ref = para4[0];`
  - `OutPut[12] = grid_side.ref.P_active_power_ref;`
- 已通过“`OutPut[12]=Pref+1` 临时注入”验证编译链与观测链有效，随后已恢复正常表达式。
- `MOTOR_CONTROL1` 默认参数已固定为：
  - `Pref=1000000`
  - `Qref=0`
  - `ReferenceACVoltage=563`
  - `ReferenceDCVoltage=5000`
  - `S-Function1.Parameters = Pref, Qref, ReferenceACVoltage, ReferenceDCVoltage`

## 2) 无扰动基线（关键观测）
- 在无风速阶跃、无故障、无阻尼附加控制下，`Pref=1MW`：
  - `Ppcc` 约 `0.43 MW`（明显低于目标 1 MW）
  - `Udc` 未稳定（斜率非零，存在持续漂移）
  - 机械侧未收敛（`omega_g/omega_t/theta_tw` 末端斜率不满足稳态判据）
- 结论：当前系统**尚未形成稳定功率平衡点**，不能直接进入小扰动验证。

## 3) 已做 A/B 排查
- 功率环符号测试：
  - `w_ref = 314 - PI` 与 `w_ref = 314 + PI` 均测试过。
  - 结果：`Ppcc`提升不明显，仍远离 1MW；说明单改符号不是主矛盾。
- 功率环输出限幅测试（`±5` -> `±50`）：
  - `Ppcc`改善不明显，核心瓶颈未解除。
  - 说明问题更可能在**DC-link功率平衡/并网时序/内环可达性**。

## 4) 新增诊断探针
- 新增并保存了 3 个 ToWorkspace：
  - `idref_out`（MOTOR_CONTROL1 Out11）
  - `id_out`（MOTOR_CONTROL1 Out12）
  - `ud1ref_out`（MOTOR_CONTROL1 Out5）
- 用于判断：是功率环未给出有效电流参考，还是电压/电流环未跟踪到位。

## 5) 当前最可能的主因路径（按优先级）
1. **并网时序/同步状态**  
   `presyn_out` 在部分窗口出现异常均值（需进一步固定窗口和时序逻辑后再判定）。
2. **DC-link 能量平衡未闭合**  
   `Udc` 斜率持续偏离 0，指向 MSC->DC 与 GSC->PCC 功率不平衡。
3. **网侧内环可达性限制**  
   需结合 `Id_ref / Id / Ud1_ref` 时间历程确认是否受限幅或参考不可达。

## 6) 下一步执行顺序（论文向）
1. 固定无扰动稳态工况，先把 `presyn` 逻辑与断路器状态收敛到稳定并网。  
2. 在并网稳定前提下，闭合 `Udc` 稳态（`dUdc -> 0`）并记录可持续 `Ppcc` 上限。  
3. 当 `Ppcc` 可稳定接近目标值后，再进行方案A参数映射一致性验证（小信号→C控制器）。  
4. 最后再进入小扰动验证（轴系2Hz附近模态对应验证）。

---

## 7) 新增证据（同日继续排查）

### 7.1 显式参数注入（独立重载）结果
采用“直接写 `MOTOR_CONTROL1` 与 `S-Function1.Parameters` 数值 + 每个工况独立重载”的方式，得到：

- `Pref=850000`：`Pref_out≈850000`，`Pmeas≈1.168 MW`，`Ppcc≈1.173 MW`，`Udc≈5013.7 V`，`dUdc≈+152.4 V/s`
- `Pref=1000000`：`Pref_out≈1000000`，`Pmeas≈1.156 MW`，`Ppcc≈1.161 MW`，`Udc≈5023.9 V`，`dUdc≈+120.3 V/s`

解释：
- 送功能力不是“上不去”类型，而是出现“高于命令值”的超送功趋势；
- `Udc` 仍在上升，稳态功率平衡尚未闭合。

### 7.2 功率环符号 A/B
- 临时测试 `w_ref = 314 + power_loop_pi.Out` 后：
  - 进入负送功（`Ppcc<0`）并出现频率参考异常放大趋势；
- 结论：当前实现应保持 `w_ref = 314 - power_loop_pi.Out`（已恢复）。

### 7.3 机械侧仍未满足稳态判据
- 以 4~5 s 末段斜率看，`omega_g/theta_tw` 仍未达到近零；
- 需要在功率平衡闭合后再判断轴系最终稳态。

## 8) 进一步显式工况（独立重载）
在 `w_ref = 314 - power_loop_pi.Out` 保持不变、每个工况独立重载仿真下：

- `Pref=1.00 MW`：
  - `Pmeas ≈ 1.156 MW`
  - `Ppcc ≈ 1.161 MW`
  - `Udc ≈ 5023.9 V`, `dUdc ≈ +120.3 V/s`
  - `domega_g ≈ 0.075`, `dtheta_tw ≈ -0.0040`

- `Pref=1.40 MW`：
  - `Pmeas ≈ 0.940 MW`
  - `Ppcc ≈ 0.944 MW`
  - `Udc ≈ 5054.6 V`, `dUdc ≈ +23.34 V/s`
  - `domega_g ≈ 0.070`, `dtheta_tw ≈ -0.0061`

- `Pref=1.45 MW`：
  - `Pmeas ≈ 0.793 MW`
  - `Ppcc ≈ 0.797 MW`
  - `Udc ≈ 5065.9 V`, `dUdc ≈ +5.44 V/s`（更接近 DC 稳态）
  - `domega_g ≈ 0.060`, `dtheta_tw ≈ -0.0062`

结论：
- 当前参数下存在“`Pref` 增大但 `Ppcc` 下降”的反常映射，指向功率环/频率环结构与期望目标不一致；
- 但从 `dUdc` 看，`Pref≈1.45MW` 已接近 DC 侧稳态边界，可作为后续控制结构修正前的参考工况。

## 9) 结构性发现（关键）
- `ReferenceACVoltage` 输入在当前 `grid_forming_control.c` 中**未作为主参考通道使用**：并网后执行的是
  `p->ref.voltage_ref = 563 + 3.45/100000*(Qref - Qmeas)`
  ，即 AC 电压幅值由固定基值 + Q下垂给定。
- 因此仅修改 `MOTOR_CONTROL1.ReferenceACVoltage` 对稳态结果几乎无影响。
- 后续若要实现“小信号-非线性严格对应（方案A）”，应把 AC 电压参考、VSG状态方程和功率环目标统一到同一控制结构中。

## 2026-05-29 �����Ų��¼
- ��ȷ��ģ�� InitFcn ���ڷ���ʱ���ò����������׵�����/��д����
- �ֶ��������̣����� InitFcn + ��ʽ�趨 Pref����ʹ pref_out ��ȷ��Ϊ 1.2e6��
- run_single/run_sweep ���������Գ��ֲ�������������Ϊ���㹤�������ͬ��
- �Ѹ���ͬ�������������ǰ S_base/P_wt_rated С�ź�Ϊ 1.0e6�������Ե�ǰΪ 1.2e6����Ҫͳһ��

- ���ֹ�����ѭ����ͬһ����������� bdclose+clear mex+����InitFcn���£��õ��������㣺
  0.85MW -> Ppcc��0.147MW, pmeas��0.145MW
  1.00MW -> Ppcc��0.432MW, pmeas��0.430MW
  1.20MW -> Ppcc��0.755MW, pmeas��0.751MW
  �� Udc_slope>0, OmegaG_slope>0, ThetaTw_slope>0��δ��̬����
- �ý��˵������ǰ�������ɽ�����Pref���ӵ��͹����ƣ���δ�ﵽ1MW��̬�͹��������һ�е��/ֱ��������Ư�ơ�

- �ѽ� schemeA ���Ǹ�Ϊ���뿪�أ�Ĭ�Ϲرգ������ָ����ߣ�1MW/3s �� pref_end=1e6, pmeas_tail��0.43MW, dUdc��+183V/s��
- ������ۣ����� w_ref ���Ų��ɽ⣻���ʻ�����ӳ���ֱ��ʧ�ȡ�

