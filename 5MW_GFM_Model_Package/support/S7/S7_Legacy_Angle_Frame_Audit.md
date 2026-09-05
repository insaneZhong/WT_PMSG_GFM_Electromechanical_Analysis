# S7-5 遗留控制器角度、坐标与功率符号审计（L2）

## 1. 结论先行

当前遗留 C 控制器与 M0/S7 参考实现不能直接视为同一角度状态空间。主要差异不是 Clarke/Park 公式本身，而是：

1. 遗留控制器保存的是 `p->pf.thet_ref` 这一**绝对 alpha-beta 指令角**，并在控制周期内用 `p->Ts*w_ref` 更新；M0 状态 `delta_v` 表示相对于电网参考的相角。
2. 预同步阶段使用 `grid_phase_angle`；GFM 接管后，`grid_pll_phase` 变为测量量，实际 Park/逆 Park 仍使用 `thet_ref`。因此“PLL 角”和“VSG 指令角”不是同一个状态。
3. 生产默认 `ENABLE_VSG_EQUIV_WREF=0`，走 P/f PI；`build_idealized_controller_mex.m` 才用宏强制严格 VSG。未锁定 MEX 编译宏之前，不能把 `w_vsg_state` 当作当前实际控制状态。

## 2. 角度和坐标路径

### 2.1 PCC 电压

`grid_forming_control.c:194--203` 先由线电压恢复相电压：

```text
ua = -(uca-uab)/3
ub = -(uab-ubc)/3
uc = -(ubc-uca)/3
```

随后使用标准幅值不变 Clarke：

\[
u_\alpha=\frac{2}{3}(u_a-\tfrac12u_b-\tfrac12u_c),\qquad
u_\beta=\frac{2}{3}\frac{\sqrt3}{2}(u_b-u_c).
\]

主反馈 Park 角为 `p->pf.thet_ref`（`200--206`）。因此 GFM 控制器的电压、电流反馈坐标随指令角旋转，而不是固定使用测量 PLL 角。

### 2.2 网侧电流和 PCC 电流

网侧电流在 `268--298` 分别 Clarke 后，以同一个 `p->pf.thet_ref` Park，得到 `Id/Iq` 和 `pcc_Id/pcc_Iq`。P/Q 定义为：

\[
P=1.5(u_d i_d+u_q i_q),\qquad
Q=1.5(u_d i_q-u_q i_d).
\]

在 d 轴对准且 \(u_q\simeq0\) 时，\(P\simeq1.5u_di_d\)，\(Q\simeq1.5u_di_q\)；但 GFL 分支的电流给定明确使用 `Iq_ref=-Q_ref/(1.5 Ud)`（`472--477`），说明无功电流正方向与某些常见 GFM 文献约定相反，必须在对齐测试中保留该负号。

### 2.3 PLL 角与 VSG 角

| 变量 | 源码位置 | 物理用途 | 是否驱动实际变流器坐标 |
|---|---|---|---|
| `grid_side.val.grid_phase_angle` | `210--229, 395--400` | 预同步 PLL 角；接管前使用 | 接管前是；接管后不是主角 |
| `grid_pll_phase` | `245--266` | GFM 接管后的测量 PLL 角 | 否，仅监测/同步可选输入 |
| `grid_pll_freq` | `245--266` | 测量电网频率 | GFL 分支使用；严格 VSG 不直接跟随 |
| `w_vsg_state` | `334--380` | 严格 VSG 频率状态 | 通过 `p->pf.w_ref` 间接驱动 |
| `p->pf.thet_ref` | `202,278,294,391--424` | 变流器指令角 | 是，Park/逆 Park 主角 |

严格 VSG 分支的更新为：

\[
\omega_{v,k+1}=\omega_{v,k}+T_s\dot\omega_v,
\qquad
\theta_{k+1}=\theta_k+T_s\omega_{v,k+1},
\]

其中源码先更新 `w_vsg_state`（`374`），再将其赋给 `p->pf.w_ref`（`379`），最后更新 `thet_ref`（`392`）。这是“更新后频率用于角度”的顺序，不能默认等同于 S7 参考实现的先保持后更新。

## 3. 启动和接管的角度不连续风险

- `Pre_syn==0`：PLL PI 调整 `grid_phase_angle`，同时 `p->pf.thet_ref` 被同步到该角（`395--400`）。
- `Pre_syn==1` 且 `gfm_enabled`：GFM 或 GFL 分支生成新 `w_ref`；严格 VSG 使用 `w_vsg_state`，P/f 分支使用 `power_loop_pi.Out`。
- `GSI_BUMPLESS_TAKEOVER` 默认关闭（由头文件宏控制），因此若编译配置未启用，电压指令从预同步分支到 GFM 分支可能发生瞬时改变。
- 角度只做一次 `±2π` 修正（`421--424`），不是通用 modulo；长时间或异常频率下不能假设其数值连续。

## 4. 生产与理想化编译的区别

`build_idealized_controller_mex.m:10--26` 明确设置：

```text
IDEAL_AVG_OUTPUTS=1
IDEAL_CONTINUOUS_CONTROLLER=1
ENABLE_VSG_EQUIV_WREF=1
VSG_POWER_ERROR_SIGN=1
VSG_EQUIV_H=3
VSG_EQUIV_SBASE_W=5e6
VSG_GRID_SYNC_ENABLE=0
```

而 `grid_forming_control.h:27--34,48--55,121--124` 的源代码默认值为：

```text
ENABLE_VSG_EQUIV_WREF=0
VSG_EQUIV_H=10
VSG_EQUIV_SBASE_W=1e6
VSG_POWER_ERROR_SIGN=+1
```

所以“源码默认”“理想化 MEX”“生产遗留 MEX”是三个必须分开的对象。当前没有完成生产 MEX 的编译元数据闭锁，不能用角度符号或 VSG 状态做最终 LC1 结论。

## 5. L2 判定

| 审计项 | 判定 | 依据 |
|---|---|---|
| Clarke/Park 数学形式 | `EQUIVALENT_LOCAL` | 两者均为标准变换，但缩放、线电压恢复和信号面仍需逐点核对 |
| P/Q 正负号 | `MISMATCH_UNVERIFIED` | 遗留 `Q=1.5(ud iq-uq id)`，GFL 给定又显式带负号 |
| VSG 频率更新 | `MISMATCH` | 宏、状态和更新后角度顺序未与 S7 参考锁定 |
| VSG 相角定义 | `MISMATCH` | 绝对 `thet_ref` 与 M0 相对 `delta_v` 不是同一坐标 |
| PLL/预同步 | `MISMATCH` | M0/S7 参考模型没有该启动分支 |
| 接管平滑 | `MISMATCH` | `GSI_BUMPLESS_TAKEOVER` 可选且默认不保证开启 |

**L2 结论：** 在 LC1/LC2 之前，必须把实际编译宏、初始角度、网格相角和 `thet_ref` 的零点写入一步测试输入；不能仅凭稳态频率接近认定角度状态已对齐。

