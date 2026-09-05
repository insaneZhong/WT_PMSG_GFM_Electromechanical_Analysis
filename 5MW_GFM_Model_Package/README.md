# 5 MW构网型PMSG模型包

这是从当前研究工作区整理出的可追溯模型快照。模型按研究用途分层，不包含 `results`、`slprj`、`.slxc`、MEX 二进制或大规模时序数据。

## 当前主线

论文机电耦合、轴系模态、复转矩和扰动通道研究使用：

```text
models/M0/Grid_Forming_PMSG5MW_TwoMass_Idealized.slx
ssm/M0_5MW_Aligned_Workpoint_and_SSM.mat
```

M0 是理想连续平均模型；它与同源小信号模型对应，适合主线机理分析。M1、GFL、刚性轴和 Legacy/S7 模型只用于交叉验证或工程实现边界，不应自动替代 M0。

## 目录

```text
5MW_GFM_Model_Package/
├─ models/
│  ├─ M0/             理想连续平均主模型
│  ├─ M1/             物理平均 VSC 反事实模型
│  ├─ Comparators/    GFL 与刚性轴对照
│  ├─ Engineering/   Legacy、MPPT、Pitch 工程变体
│  ├─ S7/             Legacy 平均、热启动和同源离散参考
│  └─ Reference/      源对齐/早期参考模型
├─ ssm/               M0 同源平衡点与小信号矩阵
├─ support/           MATLAB、C/H 和验证脚本
├─ docs/              关键验收报告
└─ 模型与功能清单_5MW构网型风电机组.md
```

## MATLAB 使用

在 MATLAB 当前目录切换到本目录后运行：

```matlab
pkg = setup_5mw_gfm_model_package;
open_system(fullfile(pkg,'models','M0', ...
    'Grid_Forming_PMSG5MW_TwoMass_Idealized.slx'));
```

`setup_5mw_gfm_model_package.m` 会递归加入模型包内的 `models`、`ssm` 和 `support` 路径，但不会修改原始工作区文件。

主线 M0 的初始化、平衡点和线性化脚本位于 `support/M0_Root` 与 `support/M0_CurrentModel`；M1 脚本位于 `support/M1`。Legacy 控制器的 C/H 源文件位于 `support/Legacy`，需要时再单独编译，不能把它们当作 M0 连续控制器。

## 模型状态边界

| 模型 | 状态 | 说明 |
|---|---|---|
| M0 + SSM | 主线可用 | 5 MW 理想连续平均机电耦合研究 |
| M1 | 机制交叉验证 | 检查 `Udc` 相关平均 VSC 假设 |
| GFL/刚性轴 | 对照可用 | 比较同步方式或轴系结构影响 |
| MPPT/Pitch | 工程补充 | 额定以上和复杂风况验证 |
| S7/S7A | 实现验证 | 不等同于真实 Legacy EMT 认证 |
| EMT/热启动 | 辅助/历史 | 不纳入当前理想连续主线结论 |

## 复现原则

1. 先运行 M0 主线，再运行 M1 或对照模型。
2. 参数扫描只保存摘要；不要把 `results`、求解历史或完整时序复制进模型包。
3. 任何稳定性结论同时记录模型层级、工作点、参数范围和验证状态。
4. 不把 M0 的理想连续结果直接升级为 Legacy、EMT 或 OpenFAST 结论。

