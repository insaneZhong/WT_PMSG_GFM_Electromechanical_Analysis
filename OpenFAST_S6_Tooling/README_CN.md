# M3 / OpenFAST S6 工具程序

本目录只保存可复用的 MATLAB 程序，不保存 OpenFAST 可执行文件、MEX/DLL、模型文件、原始时序或临时构建目录。它们用于 M3“反例驱动的机电耦合机制有效域与跨模型稳健性研究”的 S6 阶段诊断。

## 文件

- `run_m3_workpoint_generalization.m`：M3 跨运行点主入口，按 `StopAfter` 选择 S1–S6 阶段。
- `analyze_v500_baseline.m`：OpenFAST v5.0.0 周期线性化与 MBC/Floquet 基线审计。
- `analyze_frozenwake_torque_probe.m`：冻结尾流 OpenFAST 独立转矩探针；只用于诊断，不是 Simulink 联合仿真。

## 使用边界

这些程序默认从原 5 MW `5MW_Ideal` 工程的目录结构读取模型、运行文件和 `matlab-toolbox`。因此将本目录克隆到其他位置后，需要把工程根目录加入 MATLAB 路径，或按脚本中的相对路径准备对应测试资产；本目录本身不是自包含模型包。

当前 S6 结果只表示：OpenFAST 柔性机械线性模型与 M3 电气小信号模型的混合诊断。它不等同于 Simulink—OpenFAST 时域联合仿真，也不能单独证明 GFM 必然恶化轴系稳定性。

## 推荐调用

```matlab
% 在原 5MW_Ideal/M3_Workpoint_Generalization 目录运行
R = run_m3_workpoint_generalization('StopAfter','S6', ...
    'S6OpenFASTRoot','<OpenFAST 运行资产目录>', ...
    'S6Variant','FROZEN_WAKE');
```

独立 OpenFAST 探针在其测试目录中运行：

```matlab
S = analyze_frozenwake_torque_probe();
```

若输出 `NO_ISOLATED_DECAY`，只能记录“有限响应可观察”，不得把结果当作可辨识阻尼或联合验证证据。

## 版本与资产

- OpenFAST 源码基线：v5.0.0，提交 `2895884d2be01862173c88d70f86b358d2f1a50a`。
- 当前仓库不提交预编译二进制；请从官方发布页取得 OpenFAST Windows 资产，并在本地临时目录配置。
- 本次环境验证：官方 v5.0.0 `FAST_SFunc.mexw64` 可被 MATLAB R2024b 识别为 Level-2 S-Function；预编译包的 `OpenFAST-Simulink.dll` 需在本地按 MEX 依赖名提供 `OpenFAST-Simulink_Matlab_Release.dll`（可用硬链接，不必复制数据）。这只证明 MEX 可加载，不等同于已完成 Te→OpenFAST→转速的时域联合 Gate。
- S6 之前的 M3 结果仍需在原 5 MW 工程中按报告中的 Gate 状态解释，不能仅凭本目录程序推断论文结论。
