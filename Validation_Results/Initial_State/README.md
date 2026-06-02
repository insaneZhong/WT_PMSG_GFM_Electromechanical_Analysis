# GFM 初始状态快照

本目录存放由本模型自己生成的稳态 operating point 快照。快照包含
Simulink 状态和 C MEX S-Function 内部控制器状态。

## 生成方式

优先通过 `run_gfm_final_settling_validation_60s.m` 生成：

```matlab
run_gfm_final_settling_validation_60s
```

脚本会在长时无扰动验证后，把最终状态保存为：

- `Grid_Forming_PMSG_Init_candidate.mat`：每次长时运行产生的候选快照。
- `Grid_Forming_PMSG_Init.mat`：仅当电气与机械稳态判据全部通过时生成。

也可以先运行：

```matlab
run_gfm_initial_state_candidate_30s
```

再分段续跑：

```matlab
extend_gfm_initial_state_candidate(15)
```

## 加载方式

使用：

```matlab
run_gfm_with_initial_state
```

或在其他脚本中显式传入：

```matlab
ctrl.InitialStateFile = fullfile(pwd, 'Validation_Results', 'Initial_State', 'Grid_Forming_PMSG_Init.mat');
```

## 注意

- 该快照只应来自当前 `Grid_Forming_PMSG` 模型。
- 控制参数、拓扑或额定值变化后，应重新生成。
- 不要把名称含 `candidate` 或 `unsettled` 的文件用于扰动试验。
