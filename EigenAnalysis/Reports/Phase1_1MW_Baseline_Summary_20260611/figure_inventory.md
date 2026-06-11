# 阶段 1 已有图像成果清单

## 1. 小信号控制结构对比

目录：

```text
D:\博士工作\论文工作\（1）小信号模型\WT_PMSG_GFM_Electromechanical_Validation\EigenAnalysis\Results\Control_Mode_Comparison_Results
```

可用图：

| 文件 | 用途 |
|---|---|
| `baseline_damping_bar.png` | 四种结构扭振阻尼柱状对比 |
| `torsional_damping_comparison.png` | SCR/XR/Cdc 扫描下扭振阻尼变化 |
| `torsional_real_part_comparison.png` | SCR/XR/Cdc 扫描下扭振模态实部变化 |
| `overall_stability_comparison.png` | 整体最大实部稳定性对比 |
| `common_condition_sweeps.csv` | 扫描数据源 |
| `baseline_torsional_participation_top10.csv` | 参与因子表 |

建议用途：

```text
用于论文中说明 GFL-WT、GFM-GWT、GFM-MWT、GFM-MWT+AD 的阶段式因果比较。
```

## 2. DVC TypeA/TypeC/TypeC+AD 对比

目录：

```text
D:\博士工作\论文工作\（1）小信号模型\WT_PMSG_GFM_Electromechanical_Validation\EigenAnalysis\Results\DVC_Type_Comparison_Results
```

可用图：

| 文件 | 用途 |
|---|---|
| `baseline_damping_bar.png` | TypeA、TypeC、TypeC+AD 扭振阻尼柱状对比 |
| `torsional_damping_comparison.png` | DVC 结构在参数扫描下的阻尼差异 |
| `torsional_real_part_comparison.png` | DVC 结构对模态实部的影响 |
| `overall_stability_comparison.png` | 整体稳定性对比 |

最重要结论：

```text
TypeC 相比 TypeA 对当前扭振模态改善有限；
TypeC+AD 能把约 2 Hz 附近的负阻尼模态拉回稳定。
```

## 3. 控制参数扫描和模态轨迹

目录：

```text
D:\博士工作\论文工作\（1）小信号模型\WT_PMSG_GFM_Electromechanical_Validation\EigenAnalysis\Results\Control_Parameter_Scan_Results
D:\博士工作\论文工作\（1）小信号模型\WT_PMSG_GFM_Electromechanical_Validation\EigenAnalysis\Results\Mode_Trajectory_Results
```

可用图：

| 文件/类型 | 用途 |
|---|---|
| `h_mp_stability_map.png` | 虚拟惯量和有功下垂参数稳定域 |
| `dvc_stability_map.png` | DC 电压控制参数稳定域 |
| `kdamp_scan_*.png` | 附加阻尼增益对稳定性的影响 |
| `*_tracked_trajectory.png` | 参数变化时扭振模态移动轨迹 |

建议用途：

```text
用于回答“传统论文是否只看参与因子”的问题：
参与因子用于识别模态，参数轨迹用于解释参数如何移动模态和改变阻尼。
```

## 4. 小信号小扰动响应图

目录：

```text
D:\博士工作\论文工作\（1）小信号模型\WT_PMSG_GFM_Electromechanical_Validation\EigenAnalysis\Results\Small_Disturbance_Response_Results
```

可用图：

| 子目录 | 文件 | 用途 |
|---|---|---|
| `four_topology` | `wind_step_mechanical_response.png` | 风速小阶跃下四拓扑轴系响应 |
| `four_topology` | `pref_step_mechanical_response.png` | 有功参考阶跃下四拓扑轴系响应 |
| `four_topology` | `wind_step_dc_power_response.png` | 风速扰动下 DC 和功率响应 |
| `four_topology` | `pref_step_dc_power_response.png` | 有功扰动下 DC 和功率响应 |
| `four_topology` | `wind_step_torsion_fft.png` | 小信号轴系扭矩频谱 |
| `dvc_type` | `wind_step_mechanical_response.png` | TypeA/TypeC/TypeC+AD 轴系响应 |
| `dvc_type` | `pref_step_dc_power_response.png` | Type 结构对 DC 响应的影响 |
| `dvc_type` | `wind_step_torsion_fft.png` | Type 结构下轴系扭矩频谱 |

注意：

```text
这些图默认使用 0-10 Hz 低频模态截断响应，用于突出机电耦合频段。
```

## 5. 非线性验证成果

目录：

```text
D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\GFM_1MW _nonlinear\Validation_Results
```

重点子目录：

| 子目录 | 用途 |
|---|---|
| `LongTerm_1000V` | 1000V 基线长时稳定与热启动文件 |
| `Small_Perturbation_Stable_20260609` | 1000V 小扰动、FFT、Bode、三相电流频谱 |
| `Baseline_Ksh_Study_20260610` | 轴系刚度变化与小信号/非线性频率对照 |
| `Initial_State` | 热启动初值文件 |

可用于论文的非线性图：

| 图类型 | 作用 |
|---|---|
| 长时趋势图 | 证明模型可以稳定运行 |
| 小扰动时域图 | 观察扰动后功率、电压、轴系响应 |
| 机械量 FFT/PSD | 验证非线性中是否出现轴系频率 |
| 电流 FFT/PSD | 验证电气量中是否出现低频调制 |
| Bode 图 | 补充频域响应解释 |

## 6. 后续建议补图

优先补充：

1. 小信号扭振模态频率与非线性 FFT 峰值对应图。
2. GFL 非线性对照下的同类小扰动图。
3. 2MW/5MW 参数替换后的同类图。
4. 论文级总框图：结构、控制位置、扰动输入和观测输出。
