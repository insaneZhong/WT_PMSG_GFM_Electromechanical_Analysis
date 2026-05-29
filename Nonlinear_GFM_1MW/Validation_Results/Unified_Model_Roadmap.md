# GFM-MWT 统一建模与验证路线（当前版）

## 1) 已完成的统一基础
- 新增参数源定位脚本：`locate_ssm_parameters_mat.m`  
  作用：统一从小信号模型中定位 `Parameters.mat`（优先 `WT_PMSG_GFM_Electromechanical_Validation`）。
- 重写 `GFM_MWT_Nonlinear_Params.m`  
  作用：非线性参数初始化直接继承同一份小信号参数（可开关）。
- 重写 `build_same_object_parameter_table.m`  
  作用：生成“小信号 vs 非线性当前值 vs 比值”统一参数表（CSV+MD）。
- 重写 `export_schemeA_c_tuning_from_small_signal.m`  
  作用：导出方案A的 C 侧参数建议（电流环/电压环/功率环/VSG等价量）。

## 2) 目前确认的控制结构差异（核心问题）
- 小信号 GFM-GWT 中，频率环为 VSG 形式：  
  `dw/dt = (P_ref - P - (w-w0)/mp)/(2*h*w0)`（二阶状态环）
- 非线性 C 侧当前为：  
  `P环 -> w_ref`（PI输出直接修正频率）+ `Q下垂 -> E`
- 结论：两者“参数可对齐”，但“控制结构尚未完全同构”。  
  下一步要在 C 侧补齐等价 VSG 状态环，才能做到论文层面的严格对应。

## 3) 你现在可以在 MATLAB 里直接执行的顺序
在目录 `GFM_1MW _nonlinear` 下：

```matlab
T = build_same_object_parameter_table();
rec = export_schemeA_c_tuning_from_small_signal();
```

然后继续（无扰动稳态）：

```matlab
T1 = scan_steady_nondisturbance_timing_powerloop();
T2 = scan_steady_nondisturbance_voltage_loop();
```

目标判据（先稳态再扰动）：
- `Ppcc_mean` 接近 `1e6 W`
- `Udc_slope` 接近 `0`
- `OmegaG_slope` 与 `ThetaTw_slope` 接近 `0`

## 4) 下一步改造清单（按优先级）
1. 在 `grid_forming_control.c` 中将 `P环->w_ref` 改为“等价 VSG 状态更新”。
2. 将 `h/mp` 与 `k_pdc/k_idc` 的对应关系写入 C 参数头文件（保留开关宏）。
3. 仅在“无扰动稳态达标”后，加入微小扰动（不加故障/不加大风速阶跃）做线性一致性验证。

