# 5 MW 两质量块构网型 PMSG 稳定版本

本版本以 `Grid_Forming_PMSG5MW_Liu2024_TwoMass.slx` 为唯一主模型，包含：

- 5 MW、690 V、1500 V 直流母线构网型 PMSG；
- 两质量块传动轴系；
- `Cp(lambda,beta)` 气动模型；
- 额定以下 `Kopt*omega^3` MPPT；
- 0.75–0.98 pu 转速区间的 Region 2.5 平滑过渡；
- 额定以上 5 MW 恒功率和桨距控制；
- MSC Type-c 直流电压控制；
- GSC VSG 和 Q-V 下垂控制；
- 气动与电功率协调启动。

## 运行环境

- MATLAB/Simulink（本版本在 Windows 上验证）；
- MinGW64 C 编译器仅在重新生成 S-Function 时需要；
- 仿真固定步长为 1 us，因此 60 s 开关级 EMT 验证耗时较长。

## 直接运行

```matlab
cd('D:/博士工作/论文工作/（1）构网型风电机组/构网型风电机组/GFM_1MW _nonlinear')
report = run_liu2024_5mw_full_load_validation();
```

统一参数入口为 `Liu2024_5MW_Params.m`。若修改控制器编译参数，执行：

```matlab
rebuild_liu2024_5mw_controller();
```

## 已验证额定工况

- 风速：12.20 m/s；
- 并网功率：5.0013 MW；
- 直流电压：1499.1 V；
- 风轮/发电机平均转速：1.3270 rad/s；
- 功率斜率：-0.049 kW/s；
- 直流电压斜率：0.187 V/s；
- 转速斜率：约 -0.00160 rad/s^2；
- 扭转角斜率：7.71e-6 rad/s；
- 最大调制度：0.547；
- 58–60 s 综合验收：`all_steady_pass = true`。

最新验证数据保存在 `Validation_Results/liu2024_5mw_active_run.mat`，每次运行验证脚本时覆盖该文件，不生成新的模型副本。

## 参数来源说明

5 MW 容量、电压、直流母线、轴系与控制参数以 Liu 2024 参数体系和可信 5 MW 风机量级为基础；叶轮半径采用 63 m。为适配本开关级 EMT 对象的实际损耗，额定验证风速取约 12 m/s，并对电感、控制增益、启动协调和桨距参数进行了可运行性整定。不能将全部参数表述为对论文表格的逐项原值照搬。
