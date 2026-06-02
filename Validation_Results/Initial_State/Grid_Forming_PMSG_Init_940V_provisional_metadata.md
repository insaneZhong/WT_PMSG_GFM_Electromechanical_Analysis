# GFM-MWT 940 V 临时热启动快照元数据

- 状态: 临时非线性基础工况，尚未完成同对象直流电压对齐。
- 模型: `D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\GFM_1MW _nonlinear\Grid_Forming_PMSG.mdl`
- 快照: `D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\GFM_1MW _nonlinear\Validation_Results\Initial_State\Grid_Forming_PMSG_Init_940V_provisional.mat`
- 快照时刻: 45.000 s
- Operating point 类型: `Simulink.op.ModelOperatingPoint`
- 额定功率: 1000000 W
- 风速: 12.000 m/s，无风速扰动
- AC 参考值: 563.000 V
- DC 控制参考值: 940.000 V
- DC 电容初值: 1200.000 V
- 小信号模型 Vdc 参数: 1500.000 V
- SCR: 待网络等值阻抗对齐后计算；当前非线性模型未显式参数化 SCR。
- MEX 编译时间: `2026-06-02 23:22:50`

## 控制参数

- MSC-DVC: `Kp=0.1`, `Ki=0.0007`, `Iq` 功率前馈 `0.0002 A/W`
- GSC P-f: `Kp=1e-06`, `Ki=2e-05`, 有功爬坡 `2e+06 W/s`
- GSC 电压环: `Kp=1.130973`, `Ki=0.0282743`

## 验证结果

- 冷启动: 0-30 s；热启动续跑: 30-45 s。
- `Ppcc=1000.553 kW`, 误差 `0.000553416 pu`。
- `Udc=876.026 V`, 尾段斜率 `-0.707787 V/s`。
- `omega_g` 尾段斜率 `-0.00565682 rad/s^2`。
- `omega_t` 尾段斜率 `-0.00770272 rad/s^2`。
- `T_sh` 尾段斜率 `-12006.6 N*m/s`。
- 门槛: operational=1, dc_settled=1, mech_settled=1。
- 冷启动末点与热启动首点: `Udc`, `omega_g`, `omega_t`, `theta_tw`, `T_sh`, `Pmeas`, `Ppcc` 接口差值均为 0。
