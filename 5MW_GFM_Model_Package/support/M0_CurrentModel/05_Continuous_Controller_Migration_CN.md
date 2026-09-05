# 当前5 MW理想化副本：显式连续控制器迁移

- 唯一模型：`Grid_Forming_PMSG5MW_TwoMass_Idealized.slx`
- 已替换：`MOTOR_CONTROL1/S-Function1`
- 新控制器状态：11 个可见连续 Integrator
- 已移除：MEX 内部状态、控制采样、SVPWM、数字延迟、限幅、PLL/预同步、主动阻尼。
- 保留：原始两质量轴系、PMSG、LCL、电网、测量面和理想VSC端口。
- 后续门槛：先验证功率面/DC-link与平衡点，再比较六个输出的小扰动。
