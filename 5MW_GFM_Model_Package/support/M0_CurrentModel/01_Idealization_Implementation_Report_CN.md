# 当前5 MW模型理想化副本实施报告

- 副本：`Grid_Forming_PMSG5MW_TwoMass_Idealized.slx`
- 原始模型：未修改
- 控制器：`main_5mw_idealized.mexw64`
- 控制器执行：每个4 us主步执行，PI状态更新步长取4 us
- VSG：严格VSG、物理相对角、启动PLL/接管分支关闭
- 主动阻尼：关闭
- 重复GSC直流能量修正：关闭
- 平均VSC命令延迟：6个延迟块/变流器均改为直通
- 外部Saturate块：10个移至非作用区
- powergui连续模式：1
- 求解器Variable-step：1

## 当前边界

本副本已去除控制器的PWM调度、SVPWM输出对控制电压的依赖、命令延迟和限幅路径；但控制器仍以专用MEX形式承载内部PI/VSG状态，因此“严格同源小信号”验收前还必须将这些内部状态迁移为显式连续Integrator，并逐项核对功率面。
