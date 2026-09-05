# GFM风机多模态扰动塑形、双向机电传播及机制区域分析

## 结论边界

本报告仅使用已对齐的5 MW理想连续平均模型及同源23状态小信号模型；未加入EMT、PWM、离散PI、限幅、启动或保护。

## Gate结果

- Gate 1（3–5主导模态阶跃重构）：PASS。
- Gate 2（机制区域类别和少量NL代表点）：PASS。

## H非单调机理

TOR残差峰值 H=3 s，TOR输入投影峰值 H=3 s，当前分类为 **INPUT_PROJECTION_DOMINATED**；TOR阻尼最大相对变化 0.000%。

## 双向传播

详见 `Bidirectional_Disturbance_Transfer_Summary.csv` 与 `Bidirectional_Coupling_Matrix_Summary.csv`。使用“方向依赖扰动传递”，不把非共轭输入输出的差异误称为非互易性。

## 机制区域

连续指标采用 I_pole=|zeta_tor-zeta_ref|/zeta_ref 与 I_path=|log10(Gamma_path)|；5%和20%仅作分类可视化阈值，不是稳定边界。

## 理想连续非线性代表点

|Case|Class|Output|Full SSM NRMSE|Minimal NRMSE|Full peak error %|PASS|
|---|---|---|---:|---:|---:|---|
|SCR_H: SCR=4, value=1.5|PATH_SHAPING_DOMINATED|omega_sh|0.001721|0.003584|0.06685|1|
|SCR_H: SCR=4, value=1.5|PATH_SHAPING_DOMINATED|T_sh|0.01841|0.01713|0.8145|1|
|SCR_H: SCR=2, value=1.5|JOINT_POLE_PATH_SHAPING|omega_sh|0.002532|0.007715|0.01183|1|
|SCR_H: SCR=2, value=1.5|JOINT_POLE_PATH_SHAPING|T_sh|0.01825|0.01647|0.8161|1|
|SCR_DVC: SCR=2, value=1|WEAK_CONTROL_EFFECT|omega_sh|0.002325|0.007795|0.005347|1|
|SCR_DVC: SCR=2, value=1|WEAK_CONTROL_EFFECT|T_sh|0.01871|0.01745|0.8219|1|
