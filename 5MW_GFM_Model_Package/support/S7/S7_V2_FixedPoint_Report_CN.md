# S7-2 离散平均模型 V2 固定点与非线性—离散SSM验证

生成时间：2026-09-03 17:55:05

## 结论等级

本轮结论为 **CONDITIONAL_REFERENCE_DIGITAL_AVERAGE**。模型 `S7A_DiscreteAvg_5MW.slx` 是从 M0 方程直接实现的离散平均参考模型，用于验证 S7-1 的离散化映射；它不是旧 C/S-Function 控制器的复刻，也不包含 PWM、开关、限幅、保护或数字采样器。

## 输入与范围

- M0工作点：`D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\5MW_Ideal\M0_5MW_Aligned_Workpoint_and_SSM.mat`
- StopTime=0.2 s，扰动时刻=0.02 s，阶跃幅值=0.0005 pu。
- 工况：D1(Ts=Ts0, tau=Ts0)、D2(Ts=Ts0, tau=1.5Ts0)、D3(Ts=2Ts0, tau=Ts0)。
- 每个工况分别施加机械转矩和电网频率小阶跃。

## V2验收结果

|工况|Ts(s)|tau(s)|固定点最大残差|归一化残差|轴系模态(Hz)|阻尼|全部极点|
|---|---:|---:|---:|---:|---:|---:|---|
|D1|0.0001|0.0001|1.475e-07|2.979e-14|2.482747|0.027106|true|
|D2|0.0001|0.00015|1.475e-07|2.979e-14|2.482746|0.027106|true|
|D3|0.0002|0.0001|2.951e-07|5.958e-14|2.482747|0.027107|true|

通道对照共 54 行；具体 NRMSE、峰值和各扰动结果见 `D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\5MW_Ideal\M3_Workpoint_Generalization\S7_V2_NL_SSM_Validation.csv`。

D1机械扰动幅值线性验收已补做（0.025%、0.05%、0.1% pu）：Te与轴系相对速度峰值斜率最大偏差为 `1.1245e-5`（约0.0012%），通过5%线性门槛；摘要见 `S7_V2_Amplitude_Linearity.csv`。

## 结果解释边界

1. 这里的“非线性”指离散平均方程的非线性时域迭代，和同一离散映射数值线性化得到的离散SSM具有同源性；因此它只能证明离散化映射内部的一致性，不能替代旧 EMT 或真实遗留数字控制器的独立验证。
2. 轴系模态由离散映射特征值提取；当前脚本未把短时域FFT误称为精确频率。
3. 固定点残差继承M0工作点的数值平衡误差；若要升级为严格 Gate V2，需要后续用真实控制器状态映射和独立平衡点重新验收。

## 产物

- 离散模型：`D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\5MW_Ideal\M3_Workpoint_Generalization\S7A_DiscreteAvg_5MW.slx`
- 模态摘要：`D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\5MW_Ideal\M3_Workpoint_Generalization\S7_V2_Discrete_Modes.csv`
- 非线性—离散SSM摘要：`D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\5MW_Ideal\M3_Workpoint_Generalization\S7_V2_NL_SSM_Validation.csv`
- 幅值线性摘要：`D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\5MW_Ideal\M3_Workpoint_Generalization\S7_V2_Amplitude_Linearity.csv`
- 综合对照图：`D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\5MW_Ideal\M3_Workpoint_Generalization\S7_V2_D1_D2_D3_NL_SSM_Comparison.png`
