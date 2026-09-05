# 实验事实与后续论文支撑矩阵

| Dimension | Existing-style analysis | Current work evidence |
|---|---|---|
| Torsional pole | eigenvalue/damping | 同一或近似极点下仍可出现不同的扰动响应；采用 I_pole 与 I_path 区分。 |
| Complex torque | G_Te,omega_g | 当前架构复转矩近似相同，仍通过残差和多模态叠加解释响应差异。 |
| DVC | damping effect | 扫描同时记录DVC通道增益、极点指标和扰动残差。 |
| GFM architecture | controller comparison | GWT在MSC iq*节点截断，MWT保留Udc->iq*->Te路径。 |
| Modal analysis | torsional mode | 3–5个TOR/DC/SYNC/GSC/SPEED模态重构时域响应。 |
| Coupling direction | machine/grid interaction | Grid->Machine 与 Machine->Grid 的方向依赖传递、PCC可观测性摘要。 |
| Grid strength | damping sensitivity | SCR-H/SCR-DVC机制区域区分路径塑形和共同塑形。 |

本文档仅记录当前实验事实，不宣称“首次”或“文献从未提出”。
