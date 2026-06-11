# MSC-DVC Type a / Type c 对比说明

## 运行入口

主程序：

`D:\博士工作\论文工作\（1）小信号模型\WT_PMSG_GFM_Electromechanical_Validation\EigenAnalysis\Compare_DVC_TypeA_TypeC_Run.m`

需要先保证以下模型矩阵文件已经生成并复制到 `EigenAnalysis`：

- `Unified_WT_PMSG_VSG_TypeA.mat`
- `Unified_WT_PMSG_VSG_TypeC.mat`
- `Unified_WT_PMSG_VSG_TypeC_Damping.mat`

对应生成脚本位于：

- `Generate_SSM\WT_PMSG_VSG_TypeA_Model_Export.m`
- `Generate_SSM\WT_PMSG_VSG_TypeC_Model_Export.m`
- `Generate_SSM\WT_PMSG_VSG_TypeC_Damping_Model_Export.m`

## 模型含义

`GFM-MWT-TypeA`：MSC 侧采用直流电压反馈型 DVC，即机侧 q 轴电流参考只由直流电压误差产生。

`GFM-MWT-TypeC`：MSC 侧采用 Type c DVC，在直流电压反馈基础上加入输出有功功率前馈，形式为：

`Delta i_m_q_ref = G_dvc(s) * Delta v_dc + K_ff * Delta p_m`

`GFM-MWT-TypeC+AD`：在 Type c 基础上加入 APCAD 阻尼控制，用于改善轴系扭振模态阻尼。

## 本次基准结果

本次结果采用 `beta_v = 0`，即关闭电流控制器中的电容电压前馈项。该设置用于消除 LCL 谐振频率附近的高频不稳定模态。

| 模型 | 扭振频率 / Hz | 扭振实部 sigma | 阻尼比 | 全系统稳定 |
|---|---:|---:|---:|---|
| GFM-MWT-TypeA | 2.0011 | 0.1728 | -0.01374 | 否 |
| GFM-MWT-TypeC | 2.0011 | 0.1723 | -0.01370 | 否 |
| GFM-MWT-TypeC+AD | 1.9538 | -0.1424 | 0.01160 | 是 |

## 结果解释

Type a 到 Type c 的变化主要影响 MSC-DVC 与直流链路/有功功率平衡之间的耦合。当前参数下，Type c 使 2 Hz 附近扭振模态实部从 `0.1727` 降到 `0.1722`，阻尼比从 `-0.01374` 提高到 `-0.01370`，改善幅度很小。

APCAD 加入后，2 Hz 附近扭振模态实部从正值变为负值，阻尼比从负值变为正值，说明它对机电耦合扭振模态有明确抑制作用。

与 `beta_v = 0.5` 的旧结果相比，当前 `beta_v = 0` 已消除 2.2 kHz 附近由 LCL/延时/内环耦合引起的高频不稳定模态。因此 `GFM-MWT-TypeC+AD` 在基准工况下已经实现全模态稳定。

需要注意：TypeA 与 TypeC 仍为不稳定，是因为 2 Hz 轴系扭振模态本身仍为负阻尼；这正好说明仅改变 MSC-DVC 的 Type a / Type c 不足以抑制机电耦合振荡，必须引入面向轴系模态的附加阻尼控制。

## 高频不稳定处理说明

旧结果中最大实部模态约为 `1298.8 +/- j13771.6`，频率约 `2191.8 Hz`。参与度最高的状态是 `Vcf_d/Vcf_q`、`i1_d/i1_q`、`x_del1~x_del6`、`ig_D/ig_Q`，对应 LCL 滤波器、电流环和延时环节，而不是轴系状态。

当前将 `beta_v` 从 `0.5` 调为 `0` 后，该高频模态不再主导系统稳定性。该调整属于控制参数调整，不改变模型拓扑。

## 后续建议

后续应继续在 `beta_v = 0` 的稳定基线上扫描 `SCR`、`XR`、`h`、`mp`、`k_ff_msc_typec` 和 `K_damp`。机电耦合主线仍以 `theta_tw`、`omega_g`、`omega_t` 的参与度和 2 Hz 附近模态轨迹作为核心证据。
