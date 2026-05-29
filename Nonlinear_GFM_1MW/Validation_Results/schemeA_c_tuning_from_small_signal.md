# 方案A：小信号到C控制器参数映射（建议首版）

- 来源：`D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\GFM_1MW _nonlinear\..\..\..\（1）小信号模型\WT_PMSG_GFM_小信号分析_最新整理包_20260526\EigenAnalysis\Parameters.mat`

| 参数 | 建议值 |
|---|---:|
| MOTOR_ID_KP | 1.7 |
| MOTOR_ID_KI | 0.0050833333 |
| MOTOR_IQ_KP | 1.7 |
| MOTOR_IQ_KI | 0.0050833333 |
| GSC_ID_KP | 0.23333333 |
| GSC_ID_KI | 0.00991875 |
| GSC_IQ_KP | 0.23333333 |
| GSC_IQ_KI | 0.00991875 |
| GSC_V_KP | 0.668 |
| GSC_V_KI | 0.4008 |
| GSC_P_KP | 1.9792034 |
| GSC_P_KI | 0.0002220661 |
| VSG_H | 506.60592 |
| VSG_MP | 1.5707963e-06 |
| VSG_W0 | 314.15927 |
| VSG_M_equiv | 318309.89 |
| VSG_D_equiv | 636619.77 |

> 说明：含 `*_KI` 的值已按 C 控制器 `Ts=0.00025` 的离散实现做了首版换算，后续需闭环验证再细调。
