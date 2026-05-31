# 方案A：小信号到C控制器参数映射（首版）

- Source MAT: `D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\GFM_1MW _nonlinear\..\..\..\（1）小信号模型\WT_PMSG_GFM_Electromechanical_Validation\EigenAnalysis\Parameters.mat`

| Key | Suggested value |
|---|---:|
| MOTOR_ID_KP | 1.4 |
| MOTOR_ID_KI | 0.0040666667 |
| MOTOR_IQ_KP | 1.4 |
| MOTOR_IQ_KI | 0.0040666667 |
| GSC_ID_KP | 0.16 |
| GSC_ID_KI | 0.0027666667 |
| GSC_IQ_KP | 0.16 |
| GSC_IQ_KI | 0.0027666667 |
| GSC_V_KP | 0.11 |
| GSC_V_KI | 0.0825 |
| GSC_P_KP | 1.9792034 |
| GSC_P_KI | 0.0002220661 |
| VSG_H | 506.60592 |
| VSG_MP | 1.5707963e-06 |
| VSG_W0 | 314.15927 |
| VSG_M_equiv | 318309.89 |
| VSG_D_equiv | 636619.77 |

> Note: `*_KI` values already include first-pass Ts conversion for the C discrete realization.
