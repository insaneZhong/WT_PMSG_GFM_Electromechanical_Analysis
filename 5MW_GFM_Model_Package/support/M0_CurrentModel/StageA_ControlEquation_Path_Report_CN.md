# 阶段A：GWT/MWT/GFL控制方程通道结构判别

## 1. 范围与方法

本报告只使用已对齐的理想连续非线性模型及其同源23状态SSM。没有引入EMT、PWM、离散PI、采样、延迟、限流、LVRT或MPPT/Pitch动态，也没有修改公共plant和共同工作点。

为单独识别由直流电压出发的控制通道，计算时将 $U_{dc}$ 能量积分状态从状态矩阵中移除，并把其Jacobian列作为外部规定输入。由此得到

$$G_{y,U_{dc}}(j\omega)=C_r(j\omega I-A_r)^{-1}B_{U_{dc}}+D_{y,U_{dc}}.$$

同时在 $U_{dc0}\pm0.2\%$ 的局部点重复线性化。该偏移只用于排除工作点抵消，不作为新平衡点。

## 2. 实际控制方程

- **GFM-GWT：** MSC采用固定转矩/电流参考，$i_{q,MSC}^*=i_{q0}^*$，因此 $\partial i_q^*/\partial U_{dc}=0$，且不存在积分动态通道。DC-link误差进入GSC-DVC：$P_{ctrl}=P_{ref}-K_{p,gdc}(U_{dc0}-U_{dc})-\xi_{gdc}$。
- **GFM-MWT与GFL：** MSC-DVC为 $i_q^*=K_{p,dc}(U_{dc0}-U_{dc})+\xi_{dc}$、$\dot\xi_{dc}=K_{i,dc}(U_{dc0}-U_{dc})$，故 $G_{i_q^*,U_{dc}}=-(K_{p,dc}+K_{i,dc}/s)$。
- **当前理想MWT与GFL的网侧：** GSC交流控制和理想受控电压源不含 $U_{dc}$；DC-link只通过MSC-DVC闭合。因此在规定 $U_{dc}$ 的方向性测试中，$U_{dc}\to P_{GSC}/P_{PCC}$ 没有有向控制边。

## 3. 数值结果

| 架构 | 路径 | $f_{tor}$ (Hz) | 直接偏导(解析/数值) | $|G(j\omega_{tor})|$ | -0.2% / +0.2% | 可达 | 分类 | 一致 |
|---|---|---:|---:|---:|---:|:---:|---|:---:|
| GFL | Udc -> iq_MSC_ref | 2.494210 | -7.89832 / -7.89832 | 7.91739 | 7.91739 / 7.91739 | 是 | ACTIVE_COUPLING | 是 |
| GFL | Udc -> P_GSC | 2.494210 | 0 / 0 | 0 | 0 / 0 | 否 | STRUCTURAL_ZERO | 是 |
| GFL | Udc -> P_PCC | 2.494210 | 0 / 0 | 0 | 0 / 0 | 否 | STRUCTURAL_ZERO | 是 |
| GFM-GWT (GSC-DVC + MSC-MPPT/转矩) | Udc -> iq_MSC_ref | 2.502816 | 0 / 0 | 0 | 0 / 0 | 否 | STRUCTURAL_ZERO | 是 |
| GFM-GWT (GSC-DVC + MSC-MPPT/转矩) | Udc -> P_GSC | 2.502816 | 0 / 0 | 3376.22 | 3376.22 / 3376.22 | 是 | ACTIVE_COUPLING | 是 |
| GFM-GWT (GSC-DVC + MSC-MPPT/转矩) | Udc -> P_PCC | 2.502816 | 0 / 0 | 3334.72 | 3334.72 / 3334.72 | 是 | ACTIVE_COUPLING | 是 |
| GFM-MWT (MSC-DVC + GSC-VSG) | Udc -> iq_MSC_ref | 2.494210 | -7.89832 / -7.89832 | 7.91739 | 7.91739 / 7.91739 | 是 | ACTIVE_COUPLING | 是 |
| GFM-MWT (MSC-DVC + GSC-VSG) | Udc -> P_GSC | 2.494210 | 0 / 0 | 0 | 0 / 0 | 否 | STRUCTURAL_ZERO | 是 |
| GFM-MWT (MSC-DVC + GSC-VSG) | Udc -> P_PCC | 2.494210 | 0 / 0 | 0 | 0 / 0 | 否 | STRUCTURAL_ZERO | 是 |

## 4. 判定

**阶段A一致性门：PASS。** 解析偏导、Jacobian有向可达性、轴系频率处频响和正负0.2%局部检查相互一致。

- GFL 的轴系评价频率：2.494210 Hz。
- GFM-GWT (GSC-DVC + MSC-MPPT/转矩) 的轴系评价频率：2.502816 Hz。
- GFM-MWT (MSC-DVC + GSC-VSG) 的轴系评价频率：2.494210 Hz。

最终分类含义：

- `STRUCTURAL_ZERO`：实际控制方程不存在有向边，Jacobian不可达，基准与偏移点频响均为机器零。
- `WEAK_COUPLING`：方程和Jacobian存在通道，但轴系频率处增益很小。
- `OPERATING_POINT_CANCELLATION`：方程存在通道，基准点近零，但偏移后显著恢复。
- `ACTIVE_COUPLING`：解析与数值均显示有限动态通道；它不是三类“近似零”之一。

## 5. 阶段A结论

当前模型的双向传播差异首先来自**DC-link调节责任的结构分配**：GWT把直流调节放在GSC侧，因此切断了 $U_{dc}\to i_{q,MSC}^*$，但保留 $U_{dc}\to P_{GSC}/P_{PCC}$；MWT与GFL把直流调节放在MSC侧，因此保留前一通道，并在当前理想网侧实现中切断后一通道。若表中不存在 `WEAK_COUPLING` 或 `OPERATING_POINT_CANCELLATION`，则不能把这些机器零解释成参数过小或单一工作点偶然抵消。

本报告到阶段A为止，不继续构造归一化双向矩阵、$\alpha_{dc}$ 或SCR/H/DVC统一扫描。
