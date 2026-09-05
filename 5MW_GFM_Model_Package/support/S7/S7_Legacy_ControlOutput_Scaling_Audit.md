# S7 Legacy 平均控制输出与尺度审计

## 当前副本的接口

`main_s7_legacy_avg` 使用 `IDEAL_AVG_OUTPUTS=1` 编译，因此输出端口 38--41 是：

1. `motor.out.Us_alfa`
2. `motor.out.Us_beta`
3. `grid_side.out.Us_alfa`
4. `grid_side.out.Us_beta`

这四个量是以 V 表示的 αβ 交流电压指令，直接接入复制模型中的 `Ideal_MSC_AverageVSC` 和 `Ideal_GSC_AverageVSC` 命令端口。当前路径**不再做 `Udc/2` 或 `Udc` 归一化**；平均 VSC 内部原有的命令极性和 αβ 到三相映射保持不变。

## 与调制量接口的区别

如果未来改为导出调制比 `m_alpha,m_beta`，物理受控电压应按

\[
v_{conv,\alpha\beta}=\frac{U_{dc}}{2}m_{\alpha\beta}
\]

重建；本 S7 副本目前不是这种接口，禁止重复乘以 `Udc/2`。

## 诊断端口

端口 1--37 保留遗留控制器诊断量；包装器把端口 13--30 映射到原 `MOTOR_CONTROL1` 的 18 个外部诊断输出，供既有顶层监视信号继续使用。未映射的端口由 Terminator 吸收，不参与物理控制。

## 验收要求

- MEX 必须来自当前 Legacy C 源码，且不覆盖生产 MEX；
- MSC/GSC 两个 αβ 指令只各出现一条到平均 VSC 的控制路径；
- 不得在包装器内增加额外的电压、调制比或功率比例；
- 端口 4 的 Udc 只作为 Legacy 输入，功率面由平均 VSC 的交流端口测量；
- 任何输出极性差异必须通过稳态功率符号试验确认，不能凭接口名称推断。
