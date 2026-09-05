# GFM网侧扰动多模态传递机理与参数规律分析

## 1. 基准模型和公平性

本报告仅使用已对齐的5 MW理想连续平均模型及其同源23状态SSM；不包含EMT、PWM、采样、离散PI、延迟、限幅、LVRT、启动或故障逻辑。三架构使用同一个Gate A参数源和共同工作点。

## 2. Grid-frequency输入公平性审计

|Architecture|InputName|PhysicalMeaning|BaseValue_radps|Unit|InputLocation|ThetaIntegrator|ScaleFactor|BfNorm|BfNonzeroStates|PASS|
|---|---|---|---|---|---|---|---|---|---|---|
|GFL|DeltaOmegaGrid|同一外部电网角频率增量 Delta omega_grid|314.15927|rad/s|wctrl=omega_grid，理想PLL直接跟随同一网源|无独立相对角积分；理想PLL已直接跟随，不重复积分|1|6283.7081|Udc,xi_GSC_iq,i_f_d,i_f_q|1|
|GFMGWT|DeltaOmegaGrid|同一外部电网角频率增量 Delta omega_grid|314.15927|rad/s|delta_dot=omega_ctrl-(omega0+DeltaOmegaGrid)|仅相对功角 delta 的一阶积分；不存在重复积分|1|1|delta|1|
|VSG|DeltaOmegaGrid|同一外部电网角频率增量 Delta omega_grid|314.15927|rad/s|delta_dot=omega_ctrl-(omega0+DeltaOmegaGrid)|仅相对功角 delta 的一阶积分；不存在重复积分|1|1|delta|1|

**Gate A：PASS。** 三架构外部输入均为同一个物理量 \Delta\omega_{grid}（rad/s）。GFL表示理想PLL直接跟随；GFM-GWT/MWT通过唯一的相对功角方程 \dot\delta=\omega_{ctrl}-\omega_{grid} 接收该输入。不存在Hz/rad/s混用或重复积分。

## 3. GFL/GFM-GWT/GFM-MWT逐级传递差异

|Architecture|Disturbance|First_Divergence_Node|
|---|---|---|
|GFL|Grid angle|NONE|
|GFL|Grid frequency|NONE|
|GFM-GWT (GSC-DVC + MSC-MPPT/转矩)|Grid angle|P_GSC|
|GFM-GWT (GSC-DVC + MSC-MPPT/转矩)|Grid frequency|P_GSC|
|GFM-MWT (MSC-DVC + GSC-VSG)|Grid angle|P_GSC|
|GFM-MWT (MSC-DVC + GSC-VSG)|Grid frequency|P_GSC|

在共同参考频率2.4942 Hz处，三种架构相对GFL的逐级传递函数已保存在 `GridToShaft_TransferChain_Summary.csv`。非GFL架构的首次显著偏离节点由表中 `First_Divergence_Node` 标识。

## 4. GFM-GWT与GFM-MWT关键路径差异

|Disturbance|Signal|GWT_Magnitude|MWT_Magnitude|MWT_to_GWT_Ratio|Interpretation|
|---|---|---|---|---|---|
|Grid angle|P_GSC|49899398|48747565|0.9769169|两条路径近似一致|
|Grid angle|Udc|7075.7365|4938.6334|0.6979674|两条路径近似一致|
|Grid angle|iq_MSC_ref|0|39101.09|3.910109e+22|GWT在该节点已截断；MWT保持非零传播|
|Grid angle|iq_MSC|4.707343e-19|39131.306|3.9131306e+22|GWT在该节点已截断；MWT保持非零传播|
|Grid angle|T_e|1.2201433e-16|10142835|8.3128224e+22|GWT在该节点已截断；MWT保持非零传播|
|Grid angle|omega_sh|2.3283528e-09|2.1174037|9.0939986e+08|MWT相对GWT放大|
|Grid angle|T_sh|0.12795736|1.1746062e+08|9.1796692e+08|MWT相对GWT放大|
|Grid frequency|P_GSC|3184081.4|3110582.9|0.9769169|两条路径近似一致|
|Grid frequency|Udc|451.50286|315.13428|0.6979674|两条路径近似一致|
|Grid frequency|iq_MSC_ref|0|2495.0412|2.4950412e+21|GWT在该节点已截断；MWT保持非零传播|
|Grid frequency|iq_MSC|2.2002089e-20|2496.9693|2.4969693e+21|GWT在该节点已截断；MWT保持非零传播|
|Grid frequency|T_e|5.7029416e-18|647214.44|1.1348783e+23|GWT在该节点已截断；MWT保持非零传播|
|Grid frequency|omega_sh|1.0882713e-10|0.13511157|1.2415246e+09|MWT相对GWT放大|
|Grid frequency|T_sh|0.0059807226|7495164.3|1.2532205e+09|MWT相对GWT放大|
|Grid angle|w_tor^H B_d|0.00130976|2361.333|1802874.6|轴系模态输入投影|
|Grid frequency|w_tor^H B_d|9.9680616e-05|150.60914|1510917|轴系模态输入投影|

若 `iq_MSC_ref`、`iq_MSC` 与 `T_e` 行显示GWT近零而MWT非零，则数据支持以下有限结论：网侧扰动在GWT中未通过MSC转矩通道传递，而MWT因MSC-DVC保留了 `Udc → iq*_MSC → Te` 路径。

## 5. 全模态贡献排序

每个输入—输出组合保留按阶跃贡献度排序的Top 10模态，见 `Full_Modal_Contribution_Ranking.csv`。机械TOR、DC-link/DVC、SYNC和GSC分类来自归一化左右特征向量参与因子。

## 6. 最小多模态重构

|Architecture|Disturbance|Output|NumModePairs|IncludedModeIDs|NRMSE|Correlation|PeakError_pct|Status|
|---|---|---|---|---|---|---|---|---|
|GFL|Grid angle|omega_sh|4|M06,M04,M08,M07|0.026948611|0.99999011|0.32237533|PASS|
|GFL|Grid angle|T_sh|4|M07,M06,M08,M04|0.0084087242|0.99999809|0.18032952|PASS|
|GFL|Grid frequency|omega_sh|4|M04,M06,M20,M18|0.03003799|0.9999526|0.21134706|PASS|
|GFL|Grid frequency|T_sh|5|M06,M20,M18,M04,M08|0.0090339305|0.99999031|0.014013696|PASS|
|GFM-GWT (GSC-DVC + MSC-MPPT/转矩)|Grid angle|omega_sh|0||0|1|0|NO_EXCITATION|
|GFM-GWT (GSC-DVC + MSC-MPPT/转矩)|Grid angle|T_sh|0||0|1|0|NO_EXCITATION|
|GFM-GWT (GSC-DVC + MSC-MPPT/转矩)|Grid frequency|omega_sh|0||0|1|0|NO_EXCITATION|
|GFM-GWT (GSC-DVC + MSC-MPPT/转矩)|Grid frequency|T_sh|0||0|1|0|NO_EXCITATION|
|GFM-MWT (MSC-DVC + GSC-VSG)|Grid angle|omega_sh|3|M03,M18,M05|0.020474601|0.99999624|0.47281493|PASS|
|GFM-MWT (MSC-DVC + GSC-VSG)|Grid angle|T_sh|4|M18,M03,M05,M07|0.01197436|0.99998702|0.098454957|PASS|
|GFM-MWT (MSC-DVC + GSC-VSG)|Grid frequency|omega_sh|4|M18,M03,M05,M07|0.011458584|0.99998704|0.082581354|PASS|
|GFM-MWT (MSC-DVC + GSC-VSG)|Grid frequency|T_sh|5|M06,M18,M03,M05,M07|0.001079935|0.99999981|0.01926492|PASS|

`NO_EXCITATION` 是GFM-GWT的物理路径结果，不被作为数值错误删除。其它组合均以 NRMSE<5% 且相关系数>0.98 为通过条件。

## 7. 模态相位与叠加机理

`Modal_Superposition_At_FirstPeak.csv` 给出第一主峰处每个最小模态的同相（CONSTRUCTIVE）或反相（DESTRUCTIVE）贡献；因此响应峰值不能仅归因于单一torsional pole。

## 8. Grid-frequency与grid-angle差异来源

`Frequency_vs_Angle_Excitation_Explanation.csv` 输出每个主导模态的 \|w^HB_f\| 和 \|w^HB_\theta\|。两者输入单位不同，不能只凭(1/s)或未归一化投影的绝对数值判断强弱；必须结合逐级传递函数、DC-link/DVC和输出残差。

## 9. SCR影响

|SCR|Architecture|Status|EquilibriumResidual|f_tor_Hz|zeta_tor|Rtor_angle|Rtor_frequency|Rsync_angle|Rsync_frequency|Rdc_angle|Rdc_frequency|PeakOmegaSh_frequency|Gamma_angle_vs_GFL|Gamma_frequency_vs_GFL|Classification|
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|2|GFL|PASS|1.4456474e-15|2.501581|0.028002458|0.20952171|5.7411605e-05|6.3838817e-08|1.6616611e-11|0.27187717|6.7950348e-05|2.3650875e-06|NaN|NaN||
|2|GFM-MWT (MSC-DVC + GSC-VSG)|PASS|5.1574447e-15|2.4942032|0.029814524|0.26944357|0.017185533|0.1839312|0.019148233|0.018456367|0.0052580799|0.0011626094|1.2859936|299.33901|DISTURBANCE_PATH_RESHAPING_WITHOUT_POLE_SHIFT|
|3|GFL|PASS|2.5005792e-15|2.4997241|0.028000875|0.32754984|0.00013063292|1.7043768e-07|6.5138061e-11|0.33310267|0.0001184206|5.5134108e-06|NaN|NaN||
|3|GFM-MWT (MSC-DVC + GSC-VSG)|PASS|5.0076345e-13|2.4942091|0.029810924|0.58267302|0.037163715|0.46540912|0.038323068|0.017083649|0.0048622159|0.0015426931|1.7788835|284.48966|DISTURBANCE_PATH_RESHAPING_WITHOUT_POLE_SHIFT|
|4|GFL|PASS|1.0940034e-15|2.4942097|0.029810519|0.40184381|0.00021032327|2.8680975e-07|1.4444621e-10|0.18711165|5.6593469e-05|1.0785258e-05|NaN|NaN||
|4|GFM-MWT (MSC-DVC + GSC-VSG)|PASS|1.0940034e-15|2.4942097|0.029810519|1.1027395|0.070334278|0.99413797|0.070312707|0.016540892|0.0047072198|0.0021371985|2.7441992|334.41035|DISTURBANCE_PATH_RESHAPING_WITHOUT_POLE_SHIFT|
|6|GFL|PASS|4.2197275e-15|2.4645463|0.064064375|0.33820008|0.00026138006|2.124381e-07|1.5832208e-10|0.030090191|9.4764857e-07|1.6874381e-05|NaN|NaN||
|6|GFM-MWT (MSC-DVC + GSC-VSG)|PASS|8.6738843e-15|2.4942082|0.029811427|1.8676097|0.11911885|2.0170602|0.11608219|0.016051542|0.0045690943|0.0031149528|5.5222036|455.73043|POLE_AND_OR_PATH_CHANGE|
|8|GFL|PASS|2.4380648e-15|2.411063|0.096235995|0.38145729|0.00039009266|3.4370058e-07|3.3863775e-10|0.034983715|2.233598e-06|2.483073e-05|NaN|NaN||
|8|GFM-MWT (MSC-DVC + GSC-VSG)|PASS|7.0328791e-15|2.4942058|0.029812904|1.3506782|0.086148287|1.6718633|0.083314045|0.01581847|0.0045045683|0.0033112262|3.5408373|220.84057|POLE_AND_OR_PATH_CHANGE|
|10|GFL|PASS|9.3771722e-15|2.3387833|0.11842448|0.44390704|0.0005650222|4.9474115e-07|6.054198e-10|0.03892121|3.5943561e-06|3.4547876e-05|NaN|NaN||
|10|GFM-MWT (MSC-DVC + GSC-VSG)|PASS|2.4902831e-13|2.4942031|0.029814612|1.0741644|0.068511895|1.4880798|0.066317275|0.01567885|0.0044668939|0.0030563148|2.4197959|121.25523|POLE_AND_OR_PATH_CHANGE|

SCR扫描同时记录轴系极点与残差比。若 `DISTURBANCE_PATH_RESHAPING_WITHOUT_POLE_SHIFT` 出现，则只在该点说明残差改变比轴系阻尼改变更显著；其它点不得泛化。

## 10. H影响

|H_s|H_Factor|Status|f_tor_Hz|zeta_tor|PoleReal|PoleImag|Rtor_frequency|Rsync_frequency|Rdc_frequency|PeakOmegaSh_frequency|Gamma_frequency_vs_H0|
|---|---|---|---|---|---|---|---|---|---|---|---|
|1.5|0.5|PASS|2.4942097|0.029810519|-0.46738571|15.671582|0.037677975|0.032089464|0.0066371991|0.0013818801|0.53569861|
|2.25|0.75|PASS|2.4942097|0.029810519|-0.46738571|15.671582|0.062056768|0.059354345|0.0056564051|0.0017718298|0.88231187|
|3|1|PASS|2.4942097|0.029810519|-0.46738571|15.671582|0.070334278|0.070312706|0.0047072198|0.0021371985|1|
|4.5|1.5|PASS|2.4942097|0.029810519|-0.46738571|15.671582|0.052461501|0.053556054|0.0028977545|0.0024011151|0.7458881|
|6|2|PASS|2.4942097|0.029810519|-0.46738571|15.671582|0.042699265|0.043350729|0.0011978571|0.002565774|0.6070904|

H扫描只改变VSG惯量，不重整定其它环节；结果用于识别SYNC残差和轴系频率输入残差的变化。

## 11. DVC带宽影响

|DVC_Scale|Status|f_tor_Hz|zeta_tor|PoleReal|PoleImag|Rdc_frequency|G_Udc_frequency|G_iqref_frequency|G_Te_frequency|Rtor_frequency|PeakOmegaSh_frequency|Interpretation|
|---|---|---|---|---|---|---|---|---|---|---|---|---|
|0.5|PASS|2.4975972|0.0303255|-0.47611298|15.692866|0.00087361206|366.67727|1451.5541|376533.51|0.041790216|0.0013408629|attenuation/passive path|
|0.75|PASS|2.4957127|0.030088079|-0.47202564|15.681025|0.0066114144|339.09543|2013.5574|522317.35|0.057415826|0.0017716245|attenuation/passive path|
|1|PASS|2.4942097|0.029810519|-0.46738571|15.671582|0.0047072198|315.12636|2494.9785|647198.17|0.070334278|0.0021371985|attenuation/passive path|
|1.25|PASS|2.4930098|0.029522432|-0.46264229|15.664043|0.0051752259|294.10265|2910.6637|755027.08|0.081075287|0.0024480467|attenuation/passive path|
|1.5|PASS|2.4920471|0.02924002|-0.45803591|15.657994|0.005252284|275.52382|3272.1582|848798.92|0.090073863|0.0027132129|amplification path|

DVC扫描等比例缩放MSC-DVC PI，输出Udc→iq*→Te传递摘要和轴系残差，未补偿其它控制器。

## 12. 理想非线性模型验证

|Case|Mode|Output|f_tor_Hz|FullSSM_NRMSE|FullSSM_Correlation|FullSSM_PeakError_pct|Minimal_NRMSE|Minimal_Correlation|Minimal_PeakError_pct|
|---|---|---|---|---|---|---|---|---|---|
|Baseline GFL|GFL|omega_sh|2.4942097|0.0003354865|0.99999996|0.0058354533|0.020479581|0.99999748|0.4518402|
|Baseline GFL|GFL|T_sh|2.4942097|0.00012451475|0.99999999|0.0055368069|0.0090331983|0.9999903|0.0084761349|
|Baseline GFM-MWT|VSG|omega_sh|2.4942097|0.0015589654|0.9999997|0.14465058|0.0024304306|0.99999954|0.19185816|
|Baseline GFM-MWT|VSG|T_sh|2.4942097|0.017987649|0.99998705|0.82041931|0.017339297|0.99998673|0.8009822|
|High-Gamma representative (SCR=6)|VSG|omega_sh|2.4942082|0.0017991825|0.99999957|0.15369837|0.0019177205|0.9999995|0.16969754|
|High-Gamma representative (SCR=6)|VSG|T_sh|2.4942082|0.016684627|0.99998469|0.81242043|0.016237844|0.99998445|0.7978924|
|Low-Gamma representative (H=0.5H0)|VSG|omega_sh|2.4942097|0.0017245866|0.99999949|0.066849643|0.0035856917|0.999999|0.058374531|
|Low-Gamma representative (H=0.5H0)|VSG|T_sh|2.4942097|0.018407993|0.99999009|0.8145012|0.017131306|0.99998951|0.77712835|
|High-SCR representative (SCR=10)|VSG|omega_sh|2.4942031|0.0018766285|0.99999977|0.1735819|0.0034057096|0.99999961|0.25525126|
|High-SCR representative (SCR=10)|VSG|T_sh|2.4942031|0.01812892|0.99998737|0.81997126|0.017695483|0.99998726|0.80765241|
|Low-SCR representative (SCR=2)|VSG|omega_sh|2.4942032|0.0023248604|0.99999921|0.0053473805|0.0077950223|0.99999796|0.027099609|
|Low-SCR representative (SCR=2)|VSG|T_sh|2.4942032|0.018711929|0.99999057|0.82185359|0.017450137|0.99998983|0.78589335|

验证采用0.005 Hz小网侧频率阶跃以保持在线性区。完整SSM与理想连续非线性模型的误差、以及最小多模态重构相对非线性的误差均列于上表。

## 13. 当前结论与限制

本阶段的结论必须限定于当前理想连续工作点：GFM对轴系时域响应的影响可同时来自扰动通道投影、DC-link/DVC跨变流器传递和多模态相位叠加；不能只凭响应峰值声称轴系阻尼必然恶化。SCR、H和DVC扫描用于区分极点变化与残差变化。上述结论尚未外推至离散控制或开关EMT模型。

## 图件

- `Figures_Multimode_Mechanism/Fig20_GridToShaft_Transfer_Chain.png`：逐级传递比。
- `Figures_Multimode_Mechanism/Fig21_Minimal_Multimode_Reconstruction.png`：最小重构。
- `Figures_Multimode_Mechanism/Fig22_ParameterScan_and_NonlinearValidation.png`：参数规律与理想连续非线性验证。
