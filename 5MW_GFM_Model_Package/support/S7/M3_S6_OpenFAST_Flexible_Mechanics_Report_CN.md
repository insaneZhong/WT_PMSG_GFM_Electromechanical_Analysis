# M3 S6：OpenFAST柔性机械局部反例测试

## 研究边界

本轮使用OpenFAST官方 `5MW_Land_Linear_Aero_CalcSteady` 三方位角周期稳态线性化，经官方MBC工具变换。它是NREL 5 MW**有齿轮箱**参考机组；与当前直驱PMSG只通过低速轴等效转矩—转速功率守恒接口连接，因此属于跨模型局部反例测试，不是新的主基准。

- r-test commit：`dd5feaaaa500ba7283140107806300d551cff0a7`；
- 风速 8 m/s，转速 1.00033 rad/s，GBRatio 97；
- HSS转矩 19339.8 N m，低速轴等效转矩 1.87596e+06 N m，等效功率 1.87658 MW。

## 必须保留的反例

完整258状态MBC平均矩阵含 77 个RHP极点，最大实部 0.227021 1/s；其主导来源是AeroDyn内部诱导速度状态。因此，完整动态AeroDyn矩阵被判为 `REJECTED_FOR_COUPLING`，没有直接接入GFM。

随后只对AeroDyn内部状态做零导数静态消元：rcond=0.0299492；按常数矩阵排除近零方位分支后最大实部为 -0.0104929 1/s。后续三方位角与Floquet审计表明，全坐标准稳态气动基线仍有正增长，故该“稳定”判据不足，不能把静态消元后的模型当作严格稳定的柔性机械基线。

## Gate

- 来源可追溯/MBC/三方位角：PASS；
- 共同电气工作点：PASS；
- 低速轴接口能量误差：0.000e+00；
- 一周Floquet开环机械基线：FAIL；
- 全坐标三架构Floquet稳定：FAIL；
- 驱动链模态身份：PASS；
- **S6条件性Gate：FAIL（FAILED_UNSTABLE_QUASISTEADY_AERODYN_BASELINE）**。

## 三架构结果

|Architecture|P0_W|Omega0_radps|EquilibriumResidual|HybridMaxRealNonRigid|HybridStable|RigidModesExcluded|WorstPoleReal|WorstPoleImag|WorstFrequency_Hz|WorstDampingRatio|WorstPiOpenFAST|WorstPiElectrical|WorstPiDrivetrain|WorstDominantState|GeneratorAngleColumnNorm|NoGaugeMaxReal|NoGaugeWorstFrequency_Hz|NoGaugeStable|NoGaugeWorstDominantState|ElectricalTorqueGain1mHzReal|ElectricalTorqueGain1mHzImag|MechanicalGain1mHzReal|MechanicalGain1mHzImag|LoopGain1mHzReal|LoopGain1mHzImag|DrivePoleReal|DrivePoleImag|DriveFrequency_Hz|DriveDampingRatio|DrivePiOpenFAST|DrivePiElectrical|DrivePiDrivetrain|DriveDominantState|M3TwoMassFrequency_Hz|M3TwoMassDampingRatio|DeltaFrequency_Hz|DeltaDampingRatio|DriveResidue_GridToShaft|DriveResidue_WindToPCC|DriveResidue_GridToUdc|TopGridToShaftMode_Hz|TopGridToShaftResidue|TopWindToPCCMode_Hz|TopWindToPCCResidue|ElectricalDependenceOnThetaWt_FroNorm|DriveModeIdentity_PASS|
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|GFL|1876575.9|1.0003303|1.2451416e-13|-0.012035057|true|1|-0.012035057|2.0237679|0.32209267|0.005946751|0.99999626|3.7372339e-06|0.00064930061|ED First time derivative of 1st tower side-to-side bending mode DOF (internal DOF index = DOF_TSS1), m/s|22.05587|-0.012043115|0.32207947|true|ED First time derivative of 1st tower side-to-side bending mode DOF (internal DOF index = DOF_TSS1), m/s|3842280.4|-0.52665643|-4.753986e-07|6.4148536e-08|-1.8266147|0.24647691|-1.4103756|24.700377|3.9311871|0.057006499|0.99995423|4.5767569e-05|0.51668959|ED First time derivative of Drivetrain rotational-flexibility DOF (internal DOF index = DOF_DrTr), rad/s|2.5012616|0.047004968|1.4299255|0.010001532|8.3688961e-05|0.0032414628|7.9686301e-06|3.9311871|8.3688961e-05|0.60275338|0.069624309|0|true|
|GWT|1876575.9|1.0003303|2.4381968e-15|-0.012034806|true|1|-0.012034806|2.0237706|0.32209309|0.0059466193|0.99999355|6.4511045e-06|0.00064901878|ED First time derivative of 1st tower side-to-side bending mode DOF (internal DOF index = DOF_TSS1), m/s|22.05587|-0.012042866|0.32207989|true|ED First time derivative of 1st tower side-to-side bending mode DOF (internal DOF index = DOF_TSS1), m/s|3842280.4|-0.52600754|-4.753986e-07|6.4148536e-08|-1.8266147|0.24647691|-1.4104451|24.700364|3.931185|0.057009329|0.99994963|5.0367442e-05|0.51668872|ED First time derivative of Drivetrain rotational-flexibility DOF (internal DOF index = DOF_DrTr), rad/s|2.5011625|0.047064926|1.4300226|0.0099444036|0.02986701|0.0013108373|0.0030437676|1.6982033|0.13384357|0.60275357|0.083851796|0|true|
|MWT|1876575.9|1.0003303|3.0671167e-15|0.0023064534|false|0|0.0023064534|0|0|-1|0.99915815|0.00084185336|0.99023475|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|22.05587|-0.0013242996|0|true|ED First time derivative of Variable speed generator DOF (internal DOF index = DOF_GeAz), rad/s|-2007247.6|-593.36461|-4.753986e-07|6.4148536e-08|0.95428077|-0.12847991|-1.3529661|24.710122|3.932738|0.054671629|0.99968437|0.00031562887|0.51526105|ED First time derivative of Drivetrain rotational-flexibility DOF (internal DOF index = DOF_DrTr), rad/s|2.5002808|0.029179699|1.4324572|0.02549193|3.1591215|0.00061507501|0.088027579|1.6929299|10.394826|0.60262388|0.012240612|0|true|

## 近零频慢速分支反事实

|Architecture|Counterfactual|Factor|MaxRealPole|WorstFrequency_Hz|Stable|WorstDominantState|ElectricalTorqueGain1mHzReal|ElectricalTorqueGain1mHzImag|MechanicalGain1mHzReal|MechanicalGain1mHzImag|LoopGain1mHzReal|LoopGain1mHzImag|
|---|---|---|---|---|---|---|---|---|---|---|---|---|
|GFL|TORQUE_FEEDBACK_CLOSURE_NO_ANGLE|0|-0.010503913|0.32208361|true|ED First time derivative of 1st tower side-to-side bending mode DOF (internal DOF index = DOF_TSS1), m/s|3842280.4|-0.52665643|-4.753986e-07|6.4148536e-08|-1.8266147|0.24647691|
|GFL|TORQUE_FEEDBACK_CLOSURE_FULL_COORD|0|0.00017946168|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|3842280.4|-0.52665643|-4.7128816e-07|7.7084448e-08|-1.8108212|0.29618031|
|GFL|TORQUE_FEEDBACK_CLOSURE_NO_ANGLE|0.25|-0.010889509|0.32208406|true|ED First time derivative of 1st tower side-to-side bending mode DOF (internal DOF index = DOF_TSS1), m/s|3842280.4|-0.52665643|-4.753986e-07|6.4148536e-08|-1.8266147|0.24647691|
|GFL|TORQUE_FEEDBACK_CLOSURE_FULL_COORD|0.25|0.0001227656|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|3842280.4|-0.52665643|-4.7128816e-07|7.7084448e-08|-1.8108212|0.29618031|
|GFL|TORQUE_FEEDBACK_CLOSURE_NO_ANGLE|0.5|-0.011274672|0.32208352|true|ED First time derivative of 1st tower side-to-side bending mode DOF (internal DOF index = DOF_TSS1), m/s|3842280.4|-0.52665643|-4.753986e-07|6.4148536e-08|-1.8266147|0.24647691|
|GFL|TORQUE_FEEDBACK_CLOSURE_FULL_COORD|0.5|9.326419e-05|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|3842280.4|-0.52665643|-4.7128816e-07|7.7084448e-08|-1.8108212|0.29618031|
|GFL|TORQUE_FEEDBACK_CLOSURE_NO_ANGLE|0.75|-0.011659255|0.32208198|true|ED First time derivative of 1st tower side-to-side bending mode DOF (internal DOF index = DOF_TSS1), m/s|3842280.4|-0.52665643|-4.753986e-07|6.4148536e-08|-1.8266147|0.24647691|
|GFL|TORQUE_FEEDBACK_CLOSURE_FULL_COORD|0.75|7.5186755e-05|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|3842280.4|-0.52665643|-4.7128816e-07|7.7084448e-08|-1.8108212|0.29618031|
|GFL|TORQUE_FEEDBACK_CLOSURE_NO_ANGLE|1|-0.012043115|0.32207947|true|ED First time derivative of 1st tower side-to-side bending mode DOF (internal DOF index = DOF_TSS1), m/s|3842280.4|-0.52665643|-4.753986e-07|6.4148536e-08|-1.8266147|0.24647691|
|GFL|TORQUE_FEEDBACK_CLOSURE_FULL_COORD|1|6.2976654e-05|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|3842280.4|-0.52665643|-4.7128816e-07|7.7084448e-08|-1.8108212|0.29618031|
|GWT|TORQUE_FEEDBACK_CLOSURE_NO_ANGLE|0|-0.010503913|0.32208361|true|ED First time derivative of 1st tower side-to-side bending mode DOF (internal DOF index = DOF_TSS1), m/s|3842280.4|-0.52600754|-4.753986e-07|6.4148536e-08|-1.8266147|0.24647691|
|GWT|TORQUE_FEEDBACK_CLOSURE_FULL_COORD|0|0.00017946168|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|3842280.4|-0.52600754|-4.7128816e-07|7.7084448e-08|-1.8108212|0.29618031|
|GWT|TORQUE_FEEDBACK_CLOSURE_NO_ANGLE|0.25|-0.010889413|0.32208417|true|ED First time derivative of 1st tower side-to-side bending mode DOF (internal DOF index = DOF_TSS1), m/s|3842280.4|-0.52600754|-4.753986e-07|6.4148536e-08|-1.8266147|0.24647691|
|GWT|TORQUE_FEEDBACK_CLOSURE_FULL_COORD|0.25|0.0001227656|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|3842280.4|-0.52600754|-4.7128816e-07|7.7084448e-08|-1.8108212|0.29618031|
|GWT|TORQUE_FEEDBACK_CLOSURE_NO_ANGLE|0.5|-0.011274502|0.32208373|true|ED First time derivative of 1st tower side-to-side bending mode DOF (internal DOF index = DOF_TSS1), m/s|3842280.4|-0.52600754|-4.753986e-07|6.4148536e-08|-1.8266147|0.24647691|
|GWT|TORQUE_FEEDBACK_CLOSURE_FULL_COORD|0.5|9.326419e-05|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|3842280.4|-0.52600754|-4.7128816e-07|7.7084448e-08|-1.8108212|0.29618031|
|GWT|TORQUE_FEEDBACK_CLOSURE_NO_ANGLE|0.75|-0.011659034|0.32208231|true|ED First time derivative of 1st tower side-to-side bending mode DOF (internal DOF index = DOF_TSS1), m/s|3842280.4|-0.52600754|-4.753986e-07|6.4148536e-08|-1.8266147|0.24647691|
|GWT|TORQUE_FEEDBACK_CLOSURE_FULL_COORD|0.75|7.5186755e-05|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|3842280.4|-0.52600754|-4.7128816e-07|7.7084448e-08|-1.8108212|0.29618031|
|GWT|TORQUE_FEEDBACK_CLOSURE_NO_ANGLE|1|-0.012042866|0.32207989|true|ED First time derivative of 1st tower side-to-side bending mode DOF (internal DOF index = DOF_TSS1), m/s|3842280.4|-0.52600754|-4.753986e-07|6.4148536e-08|-1.8266147|0.24647691|
|GWT|TORQUE_FEEDBACK_CLOSURE_FULL_COORD|1|6.2976654e-05|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|3842280.4|-0.52600754|-4.7128816e-07|7.7084448e-08|-1.8108212|0.29618031|
|MWT|TORQUE_FEEDBACK_CLOSURE_NO_ANGLE|0|-0.010503913|0.32208361|true|ED First time derivative of 1st tower side-to-side bending mode DOF (internal DOF index = DOF_TSS1), m/s|-2007247.6|-593.36461|-4.753986e-07|6.4148536e-08|0.95428077|-0.12847991|
|MWT|TORQUE_FEEDBACK_CLOSURE_FULL_COORD|0|0.00017946168|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|-2007247.6|-593.36461|-4.7128816e-07|7.7084448e-08|0.94603777|-0.15444793|
|MWT|TORQUE_FEEDBACK_CLOSURE_NO_ANGLE|0.25|-0.010249943|0.32207962|true|ED First time derivative of 1st tower side-to-side bending mode DOF (internal DOF index = DOF_TSS1), m/s|-2007247.6|-593.36461|-4.753986e-07|6.4148536e-08|0.95428077|-0.12847991|
|MWT|TORQUE_FEEDBACK_CLOSURE_FULL_COORD|0.25|0.00023634516|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|-2007247.6|-593.36461|-4.7128816e-07|7.7084448e-08|0.94603777|-0.15444793|
|MWT|TORQUE_FEEDBACK_CLOSURE_NO_ANGLE|0.5|-0.00999629|0.32207518|true|ED First time derivative of 1st tower side-to-side bending mode DOF (internal DOF index = DOF_TSS1), m/s|-2007247.6|-593.36461|-4.753986e-07|6.4148536e-08|0.95428077|-0.12847991|
|MWT|TORQUE_FEEDBACK_CLOSURE_FULL_COORD|0.5|0.0003452837|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|-2007247.6|-593.36461|-4.7128816e-07|7.7084448e-08|0.94603777|-0.15444793|
|MWT|TORQUE_FEEDBACK_CLOSURE_NO_ANGLE|0.75|-0.0097430008|0.3220703|true|ED First time derivative of 1st tower side-to-side bending mode DOF (internal DOF index = DOF_TSS1), m/s|-2007247.6|-593.36461|-4.753986e-07|6.4148536e-08|0.95428077|-0.12847991|
|MWT|TORQUE_FEEDBACK_CLOSURE_FULL_COORD|0.75|0.00063196287|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|-2007247.6|-593.36461|-4.7128816e-07|7.7084448e-08|0.94603777|-0.15444793|
|MWT|TORQUE_FEEDBACK_CLOSURE_NO_ANGLE|1|-0.0013242996|0|true|ED First time derivative of Variable speed generator DOF (internal DOF index = DOF_GeAz), rad/s|-2007247.6|-593.36461|-4.753986e-07|6.4148536e-08|0.95428077|-0.12847991|
|MWT|TORQUE_FEEDBACK_CLOSURE_FULL_COORD|1|0.0023064534|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|-2007247.6|-593.36461|-4.7128816e-07|7.7084448e-08|0.94603777|-0.15444793|
|MWT|MSC_DVC_SCALE_NO_ANGLE|0|-4.8962501e-08|0|true|Udc|-72199.676|380.88532|-4.753986e-07|6.4148536e-08|0.034299191|-0.0048125758|
|MWT|MSC_DVC_SCALE_FULL_COORD|0|0.00018590391|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|-72199.676|380.88532|-4.7128816e-07|7.7084448e-08|0.033997492|-0.0057449788|
|MWT|MSC_DVC_SCALE_NO_ANGLE|0.25|-0.0013243499|0|true|ED First time derivative of Variable speed generator DOF (internal DOF index = DOF_GeAz), rad/s|-2007297.7|-593.10421|-4.753986e-07|6.4148536e-08|0.95430456|-0.12848325|
|MWT|MSC_DVC_SCALE_FULL_COORD|0.25|0.0023063944|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|-2007297.7|-593.10421|-4.7128816e-07|7.7084448e-08|0.94606136|-0.15445191|
|MWT|MSC_DVC_SCALE_NO_ANGLE|0.5|-0.0013243164|0|true|ED First time derivative of Variable speed generator DOF (internal DOF index = DOF_GeAz), rad/s|-2007264.3|-593.27781|-4.753986e-07|6.4148536e-08|0.9542887|-0.12848102|
|MWT|MSC_DVC_SCALE_FULL_COORD|0.5|0.0023064337|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|-2007264.3|-593.27781|-4.7128816e-07|7.7084448e-08|0.94604564|-0.15444926|
|MWT|MSC_DVC_SCALE_NO_ANGLE|0.75|-0.0013243052|0|true|ED First time derivative of Variable speed generator DOF (internal DOF index = DOF_GeAz), rad/s|-2007253.2|-593.33568|-4.753986e-07|6.4148536e-08|0.95428341|-0.12848028|
|MWT|MSC_DVC_SCALE_FULL_COORD|0.75|0.0023064468|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|-2007253.2|-593.33568|-4.7128816e-07|7.7084448e-08|0.94604039|-0.15444837|
|MWT|MSC_DVC_SCALE_NO_ANGLE|1|-0.0013242996|0|true|ED First time derivative of Variable speed generator DOF (internal DOF index = DOF_GeAz), rad/s|-2007247.6|-593.36461|-4.753986e-07|6.4148536e-08|0.95428077|-0.12847991|
|MWT|MSC_DVC_SCALE_FULL_COORD|1|0.0023064534|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|-2007247.6|-593.36461|-4.7128816e-07|7.7084448e-08|0.94603777|-0.15444793|

上述反事实分别保留或移除发电机方位状态，再连续闭合电磁转矩反馈和MSC-DVC。该方位状态在本准稳态气动化简中与叶片柔性状态存在非零耦合；“ANGLE_REMOVED”仅是坐标敏感性测试，不是可直接采纳的物理模型。

## 三方位角敏感性

|PhaseIndex|Azimuth_deg|Architecture|CoordinateTreatment|MaxRealPole|WorstFrequency_Hz|Stable|WorstDominantState|
|---|---|---|---|---|---|---|---|
|1|0.37242257|GFL|FULL_COORD|7.0123725e-05|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|
|1|0.37242257|GFL|ANGLE_REMOVED|-0.012043603|0.32207954|true|ED First time derivative of 1st tower side-to-side bending mode DOF (internal DOF index = DOF_TSS1), m/s|
|1|0.37242257|GFL|TORQUE_FEEDBACK_OPEN|0.00019974977|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|
|1|0.37242257|GWT|FULL_COORD|7.0123724e-05|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|
|1|0.37242257|GWT|ANGLE_REMOVED|-0.012043353|0.32207997|true|ED First time derivative of 1st tower side-to-side bending mode DOF (internal DOF index = DOF_TSS1), m/s|
|1|0.37242257|GWT|TORQUE_FEEDBACK_OPEN|0.00019974977|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|
|1|0.37242257|MWT|FULL_COORD|0.0024624027|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|
|1|0.37242257|MWT|ANGLE_REMOVED|-0.0013247944|0|true|ED First time derivative of Variable speed generator DOF (internal DOF index = DOF_GeAz), rad/s|
|1|0.37242257|MWT|TORQUE_FEEDBACK_OPEN|0.00019974977|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|
|2|120.14352|GFL|FULL_COORD|5.0198069e-05|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|
|2|120.14352|GFL|ANGLE_REMOVED|-0.012042196|0.32207947|true|ED First time derivative of 1st tower side-to-side bending mode DOF (internal DOF index = DOF_TSS1), m/s|
|2|120.14352|GFL|TORQUE_FEEDBACK_OPEN|0.00014315737|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|
|2|120.14352|GWT|FULL_COORD|5.0198071e-05|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|
|2|120.14352|GWT|ANGLE_REMOVED|-0.012041946|0.3220799|true|ED First time derivative of 1st tower side-to-side bending mode DOF (internal DOF index = DOF_TSS1), m/s|
|2|120.14352|GWT|TORQUE_FEEDBACK_OPEN|0.00014315737|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|
|2|120.14352|MWT|FULL_COORD|0.0020067102|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|
|2|120.14352|MWT|ANGLE_REMOVED|-0.0013183663|0|true|ED First time derivative of Variable speed generator DOF (internal DOF index = DOF_GeAz), rad/s|
|2|120.14352|MWT|TORQUE_FEEDBACK_OPEN|0.00014315738|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|
|3|240.48758|GFL|FULL_COORD|6.8628516e-05|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|
|3|240.48758|GFL|ANGLE_REMOVED|-0.012043545|0.32207939|true|ED First time derivative of 1st tower side-to-side bending mode DOF (internal DOF index = DOF_TSS1), m/s|
|3|240.48758|GFL|TORQUE_FEEDBACK_OPEN|0.00019549392|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|
|3|240.48758|GWT|FULL_COORD|6.8628516e-05|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|
|3|240.48758|GWT|ANGLE_REMOVED|-0.012043295|0.32207981|true|ED First time derivative of 1st tower side-to-side bending mode DOF (internal DOF index = DOF_TSS1), m/s|
|3|240.48758|GWT|TORQUE_FEEDBACK_OPEN|0.00019549392|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|
|3|240.48758|MWT|FULL_COORD|0.0024286207|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|
|3|240.48758|MWT|ANGLE_REMOVED|-0.0013297286|0|true|ED First time derivative of Variable speed generator DOF (internal DOF index = DOF_GeAz), rad/s|
|3|240.48758|MWT|TORQUE_FEEDBACK_OPEN|0.00019549392|0|false|ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad|

MWT全坐标分支跨方位角实部符号一致性：`YES`。若符号跨方位角改变，平均矩阵中的近零频极点只能列为周期相位敏感候选，不得作为LTI普适稳定边界。

## 一周Floquet插值审计

|Architecture|CoordinateTreatment|MaxFloquetReal|WorstFrequency_Hz|Stable|WorstDominantState|WorstMultiplierMagnitude|
|---|---|---|---|---|---|---|
|GFL|FULL_COORD|6.2967042e-05|0|false|P_f|1.0003956|
|GFL|ANGLE_REMOVED|-0.012043113|0.0036644356|true|P_f|0.9271461|
|GFL|TORQUE_FEEDBACK_OPEN|0.00017943437|0|false|P_f|1.0011277|
|GWT|FULL_COORD|6.296704e-05|0|false|xi_DVC|1.0003956|
|GWT|ANGLE_REMOVED|-0.012042864|0.003664863|true|Q_f|0.92714755|
|GWT|TORQUE_FEEDBACK_OPEN|0.00017943438|0|false|xi_DVC|1.0011277|
|MWT|FULL_COORD|0.0023062419|0|false|i_mq|1.0145912|
|MWT|ANGLE_REMOVED|-0.0013242896|0|true|xi_DVC|0.99171649|
|MWT|TORQUE_FEEDBACK_OPEN|0.00017943438|0|false|P_f|1.0011277|

Floquet审计将官方MBC固定坐标系下的三个方位角矩阵按转子一周分段指数传播。它用于判定平均矩阵近零频分支是否在该三点周期插值下保留，不能替代完整动态AeroDyn或直接驱动机组的时变非线性验证。

## 解释纪律

1. M3两质量模态与OpenFAST柔性驱动链模态并非同一模态，频率/阻尼差只说明机械模型层级改变了谱结构，不能写成GFM效应。
2. `GridToShaft` 与 `WindToPCC` 残差用于筛选新增机械模态是否改变扰动排序；单个点不能证明全局方向性。
3. 本轮若通过，只允许说候选机电耦合分析在一个来源可追溯的柔性机械、准稳态气动、低速轴等效接口上可执行。完整动态AeroDyn、直驱OpenFAST模型和跨风速稳健性仍未验证。

## 决策

S6 Gate失败：当前静态AeroDyn消元后的全坐标开环柔性机械基线已在一周Floquet插值中出现正增长；MWT只是在该失效基线上进一步放大慢速分支。S7保持阻塞。下一步必须获取或生成经周期稳态配平且无需静态AeroDyn状态消元的严格稳定柔性机械模型，不能以额外控制调参掩盖基线问题。
