# 当前5 MW模型副本结构审计

- 模型：`Grid_Forming_PMSG5MW_Liu2024_TwoMass.slx`
- 审计时间：2026-08-11 10:39:41
- 总块数：1079
- 命中候选离散/延迟/桥/限幅/开关块：255

## 审计结论

本轮只读审计，未修改副本，也未修改原始模型。后续理想化只在本目录副本进行。

## 必须替换或旁路的候选块

|路径|BlockType|MaskType|SampleTime|FunctionName|DelayLength|TimeDelay|名称|
|---|---|---|---|---|---|---|---|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Inverter1|SubSystem|Universal Bridge|||||Inverter1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Inverter1/ISWITCH|Terminator||||||ISWITCH|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Inverter1/ISWITCH1|Terminator||||||ISWITCH1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Inverter1/Model|SubSystem|DiscreteGTO|||||Model|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Inverter1/Model/Uswitch|Inport||Ts||||Uswitch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Inverter1/Model/Saturation|Saturate||Ts||||Saturation|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Inverter1/Model/Switch|Switch||Ts||||Switch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Inverter1/Model/Unit Delay|UnitDelay||Ts||||Unit Delay|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Inverter1/Model/Vf 1/Switch|Switch||-1||||Switch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Inverter1/Model/iSwitch|Outport||-1||||iSwitch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Inverter1/Uswitch|From||||||Uswitch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/MOTOR_CONTROL1/S-Function1|S-Function|||main_liu2024_5mw_vsg_vdcref|||S-Function1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1|SubSystem|On/Off Delay |||||On/Off Delay1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/IN|Inport||-1||||IN|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/Clock|Clock||||||Clock|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/Constant|Constant||inf||||Constant|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/Constant1|Constant||inf||||Constant1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/Constant2|Constant||inf||||Constant2|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/Data Type Conversion1|DataTypeConversion||-1||||Data Type Conversion1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/Logical Operator1|Logic||-1||||Logical Operator1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/Logical Operator2|Logic||-1||||Logical Operator2|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay|SubSystem||||||OFF Delay|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/IN|Inport||-1||||IN|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/clock|Inport||-1||||clock|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/DELAY|Inport||-1||||DELAY|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Enable|EnablePort||-1||||Enable|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Edge Detector|SubSystem|Edge Detector|||||Edge Detector|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Edge Detector/In|Inport||-1||||In|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Edge Detector/Constant1|Constant||inf||||Constant1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Edge Detector/Demux|Demux||||||Demux|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Edge Detector/Logical Operator1|Logic||-1||||Logical Operator1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Edge Detector/Memory|Memory||||||Memory|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Edge Detector/Multiport Switch|MultiPortSwitch||-1||||Multiport Switch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Edge Detector/NEGATIVE Edge|SubSystem||||||NEGATIVE Edge|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Edge Detector/NEGATIVE Edge/IN|Inport||-1||||IN|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Edge Detector/NEGATIVE Edge/IN previous|Inport||-1||||IN previous|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Edge Detector/NEGATIVE Edge/Enable|EnablePort||-1||||Enable|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Edge Detector/NEGATIVE Edge/Relational Operator1|RelationalOperator||-1||||Relational Operator1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Edge Detector/NEGATIVE Edge/OUT|Outport||-1||||OUT|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Edge Detector/POSITIVE Edge|SubSystem||||||POSITIVE Edge|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Edge Detector/POSITIVE Edge/IN|Inport||-1||||IN|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Edge Detector/POSITIVE Edge/IN previous|Inport||-1||||IN previous|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Edge Detector/POSITIVE Edge/Enable|EnablePort||-1||||Enable|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Edge Detector/POSITIVE Edge/Relational Operator1|RelationalOperator||-1||||Relational Operator1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Edge Detector/POSITIVE Edge/OUT|Outport||-1||||OUT|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Edge Detector/either edge|Constant||inf||||either edge|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Edge Detector/neg. edge|Constant||inf||||neg. edge|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Edge Detector/pos. edge|Constant||inf||||pos. edge|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Edge Detector/Out|Outport||-1||||Out|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Logical Operator|Logic||-1||||Logical Operator|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Logical Operator1|Logic||-1||||Logical Operator1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Logical Operator2|Logic||-1||||Logical Operator2|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Relational Operator|RelationalOperator||-1||||Relational Operator|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Sample & Hold|SubSystem|Sample & Hold |||||Sample & Hold|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Sample & Hold/In|Inport||-1||||In|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Sample & Hold/S|Inport||-1||||S|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Sample & Hold/IC=ic|Memory||||||IC=ic|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Sample & Hold/Switch|Switch||-1||||Switch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Sample & Hold/   |Outport||-1||||   |
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/Sum|Sum||-1||||Sum|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OFF Delay/OUT|Outport||-1||||OUT|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay|SubSystem||||||ON Delay|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/IN|Inport||-1||||IN|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/clock|Inport||-1||||clock|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/DELAY|Inport||-1||||DELAY|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Enable|EnablePort||-1||||Enable|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Edge Detector|SubSystem|Edge Detector|||||Edge Detector|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Edge Detector/In|Inport||-1||||In|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Edge Detector/Constant1|Constant||inf||||Constant1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Edge Detector/Demux|Demux||||||Demux|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Edge Detector/Logical Operator1|Logic||-1||||Logical Operator1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Edge Detector/Memory|Memory||||||Memory|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Edge Detector/Multiport Switch|MultiPortSwitch||-1||||Multiport Switch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Edge Detector/NEGATIVE Edge|SubSystem||||||NEGATIVE Edge|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Edge Detector/NEGATIVE Edge/IN|Inport||-1||||IN|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Edge Detector/NEGATIVE Edge/IN previous|Inport||-1||||IN previous|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Edge Detector/NEGATIVE Edge/Enable|EnablePort||-1||||Enable|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Edge Detector/NEGATIVE Edge/Relational Operator1|RelationalOperator||-1||||Relational Operator1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Edge Detector/NEGATIVE Edge/OUT|Outport||-1||||OUT|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Edge Detector/POSITIVE Edge|SubSystem||||||POSITIVE Edge|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Edge Detector/POSITIVE Edge/IN|Inport||-1||||IN|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Edge Detector/POSITIVE Edge/IN previous|Inport||-1||||IN previous|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Edge Detector/POSITIVE Edge/Enable|EnablePort||-1||||Enable|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Edge Detector/POSITIVE Edge/Relational Operator1|RelationalOperator||-1||||Relational Operator1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Edge Detector/POSITIVE Edge/OUT|Outport||-1||||OUT|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Edge Detector/either edge|Constant||inf||||either edge|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Edge Detector/neg. edge|Constant||inf||||neg. edge|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Edge Detector/pos. edge|Constant||inf||||pos. edge|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Edge Detector/Out|Outport||-1||||Out|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Logical Operator2|Logic||-1||||Logical Operator2|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Relational Operator|RelationalOperator||-1||||Relational Operator|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Sample & Hold|SubSystem|Sample & Hold |||||Sample & Hold|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Sample & Hold/In|Inport||-1||||In|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Sample & Hold/S|Inport||-1||||S|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Sample & Hold/IC=ic|Memory||||||IC=ic|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Sample & Hold/Switch|Switch||-1||||Switch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Sample & Hold/   |Outport||-1||||   |
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/Sum|Sum||-1||||Sum|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/ON Delay/OUT|Outport||-1||||OUT|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/Relational Operator1|RelationalOperator||-1||||Relational Operator1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay1/OUT|Outport||-1||||OUT|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2|SubSystem|On/Off Delay |||||On/Off Delay2|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/IN|Inport||-1||||IN|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/Clock|Clock||||||Clock|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/Constant|Constant||inf||||Constant|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/Constant1|Constant||inf||||Constant1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/Constant2|Constant||inf||||Constant2|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/Data Type Conversion1|DataTypeConversion||-1||||Data Type Conversion1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/Logical Operator1|Logic||-1||||Logical Operator1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/Logical Operator2|Logic||-1||||Logical Operator2|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay|SubSystem||||||OFF Delay|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/IN|Inport||-1||||IN|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/clock|Inport||-1||||clock|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/DELAY|Inport||-1||||DELAY|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Enable|EnablePort||-1||||Enable|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Edge Detector|SubSystem|Edge Detector|||||Edge Detector|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Edge Detector/In|Inport||-1||||In|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Edge Detector/Constant1|Constant||inf||||Constant1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Edge Detector/Demux|Demux||||||Demux|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Edge Detector/Logical Operator1|Logic||-1||||Logical Operator1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Edge Detector/Memory|Memory||||||Memory|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Edge Detector/Multiport Switch|MultiPortSwitch||-1||||Multiport Switch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Edge Detector/NEGATIVE Edge|SubSystem||||||NEGATIVE Edge|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Edge Detector/NEGATIVE Edge/IN|Inport||-1||||IN|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Edge Detector/NEGATIVE Edge/IN previous|Inport||-1||||IN previous|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Edge Detector/NEGATIVE Edge/Enable|EnablePort||-1||||Enable|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Edge Detector/NEGATIVE Edge/Relational Operator1|RelationalOperator||-1||||Relational Operator1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Edge Detector/NEGATIVE Edge/OUT|Outport||-1||||OUT|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Edge Detector/POSITIVE Edge|SubSystem||||||POSITIVE Edge|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Edge Detector/POSITIVE Edge/IN|Inport||-1||||IN|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Edge Detector/POSITIVE Edge/IN previous|Inport||-1||||IN previous|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Edge Detector/POSITIVE Edge/Enable|EnablePort||-1||||Enable|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Edge Detector/POSITIVE Edge/Relational Operator1|RelationalOperator||-1||||Relational Operator1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Edge Detector/POSITIVE Edge/OUT|Outport||-1||||OUT|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Edge Detector/either edge|Constant||inf||||either edge|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Edge Detector/neg. edge|Constant||inf||||neg. edge|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Edge Detector/pos. edge|Constant||inf||||pos. edge|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Edge Detector/Out|Outport||-1||||Out|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Logical Operator|Logic||-1||||Logical Operator|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Logical Operator1|Logic||-1||||Logical Operator1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Logical Operator2|Logic||-1||||Logical Operator2|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Relational Operator|RelationalOperator||-1||||Relational Operator|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Sample & Hold|SubSystem|Sample & Hold |||||Sample & Hold|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Sample & Hold/In|Inport||-1||||In|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Sample & Hold/S|Inport||-1||||S|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Sample & Hold/IC=ic|Memory||||||IC=ic|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Sample & Hold/Switch|Switch||-1||||Switch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Sample & Hold/   |Outport||-1||||   |
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/Sum|Sum||-1||||Sum|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OFF Delay/OUT|Outport||-1||||OUT|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay|SubSystem||||||ON Delay|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/IN|Inport||-1||||IN|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/clock|Inport||-1||||clock|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/DELAY|Inport||-1||||DELAY|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Enable|EnablePort||-1||||Enable|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Edge Detector|SubSystem|Edge Detector|||||Edge Detector|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Edge Detector/In|Inport||-1||||In|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Edge Detector/Constant1|Constant||inf||||Constant1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Edge Detector/Demux|Demux||||||Demux|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Edge Detector/Logical Operator1|Logic||-1||||Logical Operator1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Edge Detector/Memory|Memory||||||Memory|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Edge Detector/Multiport Switch|MultiPortSwitch||-1||||Multiport Switch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Edge Detector/NEGATIVE Edge|SubSystem||||||NEGATIVE Edge|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Edge Detector/NEGATIVE Edge/IN|Inport||-1||||IN|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Edge Detector/NEGATIVE Edge/IN previous|Inport||-1||||IN previous|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Edge Detector/NEGATIVE Edge/Enable|EnablePort||-1||||Enable|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Edge Detector/NEGATIVE Edge/Relational Operator1|RelationalOperator||-1||||Relational Operator1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Edge Detector/NEGATIVE Edge/OUT|Outport||-1||||OUT|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Edge Detector/POSITIVE Edge|SubSystem||||||POSITIVE Edge|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Edge Detector/POSITIVE Edge/IN|Inport||-1||||IN|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Edge Detector/POSITIVE Edge/IN previous|Inport||-1||||IN previous|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Edge Detector/POSITIVE Edge/Enable|EnablePort||-1||||Enable|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Edge Detector/POSITIVE Edge/Relational Operator1|RelationalOperator||-1||||Relational Operator1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Edge Detector/POSITIVE Edge/OUT|Outport||-1||||OUT|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Edge Detector/either edge|Constant||inf||||either edge|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Edge Detector/neg. edge|Constant||inf||||neg. edge|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Edge Detector/pos. edge|Constant||inf||||pos. edge|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Edge Detector/Out|Outport||-1||||Out|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Logical Operator2|Logic||-1||||Logical Operator2|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Relational Operator|RelationalOperator||-1||||Relational Operator|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Sample & Hold|SubSystem|Sample & Hold |||||Sample & Hold|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Sample & Hold/In|Inport||-1||||In|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Sample & Hold/S|Inport||-1||||S|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Sample & Hold/IC=ic|Memory||||||IC=ic|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Sample & Hold/Switch|Switch||-1||||Switch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Sample & Hold/   |Outport||-1||||   |
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/Sum|Sum||-1||||Sum|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/ON Delay/OUT|Outport||-1||||OUT|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/Relational Operator1|RelationalOperator||-1||||Relational Operator1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/On//Off Delay2/OUT|Outport||-1||||OUT|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/PMSM1/Mechanical model/Discrete-Time Integrator|DiscreteIntegrator||Ts_step||||Discrete-Time Integrator|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/PMSM1/Mechanical model/Discrete-Time Integrator1|DiscreteIntegrator||Ts_step||||Discrete-Time Integrator1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/PMSM1/Mechanical model/Switch|Switch||-1||||Switch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/PMSM1/elemodel3/iq,id/id/Discrete-Time Integrator|DiscreteIntegrator||Ts_step||||Discrete-Time Integrator|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/PMSM1/elemodel3/iq,id/iq/Discrete-Time Integrator|DiscreteIntegrator||Ts_step||||Discrete-Time Integrator|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/P_Grid_Paper_20ms_Average|DiscreteTransferFcn||0.001||||P_Grid_Paper_20ms_Average|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/ProgrammableGridSource/Model/Harmonic Generator/Harmonic A generation/Multiport Switch|MultiPortSwitch||-1||||Multiport Switch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/ProgrammableGridSource/Model/Harmonic Generator/Harmonic B generation/Multiport Switch|MultiPortSwitch||-1||||Multiport Switch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/ProgrammableGridSource/Model/Signal Generator/Switch2|Switch||-1||||Switch2|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/ProgrammableGridSource/Model/Signal Generator/Switch3|Switch||-1||||Switch3|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/ProgrammableGridSource/Model/Signal Generator/Switch4|Switch||-1||||Switch4|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/ProgrammableGridSource/Model/Signal Generator/Variation SubSystem/MULTIPORT SWITCH|MultiPortSwitch||-1||||MULTIPORT SWITCH|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/ProgrammableGridSource/Model/Signal Generator/Variation SubSystem/Switch|Switch||-1||||Switch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/ProgrammableGridSource/Model/Signal Generator/Variation SubSystem/Switch1|Switch||-1||||Switch1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/ProgrammableGridSource/Model/Signal Generator/Variation SubSystem/Switch2|Switch||-1||||Switch2|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/ProgrammableGridSource/Model/Signal Generator/Variation SubSystem/Switch3|Switch||-1||||Switch3|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/ProgrammableGridSource/Model/Switch1|Switch||-1||||Switch1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/ProgrammableGridSource/Model/Switch5|Switch||-1||||Switch5|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Saturation|Saturate||-1||||Saturation|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Three-Phase Breaker/Breaker A/Model/Uswitch|Inport||Ts||||Uswitch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Three-Phase Breaker/Breaker A/Model/Switch3|Switch||-1||||Switch3|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Three-Phase Breaker/Breaker A/Uswitch|From||||||Uswitch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Three-Phase Breaker/Breaker B/Model/Uswitch|Inport||Ts||||Uswitch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Three-Phase Breaker/Breaker B/Model/Switch3|Switch||-1||||Switch3|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Three-Phase Breaker/Breaker B/Uswitch|From||||||Uswitch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Three-Phase Breaker/Breaker C/Model/Uswitch|Inport||Ts||||Uswitch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Three-Phase Breaker/Breaker C/Model/Switch3|Switch||-1||||Switch3|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Three-Phase Breaker/Breaker C/Uswitch|From||||||Uswitch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Three-Phase Breaker/Switch|Switch||-1||||Switch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Three-Phase Breaker/Switch1|Switch||-1||||Switch1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Three-Phase Breaker/Switch2|Switch||-1||||Switch2|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Three-Phase Breaker/Switch3|Switch||-1||||Switch3|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge|SubSystem|Universal Bridge|||||Universal Bridge|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/g|Inport||-1||||g|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/Goto|Goto||||||Goto|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/ISWITCH|Terminator||||||ISWITCH|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/ISWITCH1|Terminator||||||ISWITCH1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/ITAIL|Goto||||||ITAIL|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/Model|SubSystem|DiscreteMOSFET|||||Model|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/Model/gate|Inport||-1||||gate|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/Model/Uswitch|Inport||-1||||Uswitch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/Model/status|Inport||-1||||status|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/Model/Constant|Constant||inf||||Constant|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/Model/Data Type Conversion|DataTypeConversion||-1||||Data Type Conversion|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/Model/ddd|Terminator||||||ddd|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/Model/g1,d1,g2,d2, ...|Selector||-1||||g1,d1,g2,d2, ...|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/Model/iii|Ground||||||iii|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/Model/uswitch|Terminator||||||uswitch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/Model/vf1|Ground||||||vf1|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/Model/iSwitch|Outport||-1||||iSwitch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/Model/m|Outport||-1||||m|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/Model/vf|Outport||-1||||vf|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/Model/Gate|Outport||-1||||Gate|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/Model/itail|Outport||-1||||itail|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/Status|From||||||Status|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/UniversalBridge|PMComponent|InnerPowersysBlock|||||UniversalBridge|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/Uswitch|From||||||Uswitch|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/VF|Terminator||||||VF|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/A|PMIOPort||-1||||A|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/B|PMIOPort||-1||||B|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/C|PMIOPort||-1||||C|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/+|PMIOPort||-1||||+|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Universal Bridge/-|PMIOPort||-1||||-|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/VdcRefProfileSelector|Switch||-1||||VdcRefProfileSelector|
|Grid_Forming_PMSG5MW_Liu2024_TwoMass/Wind_Turbine_Aero_MPPT_Pitch/Pitch_Controller/Pitch_Rate_Limit|RateLimiter||||||Pitch_Rate_Limit|

## 理想化边界

保留：两质量轴系、PMSG电流动态、MSC/GSC连续PI状态、DC-link能量、P/Q滤波、VSG惯量/功角、LCL和电网。

删除或替换：控制采样调度、PWM/SVPWM、数字延迟、PI限幅/抗积分饱和、参考斜率限制、PLL/预同步/GFM接管、主动阻尼，以及PMSG内部与外部两质量轴系重复的离散机械积分。

## 下一步

1. 复制完成并确认原始模型不变；2. 替换控制器S-Function为透明连续状态实现；3. 用理想连续三相受控电压源替换两个物理桥；4. 统一MSC/GSC交流端口功率面并重建唯一DC-link能量状态；5. 编译、求平衡点并检查极点。
