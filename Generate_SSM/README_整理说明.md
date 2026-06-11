# Generate_SSM 文件夹整理说明

本目录用于生成小信号统一模型。建模脚本保留在根目录，导出的 `.mat` 模型统一放入 `Generated_Models`。

## 根目录保留内容

- `WT_PMSG_VSG_Model.mlx`：原始/基础构网型风机小信号建模文件。
- `WT_PMSG_VSG_TypeA_Model_Export.m`：Type-a 机侧 DC 电压控制模型导出脚本。
- `WT_PMSG_VSG_TypeC_Model_Export.m`：Type-c 机侧 DC 电压控制模型导出脚本。
- `WT_PMSG_VSG_TypeC_Damping_Model_Export.m`：Type-c + APCAD 阻尼控制模型导出脚本。

## Generated_Models

存放脚本生成的模型文件：

- `Unified_WT_PMSG_VSG_TypeA.mat`
- `Unified_WT_PMSG_VSG_TypeC.mat`
- `Unified_WT_PMSG_VSG_TypeC_Damping.mat`

三份导出脚本已改为自动保存到本目录下的 `Generated_Models`，避免后续过程产物散落在根目录。

## 后续使用

1. 在 `Generate_SSM` 中运行导出脚本生成模型。
2. 将需要参与特征值分析的 `.mat` 复制或移动到 `EigenAnalysis\Generated_Models`。
3. 在 `EigenAnalysis` 中运行对应分析脚本。

本次整理没有删除文件，只进行了明确文件移动和路径修正。
