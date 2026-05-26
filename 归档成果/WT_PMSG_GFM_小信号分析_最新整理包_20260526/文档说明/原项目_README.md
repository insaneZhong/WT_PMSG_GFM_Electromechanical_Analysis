# Modularized-Small-Signal-Modeling-of-Grid-Forming-Inverters
# 模块化并网型逆变器小信号建模

## Overview
## 概述

This repository provides a comprehensive power system toolkit for the modular small-signal modeling of Grid-Forming Inverters (GFMIs). It implements the complete development of small-signal models for various GFMI control strategies, along with analytical tools for detailed stability studies including eigenvalue analysis.
本仓库提供了一个完整的电力系统工具包，用于模块化并网型逆变器（GFMI）的分段小信号建模。它实现了多种GFMI控制策略的小信号模型的完整构建，并附带用于稳定性分析（包括特征值分析）的工具。

The files in this repository are designed to guide you through building small-signal models of GFMIs from first principles, and enable you to conduct systematic analyses of these models under a wide range of control and grid conditions.
仓库中的文件旨在从基本原理引导您构建GFMI的小信号模型，并使您能够在各种控制策略和电网条件下对这些模型进行系统分析。

## Citing this repository
## 引用本仓库

If you find this repository helpful in your own research or modeling activities, we would be sincerely grateful if you consider citing the following publication:
如果您在研究或建模工作中使用本仓库，请考虑引用以下论文，我们将非常感谢您的支持：

> Sohail A. Ali, Paul Serna-Torre, Patricia Hidalgo-Gonzalez, Mehdi Ghazavi Dozein, and Behrooz Bahrani, 
> "Modularized Small-Signal Modeling of Grid-Forming Inverters", IEEE Open Access, 2025

You can find the corresponding citation (.bib) on the IEEE website.
该论文的BibTeX引用可以在IEEE网站上找到。

## Get Started with this Repository
## 开始使用本仓库

While the repository is self-contained, we strongly recommend reading the accompanying paper to fully understand the modeling framework and analysis procedures implemented here.
虽然仓库本身是自包含的，但我们强烈建议先阅读配套论文，以便充分理解这里实现的建模框架和分析过程。

You can begin using the files of this repository by following these steps:
您可以按以下步骤开始使用本仓库的文件：

1.  Install [Matpower](https://matpower.org/) if you do not have it already installed on your computer. 
1.  如果您的电脑上还没有安装 [Matpower](https://matpower.org/)，请先安装它。

    - During MATPOWER installation, select option 3 to ensure proper integration.
    - 在安装MATPOWER时，选择选项3以确保正确集成。
    - Matpower is required to obtain a power flow solution which is used to calculate linear operating point of the small-signal
    models. (It is important to perform power flow when you vary circuit parameters, as these affect your operating point and consequently your eigenvalues)
    - Matpower用于计算潮流解，这些结果用于小信号模型的线性工作点计算。（当改变电路参数时，执行潮流计算非常重要，因为它们会影响工作点并进而影响特征值）

2.  Go to the folder "Generate_SSM". You will find all the models developed in our paper. It contains several MATLAB .mlx and .mat files that correspond to each model. For example, running the "VSG_model.mlx" does the following:
2.  进入文件夹 "Generate_SSM"，您会找到论文中开发的所有模型。该文件夹包含多个与每个模型对应的 MATLAB .mlx 和 .mat 文件。例如，运行 "VSG_model.mlx" 会执行以下操作：
    - Builds subsystems, algebraic loops and performs the Component Connection Method (CCM) to provide symbolic model of the VSG.
    - 构建子系统、代数环路，并执行组件连接方法（CCM），以生成VSG的符号模型。
    - This model is saved as a .mat file and can be used for eigenvalue analysis (Part 3) and validation with an EMT model.
    - 该模型会保存为 .mat 文件，可用于特征值分析（第3部分）和与电磁暂态（EMT）模型的验证。
    - The same process is repeated for all models
    - 所有模型都会重复相同的流程。

3.  Go to the folder "EigenAnalysis". This file helps you study how the stability of your system changes when you adjust control or grid parameters.
3.  进入文件夹 "EigenAnalysis"。这里的脚本帮助您研究当调节控制或电网参数时系统稳定性如何变化。
    - Copy the symbolic models you created in Step 2 (e.g Unified_VSG.mat) into this folder.
    - 将步骤2中生成的符号模型（例如 Unified_VSG.mat）复制到此文件夹中。
    - The scripts will convert all your variables in the symbolic models into numeric with values in the "Parameters.m" except for the parameter you intend to vary.
    - 脚本会将符号模型中的变量转换为数值，数值来自 "Parameters.m"，除非该参数是您希望变化的参数。
    - To help you get started, the provided files can vary feedforward, inertia, short-circuit ratio and XR ratio for VSG.
    - 为了帮助您入门，提供的文件可以用于改变VSG的前馈、惯量、短路比和 XR 比率。
    - You can can do this for any model by simplying loading another model in the folder.
    - 对于任何模型，您只需加载该文件夹中的其他模型即可执行类似分析。

## Description of executable files in this repository.
## 本仓库可执行文件说明

1. SMIB_PowerFlow.m: Computes a power flow solution for the GFMI connected to an infinite bus using Matpower.
1. SMIB_PowerFlow.m：使用Matpower计算连接到无限总线的GFMI的潮流解。
2. Parameters.m: Contains the numerical parameters for the GFMI models.
2. Parameters.m：包含GFMI模型的数值参数。
3. CGVSG.mxl: Generates the symbolic small-signal model for a Compensated generalized virtual synchronous generator.
3. CGVSG.mlx：生成补偿广义虚拟同步发电机的符号小信号模型。
4. Droop_Model.mlx: Generates the symbolic small-signal model for a Droop-based GFMI.
4. Droop_Model.mlx：生成基于下垂控制的GFMI的符号小信号模型。
5. LPF_Model.mlx: Generates the symbolic small-signal model for a Low-pass-filter-based GFMI.
5. LPF_Model.mlx：生成基于低通滤波器的GFMI的符号小信号模型。
6. VSG_Model.mlx: Generates the symbolic small-signal model for a Virtual synchronous generator.
6. VSG_Model.mlx：生成虚拟同步发电机的符号小信号模型。
7. VSG_Model_VI.mlx: Generates the symbolic small-signal model for a Virtual Synchronous generator with algebraic virtual impedance.
7. VSG_Model_VI.mlx：生成具有代数虚拟阻抗的虚拟同步发电机的符号小信号模型。
8. VSG_model_No_CC.mlx: Generates the symbolic small-signal model for a Virtual Synchronous generator with no current controller. This is reduced model version of
the VSG_Model.
8. VSG_model_No_CC.mlx：生成无电流控制器的虚拟同步发电机的符号小信号模型。这是VSG_Model的约简模型版本。
9. VSG_model_No_VC.mlx: Generates the symbolic small-signal model for a Virtual Synchronous generator without voltage or current controller. This is reduced model version of
the VSG_Model.
9. VSG_model_No_VC.mlx：生成无电压或电流控制器的虚拟同步发电机的符号小信号模型。这是VSG_Model的约简模型版本。
10. VSG_SCR.mlx: Conducts the eigenvalue analysis for a VSG for grid strength variation.
10. VSG_SCR.mlx：进行VSG的特征值分析，以考察电网强度变化影响。

## Run into some issues?
## 遇到问题？

If you have any question or run into some issues, please contact: sohail.ali[at]monash.edu, or psernatorre[at]ucsd.edu.
如果您有任何问题或遇到麻烦，请联系：sohail.ali[at]monash.edu，或 psernatorre[at]ucsd.edu。
