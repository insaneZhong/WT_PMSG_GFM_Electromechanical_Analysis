function pkg = setup_5mw_gfm_model_package()
%SETUP_5MW_GFM_MODEL_PACKAGE
% 中文说明：为模型包加入可复现的相对路径，并返回模型包根目录。
% 本函数只修改 MATLAB 当前会话的路径，不修改原始模型或工作区文件。

pkg = fileparts(mfilename('fullpath'));
addpath(genpath(pkg));

fprintf('5 MW 构网型 PMSG 模型包已加载：%s\n', pkg);
fprintf('主线 M0：models/M0/Grid_Forming_PMSG5MW_TwoMass_Idealized.slx\n');
fprintf('同源 SSM：ssm/M0_5MW_Aligned_Workpoint_and_SSM.mat\n');
end
