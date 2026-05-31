function ssmMat = locate_ssm_parameters_mat()
% Locate the active small-signal Parameters.mat with a deterministic priority.
%
% Priority:
% 1) WT_PMSG_GFM_Electromechanical_Validation/EigenAnalysis/Parameters.mat
% 2) WT_PMSG_GFM_小信号分析_最新整理包_20260526/EigenAnalysis/Parameters.mat
% 3) Modularized-Small-Signal-Modeling-of-Grid-Forming-Inverters-main/EigenAnalysis/Parameters.mat

root = fileparts(mfilename('fullpath'));
smallSignalRoot = fullfile(root, '..', '..', '..', '（1）小信号模型');

candidates = { ...
    fullfile(smallSignalRoot, 'WT_PMSG_GFM_Electromechanical_Validation', 'EigenAnalysis', 'Parameters.mat'), ...
    fullfile(smallSignalRoot, 'WT_PMSG_GFM_小信号分析_最新整理包_20260526', 'EigenAnalysis', 'Parameters.mat'), ...
    fullfile(smallSignalRoot, 'Modularized-Small-Signal-Modeling-of-Grid-Forming-Inverters-main', 'EigenAnalysis', 'Parameters.mat') ...
    };

ssmMat = '';
for i = 1:numel(candidates)
    if exist(candidates{i}, 'file') == 2
        ssmMat = candidates{i};
        return;
    end
end

error('Cannot locate small-signal Parameters.mat under: %s', smallSignalRoot);
end

