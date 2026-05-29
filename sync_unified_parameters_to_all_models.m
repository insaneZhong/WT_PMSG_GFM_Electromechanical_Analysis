function sync_unified_parameters_to_all_models()
% Sync one unified Parameters.m to all small-signal model copies and
% regenerate each local Parameters.mat.

root = fileparts(mfilename('fullpath'));
src = fullfile(root, 'WT_PMSG_GFM_Electromechanical_Validation', 'EigenAnalysis', 'Parameters.m');
if exist(src, 'file') ~= 2
    error('Source Parameters.m not found: %s', src);
end

targets = { ...
    fullfile(root, 'WT_PMSG_GFM_小信号分析_最新整理包_20260526', 'EigenAnalysis', 'Parameters.m'), ...
    fullfile(root, 'Modularized-Small-Signal-Modeling-of-Grid-Forming-Inverters-main', 'EigenAnalysis', 'Parameters.m'), ...
    fullfile(root, 'Modularized-Small-Signal-Modeling-of-Grid-Forming-Inverters-main', '归档成果', 'WT_PMSG_GFM_小信号分析_最新整理包_20260526', 'EigenAnalysis', 'Parameters.m') ...
    };

fprintf('Source: %s\n', src);
for i = 1:numel(targets)
    t = targets{i};
    if exist(t, 'file') == 2
        copyfile(src, t, 'f');
        fprintf('[OK] synced: %s\n', t);
    else
        fprintf('[SKIP] missing target: %s\n', t);
    end
end

% Regenerate Parameters.mat in each EigenAnalysis directory
regenDirs = unique([{fileparts(src)}, cellfun(@fileparts, targets, 'UniformOutput', false)]);
for i = 1:numel(regenDirs)
    d = regenDirs{i};
    if exist(fullfile(d, 'Parameters.m'), 'file') == 2
        % Parameters.m ends with "clear all". Running it directly inside
        % this function may clear local variables and interrupt the sync.
        % Execute it in base workspace and load/save by file side effect.
        oldPwd = pwd;
        cmd = sprintf('cd(''%s''); run(''Parameters.m''); cd(''%s'')', escape_path(d), escape_path(oldPwd));
        evalin('base', cmd);
        fprintf('[OK] regenerated Parameters.mat in: %s\n', d);
    end
end

fprintf('Done.\n');
end

function p = escape_path(p)
p = strrep(p, '''', '''''');
end
