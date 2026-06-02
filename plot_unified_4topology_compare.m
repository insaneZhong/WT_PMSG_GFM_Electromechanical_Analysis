function out = plot_unified_4topology_compare()
% 统一对照图：
% GFL-WT / GFM-GWT / GFM-MWT / GFM-MWT+AD
% 数据来源：
% 1) 小信号：run_ssm_control_mode_comparison_modular 输出
% 2) 非线性：当前模型可直接得到 baseline 与 schemeA 两组（映射到 GFM-MWT 与 GFM-MWT+AD）

root = fileparts(mfilename('fullpath'));
outDir = fullfile(root, 'Validation_Results', 'Unified_4Topology');
if ~exist(outDir, 'dir'), mkdir(outDir); end

% ---------- Small-signal summary ----------
ssmMat = fullfile(root, 'Validation_Results', 'Master_Run', 'small_signal_comparison_results.mat');
if exist(ssmMat, 'file') ~= 2
    ssmMat = fullfile(root, 'Validation_Results', 'small_signal_comparison_results.mat');
end
hasSsm = exist(ssmMat, 'file') == 2;
if hasSsm
    S = load(ssmMat);
    ssmTbl = [];
    if isfield(S, 'ssmResults') && isfield(S.ssmResults, 'modeSummary')
        ssmTbl = S.ssmResults.modeSummary;
    elseif isfield(S, 'ssmResults') && isfield(S.ssmResults, 'summary')
        ssmTbl = S.ssmResults.summary;
    end
    if isempty(ssmTbl)
        hasSsm = false;
    end
end

models = ["GFL","GFM_GWT","GFM_MWT","GFM_MWT_AD"];
labels = ["GFL-WT","GFM-GWT","GFM-MWT","GFM-MWT+AD"];
zeta = nan(1,4); freq = nan(1,4);
if hasSsm
    varNames = string(ssmTbl.Properties.VariableNames);
    colModel = find(varNames=="Model",1);
    colZeta = find(contains(lower(varNames), "damping"),1);
    colFreq = find(contains(lower(varNames), "freq"),1);
    if ~isempty(colModel) && ~isempty(colZeta) && ~isempty(colFreq)
        for i = 1:4
            idx = find(string(ssmTbl{:,colModel}) == models(i), 1, 'first');
            if ~isempty(idx)
                zeta(i) = ssmTbl{idx,colZeta};
                freq(i) = ssmTbl{idx,colFreq};
            end
        end
    end
end

% ---------- Nonlinear summary ----------
% baseline -> GFM-MWT ; schemeA -> GFM-MWT+AD
nlCsv = fullfile(root, 'Validation_Results', 'publication_figure_summary.csv');
udcSlope = nan(1,4);
if exist(nlCsv, 'file') == 2
    Tnl = readtable(nlCsv);
    if any(strcmp(Tnl.Properties.VariableNames,'scenario')) && any(strcmp(Tnl.Properties.VariableNames,'Udc_slope'))
        iBase = find(strcmp(string(Tnl.scenario), "baseline"),1);
        iAd   = find(strcmp(string(Tnl.scenario), "schemeA"),1);
        if ~isempty(iBase), udcSlope(3) = Tnl.Udc_slope(iBase); end
        if ~isempty(iAd),   udcSlope(4) = Tnl.Udc_slope(iAd); end
    end
end

% ---------- Plot ----------
fig = figure('Color','w','Position',[100 80 1180 720]);
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');

nexttile;
bar(categorical(labels), zeta);
ylabel('Damping Ratio');
title('小信号主模态阻尼比');
grid on;

nexttile;
bar(categorical(labels), freq);
ylabel('Frequency (Hz)');
title('小信号主模态频率');
grid on;

nexttile;
bar(categorical(labels), udcSlope);
ylabel('Udc Tail Slope (V/s)');
title('非线性10s尾段斜率（可用模型）');
grid on;

nexttile;
text(0.01, 0.80, '说明:', 'FontWeight','bold');
text(0.01, 0.65, '1) GFL/GFM-GWT 当前仅有小信号对照；');
text(0.01, 0.50, '2) 非线性栏位已接入 GFM-MWT 与 GFM-MWT+AD；');
text(0.01, 0.35, '3) 后续补充 GFL/GFM-GWT 非线性模型后可自动填充。');
axis off;

saveas(fig, fullfile(outDir, 'unified_4topology_compare.png'));

out = struct();
out.labels = labels;
out.zeta = zeta;
out.freq = freq;
out.udcSlope = udcSlope;
save(fullfile(outDir, 'unified_4topology_compare.mat'), 'out');
writetable(table(labels(:), zeta(:), freq(:), udcSlope(:), ...
    'VariableNames', {'Topology','DampingRatio','FrequencyHz','UdcSlope'}), ...
    fullfile(outDir, 'unified_4topology_compare.csv'));
fprintf('Saved: %s\n', fullfile(outDir, 'unified_4topology_compare.png'));
end
