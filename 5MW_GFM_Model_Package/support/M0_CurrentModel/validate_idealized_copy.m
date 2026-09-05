function result = validate_idealized_copy(stopTime, writeReport)
%VALIDATE_IDEALIZED_COPY 运行理想化副本的零扰动短时验收。
% 只在内存中读取 ToWorkspace 输出，不保存完整 SimulationOutput 或原始时序。
% 该脚本用于发现结构/符号/启动问题，不把短时运行自动判定为长期稳定。

if nargin < 1 || isempty(stopTime), stopTime = 0.2; end
if nargin < 2 || isempty(writeReport), writeReport = true; end
here = fileparts(mfilename('fullpath'));
mdl = 'Grid_Forming_PMSG5MW_TwoMass_Idealized';
addpath(here);

in = Simulink.SimulationInput(mdl);
in = in.setModelParameter('StopTime', num2str(stopTime, '%.15g'), ...
    'ReturnWorkspaceOutputs', 'on');
tic;
out = sim(in);
elapsed = toc;

names = {'stage4_Ppcc','stage4_Udc','tm_T_e','tm_T_sh', ...
    'tm_delta_omega_sh','tm_omega_g','tm_omega_t','stage4_PreSyn', ...
    'stage4_Pref_raw'};
result = struct();
result.model = mdl;
result.stopTime = stopTime;
result.elapsed_s = elapsed;
result.signals = struct();
for k = 1:numel(names)
    n = names{k};
    s = struct('available',false,'finite',false,'n',0,'t0',NaN,'t1',NaN,...
        'first',NaN,'last',NaN,'tailMean',NaN,'tailStd',NaN,...
        'min',NaN,'max',NaN);
    try
        ts = out.get(n);
        if isa(ts, 'timeseries')
            d = double(ts.Data(:));
            t = double(ts.Time(:));
            s.available = true;
            s.finite = all(isfinite(d));
            s.n = numel(d);
            s.t0 = t(1); s.t1 = t(end);
            s.first = d(1); s.last = d(end);
            idx = max(1, numel(d)-100):numel(d);
            s.tailMean = mean(d(idx));
            s.tailStd = std(d(idx));
            s.min = min(d); s.max = max(d);
        end
    catch
        % 缺少诊断量不终止结构验收，由报告明确标记。
    end
    result.signals.(n) = s;
end

% 仅作“运行完整性”判断；不把 0.2 s 零扰动运行等同于稳态通过。
result.runFinite = all(cellfun(@(n) result.signals.(n).available && ...
    result.signals.(n).finite, names));
result.longTermStable = false;
result.note = ['当前副本仍保留 MEX 内部 PI/VSG 状态；本脚本的 PASS 只表示仿真输出有限，' ...
    '不表示已经达到与显式连续小信号模型严格同源。'];

if writeReport
    report = fullfile(here, '02_Idealization_Validation_Report_CN.md');
    fid = fopen(report, 'w', 'n', 'UTF-8');
    assert(fid > 0, '无法写入验收报告。');
    c = onCleanup(@() fclose(fid));
    fprintf(fid, '# 5 MW 当前模型理想化副本验收报告\n\n');
    fprintf(fid, '- 模型：`%s.slx`\n- 仿真时长：%.6g s\n- 运行耗时：%.3f s\n', mdl, stopTime, elapsed);
    fprintf(fid, '- 运行完整性：**%s**\n- 长期稳定性：**尚未判定**\n\n', ternary(result.runFinite,'PASS','FAIL'));
    fprintf(fid, '## 末段统计（仅汇总，不保存原始时序）\n\n');
    fprintf(fid, '| 信号 | 样本数 | 首值 | 末值 | 末段均值 | 末段标准差 | 最小值 | 最大值 |\n|---|---:|---:|---:|---:|---:|---:|---:|\n');
    for k = 1:numel(names)
        s = result.signals.(names{k});
        if s.available
            fprintf(fid, '| `%s` | %d | %.6g | %.6g | %.6g | %.6g | %.6g | %.6g |\n', ...
                names{k},s.n,s.first,s.last,s.tailMean,s.tailStd,s.min,s.max);
        else
            fprintf(fid, '| `%s` | - | unavailable | unavailable | - | - | - | - |\n', names{k});
        end
    end
    fprintf(fid, '\n## 当前判定边界\n\n');
    fprintf(fid, '%s\n\n', result.note);
    fprintf(fid, '当前副本的主要未完成项：\n\n');
    fprintf(fid, '1. 将 MEX 控制器内部 PI/VSG 状态迁移为显式连续 Integrator；\n');
    fprintf(fid, '2. 重新核对 MSC/GSC 交流端口功率面、源极性和 DC-link 能量方向；\n');
    fprintf(fid, '3. 以同一平衡点生成小信号矩阵，并通过特征值、复转矩和小扰动响应验收。\n');
end
end

function y = ternary(cond, a, b)
if cond, y = a; else, y = b; end
end
