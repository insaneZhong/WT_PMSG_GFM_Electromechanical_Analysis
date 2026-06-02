function out = analyze_nonlinear_blind_fft_psd(cfg)
% 盲频谱识别：自动找主峰，不预设2Hz
if nargin < 1
    cfg = struct();
end
cfg = set_default(cfg, 'simStop', 10.0);
cfg = set_default(cfg, 'windStep', 0.0);
cfg = set_default(cfg, 'fmin', 0.2);
cfg = set_default(cfg, 'fmax', 10.0);
cfg = set_default(cfg, 'tailSec', 8.0);
cfg = set_default(cfg, 'useSchemeA', false);
cfg = set_default(cfg, 'ctrlOverrides', struct());

root = fileparts(mfilename('fullpath'));
outDir = fullfile(root, 'Validation_Results');
figDir = fullfile(outDir, 'Figures');
if ~exist(outDir, 'dir'), mkdir(outDir); end
if ~exist(figDir, 'dir'), mkdir(figDir); end

ovr = cfg.ctrlOverrides;
ovr.SaveSeries = true;
diag = run_no_disturbance_diagnosis(cfg.useSchemeA, cfg.simStop, cfg.windStep, ovr);

channels = {'omega_g','omega_t','theta_tw','T_sh','T_e','pmeas_out','udc_meas'};
rows = struct('channel',{},'f_peak_hz',{},'amp_peak',{},'bw_hz',{},'notes',{});

fig = figure('Name','Blind PSD Peaks','Color','w','Position',[80 80 1200 700]); %#ok<NASGU>
tiledlayout(3,3,'TileSpacing','compact','Padding','compact');

for k = 1:numel(channels)
    ch = channels{k};
    nexttile; hold on; grid on;
    if ~isfield(diag, 'series') || ~isfield(diag.series, ch) || isempty(diag.series.(ch).t)
        title(sprintf('%s (missing)', ch), 'Interpreter', 'none');
        rows(end+1) = struct('channel',string(ch),'f_peak_hz',NaN,'amp_peak',NaN,'bw_hz',NaN,'notes',"missing"); %#ok<AGROW>
        continue;
    end
    s = diag.series.(ch);
    [f, pxx, info] = local_psd_peak(s.t, s.y, cfg.fmin, cfg.fmax, cfg.tailSec);
    if isempty(f)
        title(sprintf('%s (insufficient data)', ch), 'Interpreter', 'none');
        rows(end+1) = struct('channel',string(ch),'f_peak_hz',NaN,'amp_peak',NaN,'bw_hz',NaN,'notes',"insufficient"); %#ok<AGROW>
        continue;
    end
    plot(f, pxx, 'LineWidth', 1.2);
    if isfinite(info.fPeak)
        xline(info.fPeak, 'r--', sprintf('%.3f Hz', info.fPeak));
    end
    xlabel('Frequency (Hz)');
    ylabel('PSD');
    title(ch, 'Interpreter', 'none');
    rows(end+1) = struct('channel',string(ch), ...
        'f_peak_hz',info.fPeak, 'amp_peak',info.pPeak, 'bw_hz',info.bw3dB, 'notes',"ok"); %#ok<AGROW>
end

T = struct2table(rows);
writetable(T, fullfile(outDir, 'blind_psd_peaks.csv'));
saveas(gcf, fullfile(figDir, 'blind_psd_peaks.png'));
save(fullfile(outDir, 'blind_psd_peaks.mat'), 'T', 'diag', 'cfg');

out = struct('diag', diag, 'table', T, 'cfg', cfg);
disp(T);
fprintf('Saved: %s\n', fullfile(outDir, 'blind_psd_peaks.csv'));
end

function [f, pxx, info] = local_psd_peak(t, y, fmin, fmax, tailSec)
info = struct('fPeak', NaN, 'pPeak', NaN, 'bw3dB', NaN);
t = t(:); y = y(:);
if numel(t) < 128 || numel(y) ~= numel(t)
    f = []; pxx = [];
    return;
end

idx = t >= (t(end) - tailSec);
if nnz(idx) < 128
    f = []; pxx = [];
    return;
end
t1 = t(idx); y1 = detrend(y(idx));
dt = median(diff(t1));
fs = 1/max(dt, eps);

[pxxAll, fAll] = pwelch(y1, [], [], [], fs);
band = (fAll >= fmin) & (fAll <= fmax);
f = fAll(band); pxx = pxxAll(band);
if isempty(f)
    return;
end

[pmax, imax] = max(pxx);
info.fPeak = f(imax);
info.pPeak = pmax;
halfPow = pmax / 2;
left = find(pxx(1:imax) <= halfPow, 1, 'last');
right = imax - 1 + find(pxx(imax:end) <= halfPow, 1, 'first');
if isempty(left) || isempty(right)
    info.bw3dB = NaN;
else
    info.bw3dB = f(right) - f(left);
end
end

function cfg = set_default(cfg, name, val)
if ~isfield(cfg, name) || isempty(cfg.(name))
    cfg.(name) = val;
end
end
