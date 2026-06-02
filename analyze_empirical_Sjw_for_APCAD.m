function R = analyze_empirical_Sjw_for_APCAD(cfg)
% 基于非线性仿真结果的经验频域指标：
% 1) S(jw) 近似：输出(omega_g/theta_tw/T_sh) 对扰动输入(风速阶跃)的频域比值
% 2) Ws/Wd 建议：围绕识别主峰自动给出权重中心频率与带宽建议

if nargin < 1, cfg = struct(); end
cfg = set_default(cfg, 'simStop', 10.0);
cfg = set_default(cfg, 'windStepMps', 0.5);
cfg = set_default(cfg, 'useSchemeA', false);
cfg = set_default(cfg, 'tailSec', 8.0);
cfg = set_default(cfg, 'fmin', 0.2);
cfg = set_default(cfg, 'fmax', 10.0);

root = fileparts(mfilename('fullpath'));
outDir = fullfile(root, 'Validation_Results', 'Freq_Design');
if ~exist(outDir,'dir'), mkdir(outDir); end

% 先跑基线与风速扰动
suite = run_disturbance_suite_unified(struct( ...
    'simStop', cfg.simStop, 'windStepMps', cfg.windStepMps, 'useSchemeA', cfg.useSchemeA));

base = suite.baseline.signals;
dist = suite.wind_step.signals;

chans = {'omega_g','theta_tw','T_sh'};
rows = struct('channel',{},'f_peak_hz',{},'mag_peak',{},'ws_center_hz',{},'ws_bw_hz',{});

fig = figure('Color','w','Position',[80 80 1200 700]);
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');

for k = 1:numel(chans)
    nm = chans{k};
    [f, Hmag, fpk, mpk] = local_empirical_tf(base.(nm), dist.(nm), base.omega_g.t, cfg);
    nexttile; hold on; grid on;
    if isempty(f)
        title([nm ' (insufficient)'], 'Interpreter','none');
        rows(end+1)=struct('channel',string(nm),'f_peak_hz',NaN,'mag_peak',NaN,'ws_center_hz',NaN,'ws_bw_hz',NaN); %#ok<AGROW>
        continue;
    end
    semilogy(f, Hmag, 'LineWidth',1.2);
    xline(fpk,'r--',sprintf('%.3f Hz',fpk));
    xlabel('Frequency (Hz)'); ylabel('|S_{emp}(j\omega)|');
    title(nm, 'Interpreter','none');
    rows(end+1)=struct('channel',string(nm),'f_peak_hz',fpk,'mag_peak',mpk, ... %#ok<AGROW>
        'ws_center_hz',fpk,'ws_bw_hz',max(0.2, 0.5*fpk));
end

nexttile;
axis off;
text(0.01,0.85,'APCAD 频域设计建议','FontWeight','bold');
text(0.01,0.65,'1) Ws 中心频率可取机械主峰附近（自动识别）');
text(0.01,0.50,'2) Ws 带宽先取 0.5*fp（再按时域超调/收敛迭代）');
text(0.01,0.35,'3) Wd 可在低频维持较大权重，抑制慢漂移');

T = struct2table(rows);
writetable(T, fullfile(outDir, 'empirical_Sjw_apcad.csv'));
saveas(fig, fullfile(outDir, 'empirical_Sjw_apcad.png'));
R = struct('table', T, 'cfg', cfg);
save(fullfile(outDir, 'empirical_Sjw_apcad.mat'), 'R');
fprintf('Saved: %s\n', fullfile(outDir, 'empirical_Sjw_apcad.csv'));
end

function [fBand, Hmag, fpk, mpk] = local_empirical_tf(baseSig, distSig, tRef, cfg)
fBand=[]; Hmag=[]; fpk=NaN; mpk=NaN;
if isempty(baseSig.t) || isempty(distSig.t), return; end
t = tRef(:);
if numel(t)<200, return; end
idx = t >= (t(end)-cfg.tailSec);
if nnz(idx)<200, return; end
yb = detrend(baseSig.y(idx));
yd = detrend(distSig.y(idx));
du = yd - yb;
dt = median(diff(t(idx))); fs = 1/max(dt,eps);
[Pdu, f] = pwelch(du, [], [], [], fs);
[Pyb, ~] = pwelch(yb, [], [], [], fs);
H = sqrt(Pdu ./ max(Pyb, eps)); % 经验比值（近似指标）
band = (f>=cfg.fmin) & (f<=cfg.fmax);
fBand = f(band); Hmag = H(band);
if isempty(fBand), return; end
[mpk, ii] = max(Hmag);
fpk = fBand(ii);
end

function cfg = set_default(cfg, name, val)
if ~isfield(cfg,name) || isempty(cfg.(name)), cfg.(name)=val; end
end
