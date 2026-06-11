%% Small-disturbance response figures for WT-PMSG small-signal models
% 本脚本用于补充论文实验中的小扰动时域响应图。
% 现有分析主要展示特征值、阻尼比和参与因子；本脚本进一步展示
% dx = A*x + B*u, dy = C*x + D*u 下的小扰动输出响应。
%
% 输出均为运行点附近的增量量，例如 Delta omega_g、Delta T_sh、Delta v_dc。

close all
clear
clc

%% 基本路径
this_dir = fileparts(mfilename('fullpath'));
if isempty(this_dir)
    this_dir = pwd;
end
cd(this_dir);

% MATPOWER 用于生成与特征值分析一致的潮流运行点。
addpath(genpath('D:\apps\matlab\R2024b\bin\matpower8.0'));

%% 加载参数
% Parameters.m 会保存 Parameters.mat 并 clear 工作区，因此运行后重新加载，
% 且所有用户配置都必须放在 run("Parameters.m") 之后。
run("Parameters.m");
this_dir = pwd;
params = load("Parameters.mat");
mpopt = mpoption('verbose', 0, 'out.all', 0);
generated_model_dir = fullfile(this_dir, 'Generated_Models');

%% 响应图配置
% 需要哪一组实验就保留 true；不需要可手动改成 false。
run_four_topology_set = true;  % GFL-WT / GFM-GWT / GFM-MWT / GFM-MWT+AD
run_dvc_type_set = true;       % Type-a / Type-c / Type-c+AD

% 小扰动幅值。建议保持小量，避免超出线性化适用范围。
cfg.t_end = 8.0;               % s
cfg.dt = 0.002;                % s
cfg.wind_step_mps = 0.10;      % Delta v_w, m/s
cfg.pref_step_pu = 0.01;       % Delta p_ref, pu of S_base
cfg.target_frequency_hz = 2.0; % 仅用于标注当前模态附近频段，不作为滤波先验
cfg.response_method = "low_frequency_modal"; % low_frequency_modal 或 full_lsim
cfg.max_modal_frequency_hz = 10.0; % 仅保留低频机电模态，避免高频变流器模态支配论文图

result_dir = fullfile(this_dir, 'Results', 'Small_Disturbance_Response_Results');
if ~exist(result_dir, 'dir')
    mkdir(result_dir);
end

all_metrics = table();

%% 四拓扑小扰动响应
if run_four_topology_set
    gfl = load("Unified_WT_PMSG_GFL.mat");
    gfm_gwt = load("Unified_WT_PMSG_GFM_GWT.mat");
    gfm_mwt = load("Unified_WT_PMSG_VSG.mat");
    gfm_mwt_ad = load("Unified_WT_PMSG_VSG_Damping.mat");

    models = struct( ...
        'key', {"GFL_WT", "GFM_GWT", "GFM_MWT", "GFM_MWT_AD"}, ...
        'label', {"GFL-WT", "GFM-GWT", "GFM-MWT", "GFM-MWT+AD"}, ...
        'data', {gfl.Unified_GFMI, gfm_gwt.Unified_GFMI, gfm_mwt.Unified_GFMI, gfm_mwt_ad.Unified_GFMI}, ...
        'color', {[0.05 0.42 0.62], [0.91 0.60 0.10], [0.78 0.28 0.18], [0.12 0.55 0.34]});

    set_dir = fullfile(result_dir, 'four_topology');
    if ~exist(set_dir, 'dir')
        mkdir(set_dir);
    end
    metrics = run_response_set(models, params, mpopt, cfg, set_dir, "Four topology");
    all_metrics = [all_metrics; metrics]; %#ok<AGROW>
end

%% DVC Type-a / Type-c 小扰动响应
if run_dvc_type_set
    type_a = load_model_mat(this_dir, generated_model_dir, "Unified_WT_PMSG_VSG_TypeA.mat");
    type_c = load_model_mat(this_dir, generated_model_dir, "Unified_WT_PMSG_VSG_TypeC.mat");
    type_c_ad = load_model_mat(this_dir, generated_model_dir, "Unified_WT_PMSG_VSG_TypeC_Damping.mat");

    models = struct( ...
        'key', {"GFM_MWT_TypeA", "GFM_MWT_TypeC", "GFM_MWT_TypeC_AD"}, ...
        'label', {"GFM-MWT-TypeA", "GFM-MWT-TypeC", "GFM-MWT-TypeC+AD"}, ...
        'data', {type_a.Unified_GFMI, type_c.Unified_GFMI, type_c_ad.Unified_GFMI}, ...
        'color', {[0.78 0.28 0.18], [0.05 0.42 0.62], [0.12 0.55 0.34]});

    set_dir = fullfile(result_dir, 'dvc_type');
    if ~exist(set_dir, 'dir')
        mkdir(set_dir);
    end
    metrics = run_response_set(models, params, mpopt, cfg, set_dir, "DVC type");
    all_metrics = [all_metrics; metrics]; %#ok<AGROW>
end

writetable(all_metrics, fullfile(result_dir, 'small_disturbance_response_metrics.csv'));
save(fullfile(result_dir, 'small_disturbance_response_summary.mat'), 'all_metrics', 'cfg');

fprintf('\nSmall-disturbance response figures written to:\n%s\n', result_dir);
disp(all_metrics);

%% Supporting functions
function metrics = run_response_set(models, params, mpopt, cfg, result_dir, set_label)
t = (0:cfg.dt:cfg.t_end).';
case_defs = struct( ...
    'name', {"wind_step", "pref_step"}, ...
    'input', {"v_w", "p_ref"}, ...
    'amplitude', {cfg.wind_step_mps, cfg.pref_step_pu * params.S_base}, ...
    'description', {"Wind-speed small step", "Active-power reference small step"});

metrics = table();
for c = 1:numel(case_defs)
    response = struct();
    for m = 1:numel(models)
        model = models(m).data;
        [A, B, C, D] = make_ss(model, mpopt, params);
        input_names = sym_names(model.U_unified);
        output_names = sym_names(model.Y_unified);

        u = zeros(numel(t), numel(input_names));
        input_index = find_name(input_names, case_defs(c).input);
        if isnan(input_index)
            warning('%s does not contain input %s. Skip this response.', models(m).label, case_defs(c).input);
            continue;
        end
        u(:, input_index) = case_defs(c).amplitude;

        if cfg.response_method == "full_lsim"
            sys = ss(A, B, C, D);
            y = lsim(sys, u, t);
            response_method = "full_lsim";
        else
            y = modal_step_response(A, B, C, D, input_index, case_defs(c).amplitude, t, cfg.max_modal_frequency_hz);
            response_method = sprintf('low_frequency_modal_0_%.3gHz', cfg.max_modal_frequency_hz);
        end

        response(m).key = models(m).key; %#ok<AGROW>
        response(m).label = models(m).label; %#ok<AGROW>
        response(m).color = models(m).color; %#ok<AGROW>
        response(m).t = t; %#ok<AGROW>
        response(m).y = y; %#ok<AGROW>
        response(m).outputs = output_names; %#ok<AGROW>
        response(m).method = response_method; %#ok<AGROW>

        metrics = [metrics; response_metrics(set_label, case_defs(c), models(m).label, response_method, t, y, output_names)]; %#ok<AGROW>
    end

    if ~isempty(response)
        plot_mechanical_response(response, case_defs(c), result_dir);
        plot_dc_power_response(response, case_defs(c), result_dir);
        plot_torsion_fft(response, case_defs(c), result_dir, cfg);
        save(fullfile(result_dir, sprintf('%s_response_data.mat', case_defs(c).name)), 'response', 'case_defs', 'cfg');
    end
end
end

function [A, B, C, D] = make_ss(model, mpopt, params)
params = add_operating_point(params, mpopt);
names = fieldnames(params);
values = cell(size(names));
for k = 1:numel(names)
    values{k} = params.(names{k});
end

A = double(subs(model.sym_A, names, values));
B = double(subs(model.sym_B, names, values));
C = double(subs(model.sym_C, names, values));
D = double(subs(model.sym_D, names, values));
end

function y = modal_step_response(A, B, C, D, input_index, amplitude, t, max_frequency_hz)
% 低频模态截断阶跃响应。
% 目的：展示机电耦合频段的小扰动响应，避免完整模型中的高频变流器模态
% 或数值不稳定模态掩盖 0~10 Hz 轴系动态。
[V, Lambda] = eig(A);
lambda = diag(Lambda);
modal_speed_hz = max(abs(real(lambda)), abs(imag(lambda))) / (2*pi);
keep = modal_speed_hz <= max_frequency_hz;
if ~any(keep)
    error('No modes are retained below %.3g Hz.', max_frequency_hz);
end

Vk = V(:, keep);
lk = lambda(keep);
modal_input = V \ B(:, input_index);
bk = amplitude * modal_input(keep);
direct = D(:, input_index).' * amplitude;

y = zeros(numel(t), size(C, 1));
for n = 1:numel(t)
    tn = t(n);
    modal_factor = zeros(size(lk));
    nonzero = abs(lk) > 1e-10;
    modal_factor(nonzero) = (exp(lk(nonzero) * tn) - 1) ./ lk(nonzero);
    modal_factor(~nonzero) = tn;
    x = Vk * (modal_factor .* bk);
    y(n, :) = real((C * x).' + direct);
end
end

function params = add_operating_point(params, mpopt)
mpc = SMIB_PowerFlow(params.rg, params.lg);
pf = runpf(mpc, mpopt);
assert(pf.success == 1, 'Power flow failed for the small-disturbance response case.');

angle_rad = deg2rad(pf.bus(1, 9));
voltage_mag = pf.bus(1, 8);
vc_phasor = voltage_mag * params.V_LL / sqrt(3) * exp(1j * angle_rad);
vg_phasor = params.V_LL / sqrt(3);
zt = params.rf2 + params.rg + 1j * 2*pi*params.f_base*(params.lf2 + params.lg);
z2 = params.rf2 + 1j * 2*pi*params.f_base*params.lf2;
i2_phasor = (vc_phasor - vg_phasor) / zt;
vpcc_phasor = (vc_phasor - i2_phasor*z2) * sqrt(2);

params.delta0 = angle_rad;
params.Vpcc_D0 = real(vpcc_phasor);
params.Vpcc_Q0 = imag(vpcc_phasor);
vc_dq0 = vc_phasor * exp(-1j*angle_rad) * sqrt(2);
params.Vc_d0 = real(vc_dq0);
params.Vc_q0 = imag(vc_dq0);
i2_dq0 = i2_phasor * exp(-1j*angle_rad) * sqrt(2);
params.i2_d0 = real(i2_dq0);
params.i2_q0 = imag(i2_dq0);
end

function names = sym_names(sym_vector)
names = string(arrayfun(@char, sym_vector(:), 'UniformOutput', false));
end

function idx = find_name(names, target)
idx = find(names == string(target), 1);
if isempty(idx)
    idx = NaN;
end
end

function values = get_output(y, output_names, name)
idx = find_name(output_names, name);
if isnan(idx)
    values = nan(size(y, 1), 1);
else
    values = y(:, idx);
end
end

function row = response_metrics(set_label, case_def, model_label, response_method, t, y, output_names)
omega_g = get_output(y, output_names, "omega_g");
omega_t = get_output(y, output_names, "omega_t");
theta_tw = get_output(y, output_names, "theta_tw");
T_sh = get_output(y, output_names, "T_sh");
v_dc = get_output(y, output_names, "v_dc");
p_m = get_output(y, output_names, "p_m");

row = table( ...
    string(set_label), string(case_def.name), string(case_def.description), string(model_label), string(response_method), ...
    peak_abs(omega_g), peak_abs(omega_t), peak_abs(theta_tw), peak_abs(T_sh), peak_abs(v_dc), peak_abs(p_m), ...
    final_value(omega_g), final_value(theta_tw), final_value(T_sh), final_value(v_dc), final_value(p_m), ...
    'VariableNames', {'Set', 'Case', 'CaseDescription', 'Model', 'ResponseMethod', ...
    'PeakAbs_omega_g', 'PeakAbs_omega_t', 'PeakAbs_theta_tw', 'PeakAbs_T_sh', 'PeakAbs_v_dc', 'PeakAbs_p_m', ...
    'Final_omega_g', 'Final_theta_tw', 'Final_T_sh', 'Final_v_dc', 'Final_p_m'});

if numel(t) > 2
    row.EndSlope_omega_g = end_slope(t, omega_g);
    row.EndSlope_T_sh = end_slope(t, T_sh);
    row.EndSlope_v_dc = end_slope(t, v_dc);
end
end

function val = peak_abs(x)
x = x(:);
x = x(~isnan(x));
if isempty(x)
    val = NaN;
else
    val = max(abs(x));
end
end

function val = final_value(x)
x = x(:);
x = x(~isnan(x));
if isempty(x)
    val = NaN;
else
    val = x(end);
end
end

function val = end_slope(t, x)
x = x(:);
valid = ~isnan(x);
t = t(valid);
x = x(valid);
if numel(x) < 5
    val = NaN;
    return;
end
tail = t >= max(t) - min(1.0, 0.25*max(t));
p = polyfit(t(tail), x(tail), 1);
val = p(1);
end

function plot_mechanical_response(response, case_def, result_dir)
fig = figure('Color', 'w', 'Position', [80 80 980 720]);
tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
channels = ["omega_g", "theta_tw", "T_sh"];
ylabels = {"\Delta\omega_g (rad/s)", "\Delta\theta_{tw} (rad)", "\Delta T_{sh} (N m)"};
for k = 1:numel(channels)
    ax = nexttile;
    ax.Toolbar.Visible = 'off';
    hold(ax, 'on');
    for m = 1:numel(response)
        y = get_output(response(m).y, response(m).outputs, channels(k));
        plot(ax, response(m).t, y, 'LineWidth', 1.5, 'Color', response(m).color, 'DisplayName', response(m).label);
    end
    grid(ax, 'on');
    ylabel(ax, ylabels{k});
    if k == 1
        title(ax, sprintf('%s: mechanical small-disturbance response', case_def.description));
        legend(ax, 'Location', 'best');
    end
    if k == numel(channels)
        xlabel(ax, 'Time (s)');
    end
end
exportgraphics(fig, fullfile(result_dir, sprintf('%s_mechanical_response.png', case_def.name)), 'Resolution', 300);
end

function plot_dc_power_response(response, case_def, result_dir)
fig = figure('Color', 'w', 'Position', [120 120 980 560]);
tiledlayout(2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
channels = ["v_dc", "p_m"];
ylabels = {"\Delta v_{dc} (V)", "\Delta P_{PCC} (W)"};
for k = 1:numel(channels)
    ax = nexttile;
    ax.Toolbar.Visible = 'off';
    hold(ax, 'on');
    for m = 1:numel(response)
        y = get_output(response(m).y, response(m).outputs, channels(k));
        plot(ax, response(m).t, y, 'LineWidth', 1.5, 'Color', response(m).color, 'DisplayName', response(m).label);
    end
    grid(ax, 'on');
    ylabel(ax, ylabels{k});
    if k == 1
        title(ax, sprintf('%s: DC-link and power response', case_def.description));
        legend(ax, 'Location', 'best');
    else
        xlabel(ax, 'Time (s)');
    end
end
exportgraphics(fig, fullfile(result_dir, sprintf('%s_dc_power_response.png', case_def.name)), 'Resolution', 300);
end

function plot_torsion_fft(response, case_def, result_dir, cfg)
fig = figure('Color', 'w', 'Position', [160 160 980 460]);
ax = axes(fig);
ax.Toolbar.Visible = 'off';
hold(ax, 'on');
for m = 1:numel(response)
    y = get_output(response(m).y, response(m).outputs, "T_sh");
    [f, amp] = single_sided_fft(response(m).t, y);
    band = f >= 0.1 & f <= 8.0;
    plot(ax, f(band), amp(band), 'LineWidth', 1.5, 'Color', response(m).color, 'DisplayName', response(m).label);
end
xline(ax, cfg.target_frequency_hz, 'k--', 'LineWidth', 0.8, 'DisplayName', 'current nominal torsion marker');
grid(ax, 'on');
xlabel(ax, 'Frequency (Hz)');
ylabel(ax, '|\Delta T_{sh}|');
title(ax, sprintf('%s: shaft-torque response spectrum', case_def.description));
legend(ax, 'Location', 'best');
exportgraphics(fig, fullfile(result_dir, sprintf('%s_torsion_fft.png', case_def.name)), 'Resolution', 300);
end

function [f, amp] = single_sided_fft(t, y)
t = t(:);
y = y(:);
valid = ~isnan(y);
t = t(valid);
y = y(valid);
if numel(y) < 4
    f = NaN;
    amp = NaN;
    return;
end
y = detrend(y);
dt = median(diff(t));
Fs = 1 / dt;
N = numel(y);
win = hann(N);
Y = fft(y .* win);
P2 = abs(Y / N);
amp = P2(1:floor(N/2)+1);
if numel(amp) > 2
    amp(2:end-1) = 2*amp(2:end-1);
end
f = Fs * (0:floor(N/2)) / N;
end

function data = load_model_mat(this_dir, generated_model_dir, file_name)
candidates = [
    fullfile(generated_model_dir, file_name)
    fullfile(this_dir, file_name)
    ];
for k = 1:numel(candidates)
    if exist(candidates(k), 'file') == 2
        data = load(candidates(k));
        return;
    end
end
error('Cannot find model MAT file: %s', file_name);
end
