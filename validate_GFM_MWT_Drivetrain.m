function metrics = validate_GFM_MWT_Drivetrain()
% Run no-disturbance/step validation and estimate observed torsional behavior.

root = fileparts(mfilename('fullpath'));
oldFolder = pwd;
cleanup = onCleanup(@() cd(oldFolder));
cd(root);

run('GFM_MWT_Nonlinear_Params.m');
mdl = 'Grid_Forming_PMSG';
load_system(mdl);

in = Simulink.SimulationInput(mdl);
in = in.setModelParameter('StopTime', num2str(sim_stop_time), ...
    'ReturnWorkspaceOutputs', 'on', ...
    'SimulationMode', 'normal');
out = sim(in);

omega_t = out.omega_t;
omega_g = out.omega_g;
theta_tw = out.theta_tw;
T_sh = out.T_sh;

t = omega_g.Time;
speedDiff = omega_t.Data - omega_g.Data;
after = t >= wind_step_time;
if wind_step_mps == 0
    analysisWindow = t >= 2.0;
else
    analysisWindow = t >= frequency_estimation_start;
end
steady = t >= (t(end) - 1.0);
trendWindow = t >= (t(end) - 1.0);

metrics = struct();
metrics.windStepTime_s = wind_step_time;
metrics.samples = numel(t);
metrics.initial_frequency_guess_Hz = f_sh_init_guess;
metrics.omega_t_final_radps = omega_t.Data(end);
metrics.omega_g_final_radps = omega_g.Data(end);
metrics.theta_tw_final_rad = theta_tw.Data(end);
metrics.T_sh_final_Nm = T_sh.Data(end);
metrics.max_abs_speed_difference_after_step_radps = max(abs(speedDiff(after)));
metrics.T_sh_peak_to_peak_after_step_Nm = max(T_sh.Data(after)) - min(T_sh.Data(after));
metrics.mean_T_sh_end_Nm = mean(T_sh.Data(steady));

[speedTrend, torqueTrend] = endWindowTrends(t(trendWindow), omega_g.Data(trendWindow), T_sh.Data(trendWindow));
metrics.omega_g_end_trend_radps2 = speedTrend;
metrics.T_sh_end_trend_Nmps = torqueTrend;
[metrics.observed_dominant_frequency_Hz, metrics.observed_peak_magnitude] = ...
    estimateDominantFrequency(T_sh.Time(analysisWindow), T_sh.Data(analysisWindow));
metrics.frequency_estimate_status = "provisional_not_steady_state";

% Keep placeholders for electrical/controller metrics to avoid downstream breakage.
metrics.P_pcc_end_mean_W = NaN;
metrics.P_pcc_end_trend_Wps = NaN;
metrics.Udc_end_mean_V = NaN;
metrics.Udc_end_trend_Vps = NaN;
metrics.P_ref_ctrl_end_mean_W = NaN;
metrics.P_meas_ctrl_end_mean_W = NaN;
metrics.P_meas_ctrl_end_trend_Wps = NaN;
metrics.w_ref_end_mean_radps = NaN;
metrics.PreSyn_end_mean = NaN;

resultDir = fullfile(root, 'Validation_Results');
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end
save(fullfile(resultDir, 'gfm_mwt_wind_step_results.mat'), ...
    'metrics', 'omega_t', 'omega_g', 'theta_tw', 'T_sh');

fig = figure('Visible', 'off', 'Color', 'w');
tiledlayout(4,1, 'TileSpacing', 'compact');
nexttile; plot(omega_t.Time, omega_t.Data, omega_g.Time, omega_g.Data, 'LineWidth', 1.0); grid on; ylabel('\omega (rad/s)'); legend('\omega_t','\omega_g');
nexttile; plot(theta_tw.Time, theta_tw.Data, 'LineWidth', 1.0); grid on; ylabel('\theta_{tw} (rad)');
nexttile; plot(T_sh.Time, T_sh.Data, 'LineWidth', 1.0); grid on; ylabel('T_{sh} (N m)');
nexttile; plot(t, speedDiff, 'LineWidth', 1.0); grid on; ylabel('\Delta\omega'); xlabel('Time (s)');
exportgraphics(fig, fullfile(resultDir, 'gfm_mwt_mechanical_signals.png'), 'Resolution', 180);
close(fig);

disp(metrics);
close_system(mdl, 0);
end

function [speedTrend, torqueTrend] = endWindowTrends(t, omega_g, T_sh)
pSpeed = polyfit(t(:), omega_g(:), 1);
pTorque = polyfit(t(:), T_sh(:), 1);
speedTrend = pSpeed(1);
torqueTrend = pTorque(1);
end

function [frequencyHz, peakMagnitude] = estimateDominantFrequency(t, signal)
signal = detrend(signal(:));
t = t(:);
dt = median(diff(t));
sampleRate = 1/dt;
n = numel(signal);
if n < 8 || max(abs(signal)) == 0
    frequencyHz = NaN;
    peakMagnitude = NaN;
    return;
end
window = 0.5 - 0.5*cos(2*pi*(0:n-1)'/(n-1));
spectrum = abs(fft(signal .* window));
freq = (0:n-1)' * sampleRate / n;
search = freq > 0.1 & freq < 20;
[peakMagnitude, peakIndex] = max(spectrum(search));
candidates = freq(search);
frequencyHz = candidates(peakIndex);
end

