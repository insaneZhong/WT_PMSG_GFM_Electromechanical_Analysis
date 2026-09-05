function report = run_liu2024_5mw_full_load_validation()
%RUN_LIU2024_5MW_FULL_LOAD_VALIDATION One reproducible 5 MW full-load check.
% It uses the sole active 5 MW model and overwrites only its latest result.

root = fileparts(mfilename('fullpath'));
p = Liu2024_5MW_Params();
out = run_liu2024_5mw_experiment( ...
    'StopTime', 60, 'PowerRamp', 0.50e6, 'SimulationMode', 'accelerator');

names = {'stage4_Udc','stage4_Ppcc','tm_T_e','tm_T_sh','tm_omega_g', ...
    'tm_omega_t','tm_theta_tw','stage4_Taero_effective', ...
    'wind_Pref_mppt','wind_beta_deg','wind_lambda','wind_Cp', ...
    'wind_Paero','wind_Pmppt_raw','msc_diagnostic_vector'};
series = struct();
for k = 1:numel(names)
    series.(names{k}) = out.get(names{k});
end

tailStart = 58.0;
[t, udc] = tail(series.stage4_Udc, tailStart);
[~, ppcc] = tail(series.stage4_Ppcc, tailStart);
[tg, wg] = tail(series.tm_omega_g, tailStart);
[~, wt] = tail(series.tm_omega_t, tailStart);
[~, theta] = tail(series.tm_theta_tw, tailStart);
[~, te] = tail(series.tm_T_e, tailStart);
[~, tsh] = tail(series.tm_T_sh, tailStart);
[~, pref] = tail(series.wind_Pref_mppt, tailStart);
[~, beta] = tail(series.wind_beta_deg, tailStart);
[~, lambda] = tail(series.wind_lambda, tailStart);
[~, cp] = tail(series.wind_Cp, tailStart);
[~, paero] = tail(series.wind_Paero, tailStart);

diagnostics = series.msc_diagnostic_vector;
diagIndex = diagnostics.Time >= tailStart;
diagTail = squeeze(diagnostics.Data(diagIndex,:));
if size(diagTail,2) ~= 37 && size(diagTail,1) == 37
    diagTail = diagTail.';
end

report = struct();
report.tail_window_s = [tailStart t(end)];
report.udc_mean_V = mean(udc);
report.udc_slope_Vps = slope(t, udc);
report.ppcc_mean_W = mean(ppcc);
report.ppcc_slope_Wps = slope(t, ppcc);
report.omega_g_slope_radps2 = slope(tg, wg);
report.omega_t_slope_radps2 = slope(tg, wt);
report.omega_g_mean_radps = mean(wg);
report.omega_t_mean_radps = mean(wt);
report.theta_slope_radps = slope(tg, theta);
report.theta_pp_rad = max(theta)-min(theta);
report.torque_residual_pu = abs(mean(tsh)+mean(te))/p.T_e0;
report.power_export_magnitude_pu = abs(report.ppcc_mean_W)/p.P_wt_rated;
report.pref_mean_W = mean(pref);
report.beta_mean_deg = mean(beta);
report.lambda_mean = mean(lambda);
report.cp_mean = mean(cp);
report.paero_mean_W = mean(paero);
report.gsc_id_max_A = max(abs(diagTail(:,24)));
report.msc_iq_max_A = max(abs(diagTail(:,33)));
report.msc_modulation_max = max(abs(diagTail(:,37)));
report.udc_within_2pct = abs(report.udc_mean_V-p.Vdc)/p.Vdc < 0.02;
report.power_within_1pct = abs(report.power_export_magnitude_pu-1) < 0.01;
report.udc_slope_pass = abs(report.udc_slope_Vps) < 5;
report.power_slope_pass = abs(report.ppcc_slope_Wps) < 5e3;
report.speed_slope_pass = max(abs([report.omega_g_slope_radps2, ...
    report.omega_t_slope_radps2])) < 0.01;
report.speed_magnitude_pass = max(abs([report.omega_g_mean_radps, ...
    report.omega_t_mean_radps]-p.omega_m0))/p.omega_m0 < 0.05;
report.twist_slope_pass = abs(report.theta_slope_radps) < 5e-4;
report.modulation_pass = report.msc_modulation_max < 0.90;
report.all_steady_pass = report.udc_within_2pct && ...
    report.power_within_1pct && report.udc_slope_pass && ...
    report.power_slope_pass && report.speed_slope_pass && ...
    report.speed_magnitude_pass && ...
    report.twist_slope_pass && report.modulation_pass;
report.saved_result = fullfile(root,'Validation_Results','liu2024_5mw_active_run.mat');
save(report.saved_result, 'series', 'report', '-v7.3');
analyze_5mw_gfm_diagnostics(report.saved_result, ...
    fullfile(root,'Validation_Results','5MW_GFM_Characterization'));
fprintf(['5 MW full-load: Udc=%.2f V (%.3f V/s), Ppcc=%.3f MW ' ...
    '(%.3f kW/s), omegaSlope=[%.5g %.5g], thetaSlope=%.5g, mod=%.3f, PASS=%d\n'], ...
    report.udc_mean_V,report.udc_slope_Vps,report.ppcc_mean_W/1e6, ...
    report.ppcc_slope_Wps/1e3,report.omega_t_slope_radps2, ...
    report.omega_g_slope_radps2,report.theta_slope_radps, ...
    report.msc_modulation_max,report.all_steady_pass);
end

function [t,y] = tail(ts,t0)
t = ts.Time(:); y = ts.Data(:);
idx = t >= t0; t = t(idx); y = y(idx);
end

function value = slope(t,y)
q = polyfit(t-t(1), y, 1);
value = q(1);
end
