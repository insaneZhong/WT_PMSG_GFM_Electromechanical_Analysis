function diag = run_gfm_longterm_validation_30s()
% GFM-MWT 基础模型 30 s 长时无扰动验证。
% 保存完整慢变量趋势，并保留末端 0.2 s 三相电压电流细节用于波形检查。

root = fileparts(mfilename('fullpath'));
old = pwd;
cleanup = onCleanup(@() cd(old)); %#ok<NASGU>
cd(root);

setenv('MW_MINGW64_LOC', 'C:\mingw64');
clear mex;
bdclose('all');
mex main.c svpwm.c motorcontrol.c grid_forming_control.c;

diag = run_no_disturbance_diagnosis(false, 30.0, 0.0, struct( ...
    'VdcRef_V', 1200, ...
    'VacRef_V', 563, ...
    'Pref_W', 1e6, ...
    'Qref_var', 0, ...
    'SkipMexCompile', true, ...
    'UseNumericSfunParams', true, ...
    'SaveSeries', true, ...
    'SeriesMaxPoints', 60000, ...
    'ThreePhaseTail_s', 0.2));

outDir = fullfile(root, 'Validation_Results', 'LongTerm_30s');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
save(fullfile(outDir, 'gfm_mwt_longterm_30s.mat'), 'diag', '-v7.3');

diag.series.ppcc_smooth = diag.series.ppcc;
diag.series.ppcc_smooth.y = movmean(diag.series.ppcc.y, 201);
windowRows = zeros(5, 7);
for k = 1:5
    t0 = 5 * k;
    t1 = t0 + 5;
    idxU = diag.series.udc_meas.t >= t0 & diag.series.udc_meas.t < t1;
    idxP = diag.series.ppcc.t >= t0 & diag.series.ppcc.t < t1;
    idxT = diag.series.T_sh.t >= t0 & diag.series.T_sh.t < t1;
    idxW = diag.series.omega_g.t >= t0 & diag.series.omega_g.t < t1;
    relOmega = diag.series.omega_t.y(idxW) - diag.series.omega_g.y(idxW);
    windowRows(k, :) = [t0, t1, mean(diag.series.udc_meas.y(idxU)), ...
        range(diag.series.udc_meas.y(idxU)), mean(diag.series.ppcc.y(idxP)) / 1e3, ...
        range(diag.series.T_sh.y(idxT)) / 1e3, range(relOmega)];
end
windowTable = array2table(windowRows, 'VariableNames', { ...
    'Start_s', 'End_s', 'UdcMean_V', 'UdcPP_V', 'PpccMean_kW', 'TshPP_kNm', 'RelOmegaPP_radps'});
writetable(windowTable, fullfile(outDir, 'gfm_mwt_longterm_5s_windows.csv'));
save(fullfile(outDir, 'gfm_mwt_longterm_30s.mat'), 'diag', 'windowTable', '-v7.3');

fig = figure('Color', 'w', 'Position', [80 80 1280 900]);
tiledlayout(4, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
nexttile;
plot(diag.series.udc_meas.t, diag.series.udc_meas.y, 'LineWidth', 1.0);
ylabel('U_{dc} (V)'); grid on; title('GFM-MWT 30 s 无扰动长时验证');
nexttile;
plot(diag.series.ppcc_smooth.t, diag.series.ppcc_smooth.y / 1e3, 'LineWidth', 1.0);
ylabel('P_{PCC} (kW)'); grid on;
nexttile;
plot(diag.series.omega_g.t, diag.series.omega_t.y - diag.series.omega_g.y, 'LineWidth', 1.0);
ylabel('\omega_t-\omega_g (rad/s)'); grid on;
nexttile;
plot(diag.series.T_sh.t, diag.series.T_sh.y / 1e3, 'LineWidth', 1.0);
ylabel('T_{sh} (kN m)'); xlabel('Time (s)'); grid on;
exportgraphics(fig, fullfile(outDir, 'gfm_mwt_longterm_trends.png'), 'Resolution', 180);
savefig(fig, fullfile(outDir, 'gfm_mwt_longterm_trends.fig'));
close(fig);

fig = figure('Color', 'w', 'Position', [100 100 1280 760]);
tiledlayout(2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
nexttile;
plot(diag.series.pcc_uab.t, diag.series.pcc_uab.y, ...
    diag.series.pcc_ubc.t, diag.series.pcc_ubc.y, ...
    diag.series.pcc_uca.t, diag.series.pcc_uca.y, 'LineWidth', 0.8);
ylabel('PCC line voltage (V)'); grid on; legend('u_{ab}', 'u_{bc}', 'u_{ca}');
title('末端 0.2 s 三相波形');
nexttile;
plot(diag.series.pcc_ia.t, diag.series.pcc_ia.y, ...
    diag.series.pcc_ib.t, diag.series.pcc_ib.y, ...
    diag.series.pcc_ic.t, diag.series.pcc_ic.y, 'LineWidth', 0.8);
ylabel('PCC current (A)'); xlabel('Time (s)'); grid on; legend('i_a', 'i_b', 'i_c');
exportgraphics(fig, fullfile(outDir, 'gfm_mwt_three_phase_waveforms.png'), 'Resolution', 180);
savefig(fig, fullfile(outDir, 'gfm_mwt_three_phase_waveforms.fig'));
close(fig);

fid = fopen(fullfile(outDir, 'gfm_mwt_longterm_30s_summary.md'), 'w');
fprintf(fid, '# GFM-MWT 30 s 长时无扰动验证\n\n');
fprintf(fid, '- 基础模型有界运行判定: %d\n', diag.baseline_operational_flag);
fprintf(fid, '- PCC 输出功率末端均值: %.3f kW\n', diag.Ppcc_end_mean / 1e3);
fprintf(fid, '- PCC 输出功率相对 1 MW 误差: %.5f pu\n', diag.Ppcc_target_error_pu);
fprintf(fid, '- Udc 末端均值: %.3f V\n', diag.udc_end_mean);
fprintf(fid, '- Udc 末端范围: %.3f ~ %.3f V\n', diag.udc_end_min, diag.udc_end_max);
fprintf(fid, '- Udc 末端斜率: %.3f V/s\n', diag.udc_end_slope);
fprintf(fid, '- MSC Iq_ref 末端最大绝对值: %.3f A\n', diag.msc_iqref_end_maxabs);
fprintf(fid, '- PCC 三相电流 RMS: %.3f / %.3f / %.3f A\n', ...
    diag.pcc_ia_end_rms, diag.pcc_ib_end_rms, diag.pcc_ic_end_rms);
fprintf(fid, '- PCC 三相线电压 RMS: %.3f / %.3f / %.3f V\n', ...
    diag.pcc_uab_end_rms, diag.pcc_ubc_end_rms, diag.pcc_uca_end_rms);
fprintf(fid, '- 严格 DC 静止判定: %d\n', diag.dc_settled_flag);
fprintf(fid, '- 严格机械静止判定: %d\n', diag.mech_settled_flag);
fclose(fid);

fprintf('\n=== GFM-MWT 30 s 长时无扰动验证 ===\n');
fprintf('baseline_operational = %d\n', diag.baseline_operational_flag);
fprintf('Ppcc_end_mean        = %.3f kW\n', diag.Ppcc_end_mean / 1e3);
fprintf('Udc_end_mean         = %.3f V\n', diag.udc_end_mean);
fprintf('Udc_end_range        = %.3f ~ %.3f V\n', diag.udc_end_min, diag.udc_end_max);
fprintf('Saved: %s\n', outDir);
end
