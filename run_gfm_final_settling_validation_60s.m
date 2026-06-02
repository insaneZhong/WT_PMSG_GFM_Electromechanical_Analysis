function diag = run_gfm_final_settling_validation_60s()
% GFM-MWT 基础模型 60 s 无扰动稳态确认。
% 仅验证当前控制参数，不修改模型结构或头文件参数。
% 为降低长时验证开销，本脚本保存慢变量和窗口统计，不生成图片。

root = fileparts(mfilename('fullpath'));
old = pwd;
cleanup = onCleanup(@() cd(old)); %#ok<NASGU>
cd(root);

setenv('MW_MINGW64_LOC', 'C:\mingw64');
clear mex;
bdclose('all');
mex main.c svpwm.c motorcontrol.c grid_forming_control.c;

diag = run_no_disturbance_diagnosis(false, 60.0, 0.0, struct( ...
    'VdcRef_V', 1200, ...
    'VacRef_V', 563, ...
    'Pref_W', 1e6, ...
    'Qref_var', 0, ...
    'SkipMexCompile', true, ...
    'UseNumericSfunParams', true, ...
    'SaveFinalState', true, ...
    'SaveOperatingPoint', true, ...
    'FinalStateName', 'xInitial', ...
    'SaveSeries', true, ...
    'SeriesMaxPoints', 90000, ...
    'ThreePhaseTail_s', 0.2));

outDir = fullfile(root, 'Validation_Results', 'Final_Settling_60s');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
initDir = fullfile(root, 'Validation_Results', 'Initial_State');
if ~exist(initDir, 'dir')
    mkdir(initDir);
end
save(fullfile(outDir, 'gfm_mwt_final_settling_60s_raw.mat'), 'diag', '-v7.3');
if isfield(diag, 'final_state') && ~isempty(diag.final_state)
    xInitial = diag.final_state; %#ok<NASGU>
    candidateFile = fullfile(initDir, 'Grid_Forming_PMSG_Init_candidate.mat');
    save(candidateFile, 'xInitial', 'diag', '-v7.3');
    snapshotReusable = diag.baseline_operational_flag && ...
        diag.dc_settled_flag && diag.mech_settled_flag;
    if snapshotReusable
        save(fullfile(initDir, 'Grid_Forming_PMSG_Init.mat'), 'xInitial', 'diag', '-v7.3');
    else
        warning(['Final state saved as candidate only. The run did not pass ' ...
            'the electrical and mechanical settling gates: %s'], candidateFile);
    end
end

try
    windowStart = (5:5:55)';
    windowEnd = windowStart + 5;
    n = numel(windowStart);
    windowRows = zeros(n, 7);
    for k = 1:n
        idxU = diag.series.udc_meas.t >= windowStart(k) & diag.series.udc_meas.t < windowEnd(k);
        idxP = diag.series.ppcc.t >= windowStart(k) & diag.series.ppcc.t < windowEnd(k);
        idxT = diag.series.T_sh.t >= windowStart(k) & diag.series.T_sh.t < windowEnd(k);
        idxWg = diag.series.omega_g.t >= windowStart(k) & diag.series.omega_g.t < windowEnd(k);
        idxWt = diag.series.omega_t.t >= windowStart(k) & diag.series.omega_t.t < windowEnd(k);
        relOmega = diag.series.omega_t.y(idxWt) - diag.series.omega_g.y(idxWg);
        windowRows(k, :) = [windowStart(k), windowEnd(k), ...
            mean(diag.series.udc_meas.y(idxU)), range(diag.series.udc_meas.y(idxU)), ...
            mean(diag.series.ppcc.y(idxP)) / 1e3, range(diag.series.T_sh.y(idxT)) / 1e3, ...
            range(relOmega)];
    end
    windowTable = array2table(windowRows, 'VariableNames', { ...
        'Start_s', 'End_s', 'UdcMean_V', 'UdcPP_V', 'PpccMean_kW', ...
        'TshPP_kNm', 'RelOmegaPP_radps'});

    save(fullfile(outDir, 'gfm_mwt_final_settling_60s.mat'), 'diag', 'windowTable', '-v7.3');
    writetable(windowTable, fullfile(outDir, 'gfm_mwt_final_settling_5s_windows.csv'));
catch ME
    errorPath = fullfile(outDir, 'postprocess_error.txt');
    fid = fopen(errorPath, 'w');
    fprintf(fid, '%s\n', getReport(ME, 'extended', 'hyperlinks', 'off'));
    fclose(fid);
    rethrow(ME);
end

fprintf('\n=== GFM-MWT 60 s 无扰动稳态确认 ===\n');
fprintf('baseline_operational = %d\n', diag.baseline_operational_flag);
fprintf('dc_settled           = %d\n', diag.dc_settled_flag);
fprintf('mech_settled         = %d\n', diag.mech_settled_flag);
fprintf('Ppcc_end_mean        = %.3f kW\n', diag.Ppcc_end_mean / 1e3);
fprintf('Ppcc_target_error    = %.6f pu\n', diag.Ppcc_target_error_pu);
fprintf('Udc_end_mean         = %.3f V\n', diag.udc_end_mean);
fprintf('Udc_end_slope        = %.6f V/s\n', diag.udc_end_slope);
fprintf('Saved: %s\n', outDir);
disp(windowTable);
end
