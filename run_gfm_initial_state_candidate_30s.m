function diag = run_gfm_initial_state_candidate_30s()
% Generate a reusable initial-state snapshot from a bounded GFM baseline.
% The 1200 V operating point is the current nonlinear-model baseline.

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
    'SaveFinalState', true, ...
    'SaveOperatingPoint', true, ...
    'FinalStateName', 'xInitial', ...
    'SaveSeries', true, ...
    'SeriesMaxPoints', 60000, ...
    'ThreePhaseTail_s', 0.2));

outDir = fullfile(root, 'Validation_Results', 'Initial_State');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

save(fullfile(outDir, 'gfm_mwt_initial_candidate_30s_diag.mat'), 'diag', '-v7.3');
if ~isfield(diag, 'final_state') || isempty(diag.final_state)
    error('Final state was not captured.');
end

xInitial = diag.final_state; %#ok<NASGU>
candidateFile = fullfile(outDir, 'Grid_Forming_PMSG_Init_candidate.mat');
save(candidateFile, 'xInitial', 'diag', '-v7.3');

snapshotReusable = diag.baseline_operational_flag && ...
    diag.dc_settled_flag && diag.mech_settled_flag;
if snapshotReusable
    officialFile = fullfile(outDir, 'Grid_Forming_PMSG_Init.mat');
    save(officialFile, 'xInitial', 'diag', '-v7.3');
    fprintf('Published reusable initial state: %s\n', officialFile);
else
    warning(['Candidate retained but not published. Electrical and mechanical ' ...
        'settling gates did not all pass: %s'], candidateFile);
end

fprintf('\n=== GFM 30 s initial-state candidate ===\n');
fprintf('baseline_operational = %d\n', diag.baseline_operational_flag);
fprintf('dc_settled           = %d\n', diag.dc_settled_flag);
fprintf('mech_settled         = %d\n', diag.mech_settled_flag);
fprintf('Ppcc_end_mean        = %.3f kW\n', diag.Ppcc_end_mean / 1e3);
fprintf('Udc_end_mean         = %.3f V\n', diag.udc_end_mean);
fprintf('Udc_end_slope        = %.6f V/s\n', diag.udc_end_slope);
end
