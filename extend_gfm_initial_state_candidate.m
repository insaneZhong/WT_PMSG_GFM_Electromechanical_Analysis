function diag = extend_gfm_initial_state_candidate(extraSeconds, inputFile)
% Continue an operating-point candidate and publish it only after settling.

root = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(extraSeconds)
    extraSeconds = 15.0;
end
if nargin < 2 || isempty(inputFile)
    inputFile = fullfile(root, 'Validation_Results', 'Initial_State', ...
        'Grid_Forming_PMSG_Init_candidate.mat');
end
if exist(inputFile, 'file') ~= 2
    error('Initial-state candidate not found: %s', inputFile);
end

diag = run_no_disturbance_diagnosis(false, extraSeconds, 0.0, struct( ...
    'VdcRef_V', 1200, ...
    'VacRef_V', 563, ...
    'Pref_W', 1e6, ...
    'Qref_var', 0, ...
    'SkipMexCompile', true, ...
    'UseNumericSfunParams', true, ...
    'InitialStateFile', inputFile, ...
    'InitialStateVar', 'xInitial', ...
    'SaveFinalState', true, ...
    'SaveOperatingPoint', true, ...
    'FinalStateName', 'xInitial', ...
    'SaveSeries', true, ...
    'SeriesMaxPoints', 60000, ...
    'ThreePhaseTail_s', 0.2));

if ~isfield(diag, 'final_state') || isempty(diag.final_state)
    error('Final operating point was not captured.');
end

outDir = fullfile(root, 'Validation_Results', 'Initial_State');
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
    fprintf('Candidate extended but not yet published: %s\n', candidateFile);
end

fprintf('snapshot_time        = %.3f s\n', xInitial.snapshotTime);
fprintf('baseline_operational = %d\n', diag.baseline_operational_flag);
fprintf('dc_settled           = %d\n', diag.dc_settled_flag);
fprintf('mech_settled         = %d\n', diag.mech_settled_flag);
fprintf('Ppcc_end_mean        = %.3f kW\n', diag.Ppcc_end_mean / 1e3);
fprintf('Udc_end_mean         = %.3f V\n', diag.udc_end_mean);
fprintf('Udc_end_slope        = %.6f V/s\n', diag.udc_end_slope);
end
