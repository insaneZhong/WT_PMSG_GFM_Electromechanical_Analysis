function run_after_unified_params()
% One-click runner after nonlinear/small-signal parameter unification.
% This script keeps the original workflow but makes execution order explicit.

root = fileparts(mfilename('fullpath'));
old = pwd;
cleanup = onCleanup(@() cd(old)); %#ok<NASGU>
cd(root);

fprintf('Step 1/4: refresh Parameters.mat from Parameters.m ...\n');
run('Parameters.m');

fprintf('Step 2/4: regenerate and compare four control topologies ...\n');
if exist('Compare_Control_Mode_Run.m','file') == 2
    run('Compare_Control_Mode_Run.m');
else
    warning('Compare_Control_Mode_Run.m not found, skip.');
end

fprintf('Step 3/4: run parameter sweep pipeline if available ...\n');
if exist('Scan_GFM_Control_Parameters_Run.m','file') == 2
    run('Scan_GFM_Control_Parameters_Run.m');
else
    warning('Scan_GFM_Control_Parameters_Run.m not found, skip.');
end

fprintf('Step 4/4: summarize dominant modes and damping if available ...\n');
if exist('analyze_eigen_results.m','file') == 2
    run('analyze_eigen_results.m');
end
if exist('report_eigen_samples.m','file') == 2
    run('report_eigen_samples.m');
end

fprintf('Done. Please check generated CSV/MAT figures for the ~2 Hz shaft mode tracking.\n');
end

