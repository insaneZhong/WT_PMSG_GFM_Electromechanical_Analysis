function run_single_point_steady_checks()
% Run two minimal no-disturbance steady checks:
% 1) legacy baseline
% 2) scheme-A enabled

root = fileparts(mfilename('fullpath'));
old = pwd;
cleanup = onCleanup(@() cd(old)); %#ok<NASGU>
cd(root);

fprintf('[1/2] single baseline check ...\n');
T0 = diagnose_1mw_steady_root_cause('single'); %#ok<NASGU>

fprintf('[2/2] single schemeA check ...\n');
T1 = diagnose_1mw_steady_root_cause('single_schemea'); %#ok<NASGU>

fprintf('Done. Check Validation_Results/*.csv\n');
end

