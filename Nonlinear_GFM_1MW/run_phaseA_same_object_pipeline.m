function run_phaseA_same_object_pipeline()
% Phase-A pipeline:
% 1) Refresh small-signal Parameters.mat
% 2) Build same-object parameter table
% 3) Export scheme-A C tuning map
% 4) Run no-disturbance scans (legacy and scheme-A optional)

root = fileparts(mfilename('fullpath'));
old = pwd;
cleanup = onCleanup(@() cd(old)); %#ok<NASGU>
cd(root);

fprintf('\n[1/5] Refresh small-signal Parameters.mat ...\n');
ssmMat = locate_ssm_parameters_mat();
ssmEigenDir = fileparts(ssmMat);
run(fullfile(ssmEigenDir, 'Parameters.m'));

fprintf('[2/5] Build same-object parameter table ...\n');
build_same_object_parameter_table();

fprintf('[3/5] Export scheme-A C tuning hints ...\n');
export_schemeA_c_tuning_from_small_signal();

fprintf('[4/5] Run no-disturbance baseline scan (legacy controller) ...\n');
assignin('base','use_schemeA_overrides',false);
scan_steady_nondisturbance_timing_powerloop();
scan_steady_nondisturbance_voltage_loop();

fprintf('[5/5] Run no-disturbance scan (scheme-A overrides) ...\n');
assignin('base','use_schemeA_overrides',true);
scan_steady_nondisturbance_timing_powerloop();
scan_steady_nondisturbance_voltage_loop();

fprintf('[extra] Run root-cause ranking diagnosis ...\n');
diagnose_1mw_steady_root_cause();

fprintf('\nDone. Check Validation_Results for CSV/MD outputs.\n');
end
