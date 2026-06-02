function diag = run_gfm_stability_check_10s()
% One-click GFM-only no-disturbance 10s stability check.

root = fileparts(mfilename('fullpath'));
old = pwd; cleanup = onCleanup(@() cd(old)); cd(root); %#ok<NASGU>

setenv('MW_MINGW64_LOC','C:\mingw64');
clear mex; bdclose('all');
mex main.c svpwm.c motorcontrol.c grid_forming_control.c;

diag = run_no_disturbance_diagnosis(false, 10.0, 0.0, struct( ...
    'VdcRef_V', 1200, ... % 690 V line voltage system: keep DC-link voltage above the modulation margin.
    'VacRef_V', 563, ...
    'Pref_W', 1e6, ...
    'Qref_var', 0, ...
    'SkipMexCompile', true, ...
    'UseNumericSfunParams', true));

fprintf('\n=== GFM 10s 无扰动诊断 ===\n');
fprintf('Ppcc_end_mean      = %.3f kW\n', diag.Ppcc_end_mean/1e3);
fprintf('Ppcc_target_error  = %.4f pu\n', diag.Ppcc_target_error_pu);
fprintf('Udc_end_mean       = %.2f V\n', diag.udc_end_mean);
fprintf('Udc_end_slope      = %.3f V/s\n', diag.udc_end_slope);
fprintf('omega_g_end_slope  = %.6f\n', diag.omega_g_end_slope);
fprintf('omega_t_end_slope  = %.6f\n', diag.omega_t_end_slope);
fprintf('theta_tw_end_slope = %.6f\n', diag.theta_tw_end_slope);
fprintf('T_sh_end_slope     = %.3e\n', diag.T_sh_end_slope);
fprintf('cause_hint         = %s\n', string(diag.primary_cause_hint));
fprintf('baseline_operational = %d\n', diag.baseline_operational_flag);

ok_mech = abs(diag.omega_g_end_slope) < 1e-2 && ...
          abs(diag.omega_t_end_slope) < 1e-2 && ...
          abs(diag.theta_tw_end_slope) < 5e-4;
ok_dc = abs(diag.udc_end_slope) < 5;
ok_pow = abs(diag.Ppcc_target_error_pu) < 0.05;
fprintf('mech_settled=%d, dc_settled=%d, power_reached=%d\n', ok_mech, ok_dc, ok_pow);
end
