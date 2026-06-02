function metrics = check_mechanical_equilibrium()
% Validate the no-disturbance equilibrium of the standalone two-mass shaft.
% Generation convention: T_e is negative and balances positive shaft torque.

root = fileparts(mfilename('fullpath'));
old = pwd;
cleanupObj = onCleanup(@() cd(old)); %#ok<NASGU>
cd(root);
run('GFM_MWT_Nonlinear_Params.m');

x0 = [omega_m0; omega_m0; theta_tw0];
teConst = -T_e0;
rhs = @(~, x) localRhs(x, teConst, omega_m0, T_aero0, D_aero, ...
    K_sh, D_sh, D_t, D_g, J_t, J_g);
[t, x] = ode45(rhs, [0 5], x0);

dx0 = rhs(0, x0);
theta = x(:, 3);
tsh = K_sh * theta + D_sh * (x(:, 1) - x(:, 2));

metrics = struct();
metrics.omega_m0_radps = omega_m0;
metrics.T_aero0_Nm = T_aero0;
metrics.T_e0_generation_Nm = teConst;
metrics.theta_tw0_rad = theta_tw0;
metrics.turbine_initial_accel_radps2 = dx0(1);
metrics.generator_initial_accel_radps2 = dx0(2);
metrics.theta_initial_rate_radps = dx0(3);
metrics.max_abs_omega_t_drift_radps = max(abs(x(:, 1) - omega_m0));
metrics.max_abs_omega_g_drift_radps = max(abs(x(:, 2) - omega_m0));
metrics.max_abs_theta_tw_drift_rad = max(abs(theta - theta_tw0));
metrics.max_abs_T_sh_drift_Nm = max(abs(tsh - T_aero0));
metrics.equilibrium_pass = all(abs(dx0) < 1e-10) && ...
    metrics.max_abs_omega_t_drift_radps < 1e-9 && ...
    metrics.max_abs_omega_g_drift_radps < 1e-9;

outDir = fullfile(root, 'Validation_Results');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
writetable(struct2table(metrics), fullfile(outDir, 'mechanical_equilibrium_check.csv'));
save(fullfile(outDir, 'mechanical_equilibrium_check.mat'), 'metrics', 't', 'x', 'tsh');
disp(metrics);
end

function dx = localRhs(x, te, omega0, taero0, daero, ksh, dsh, dt, dg, jt, jg)
omegaT = x(1);
omegaG = x(2);
theta = x(3);
taero = taero0 - daero * (omegaT - omega0);
tsh = ksh * theta + dsh * (omegaT - omegaG);
dx = [ ...
    (taero - tsh - dt * (omegaT - omega0)) / jt; ...
    (tsh + te - dg * (omegaG - omega0)) / jg; ...
    omegaT - omegaG];
end
