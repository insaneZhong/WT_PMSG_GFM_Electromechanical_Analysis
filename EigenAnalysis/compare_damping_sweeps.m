close all
clear
clc

run("Parameters.m")
old_model = load("Unified_WT_PMSG_VSG.mat");
new_model = load("Unified_WT_PMSG_VSG_Damping.mat");
base_params = load("Parameters.mat");
mpopt = mpoption('verbose', 0, 'out.all', 0);

cases = {
    "base", "", NaN;
    "beta_i low", "beta_i", 0;
    "beta_i high", "beta_i", 1;
    "h low", "h", 500;
    "h high", "h", 2000;
    "SCR weak", "SCR", 1.25;
    "SCR strong", "SCR", 25;
    "XR low", "XR", 1;
    "XR high", "XR", 20;
};

fprintf("case,old_sigma,old_freq_hz,old_zeta,new_sigma,new_freq_hz,new_zeta,zeta_ratio\n");
for k = 1:size(cases, 1)
    params = base_params;
    variable = cases{k, 2};
    value = cases{k, 3};
    if strlength(variable) > 0
        params.(variable) = value;
        if variable == "SCR" || variable == "XR"
            [params.rg, params.lg] = grid_impedance(params);
        end
    end

    old_A = make_A(old_model.Unified_GFMI, mpopt, params);
    new_A = make_A(new_model.Unified_GFMI, mpopt, params);
    [old_sigma, ~, old_freq, old_zeta] = torsion_mode(old_A);
    [new_sigma, ~, new_freq, new_zeta] = torsion_mode(new_A);
    fprintf("%s,%.10g,%.10g,%.10g,%.10g,%.10g,%.10g,%.10g\n", ...
        cases{k, 1}, old_sigma, old_freq, old_zeta, new_sigma, new_freq, new_zeta, new_zeta/old_zeta);
end

fprintf("\nSweep summaries for torsional mode\n");
summarize_sweep("beta_i", linspace(0, 1, 40), old_model.Unified_GFMI, new_model.Unified_GFMI, base_params, mpopt);
summarize_sweep("h", linspace(500, 2000, 40), old_model.Unified_GFMI, new_model.Unified_GFMI, base_params, mpopt);
summarize_sweep("SCR", logspace(log10(1.25), log10(25), 40), old_model.Unified_GFMI, new_model.Unified_GFMI, base_params, mpopt);
summarize_sweep("XR", linspace(1, 20, 40), old_model.Unified_GFMI, new_model.Unified_GFMI, base_params, mpopt);

function summarize_sweep(variable, values, old_model, new_model, base_params, mpopt)
    old_zeta = zeros(size(values));
    new_zeta = zeros(size(values));
    old_sigma = zeros(size(values));
    new_sigma = zeros(size(values));
    for i = 1:numel(values)
        params = base_params;
        params.(variable) = values(i);
        if variable == "SCR" || variable == "XR"
            [params.rg, params.lg] = grid_impedance(params);
        end
        old_A = make_A(old_model, mpopt, params);
        new_A = make_A(new_model, mpopt, params);
        [old_sigma(i), ~, ~, old_zeta(i)] = torsion_mode(old_A);
        [new_sigma(i), ~, ~, new_zeta(i)] = torsion_mode(new_A);
    end
    fprintf("%s: old_zeta[min,max]=[%.6g, %.6g], new_zeta[min,max]=[%.6g, %.6g], old_sigma[min,max]=[%.6g, %.6g], new_sigma[min,max]=[%.6g, %.6g]\n", ...
        variable, min(old_zeta), max(old_zeta), min(new_zeta), max(new_zeta), ...
        min(old_sigma), max(old_sigma), min(new_sigma), max(new_sigma));
end

function [rg, lg] = grid_impedance(params)
    rgpu = 1 / (params.SCR * sqrt(1 + params.XR^2));
    lgpu = params.XR * rgpu;
    rg = rgpu * params.Zb;
    lg = lgpu * params.Lb;
end

function A = make_A(model, mpopt, params)
    mpc = SMIB_PowerFlow(params.rg, params.lg);
    pf = runpf(mpc, mpopt);

    angle_rad = deg2rad(pf.bus(1, 9));
    voltage_mag = pf.bus(1, 8);
    vc_phasor = voltage_mag * params.V_LL / sqrt(3) * exp(1j * angle_rad);
    vg_phasor = params.V_LL / sqrt(3);
    zt = params.rf2 + params.rg + 1j * 2 * pi * 50 * (params.lf2 + params.lg);
    z2 = params.rf2 + 1j * 2 * pi * 50 * params.lf2;
    i2_phasor = (vc_phasor - vg_phasor) / zt;
    vpcc_phasor = (vc_phasor - i2_phasor * z2) * sqrt(2);

    params.delta0 = angle_rad;
    params.Vpcc_D0 = real(vpcc_phasor);
    params.Vpcc_Q0 = imag(vpcc_phasor);

    vc_dq0 = vc_phasor * exp(-1j * angle_rad) * sqrt(2);
    params.Vc_d0 = real(vc_dq0);
    params.Vc_q0 = imag(vc_dq0);

    i2_dq0 = i2_phasor * exp(-1j * angle_rad) * sqrt(2);
    params.i2_d0 = real(i2_dq0);
    params.i2_q0 = imag(i2_dq0);

    names = fieldnames(params);
    values = cell(size(names));
    for k = 1:numel(names)
        values{k} = params.(names{k});
    end
    A = double(subs(model.sym_A, names, values));
end

function [sigma, omega, freq_hz, zeta] = torsion_mode(A)
    poles = eig(A);
    candidates = poles(imag(poles) > 1e-6 & abs(abs(imag(poles)) - 2*pi*2) < 2*pi*1.0);
    [~, idx] = max(real(candidates));
    lam = candidates(idx);
    sigma = real(lam);
    omega = imag(lam);
    freq_hz = abs(omega) / (2*pi);
    zeta = -sigma / abs(lam);
end
