close all
clear
clc

run("Parameters.m")
load("Unified_WT_PMSG_VSG.mat")
GFMI_1 = Unified_GFMI;
base_params = load("Parameters.mat");

mpopt = mpoption('verbose', 0, 'out.all', 0);

fprintf("CASE,param,max_real,osc_sigma,osc_omega,osc_freq_hz,osc_zeta,lf_sigma,lf_omega,lf_freq_hz,lf_zeta\n");

beta_range = linspace(0, 1, 40);
for value = beta_range
    params = base_params;
    params.beta_i = value;
    A = grid_operating_point_A(GFMI_1, mpopt, params);
    print_row("beta_i", value, A);
end

h_range = linspace(500, 2000, 40);
for value = h_range
    params = base_params;
    params.h = value;
    A = grid_operating_point_A(GFMI_1, mpopt, params);
    print_row("h", value, A);
end

scr_range = logspace(log10(1.25), log10(25), 40);
for value = scr_range
    params = base_params;
    params.SCR = value;
    [params.rg, params.lg] = grid_impedance(params);
    A = grid_operating_point_A(GFMI_1, mpopt, params);
    print_row("SCR", value, A);
end

xr_range = linspace(1, 20, 40);
for value = xr_range
    params = base_params;
    params.XR = value;
    [params.rg, params.lg] = grid_impedance(params);
    A = grid_operating_point_A(GFMI_1, mpopt, params);
    print_row("XR", value, A);
end

function [rg, lg] = grid_impedance(params)
    rgpu = 1 / (params.SCR * sqrt(1 + params.XR^2));
    lgpu = params.XR * rgpu;
    rg = rgpu * params.Zb;
    lg = lgpu * params.Lb;
end

function A = grid_operating_point_A(GFMI_1, mpopt, params)
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

    A = double(substitute_params(GFMI_1.sym_A, params));
end

function expr = substitute_params(expr, params)
    names = fieldnames(params);
    values = cell(size(names));
    for idx = 1:numel(names)
        values{idx} = params.(names{idx});
    end
    expr = subs(expr, names, values);
end

function print_row(case_name, value, A)
    poles = eig(A);
    max_real = max(real(poles));

    osc_poles = poles(imag(poles) > 1e-6);
    if isempty(osc_poles)
        osc = [NaN, NaN, NaN, NaN];
    else
        [~, idx] = max(real(osc_poles));
        lam = osc_poles(idx);
        osc = mode_metrics(lam);
    end

    low_freq_poles = poles(imag(poles) > 1e-6 & abs(imag(poles)) < 20);
    if isempty(low_freq_poles)
        low_freq = [NaN, NaN, NaN, NaN];
    else
        [~, idx] = max(real(low_freq_poles));
        lam = low_freq_poles(idx);
        low_freq = mode_metrics(lam);
    end

    fprintf("%s,%.10g,%.10g,%.10g,%.10g,%.10g,%.10g,%.10g,%.10g,%.10g,%.10g\n", ...
        case_name, value, max_real, osc, low_freq);
end

function metrics = mode_metrics(lam)
    sigma = real(lam);
    omega = imag(lam);
    freq_hz = abs(omega) / (2 * pi);
    zeta = -sigma / abs(lam);
    metrics = [sigma, omega, freq_hz, zeta];
end
