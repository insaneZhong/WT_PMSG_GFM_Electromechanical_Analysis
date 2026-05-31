close all
clear
clc

run("Parameters.m")
load("Unified_WT_PMSG_VSG_Damping.mat")
base_params = load("Parameters.mat");
mpopt = mpoption('verbose', 0, 'out.all', 0);

gain_range = [-5e6 -2e6 -1e6 -5e5 -2e5 -1e5 -5e4 -2e4 -1e4 -5e3 -2e3 -1e3 0 ...
               1e3 2e3 5e3 1e4 2e4 5e4 1e5 2e5 5e5 1e6 2e6 5e6];

fprintf("K_damp,max_real,torsion_sigma,torsion_omega,torsion_freq_hz,torsion_zeta\n");
for gain = gain_range
    params = base_params;
    params.K_damp = gain;
    A = make_A(Unified_GFMI, mpopt, params);
    [sigma, omega, freq_hz, zeta] = torsion_mode(A);
    fprintf("%.10g,%.10g,%.10g,%.10g,%.10g,%.10g\n", ...
        gain, max(real(eig(A))), sigma, omega, freq_hz, zeta);
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
    if isempty(candidates)
        candidates = poles(imag(poles) > 1e-6);
    end
    [~, idx] = max(real(candidates));
    lam = candidates(idx);
    sigma = real(lam);
    omega = imag(lam);
    freq_hz = abs(omega) / (2*pi);
    zeta = -sigma / abs(lam);
end
