close all
clear
clc

run("Parameters.m")
load("Unified_WT_PMSG_VSG.mat")
params = load("Parameters.mat");
mpopt = mpoption('verbose', 0, 'out.all', 0);

cases = {
    "base", params;
    "beta_i = 0", set_param(params, "beta_i", 0);
    "beta_i = 1", set_param(params, "beta_i", 1);
    "h = 500", set_param(params, "h", 500);
    "h = 2000", set_param(params, "h", 2000);
    "SCR = 1.25", set_grid_param(params, "SCR", 1.25);
    "SCR = 25", set_grid_param(params, "SCR", 25);
    "X/R = 1", set_grid_param(params, "XR", 1);
    "X/R = 20", set_grid_param(params, "XR", 20);
};

for idx = 1:size(cases, 1)
    A = make_A(Unified_GFMI, mpopt, cases{idx, 2});
    fprintf("\n--- %s ---\n", cases{idx, 1});
    list_poles(A);
end

function params = set_param(params, name, value)
    params.(name) = value;
end

function params = set_grid_param(params, name, value)
    params.(name) = value;
    rgpu = 1 / (params.SCR * sqrt(1 + params.XR^2));
    lgpu = params.XR * rgpu;
    params.rg = rgpu * params.Zb;
    params.lg = lgpu * params.Lb;
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

function list_poles(A)
    poles = eig(A);
    [~, order] = sort(real(poles), 'descend');
    poles = poles(order);
    for k = 1:min(10, numel(poles))
        lam = poles(k);
        if abs(imag(lam)) > 1e-8
            freq_hz = abs(imag(lam)) / (2 * pi);
            zeta = -real(lam) / abs(lam);
            fprintf("%2d: %+11.6g %+11.6gj, f = %8.4g Hz, zeta = %9.5g\n", ...
                k, real(lam), imag(lam), freq_hz, zeta);
        else
            fprintf("%2d: %+11.6g %+11.6gj, real mode\n", ...
                k, real(lam), imag(lam));
        end
    end
end
