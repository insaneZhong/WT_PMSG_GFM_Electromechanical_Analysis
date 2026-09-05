function [OP,P] = solve_m0_equilibrium(P,varargin)
%SOLVE_M0_EQUILIBRIUM 求5 MW、Q=0的严格M0平衡点。
%  先在PCC功率面求解网侧电流，再由LCL稳态方程还原变流器端功率，
%  最后求解PMSG发电电流，使两个理想VSC的DC功率严格相等。

if nargin<1 || isempty(P)
    P = init_m0_5mw_parameters();
end
ip = inputParser;
ip.addParameter('Display','off',@(x)ischar(x)||isstring(x));
ip.parse(varargin{:});

w = P.omega0_radps;
Vg = P.Vphase_peak_V;
target = [P.Pref_W; P.Qref_var];
i0 = [P.Pref_W/(1.5*Vg); 0];
opts = optimoptions('fsolve','Display',char(ip.Results.Display), ...
    'FunctionTolerance',1e-12,'StepTolerance',1e-12, ...
    'OptimalityTolerance',1e-12,'MaxIterations',500);
[ig,~,exitflag] = fsolve(@gridResidual,i0,opts);
assert(exitflag>0,'M0网侧平衡点求解失败，exitflag=%d。',exitflag);

vpcc = gridPccVoltage(ig);
a = P.Rd_ohm*w*P.Cf_F;
vcf = [1,-a; a,1]\vpcc;
icap = [-w*P.Cf_F*vcf(2); w*P.Cf_F*vcf(1)];
ifdq = ig+icap;
uinv = [ ...
    vpcc(1)+P.Rf_ohm*ifdq(1)-w*P.Lf_H*ifdq(2); ...
    vpcc(2)+P.Rf_ohm*ifdq(2)+w*P.Lf_H*ifdq(1)];
Pgsc = 1.5*dot(uinv,ifdq);

we = P.pole_pairs*P.omega_m0_radps;
powerMachine = @(iq)1.5*(we*P.psi_f_Wb*iq-P.Rs_ohm*iq.^2);
iqGuess = Pgsc/(1.5*we*P.psi_f_Wb);
iq = fzero(@(z)powerMachine(z)-Pgsc, ...
    [max(1,0.25*iqGuess),4*iqGuess]);
im = [0;iq];
vmsc = [we*P.Lq_H*iq; P.Rs_ohm*iq-we*P.psi_f_Wb];
Pmsc = -1.5*dot(vmsc,im);
Tgen = P.Kt_Nm_per_A*iq;

delta = atan2(vpcc(2),vpcc(1));
c = cos(delta); s = sin(delta);
Rminus = [c,s;-s,c];
ifLocal = Rminus*ifdq;
igLocal = Rminus*ig;
uLocal = Rminus*uinv;
vLocal = Rminus*vpcc;

x0 = zeros(23,1);
x0(1) = Tgen/P.Ksh_Nm_per_rad;
x0(2:3) = P.omega_m0_radps;
x0(4:5) = im;
x0(6) = iq;                         % Type-A DVC积分输出
x0(7:8) = 0;                        % MSC解耦前馈已闭合稳态
x0(9) = P.Vdc_ref_V;
x0(10) = target(1);
x0(11) = target(2);
x0(12) = w;
x0(13) = delta;
x0(14) = ifLocal(1)-P.gsc_grid_current_feedforward*igLocal(1);
x0(15) = ifLocal(2)-P.Cf_F*w*vLocal(1)- ...
    P.gsc_grid_current_feedforward*igLocal(2);
x0(16) = uLocal(1)+w*P.Lf_H*ifLocal(2)- ...
    P.gsc_pcc_voltage_feedforward*(vLocal(1)+P.Rf_ohm*ifLocal(1));
x0(17) = uLocal(2)-w*P.Lf_H*ifLocal(1)- ...
    P.gsc_pcc_voltage_feedforward*(vLocal(2)+P.Rf_ohm*ifLocal(2));
x0(18:19) = ifdq;
x0(20:21) = vcf;
x0(22:23) = ig;

OP = struct();
OP.x0 = x0;
OP.state_names = m0_state_names();
OP.Tm0_Nm = Tgen;
OP.E0_peak_V = norm(vpcc);
OP.P_MSC_dc_W = Pmsc;
OP.P_GSC_dc_W = Pgsc;
OP.P_PCC_W = 1.5*dot(vpcc,ig);
OP.Q_PCC_var = 1.5*(vpcc(2)*ig(1)-vpcc(1)*ig(2));
OP.vpcc_dq_V = vpcc;
OP.vcf_dq_V = vcf;
OP.if_dq_A = ifdq;
OP.ig_dq_A = ig;
OP.uinv_dq_V = uinv;
OP.vmsc_dq_V = vmsc;
OP.iq_gen_A = iq;
OP.Tgen_Nm = Tgen;
OP.theta_sh_rad = x0(1);
OP.delta_v_rad = delta;
OP.grid_solver_exitflag = exitflag;

P.Tm0_Nm = OP.Tm0_Nm;
P.E0_peak_V = OP.E0_peak_V;
[pvec,pnames] = m0_pack_parameters(P,OP);
u0 = zeros(6,1);
[dx0,y0] = m0_nonlinear_dynamics(x0,u0,pvec);
scale = [1; P.omega_m0_radps; P.omega_m0_radps; ...
    P.Iphase_peak_base_A; P.Iphase_peak_base_A; ...
    P.Iphase_peak_base_A; P.Vphase_peak_V; P.Vphase_peak_V; ...
    P.Vdc_ref_V; P.Sbase_W; P.Sbase_W; P.omega0_radps; 1; ...
    P.Iphase_peak_base_A; P.Iphase_peak_base_A; ...
    P.Vphase_peak_V; P.Vphase_peak_V; ...
    P.Iphase_peak_base_A; P.Iphase_peak_base_A; ...
    P.Vphase_peak_V; P.Vphase_peak_V; ...
    P.Iphase_peak_base_A; P.Iphase_peak_base_A];
OP.max_normalized_residual = max(abs(dx0)./scale);
OP.residual_norm = norm(dx0,2);
OP.dc_power_mismatch_W = Pmsc-Pgsc;
OP.torque_mismatch_Nm = ...
    P.Ksh_Nm_per_rad*x0(1)-Tgen;
OP.dx0 = dx0;
OP.y0 = y0;
OP.parameter_names = pnames;
OP.pvec = pvec;
assert(OP.max_normalized_residual<1e-8, ...
    'M0平衡点残差过大：%.3e。',OP.max_normalized_residual);
assert(abs(OP.dc_power_mismatch_W)<1e-5*P.Sbase_W, ...
    'M0直流功率不平衡：%.6g W。',OP.dc_power_mismatch_W);

    function r = gridResidual(i)
        v = gridPccVoltage(i);
        p = 1.5*dot(v,i);
        q = 1.5*(v(2)*i(1)-v(1)*i(2));
        r = ([p;q]-target)/P.Sbase_W;
    end

    function v = gridPccVoltage(i)
        v = [ ...
            Vg+P.Rg_ohm*i(1)-w*P.Lg_H*i(2); ...
            P.Rg_ohm*i(2)+w*P.Lg_H*i(1)];
    end
end
