function OP = solve_currentmodel_source_aligned_equilibrium()
%SOLVE_CURRENTMODEL_SOURCE_ALIGNED_EQUILIBRIUM
% Solve the one 5 MW operating point used by the retained physical RC-LCL
% topology and the explicit continuous controller.  The physical PCC
% sensors measure the RC-branch node voltage; the capacitor voltage behind
% the damping resistor remains a distinct energy state.

here=fileparts(mfilename('fullpath'));
root=fileparts(here);
addpath(root);
P=init_m0_5mw_parameters();
w=P.omega0_radps;
Vg=P.Vphase_peak_V;
a=P.Rd_ohm*w*P.Cf_F;
opts=optimoptions('fsolve','Display','off','FunctionTolerance',1e-12, ...
    'StepTolerance',1e-12,'OptimalityTolerance',1e-12,'MaxIterations',500);
ig0=[P.Pref_W/(1.5*Vg);0];
[ig,~,exitflag]=fsolve(@gridResidual,ig0,opts);
assert(exitflag>0,'Source-aligned grid equilibrium solve failed.');

% RC damping branch: vNode=vCap+Rd*iCap.  The physical controller receives
% vCap from the retained PCC voltage sensors; the two inductors see vNode.
vNode=gridNodeVoltage(ig);
vCap=[1,-a;a,1]\vNode;
iCap=[-w*P.Cf_F*vCap(2);w*P.Cf_F*vCap(1)];
iF=ig+iCap;
uInv=[vNode(1)+P.Rf_ohm*iF(1)-w*P.Lf_H*iF(2); ...
      vNode(2)+P.Rf_ohm*iF(2)+w*P.Lf_H*iF(1)];
Pgsc=1.5*dot(uInv,iF);

% The ideal MSC command is expressed in the retained generator-side VSC
% voltage orientation. The retained PMSM passive dq equations give
% vq=Rs*iq+we*psi at id=0; therefore the *measured VSC AC-port power* is
% electromagnetic power plus stator copper loss in this retained current
% coordinate. This is the quantity that enters the explicit DC-link energy
% equation and was independently confirmed by the 1 us port-power probe.
we=P.pole_pairs*P.omega_m0_radps;
powerMsc=@(iq)1.5*(we*P.psi_f_Wb*iq+P.Rs_ohm*iq.^2);
iqGuess=Pgsc/(1.5*we*P.psi_f_Wb);
iq=fzero(@(z)powerMsc(z)-Pgsc,[max(1,0.25*iqGuess),4*iqGuess]);
Pmsc=powerMsc(iq);
Tgen=P.Kt_Nm_per_A*iq;

% VSG local frame is aligned with the actual physical PCC/node sensor.
vPcc=vNode;
delta=atan2(vPcc(2),vPcc(1));
c=cos(delta); s=sin(delta); Rminus=[c,s;-s,c];
vLocal=Rminus*vPcc;
iFLocal=Rminus*iF;
igLocal=Rminus*ig;
uLocal=Rminus*uInv;
Pmeas=1.5*dot(vPcc,ig);
Qmeas=1.5*(vPcc(2)*ig(1)-vPcc(1)*ig(2));

% Explicit continuous-controller states, ordered exactly as
% currentmodel_continuous_controller_io.m expects.
ctrl=zeros(11,1);
ctrl(1)=iq; % DVC integrator: iq_ref=xi_dc at Udc=Udc_ref.
ctrl(2:3)=0;
ctrl(4)=Pmeas; ctrl(5)=Qmeas;
ctrl(6)=w; ctrl(7)=delta;
ctrl(8)=iFLocal(1)-P.gsc_grid_current_feedforward*igLocal(1);
ctrl(9)=iFLocal(2)-P.Cf_F*w*vLocal(1)- ...
    P.gsc_grid_current_feedforward*igLocal(2);
ctrl(10)=uLocal(1)+w*P.Lf_H*iFLocal(2)- ...
    P.gsc_pcc_voltage_feedforward*(vLocal(1)+P.Rf_ohm*iFLocal(1));
ctrl(11)=uLocal(2)-w*P.Lf_H*iFLocal(1)- ...
    P.gsc_pcc_voltage_feedforward*(vLocal(2)+P.Rf_ohm*iFLocal(2));

% Repack the authoritative continuous-control parameter vector with the
% physical measurement-plane voltage reference and mechanical balance.
P.Tm0_Nm=Tgen;
P.E0_peak_V=norm(vPcc);
OP=struct();
OP.controller_x0=ctrl;
OP.pmsg_id0=0;
OP.pmsg_iq0=iq;
OP.theta_tw0=Tgen/P.Ksh_Nm_per_rad;
OP.omega0=P.omega_m0_radps;
OP.Tgen_Nm=Tgen;
OP.Taero_Nm=Tgen;
OP.P_msc_W=Pmsc;
OP.P_gsc_W=Pgsc;
OP.P_pcc_measurement_W=Pmeas;
OP.Q_pcc_measurement_var=Qmeas;
OP.delta_vsg_rad=delta;
OP.vcap_grid_dq_V=vCap;
OP.vnode_grid_dq_V=vNode;
OP.if_grid_dq_A=iF;
OP.ig_grid_dq_A=ig;
OP.uinv_grid_dq_V=uInv;
% Retained PMSM1 dq coordinate: vd=-we*Lq*iq at id=0 and
% vq=Rs*iq+we*psi in the generating equilibrium.
OP.vmsc_generator_dq_V=[-we*P.Lq_H*iq; P.Rs_ohm*iq+we*P.psi_f_Wb];
OP.grid_solver_exitflag=exitflag;
[OP.pvec,OP.parameter_names]=m0_pack_parameters(P,struct('Tm0_Nm',Tgen,'E0_peak_V',norm(vPcc)));
OP.energy_residual_W=Pmsc-Pgsc;
OP.torque_residual_Nm=P.Ksh_Nm_per_rad*OP.theta_tw0-Tgen;

    function r=gridResidual(i)
        vn=gridNodeVoltage(i);
        p=1.5*dot(vn,i);
        q=1.5*(vn(2)*i(1)-vn(1)*i(2));
        r=([p;q]-[P.Pref_W;P.Qref_var])/P.Sbase_W;
    end
    function vn=gridNodeVoltage(i)
        vn=[Vg+P.Rg_ohm*i(1)-w*P.Lg_H*i(2); ...
            P.Rg_ohm*i(2)+w*P.Lg_H*i(1)];
    end
end
