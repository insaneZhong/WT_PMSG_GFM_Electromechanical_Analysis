function out = run_single_explicit_case(prefW, simStop, vacRef, vdcRef, qref)
% Run one explicit nonlinear no-disturbance case with deterministic params.

if nargin < 1 || isempty(prefW),  prefW = 1e6; end
if nargin < 2 || isempty(simStop), simStop = 4.0; end
if nargin < 3 || isempty(vacRef), vacRef = 563; end
if nargin < 4 || isempty(vdcRef), vdcRef = 5000; end
if nargin < 5 || isempty(qref), qref = 0; end

assignin('base','P_wt_rated_override',prefW);
run('GFM_MWT_Nonlinear_Params.m');
assignin('base','wind_step_mps_override',0.0);
assignin('base','sim_stop_time_override',simStop);
bdclose('all');
clear mex;

mdl = 'Grid_Forming_PMSG';
load_system(mdl);
ctrlBlk = [mdl '/MOTOR_CONTROL1'];
ctrlSfun = [mdl '/MOTOR_CONTROL1/S-Function1'];
origInitFcn = get_param(mdl,'InitFcn');
set_param(mdl,'InitFcn','');
initCleanup = onCleanup(@() set_param(mdl,'InitFcn',origInitFcn));

set_param(ctrlBlk,'Pref',num2str(prefW));
set_param(ctrlBlk,'Qref',num2str(qref));
set_param(ctrlBlk,'ReferenceACVoltage',num2str(vacRef));
set_param(ctrlBlk,'ReferenceDCVoltage',num2str(vdcRef));
set_param(ctrlSfun,'Parameters',sprintf('%.12g, %.12g, %.12g, %.12g',prefW,qref,vacRef,vdcRef));
out.pref_blk_before_update = str2double(get_param(ctrlBlk,'Pref'));
out.sfun_params_before_update = get_param(ctrlSfun,'Parameters');

set_param(mdl,'StopTime',num2str(simStop));
set_param(mdl,'SimulationCommand','update');
set_param(ctrlSfun,'Parameters',sprintf('%.12g, %.12g, %.12g, %.12g',prefW,qref,vacRef,vdcRef));
out.sfun_params_before_sim = get_param(ctrlSfun,'Parameters');
sim(mdl);
out.pref_blk_after_sim = str2double(get_param(ctrlBlk,'Pref'));
out.sfun_params_after_sim = get_param(ctrlSfun,'Parameters');

pref = evalin('base','pref_out');
pmeas = evalin('base','pmeas_out');
wref = evalin('base','wref_out');
udc = evalin('base','udc_meas');
omega_g = evalin('base','omega_g');
theta_tw = evalin('base','theta_tw');
pcc_ia = evalin('base','pcc_ia');
pcc_ib = evalin('base','pcc_ib');
pcc_ic = evalin('base','pcc_ic');
pcc_uab = evalin('base','pcc_uab');
pcc_ubc = evalin('base','pcc_ubc');

tail = 1.0;
[tp, ppcc] = localPccPowerFromLineLine(pcc_uab, pcc_ubc, pcc_ia, pcc_ib, pcc_ic);

out.pref_cmd = prefW;
out.pref_out_mean = localMean(pref, tail);
out.pmeas_mean = localMean(pmeas, tail);
out.ppcc_mean = localMeanRaw(tp, ppcc, tail);
out.udc_mean = localMean(udc, tail);
out.udc_slope = localSlope(udc, tail);
out.wref_mean = localMean(wref, tail);
out.omega_g_slope = localSlope(omega_g, tail);
out.theta_tw_slope = localSlope(theta_tw, tail);
clear initCleanup;
end

function s = localSlope(ts, tailWin)
t = ts.Time(:); y = ts.Data(:);
idx = t >= (t(end) - tailWin);
p = polyfit(t(idx), y(idx), 1);
s = p(1);
end

function m = localMean(ts, tailWin)
t = ts.Time(:); y = ts.Data(:);
idx = t >= (t(end) - tailWin);
m = mean(y(idx));
end

function [t, p] = localPccPowerFromLineLine(uab, ubc, ia, ib, ic)
t = uab.Time(:);
uabv = uab.Data(:); ubcv = ubc.Data(:);
iav = ia.Data(:); ibv = ib.Data(:); icv = ic.Data(:);
ua = (2*uabv + ubcv) / 3;
ub = (-uabv + ubcv) / 3;
uc = (-uabv - 2*ubcv) / 3;
p = ua .* iav + ub .* ibv + uc .* icv;
end

function m = localMeanRaw(t, y, tailWin)
idx = t >= (t(end) - tailWin);
m = mean(y(idx));
end
