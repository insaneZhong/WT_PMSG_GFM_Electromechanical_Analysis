function diag = run_no_disturbance_case_explicit(prefW, qrefVar, vacRefV, vdcRefV, simStop)
% Explicit no-disturbance run with numeric S-Function parameters.
% This bypasses mask/workspace ambiguity and is used for steady-state debugging.

if nargin < 1, prefW = 1e6; end
if nargin < 2, qrefVar = 0; end
if nargin < 3, vacRefV = 563; end
if nargin < 4, vdcRefV = 5000; end
if nargin < 5, simStop = 5.0; end

root = fileparts(mfilename('fullpath'));
old = pwd;
cleanup = onCleanup(@() cd(old));
cd(root);

run('GFM_MWT_Nonlinear_Params.m');
assignin('base', 'wind_step_mps_override', 0.0);
assignin('base', 'sim_stop_time_override', simStop);

mdl = 'Grid_Forming_PMSG';
load_system(mdl);
ctrlBlk = [mdl '/MOTOR_CONTROL1'];

origPref = get_param(ctrlBlk, 'Pref');
origQref = get_param(ctrlBlk, 'Qref');
origVac = get_param(ctrlBlk, 'ReferenceACVoltage');
origVdc = get_param(ctrlBlk, 'ReferenceDCVoltage');

set_param(ctrlBlk, 'Pref', num2str(prefW));
set_param(ctrlBlk, 'Qref', num2str(qrefVar));
set_param(ctrlBlk, 'ReferenceACVoltage', num2str(vacRefV));
set_param(ctrlBlk, 'ReferenceDCVoltage', num2str(vdcRefV));

set_param(mdl, 'StopTime', num2str(simStop));
set_param(mdl, 'SimulationCommand', 'update');
sim(mdl);

set_param(ctrlBlk, 'Pref', origPref);
set_param(ctrlBlk, 'Qref', origQref);
set_param(ctrlBlk, 'ReferenceACVoltage', origVac);
set_param(ctrlBlk, 'ReferenceDCVoltage', origVdc);

pref_out = evalin('base','pref_out');
pmeas_out = evalin('base','pmeas_out');
wref_out = evalin('base','wref_out');
udc_meas = evalin('base','udc_meas');
presyn_out = evalin('base','presyn_out');
pcc_ia = evalin('base','pcc_ia');
pcc_ib = evalin('base','pcc_ib');
pcc_ic = evalin('base','pcc_ic');
pcc_uab = evalin('base','pcc_uab');
pcc_ubc = evalin('base','pcc_ubc');

tail = 1.0;
diag.pref_end_mean = localMean(pref_out, tail);
diag.pmeas_end_mean = localMean(pmeas_out, tail);
diag.wref_end_mean = localMean(wref_out, tail);
diag.udc_end_mean = localMean(udc_meas, tail);
diag.udc_end_slope = localSlope(udc_meas, tail);
diag.presyn_end_mean = localMean(presyn_out, tail);
[tp, ppcc] = localPccPowerFromLineLine(pcc_uab, pcc_ubc, pcc_ia, pcc_ib, pcc_ic);
diag.Ppcc_end_mean = localMeanRaw(tp, ppcc, tail);
diag.Ppcc_end_slope = localSlopeRaw(tp, ppcc, tail);

disp(diag);
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

function s = localSlopeRaw(t, y, tailWin)
idx = t >= (t(end) - tailWin);
p = polyfit(t(idx), y(idx), 1);
s = p(1);
end
