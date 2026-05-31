function T = run_no_disturbance_sweep_reliable(pref_list, simStop, vacRef, vdcRef, qref)
% Reliable no-disturbance sweep:
% - one operating point per fresh model session
% - keep model InitFcn enabled so Simulink timing/powergui variables exist
% - explicitly write Pref/Qref/Vac/Vdc before update+sim

if nargin < 1 || isempty(pref_list), pref_list = [8.5e5 1.0e6 1.2e6]; end
if nargin < 2 || isempty(simStop),   simStop = 3.0; end
if nargin < 3 || isempty(vacRef),    vacRef = 563; end
if nargin < 4 || isempty(vdcRef),    vdcRef = 5000; end
if nargin < 5 || isempty(qref),      qref = 0; end

root = fileparts(mfilename('fullpath'));
old = pwd; cleanup = onCleanup(@() cd(old)); cd(root);

n = numel(pref_list);
res = zeros(n, 9);

for k = 1:n
    pref = pref_list(k);

    bdclose('all');
    clear mex;

    assignin('base','P_wt_rated_override',pref);
    assignin('base','wind_step_mps_override',0.0);
    assignin('base','sim_stop_time_override',simStop);
    run('GFM_MWT_Nonlinear_Params.m');

    mdl = 'Grid_Forming_PMSG';
    load_system(mdl);
    ctrlBlk = [mdl '/MOTOR_CONTROL1'];
    ctrlSfun = [mdl '/MOTOR_CONTROL1/S-Function1'];
    set_param(ctrlBlk,'Pref',num2str(pref));
    set_param(ctrlBlk,'Qref',num2str(qref));
    set_param(ctrlBlk,'ReferenceACVoltage',num2str(vacRef));
    set_param(ctrlBlk,'ReferenceDCVoltage',num2str(vdcRef));
    set_param(ctrlSfun,'Parameters',sprintf('%.12g, %.12g, %.12g, %.12g',pref,qref,vacRef,vdcRef));

    set_param(mdl,'StopTime',num2str(simStop));
    set_param(mdl,'SimulationCommand','update');
    fprintf('[k=%d] cmd=%.0f, blkPref=%s, sf=%s\n', k, pref, get_param(ctrlBlk,'Pref'), get_param(ctrlSfun,'Parameters'));
    sim(mdl);

    pref_out = evalin('base','pref_out');
    pmeas_out = evalin('base','pmeas_out');
    wref_out = evalin('base','wref_out');
    udc_meas = evalin('base','udc_meas');
    omega_g = evalin('base','omega_g');
    theta_tw = evalin('base','theta_tw');
    pcc_ia = evalin('base','pcc_ia');
    pcc_ib = evalin('base','pcc_ib');
    pcc_ic = evalin('base','pcc_ic');
    pcc_uab = evalin('base','pcc_uab');
    pcc_ubc = evalin('base','pcc_ubc');

    tail = min(1.0, simStop/2);
    [tp, ppcc] = localPccPowerFromLineLine(pcc_uab, pcc_ubc, pcc_ia, pcc_ib, pcc_ic);

    res(k,1) = pref;
    res(k,2) = pref_out.Data(end);
    res(k,3) = localMean(pmeas_out, tail);
    res(k,4) = localMeanRaw(tp, ppcc, tail);
    res(k,5) = localMean(udc_meas, tail);
    res(k,6) = localSlope(udc_meas, tail);
    res(k,7) = localMean(wref_out, tail);
    res(k,8) = localSlope(omega_g, tail);
    res(k,9) = localSlope(theta_tw, tail);
    fprintf('[k=%d] pref_end=%.0f, pmeas_mean=%.0f, ppcc_mean=%.0f\n', k, res(k,2), res(k,3), res(k,4));

end

T = array2table(res, 'VariableNames', { ...
    'Pref_cmd', 'Pref_out_mean', 'Pmeas_mean', 'Ppcc_mean', ...
    'Udc_mean', 'Udc_slope', 'Wref_mean', 'OmegaG_slope', 'ThetaTw_slope'});

outDir = fullfile(root, 'Validation_Results');
if ~exist(outDir,'dir'), mkdir(outDir); end
writetable(T, fullfile(outDir, 'no_disturbance_sweep_reliable.csv'));

disp(T);
fprintf('Saved: %s\n', fullfile(outDir,'no_disturbance_sweep_reliable.csv'));
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
