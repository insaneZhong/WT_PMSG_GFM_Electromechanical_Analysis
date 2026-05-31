function T = run_no_disturbance_manual_reliable(pref_list, simStop, vacRef, vdcRef, qref)
% Verified reliable no-disturbance sweep.
% This follows the manually validated sequence that correctly updates Pref
% between operating points.

if nargin < 1 || isempty(pref_list), pref_list = [8.5e5 1.0e6 1.2e6]; end
if nargin < 2 || isempty(simStop),   simStop = 3.0; end
if nargin < 3 || isempty(vacRef),    vacRef = 563; end
if nargin < 4 || isempty(vdcRef),    vdcRef = 5000; end
if nargin < 5 || isempty(qref),      qref = 0; end

root = fileparts(mfilename('fullpath'));
old = pwd; cleanup = onCleanup(@() cd(old)); cd(root);

res = zeros(numel(pref_list),9);
for k = 1:numel(pref_list)
    pref = pref_list(k);

    bdclose('all');
    clear mex;

    assignin('base','P_wt_rated_override',pref);
    assignin('base','wind_step_mps_override',0.0);
    assignin('base','sim_stop_time_override',simStop);
    run('GFM_MWT_Nonlinear_Params.m');

    mdl = 'Grid_Forming_PMSG';
    load_system(mdl);
    oldInit = get_param(mdl,'InitFcn');
    set_param(mdl,'InitFcn','');

    blk = [mdl '/MOTOR_CONTROL1'];
    sf = [blk '/S-Function1'];
    set_param(blk,'Pref',num2str(pref), ...
                 'Qref',num2str(qref), ...
                 'ReferenceACVoltage',num2str(vacRef), ...
                 'ReferenceDCVoltage',num2str(vdcRef));
    set_param(sf,'Parameters',sprintf('%.12g,0,%.12g,%.12g',pref,vacRef,vdcRef));
    set_param(mdl,'StopTime',num2str(simStop));
    set_param(mdl,'SimulationCommand','update');
    fprintf('k=%d cmd=%.0f Pref=%s sf=%s\n',k,pref,get_param(blk,'Pref'),get_param(sf,'Parameters'));

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

    t = pcc_uab.Time(:);
    uab = pcc_uab.Data(:); ubc = pcc_ubc.Data(:);
    ia = pcc_ia.Data(:); ib = pcc_ib.Data(:); ic = pcc_ic.Data(:);
    ua = (2*uab + ubc)/3;
    ub = (-uab + ubc)/3;
    uc = (-uab - 2*ubc)/3;
    ppcc = ua.*ia + ub.*ib + uc.*ic;

    iTail = t >= t(end)-1.0;
    iUdc = udc_meas.Time >= udc_meas.Time(end)-1.0;
    iW = omega_g.Time >= omega_g.Time(end)-1.0;
    iTw = theta_tw.Time >= theta_tw.Time(end)-1.0;
    iPm = pmeas_out.Time >= pmeas_out.Time(end)-1.0;
    iWr = wref_out.Time >= wref_out.Time(end)-1.0;

    pU = polyfit(udc_meas.Time(iUdc), udc_meas.Data(iUdc), 1);
    pW = polyfit(omega_g.Time(iW), omega_g.Data(iW), 1);
    pT = polyfit(theta_tw.Time(iTw), theta_tw.Data(iTw), 1);

    res(k,:) = [pref, pref_out.Data(end), mean(pmeas_out.Data(iPm)), mean(ppcc(iTail)), ...
                mean(udc_meas.Data(iUdc)), pU(1), mean(wref_out.Data(iWr)), pW(1), pT(1)];

    fprintf('k=%d pref_end=%.0f pmeas=%.0f ppcc=%.0f\n',k,res(k,2),res(k,3),res(k,4));
    set_param(mdl,'InitFcn',oldInit);
end

T = array2table(res,'VariableNames',{'Pref_cmd','Pref_out_end','Pmeas_mean','Ppcc_mean','Udc_mean','Udc_slope','Wref_mean','OmegaG_slope','ThetaTw_slope'});
outDir = fullfile(root,'Validation_Results');
if ~exist(outDir,'dir'), mkdir(outDir); end
writetable(T, fullfile(outDir,'no_disturbance_manual_reliable.csv'));
disp(T);
fprintf('Saved: %s\n', fullfile(outDir,'no_disturbance_manual_reliable.csv'));
end
