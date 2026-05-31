function T = scan_steady_nondisturbance_voltage_loop()
% Scan effective voltage-loop bandwidth (V_LOOP_BANDWITH in motorcontrol.h)
% in no-disturbance condition around 1 MW.

root = fileparts(mfilename('fullpath'));
old = pwd; cleanup = onCleanup(@() cd(old)); cd(root);

origGH = fileread(fullfile(root,'grid_forming_control.h'));
origMH = fileread(fullfile(root,'motorcontrol.h'));
restoreObj1 = onCleanup(@() restore_file(fullfile(root,'grid_forming_control.h'), origGH)); %#ok<NASGU>
restoreObj2 = onCleanup(@() restore_file(fullfile(root,'motorcontrol.h'), origMH)); %#ok<NASGU>

% keep previously better power-loop baseline
kpP = 1e-6; kiP = 2e-5;
tSync = 0.5;

% effective voltage-loop tuning candidates (through V_LOOP_BANDWITH)
vLoopBwList = [30, 60, 90, 120];

pref = 1.0e6;
simStop = 6.0;
vacRef = 563;
vdcRef = 5000;
qref = 0;

rows = [];
for bw = vLoopBwList
        patch_header(tSync, kpP, kiP, bw);
        m = run_one_case(pref, simStop, vacRef, vdcRef, qref);
        rows = [rows; tSync, kpP, kiP, bw, m]; %#ok<AGROW>
end

T = array2table(rows, 'VariableNames', { ...
    'tSync','kpp','kip','vloop_bw', ...
    'Pref_end','Pmeas_mean','Ppcc_mean','Udc_mean','Udc_slope','Wref_mean','OmegaG_slope','ThetaTw_slope'});

score = abs(T.Ppcc_mean-1e6)/1e6 + abs(T.Udc_slope)/400 + abs(T.OmegaG_slope)*5 + abs(T.ThetaTw_slope)*500;
T.score = score;
T = sortrows(T,'score');

outDir = fullfile(root, 'Validation_Results');
if ~exist(outDir,'dir'), mkdir(outDir); end
writetable(T, fullfile(outDir, 'scan_steady_nondisturbance_voltage_loop.csv'));
disp(T);
fprintf('Saved: %s\n', fullfile(outDir,'scan_steady_nondisturbance_voltage_loop.csv'));
end

function patch_header(tSync, kpP, kiP, vLoopBw)
root = fileparts(mfilename('fullpath'));

% patch grid-side synchronization and power loop
pthG = fullfile(root,'grid_forming_control.h');
txtG = fileread(pthG);
txtG = regexprep(txtG, '#define\s+PRESYN_SWITCH_TIME\s+[0-9\.]+', ...
    sprintf('#define   PRESYN_SWITCH_TIME                    %.6g', tSync), 'once');
txtG = regexprep(txtG, ['#define\s+POWER_LOOP_PI_DEFAULTS\s+\{0\.0,\s*0\.0,\s*0\.0,\s*' ...
    '[^,]+,\s*[^,]+,\s*0\.0,\s*0,'], ...
    sprintf('#define POWER_LOOP_PI_DEFAULTS   {0.0, 0.0, 0.0, %.8g, %.8g, 0.0, 0,', kpP, kiP), 'once');
fid = fopen(pthG,'w'); fwrite(fid,txtG); fclose(fid);

% patch effective voltage loop source
pthM = fullfile(root,'motorcontrol.h');
txtM = fileread(pthM);
txtM = regexprep(txtM, '#define\s+V_LOOP_BANDWITH\s+[0-9\.eE+-]+', ...
    sprintf('#define   V_LOOP_BANDWITH            %.8g', vLoopBw), 'once');
fid = fopen(pthM,'w'); fwrite(fid,txtM); fclose(fid);
end

function m = run_one_case(pref, simStop, vacRef, vdcRef, qref)
setenv('MW_MINGW64_LOC','C:\mingw64');
evalin('base','clear mex');
evalin('base','bdclose(''all'')');
useSchemeA = false;
if evalin('base','exist(''use_schemeA_overrides'',''var'')')
    useSchemeA = logical(evalin('base','use_schemeA_overrides'));
end
if useSchemeA
    mexCmd = 'mex -DENABLE_SCHEMEA_OVERRIDES main.c svpwm.c motorcontrol.c grid_forming_control.c;';
else
    mexCmd = 'mex main.c svpwm.c motorcontrol.c grid_forming_control.c;';
end
evalin('base',sprintf('cd(''%s''); %s', strrep(fileparts(mfilename('fullpath')),'\','\\'), mexCmd));

assignin('base','P_wt_rated_override',pref);
assignin('base','wind_step_mps_override',0.0);
assignin('base','sim_stop_time_override',simStop);
run('GFM_MWT_Nonlinear_Params.m');

mdl = 'Grid_Forming_PMSG';
load_system(mdl);
origInit = get_param(mdl,'InitFcn');
set_param(mdl,'InitFcn','');
cleanupInit = onCleanup(@() set_param(mdl,'InitFcn',origInit));

blk=[mdl '/MOTOR_CONTROL1']; sf=[blk '/S-Function1'];
set_param(blk,'Pref',num2str(pref),'Qref',num2str(qref),'ReferenceACVoltage',num2str(vacRef),'ReferenceDCVoltage',num2str(vdcRef));
set_param(sf,'Parameters',sprintf('%.12g, %.12g, %.12g, %.12g',pref,qref,vacRef,vdcRef));
set_param(mdl,'StopTime',num2str(simStop));
set_param(mdl,'SimulationCommand','update');
sim(mdl);

pref_out = evalin('base','pref_out');
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

[tp, ppcc] = pcc_power(pcc_uab,pcc_ubc,pcc_ia,pcc_ib,pcc_ic);
tail = 1.0;
m = [pref_out.Data(end), ...
     mean_tail(pmeas,tail), mean_tail_raw(tp,ppcc,tail), mean_tail(udc,tail), slope_tail(udc,tail), ...
     mean_tail(wref,tail), slope_tail(omega_g,tail), slope_tail(theta_tw,tail)];

clear cleanupInit;
end

function s = slope_tail(ts,win)
t=ts.Time(:); y=ts.Data(:); idx=t>=t(end)-win; p=polyfit(t(idx),y(idx),1); s=p(1);
end
function m = mean_tail(ts,win)
t=ts.Time(:); y=ts.Data(:); idx=t>=t(end)-win; m=mean(y(idx));
end
function m = mean_tail_raw(t,y,win)
idx=t>=t(end)-win; m=mean(y(idx));
end
function [t,p] = pcc_power(uab,ubc,ia,ib,ic)
t=uab.Time(:); uabv=uab.Data(:); ubcv=ubc.Data(:); iav=ia.Data(:); ibv=ib.Data(:); icv=ic.Data(:);
ua=(2*uabv+ubcv)/3; ub=(-uabv+ubcv)/3; uc=(-uabv-2*ubcv)/3;
p=ua.*iav+ub.*ibv+uc.*icv;
end
function restore_file(path,txt)
fid=fopen(path,'w'); fwrite(fid,txt); fclose(fid);
end
