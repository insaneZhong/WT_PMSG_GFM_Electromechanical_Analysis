function T = scan_delivery_limits_no_disturbance()
% Focused scan for delivery bottleneck: power-loop output limit and w_ref limit.

root = fileparts(mfilename('fullpath'));
old = pwd; cleanup = onCleanup(@() cd(old)); cd(root);

origH = fileread(fullfile(root,'grid_forming_control.h'));
restoreObj = onCleanup(@() restore_file(fullfile(root,'grid_forming_control.h'),origH)); %#ok<NASGU>

pref = 1.0e6; qref = 0; vacRef = 563; vdcRef = 5000; simStop = 3.0;
ploopMaxList = [5, 10, 20];
wrefMaxList = [450, 500, 550];

rows = [];
compiled = false;
for pmax = ploopMaxList
    for wmax = wrefMaxList
        patch_header(pmax, wmax);
        m = run_one_case(pref, simStop, vacRef, vdcRef, qref, ~compiled);
        compiled = true;
        rows = [rows; pmax, wmax, m]; %#ok<AGROW>
    end
end

T = array2table(rows, 'VariableNames', { ...
    'ploop_out_max','wref_max','Pmeas_mean','Udc_slope','Wref_mean','OmegaG_slope','ThetaTw_slope'});
T.score = abs(T.Pmeas_mean-pref)/pref + abs(T.Udc_slope)/400 + abs(T.OmegaG_slope)*6 + abs(T.ThetaTw_slope)*700;
T = sortrows(T,'score');

outDir = fullfile(root,'Validation_Results');
if ~exist(outDir,'dir'), mkdir(outDir); end
writetable(T, fullfile(outDir,'scan_delivery_limits_no_disturbance.csv'));
disp(T);
fprintf('Saved: %s\n', fullfile(outDir,'scan_delivery_limits_no_disturbance.csv'));
end

function patch_header(pmax, wmax)
pth = fullfile(fileparts(mfilename('fullpath')),'grid_forming_control.h');
txt = fileread(pth);
txt = regexprep(txt, '#define\s+GSI_PLOOP_OUT_MAX\s+[0-9\.eE+\-]+', ...
    sprintf('#define   GSI_PLOOP_OUT_MAX                   %.8g', pmax), 'once');
txt = regexprep(txt, '#define\s+GSI_PLOOP_OUT_MIN\s+[0-9\.eE+\-]+', ...
    sprintf('#define   GSI_PLOOP_OUT_MIN                  %.8g', -pmax), 'once');
txt = regexprep(txt, '#define\s+GSI_WREF_MAX\s+[0-9\.eE+\-]+', ...
    sprintf('#define   GSI_WREF_MAX                        %.8g', wmax), 'once');
fid=fopen(pth,'w'); fwrite(fid,txt); fclose(fid);
end

function m = run_one_case(pref, simStop, vacRef, vdcRef, qref, forceRecompile)
setenv('MW_MINGW64_LOC','C:\mingw64');
if forceRecompile
    evalin('base','clear mex');
    evalin('base','bdclose(''all'')');
    evalin('base',sprintf('cd(''%s''); mex main.c svpwm.c motorcontrol.c grid_forming_control.c;', strrep(fileparts(mfilename('fullpath')),'\','\\')));
end
assignin('base','P_wt_rated_override',pref);
assignin('base','wind_step_mps_override',0.0);
assignin('base','sim_stop_time_override',simStop);
run('GFM_MWT_Nonlinear_Params.m');

mdl='Grid_Forming_PMSG'; load_system(mdl);
set_param(mdl,'StopTime',num2str(simStop));
set_param([mdl '/MOTOR_CONTROL1'],'Pref',num2str(pref),'Qref',num2str(qref),'ReferenceACVoltage',num2str(vacRef),'ReferenceDCVoltage',num2str(vdcRef));
set_param([mdl '/MOTOR_CONTROL1/S-Function1'],'Parameters',sprintf('%.12g, %.12g, %.12g, %.12g',pref,qref,vacRef,vdcRef));
set_param(mdl,'SimulationCommand','update');
sim(mdl);

pmeas = evalin('base','pmeas_out');
wref = evalin('base','wref_out');
udc = evalin('base','udc_meas');
omega_g = evalin('base','omega_g');
theta_tw = evalin('base','theta_tw');

tail = 1.0;
m = [mean_tail(pmeas,tail), slope_tail(udc,tail), mean_tail(wref,tail), slope_tail(omega_g,tail), slope_tail(theta_tw,tail)];
end

function s = slope_tail(ts,win)
t=ts.Time(:); y=ts.Data(:); idx=t>=t(end)-win; p=polyfit(t(idx),y(idx),1); s=p(1);
end
function m = mean_tail(ts,win)
t=ts.Time(:); y=ts.Data(:); idx=t>=t(end)-win; m=mean(y(idx));
end
function restore_file(path,txt)
fid=fopen(path,'w'); fwrite(fid,txt); fclose(fid);
end

