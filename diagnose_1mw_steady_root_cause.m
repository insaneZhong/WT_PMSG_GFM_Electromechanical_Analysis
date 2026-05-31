function T = diagnose_1mw_steady_root_cause(mode)
% Root-cause diagnosis for no-disturbance 1 MW steady-state issues.
% Priority order follows research objective:
% 1) mechanical stability first
% 2) then GFM power delivery
% 3) if Ppcc not enough, inspect DC-link / P-loop / V-loop / sync timing.

root = fileparts(mfilename('fullpath'));
old = pwd;
cleanup = onCleanup(@() cd(old)); %#ok<NASGU>
cd(root);

% No disturbance baseline
pref = 1.0e6;
qref = 0;
vacRef = 563;
vdcRef = 5000;
simStop = 3.0;

if nargin < 1 || isempty(mode)
    mode = 'full';
end

% Candidate set
switch lower(mode)
    case 'single'
        tSyncList = 0.5;
        kppList = 1.0e-6;
        kipList = 2.0e-5;
        vLoopBwList = 30;
        schemeAList = false;
    case 'single_schemea'
        tSyncList = 0.5;
        kppList = 1.0e-6;
        kipList = 2.0e-5;
        vLoopBwList = 30;
        schemeAList = true;
    case 'ultraquick'
        tSyncList = [0.5];
        kppList = [1.0e-6];
        kipList = [2.0e-5];
        vLoopBwList = [30];
        schemeAList = [false, true];
    case 'quick'
        tSyncList = [0.5, 2.0];
        kppList = [1.0e-6];
        kipList = [2.0e-5];
        vLoopBwList = [30, 120];
        schemeAList = [false, true];
    otherwise
        tSyncList = [0.5, 1.0, 2.0];
        kppList = [0.5e-6, 1.0e-6, 2.0e-6];
        kipList = [1.0e-5, 2.0e-5, 4.0e-5];
        vLoopBwList = [30, 60, 120];
        schemeAList = [false, true];
end

rows = [];
for useSchemeA = schemeAList
    assignin('base','use_schemeA_overrides',useSchemeA);
    compiled = false;
    for tSync = tSyncList
        for kpp = kppList
            for kip = kipList
                for bw = vLoopBwList
                    apply_case_header(tSync, kpp, kip, bw);
                    m = run_one_case(pref, simStop, vacRef, vdcRef, qref, useSchemeA, ~compiled);
                    compiled = true;
                    rows = [rows; double(useSchemeA), tSync, kpp, kip, bw, m]; %#ok<AGROW>
                end
            end
        end
    end
end

T = array2table(rows, 'VariableNames', { ...
    'useSchemeA','tSync','kpp','kip','vloop_bw', ...
    'Pref_end','Pmeas_mean','Ppcc_mean','Udc_mean','Udc_slope','Wref_mean','OmegaG_slope','ThetaTw_slope'});

% Scoring:
%   mechanical first (omega/theta slopes),
%   then power transfer and DC-link stationarity.
mechScore = abs(T.OmegaG_slope)*8 + abs(T.ThetaTw_slope)*800;
powerScore = abs(T.Pmeas_mean-pref)/pref + abs(T.Udc_slope)/400;
T.score_mech = mechScore;
T.score_power = powerScore;
T.score_total = mechScore + powerScore;
T = sortrows(T, {'score_total','score_mech','score_power'});

outDir = fullfile(root,'Validation_Results');
if ~exist(outDir,'dir'), mkdir(outDir); end
writetable(T, fullfile(outDir,sprintf('diagnose_1mw_steady_root_cause_%s.csv',lower(mode))));

topN = min(20, height(T));
writetable(T(1:topN,:), fullfile(outDir,sprintf('diagnose_1mw_steady_root_cause_%s_top20.csv',lower(mode))));

disp(T(1:topN,:));
fprintf('Saved: %s\n', fullfile(outDir,sprintf('diagnose_1mw_steady_root_cause_%s.csv',lower(mode))));
fprintf('Saved: %s\n', fullfile(outDir,sprintf('diagnose_1mw_steady_root_cause_%s_top20.csv',lower(mode))));
end

function apply_case_header(tSync, kpP, kiP, vLoopBw)
root = fileparts(mfilename('fullpath'));
pthG = fullfile(root,'grid_forming_control.h');
pthM = fullfile(root,'motorcontrol.h');

txtG = fileread(pthG);
txtG = regexprep(txtG, '#define\s+PRESYN_SWITCH_TIME\s+[0-9\.]+', ...
    sprintf('#define   PRESYN_SWITCH_TIME                    %.6g', tSync), 'once');
txtG = regexprep(txtG, ['#define\s+POWER_LOOP_PI_DEFAULTS\s+\{0\.0,\s*0\.0,\s*0\.0,\s*' ...
    '[^,]+,\s*[^,]+,\s*0\.0,\s*0,'], ...
    sprintf('#define POWER_LOOP_PI_DEFAULTS   {0.0, 0.0, 0.0, %.8g, %.8g, 0.0, 0,', kpP, kiP), 'once');
fid = fopen(pthG,'w'); fwrite(fid,txtG); fclose(fid);

txtM = fileread(pthM);
txtM = regexprep(txtM, '#define\s+V_LOOP_BANDWITH\s+[0-9\.eE+\-\/A-Z_]+', ...
    sprintf('#define   V_LOOP_BANDWITH             %.8g', vLoopBw), 'once');
fid = fopen(pthM,'w'); fwrite(fid,txtM); fclose(fid);
end

function m = run_one_case(pref, simStop, vacRef, vdcRef, qref, useSchemeA, forceRecompile)
setenv('MW_MINGW64_LOC','C:\mingw64');
if forceRecompile
    evalin('base','clear mex');
    evalin('base','bdclose(''all'')');
    if useSchemeA
        mexCmd = 'mex -DENABLE_SCHEMEA_OVERRIDES main.c svpwm.c motorcontrol.c grid_forming_control.c;';
    else
        mexCmd = 'mex main.c svpwm.c motorcontrol.c grid_forming_control.c;';
    end
    evalin('base',sprintf('cd(''%s''); %s', strrep(fileparts(mfilename('fullpath')),'\','\\'), mexCmd));
end

assignin('base','P_wt_rated_override',pref);
assignin('base','wind_step_mps_override',0.0);
assignin('base','sim_stop_time_override',simStop);
run('GFM_MWT_Nonlinear_Params.m');

mdl = 'Grid_Forming_PMSG';
load_system(mdl);
% Keep model InitFcn enabled so base timing variables (e.g., dtime/Ts_step)
% are created correctly for powergui / breaker blocks.

blk=[mdl '/MOTOR_CONTROL1']; sf=[blk '/S-Function1'];
set_param(blk,'Pref',num2str(pref),'Qref',num2str(qref),'ReferenceACVoltage',num2str(vacRef),'ReferenceDCVoltage',num2str(vdcRef));
set_param(sf,'Parameters',sprintf('%.12g, %.12g, %.12g, %.12g',pref,qref,vacRef,vdcRef));
set_param(mdl,'StopTime',num2str(simStop));
set_param(mdl,'SimulationCommand','update');
sim(mdl);

pref_out = get_base_var({'pref_out','pref'});
pmeas = get_base_var({'pmeas_out','pmeas'});
wref = get_base_var({'wref_out','wref'});
udc = get_base_var({'udc_meas','udc'});
omega_g = get_base_var({'omega_g'});
theta_tw = get_base_var({'theta_tw'});
pcc_ia = get_base_var({'pcc_ia'});
pcc_ib = get_base_var({'pcc_ib'});
pcc_ic = get_base_var({'pcc_ic'});
pcc_uab = get_base_var({'pcc_uab'});
pcc_ubc = get_base_var({'pcc_ubc'});

tail = 1.0;
% NOTE:
% pcc_uab/pcc_ubc and pcc currents may not share a direct SI scaling in
% this model variant. For robust cross-case comparison, use pmeas_out as
% active-power indicator. Keep pcc reconstruction as a secondary diagnostic.
[tp, ppcc_raw] = pcc_power(pcc_uab,pcc_ubc,pcc_ia,pcc_ib,pcc_ic);
pcc_diag = mean_tail_raw(tp,ppcc_raw,tail);
m = [pref_out.Data(end), ...
     mean_tail(pmeas,tail), pcc_diag, mean_tail(udc,tail), slope_tail(udc,tail), ...
     mean_tail(wref,tail), slope_tail(omega_g,tail), slope_tail(theta_tw,tail)];
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

function v = get_base_var(candidates)
for i = 1:numel(candidates)
    name = candidates{i};
    if evalin('base', sprintf('exist(''%s'',''var'')', name))
        v = evalin('base', name);
        return;
    end
end
vars = evalin('base','who');
error('Missing required variable. Tried: %s. Base vars sample: %s', ...
    strjoin(candidates,','), strjoin(vars(1:min(end,20)),', '));
end
