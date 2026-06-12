function diag = run_no_disturbance_diagnosis(useSchemeA, simStop, windStep, ctrlOverrides)
% No-disturbance diagnosis for GFM-MWT nonlinear model.
% Goal:
% 1) keep wind constant (no step / no fault / no damping injection check)
% 2) verify mechanical-side settling first
% 3) verify whether GFM delivers ~1 MW to PCC
% 4) localize dominant non-steady cause among DC-link / power loop / breaker sequence

if nargin < 1
    useSchemeA = false;
end
if nargin < 2
    simStop = 5.0;
end
if nargin < 3
    windStep = 0.0;
end
if nargin < 4
    ctrlOverrides = struct();
end

root = fileparts(mfilename('fullpath'));
old = pwd;
cleanup = onCleanup(@() cd(old));
cd(root);

clearvars -except root old cleanup useSchemeA simStop windStep ctrlOverrides;
run('GFM_MWT_Nonlinear_Params.m');

% Force pure no-disturbance condition for this diagnosis.
wind_step_mps_override = windStep;
sim_stop_time_override = simStop;
assignin('base', 'wind_step_mps_override', wind_step_mps_override);
assignin('base', 'sim_stop_time_override', sim_stop_time_override);

mdl = 'Grid_Forming_PMSG';
if ~isempty(fieldnames(ctrlOverrides)) && isfield(ctrlOverrides, 'ModelName') && ...
        ~isempty(ctrlOverrides.ModelName)
    mdl = char(ctrlOverrides.ModelName);
end
load_system(mdl);

% Default behavior: skip repeated MEX compile in model InitFcn for faster
% diagnosis loops. Existing main.mexw64 is reused.
skipMexCompile = true;
if ~isempty(fieldnames(ctrlOverrides)) && isfield(ctrlOverrides, 'SkipMexCompile')
    skipMexCompile = logical(ctrlOverrides.SkipMexCompile);
end

% VSG-aligned model uses a dedicated S-Function (main_vsg) and source set.
% Keep the legacy main.mexw64 untouched so the P-f PI baseline and the VSG
% copy can be diagnosed independently.
if strcmp(mdl, 'Grid_FormingVSG_PMSG') && exist(fullfile(root, ['main_vsg.' mexext]), 'file') ~= 3
    compile_vsg_sfunction();
end

brkBlk = [mdl '/Three-Phase Breaker'];
origBrkExternal = get_param(brkBlk, 'External');
origBrkInit = get_param(brkBlk, 'InitialState');
dcCapBlk = [mdl '/Cd'];
hasDcCapBlk = getSimulinkBlockHandle(dcCapBlk) > 0;
if hasDcCapBlk
    origDcCapInitialVoltage = get_param(dcCapBlk, 'InitialVoltage');
else
    origDcCapInitialVoltage = '';
end
forceBreakerClosed = false;
if isfield(ctrlOverrides, 'ForceBreakerClosed')
    forceBreakerClosed = logical(ctrlOverrides.ForceBreakerClosed);
end
if forceBreakerClosed
    % Optional: eliminate breaker timing uncertainty in diagnosis.
    set_param(brkBlk, 'External', 'off');
    set_param(brkBlk, 'InitialState', 'closed');
end
if ~isempty(fieldnames(ctrlOverrides)) && isfield(ctrlOverrides, 'DcInitialVoltage_V') && hasDcCapBlk
    set_param(dcCapBlk, 'InitialVoltage', num2str(ctrlOverrides.DcInitialVoltage_V));
end

% Optional control-parameter overrides for S-Function mask.
ctrlBlk = [mdl '/MOTOR_CONTROL1'];
ctrlSfun = [mdl '/MOTOR_CONTROL1/S-Function1'];
origPref = get_param(ctrlBlk, 'Pref');
origQref = get_param(ctrlBlk, 'Qref');
origVacRef = get_param(ctrlBlk, 'ReferenceACVoltage');
origVdcRef = get_param(ctrlBlk, 'ReferenceDCVoltage');
origSfunParams = get_param(ctrlSfun, 'Parameters');
origLoadInitialState = get_param(mdl, 'LoadInitialState');
origInitialState = get_param(mdl, 'InitialState');
origSaveFinalState = get_param(mdl, 'SaveFinalState');
origFinalStateName = get_param(mdl, 'FinalStateName');
origSaveOperatingPoint = get_param(mdl, 'SaveOperatingPoint');
origSaveFormat = get_param(mdl, 'SaveFormat');
origSimulationMode = get_param(mdl, 'SimulationMode');
origFastRestart = get_param(mdl, 'FastRestart');

prefSet = str2double(origPref);
qrefSet = str2double(origQref);
vacSet = str2double(origVacRef);
vdcSet = str2double(origVdcRef);
if ~isempty(fieldnames(ctrlOverrides))
    if isfield(ctrlOverrides, 'Pref_W'), prefSet = ctrlOverrides.Pref_W; end
    if isfield(ctrlOverrides, 'Qref_var'), qrefSet = ctrlOverrides.Qref_var; end
    if isfield(ctrlOverrides, 'VacRef_V'), vacSet = ctrlOverrides.VacRef_V; end
    if isfield(ctrlOverrides, 'VdcRef_V'), vdcSet = ctrlOverrides.VdcRef_V; end

    set_param(ctrlBlk, 'Pref', num2str(prefSet));
    set_param(ctrlBlk, 'Qref', num2str(qrefSet));
    set_param(ctrlBlk, 'ReferenceACVoltage', num2str(vacSet));
    set_param(ctrlBlk, 'ReferenceDCVoltage', num2str(vdcSet));
end

% Some legacy models resolve S-function parameters from base/model workspace
% instead of mask scope. Enable deterministic numeric override when requested.
useNumericSfunParams = false;
if ~isempty(fieldnames(ctrlOverrides))
    if isfield(ctrlOverrides, 'UseNumericSfunParams')
        useNumericSfunParams = logical(ctrlOverrides.UseNumericSfunParams);
    elseif isfield(ctrlOverrides, 'Pref_W') || isfield(ctrlOverrides, 'Qref_var') || ...
            isfield(ctrlOverrides, 'VacRef_V') || isfield(ctrlOverrides, 'VdcRef_V')
        useNumericSfunParams = true;
    end
end
if useNumericSfunParams
    set_param(ctrlSfun, 'Parameters', sprintf('%.12g, %.12g, %.12g, %.12g', prefSet, qrefSet, vacSet, vdcSet));
end

useInitialState = false;
initialStateVar = 'xInitial';
initialStateStartTime = 0;
if ~isempty(fieldnames(ctrlOverrides))
    if isfield(ctrlOverrides, 'InitialStateVar') && ~isempty(ctrlOverrides.InitialStateVar)
        initialStateVar = char(ctrlOverrides.InitialStateVar);
    end
    if isfield(ctrlOverrides, 'InitialStateFile') && ~isempty(ctrlOverrides.InitialStateFile)
        initialStateFile = char(ctrlOverrides.InitialStateFile);
        if exist(initialStateFile, 'file') ~= 2
            error('InitialStateFile not found: %s', initialStateFile);
        end
        load(initialStateFile, initialStateVar);
        initialStateData = eval(initialStateVar);
        assignin('base', initialStateVar, initialStateData);
        useInitialState = true;
    elseif isfield(ctrlOverrides, 'InitialStateData')
        initialStateData = ctrlOverrides.InitialStateData;
        assignin('base', initialStateVar, initialStateData);
        useInitialState = true;
    end
end
if useInitialState
    set_param(mdl, 'LoadInitialState', 'on', 'InitialState', initialStateVar);
    if isa(initialStateData, 'Simulink.op.ModelOperatingPoint')
        initialStateStartTime = initialStateData.snapshotTime;
        sim_stop_time_override = initialStateStartTime + simStop;
        assignin('base', 'sim_stop_time_override', sim_stop_time_override);
    elseif isstruct(initialStateData) && isfield(initialStateData, 'time') && ...
            ~isempty(initialStateData.time)
        initialStateStartTime = initialStateData.time(end);
        sim_stop_time_override = initialStateStartTime + simStop;
        assignin('base', 'sim_stop_time_override', sim_stop_time_override);
    end
else
    % The saved model may still point to a stale xInitial from prior runs.
    % Force a true cold start unless an explicit initial state is supplied.
    set_param(mdl, 'LoadInitialState', 'off', 'InitialState', '');
end

saveFinalState = false;
saveOperatingPoint = false;
saveFormat = origSaveFormat;
finalStateName = 'xFinal';
if ~isempty(fieldnames(ctrlOverrides))
    if isfield(ctrlOverrides, 'SaveFinalState')
        saveFinalState = logical(ctrlOverrides.SaveFinalState);
    end
    if isfield(ctrlOverrides, 'SaveOperatingPoint')
        saveOperatingPoint = logical(ctrlOverrides.SaveOperatingPoint);
    end
    if isfield(ctrlOverrides, 'SaveFormat') && ~isempty(ctrlOverrides.SaveFormat)
        saveFormat = char(ctrlOverrides.SaveFormat);
    end
    if isfield(ctrlOverrides, 'FinalStateName') && ~isempty(ctrlOverrides.FinalStateName)
        finalStateName = char(ctrlOverrides.FinalStateName);
    end
end
if saveFinalState || saveOperatingPoint || ~strcmp(saveFormat, origSaveFormat)
    set_param(mdl, 'SaveFinalState', ternary(saveFinalState, 'on', 'off'), ...
        'SaveOperatingPoint', ternary(saveOperatingPoint, 'on', 'off'), ...
        'FinalStateName', finalStateName, ...
        'SaveFormat', saveFormat);
end

% Optional: force Scheme-A tuned C parameters via compile-time header injection.
% This avoids modifying legacy GBK-encoded C sources and remains fully reversible.
initFcnBak = get_param(mdl, 'InitFcn');
if skipMexCompile
    fastInit = [ ...
        "mdlFile = get_param(bdroot,'FileName');", ...
        "mdlDir = fileparts(mdlFile);", ...
        "dtime=4e-6;", ...
        "Ts_step=1e-6;", ...
        "run(fullfile(mdlDir,'GFM_MWT_Nonlinear_Params.m'));" ...
        ];
    set_param(mdl, 'InitFcn', strjoin(fastInit, newline));
elseif useSchemeA
    schemeInit = [ ...
        "mdlFile = get_param(bdroot,'FileName');", ...
        "mdlDir = fileparts(mdlFile);", ...
        "oldDir = pwd; cleanupObj = onCleanup(@() cd(oldDir));", ...
        "cd(mdlDir);", ...
        "setenv('MW_MINGW64_LOC','C:\mingw64');", ...
        "mex -setup C++;", ...
        "clc;", ...
        "mex main.c svpwm.c motorcontrol.c grid_forming_control.c schemeA_runtime_patch.c;", ...
        "display 'MEX compile done (SchemeA runtime patch).';", ...
        "dtime=4e-6; Ts_step=1e-6;", ...
        "run(fullfile(mdlDir,'GFM_MWT_Nonlinear_Params.m'));" ...
        ];
    set_param(mdl, 'InitFcn', strjoin(schemeInit, newline));
end

set_param(mdl, 'StopTime', num2str(sim_stop_time_override));
if ~isempty(fieldnames(ctrlOverrides)) && isfield(ctrlOverrides, 'SimulationMode') && ...
        ~isempty(ctrlOverrides.SimulationMode)
    set_param(mdl, 'SimulationMode', char(ctrlOverrides.SimulationMode));
end
if ~isempty(fieldnames(ctrlOverrides)) && isfield(ctrlOverrides, 'UseFastRestart')
    set_param(mdl, 'FastRestart', ternary(logical(ctrlOverrides.UseFastRestart), 'on', 'off'));
end
set_param(mdl, 'SimulationCommand', 'update');
if useNumericSfunParams
    % Re-apply after update in case symbolic expression is restored.
    set_param(ctrlSfun, 'Parameters', sprintf('%.12g, %.12g, %.12g, %.12g', prefSet, qrefSet, vacSet, vdcSet));
end
origReturnWorkspaceOutputs = get_param(mdl, 'ReturnWorkspaceOutputs');
set_param(mdl, 'ReturnWorkspaceOutputs', 'on');
try
    simOut = sim(mdl);
catch ME
    set_param(mdl, 'ReturnWorkspaceOutputs', origReturnWorkspaceOutputs);
    set_param(mdl, 'SimulationMode', origSimulationMode);
    set_param(mdl, 'FastRestart', origFastRestart);
    set_param(mdl, 'InitFcn', initFcnBak);
    if hasDcCapBlk
        set_param(dcCapBlk, 'InitialVoltage', origDcCapInitialVoltage);
    end
    rethrow(ME);
end
set_param(mdl, 'ReturnWorkspaceOutputs', origReturnWorkspaceOutputs);
set_param(mdl, 'SimulationMode', origSimulationMode);
set_param(mdl, 'FastRestart', origFastRestart);
set_param(mdl, 'InitFcn', initFcnBak);
if saveFinalState || saveOperatingPoint || ~strcmp(saveFormat, origSaveFormat)
    set_param(mdl, 'LoadInitialState', origLoadInitialState, ...
        'InitialState', origInitialState, ...
        'SaveFinalState', origSaveFinalState, ...
        'FinalStateName', origFinalStateName, ...
        'SaveOperatingPoint', origSaveOperatingPoint, ...
        'SaveFormat', origSaveFormat);
end
if forceBreakerClosed
    set_param(brkBlk, 'External', origBrkExternal);
    set_param(brkBlk, 'InitialState', origBrkInit);
end
if hasDcCapBlk
    set_param(dcCapBlk, 'InitialVoltage', origDcCapInitialVoltage);
end
if ~isempty(fieldnames(ctrlOverrides))
    set_param(ctrlBlk, 'Pref', origPref);
    set_param(ctrlBlk, 'Qref', origQref);
    set_param(ctrlBlk, 'ReferenceACVoltage', origVacRef);
    set_param(ctrlBlk, 'ReferenceDCVoltage', origVdcRef);
    if useNumericSfunParams
        set_param(ctrlSfun, 'Parameters', origSfunParams);
    end
end
if useInitialState && evalin('base', sprintf('exist(''%s'', ''var'')', initialStateVar)) == 1
    evalin('base', sprintf('clear(''%s'')', initialStateVar));
end

% Optional: keep key timeseries in output struct for post-analysis.
saveSeries = true;
if ~isempty(fieldnames(ctrlOverrides)) && isfield(ctrlOverrides, 'SaveSeries')
    saveSeries = logical(ctrlOverrides.SaveSeries);
end
seriesMaxPoints = inf;
threePhaseTail = inf;
if ~isempty(fieldnames(ctrlOverrides)) && isfield(ctrlOverrides, 'SeriesMaxPoints')
    seriesMaxPoints = ctrlOverrides.SeriesMaxPoints;
end
if ~isempty(fieldnames(ctrlOverrides)) && isfield(ctrlOverrides, 'ThreePhaseTail_s')
    threePhaseTail = ctrlOverrides.ThreePhaseTail_s;
end

% Pull timeseries from base workspace (written by ToWorkspace blocks).
% Keep reporting available evidence even if one observation block is absent.
missingSignals = strings(0, 1);
[omega_g, missingSignals] = localGetSeries(simOut, 'omega_g', missingSignals);
[omega_t, missingSignals] = localGetSeries(simOut, 'omega_t', missingSignals);
[theta_tw, missingSignals] = localGetSeries(simOut, 'theta_tw', missingSignals);
[T_sh, missingSignals] = localGetSeries(simOut, 'T_sh', missingSignals);
[T_e, missingSignals] = localGetSeries(simOut, 'T_e', missingSignals);
[T_aero, missingSignals] = localGetSeries(simOut, 'T_aero', missingSignals);
[pref_out, missingSignals] = localGetSeries(simOut, 'pref_out', missingSignals);
[pmeas_out, missingSignals] = localGetSeries(simOut, 'pmeas_out', missingSignals);
[wref_out, missingSignals] = localGetSeries(simOut, 'wref_out', missingSignals);
[udc_meas, missingSignals] = localGetSeries(simOut, 'udc_meas', missingSignals);
[ud1ref_out, missingSignals] = localGetSeries(simOut, 'ud1ref_out', missingSignals);
optionalSignals = strings(0, 1);
[presyn_out, optionalSignals] = localGetOptionalSeries(simOut, 'presyn_out', optionalSignals);
[gsc_voltage_ref_out, optionalSignals] = localGetOptionalSeries(simOut, 'gsc_voltage_ref_out', optionalSignals);
[gsc_uod_ref_out, optionalSignals] = localGetOptionalSeries(simOut, 'gsc_uod_ref_out', optionalSignals);
[gsc_pcc_ud_out, optionalSignals] = localGetOptionalSeries(simOut, 'gsc_pcc_ud_out', optionalSignals);
[msc_iqref, missingSignals] = localGetSeries(simOut, 'msc_iqref', missingSignals);
[msc_ud_fwd, missingSignals] = localGetSeriesWithFallback(simOut, {'msc_ud_fwd', 'idref_out'}, missingSignals);
[msc_uq_fwd, missingSignals] = localGetSeriesWithFallback(simOut, {'msc_uq_fwd', 'id_out'}, missingSignals);
[msc_ud1ref, missingSignals] = localGetSeries(simOut, 'msc_ud1ref', missingSignals);
[msc_uq1ref, missingSignals] = localGetSeries(simOut, 'msc_uq1ref', missingSignals);
[msc_id, missingSignals] = localGetSeries(simOut, 'msc_id', missingSignals);
[msc_iq, missingSignals] = localGetSeries(simOut, 'msc_iq', missingSignals);
[msc_mod_depth_signal, optionalSignals] = localGetOptionalSeries(simOut, 'msc_mod_depth', optionalSignals);
[pcc_ia, missingSignals] = localGetSeries(simOut, 'pcc_ia', missingSignals);
[pcc_ib, missingSignals] = localGetSeries(simOut, 'pcc_ib', missingSignals);
[pcc_ic, missingSignals] = localGetSeries(simOut, 'pcc_ic', missingSignals);
[pcc_uab, missingSignals] = localGetSeries(simOut, 'pcc_uab', missingSignals);
[pcc_ubc, missingSignals] = localGetSeries(simOut, 'pcc_ubc', missingSignals);
[pcc_uca, missingSignals] = localGetSeries(simOut, 'pcc_uca', missingSignals); %#ok<NASGU>

tail = 1.0; % last 1 s as steady-window check

% Mechanical-side trend
diag.omega_g_end_slope = localSlope(omega_g, tail);
diag.omega_t_end_slope = localSlope(omega_t, tail);
diag.T_sh_end_slope = localSlope(T_sh, tail);
diag.theta_tw_end_slope = localSlope(theta_tw, tail);
diag.T_sh_end_mean = localMean(T_sh, tail);
diag.T_e_end_mean = localMean(T_e, tail);
diag.T_aero_end_mean = localMean(T_aero, tail);
diag.omega_g_end_mean = localMean(omega_g, tail);
diag.omega_t_end_mean = localMean(omega_t, tail);
diag.Taero_plus_Te_end_mean = diag.T_aero_end_mean + diag.T_e_end_mean;
diag.Taero_minus_Tsh_end_mean = diag.T_aero_end_mean - diag.T_sh_end_mean;
diag.Tsh_plus_Te_end_mean = diag.T_sh_end_mean + diag.T_e_end_mean;
diag.torque_balance_Taero_Te_err_pu = localSafeDiv(abs(diag.T_aero_end_mean + diag.T_e_end_mean), max(abs(diag.T_aero_end_mean),1));
diag.torque_balance_Tsh_Te_err_pu = localSafeDiv(abs(diag.T_sh_end_mean + diag.T_e_end_mean), max(abs(diag.T_sh_end_mean),1));

% Control-side trend
diag.pref_end_mean = localMean(pref_out, tail);
diag.pmeas_end_mean = localMean(pmeas_out, tail);
diag.wref_end_mean = localMean(wref_out, tail);
diag.presyn_end_mean = localMean(presyn_out, tail);
diag.presyn_switch_happened = localPresynSwitched(presyn_out);
diag.presyn_first_switch_time = localPresynFirstSwitchTime(presyn_out);
diag.udc_end_mean = localMean(udc_meas, tail);
diag.udc_end_slope = localSlope(udc_meas, tail);
diag.udc_end_min = localMin(udc_meas, tail);
diag.udc_end_max = localMax(udc_meas, tail);
diag.gsc_ud1ref_end_mean = localMean(ud1ref_out, tail);
diag.gsc_ud1ref_end_maxabs = localMaxAbs(ud1ref_out, tail);
diag.gsc_voltage_ref_end_mean = localMean(gsc_voltage_ref_out, tail);
diag.gsc_uod_ref_end_mean = localMean(gsc_uod_ref_out, tail);
diag.gsc_pcc_ud_end_mean = localMean(gsc_pcc_ud_out, tail);
diag.gsc_pcc_ud_end_rms = localRms(gsc_pcc_ud_out, tail);
diag.msc_iqref_end_mean = localMean(msc_iqref, tail);
diag.msc_iqref_end_maxabs = localMaxAbs(msc_iqref, tail);
diag.msc_ud_fwd_end_mean = localMean(msc_ud_fwd, tail);
diag.msc_uq_fwd_end_mean = localMean(msc_uq_fwd, tail);
diag.msc_ud1ref_end_mean = localMean(msc_ud1ref, tail);
diag.msc_uq1ref_end_mean = localMean(msc_uq1ref, tail);
diag.msc_id_end_mean = localMean(msc_id, tail);
diag.msc_iq_end_mean = localMean(msc_iq, tail);
msc_iq_tracking_error = localTimeseriesDifference(msc_iqref, msc_iq);
msc_vcmd_mag = localTimeseriesMagnitude(msc_ud1ref, msc_uq1ref);
msc_iq_pi_out = localTimeseriesDifference(msc_uq1ref, msc_uq_fwd);
diag.msc_iq_tracking_error_end_mean = localMean(msc_iq_tracking_error, tail);
diag.msc_iq_tracking_error_end_maxabs = localMaxAbs(msc_iq_tracking_error, tail);
diag.msc_vcmd_mag_end_mean = localMean(msc_vcmd_mag, tail);
diag.msc_vcmd_mag_end_max = localMax(msc_vcmd_mag, tail);
diag.msc_linear_voltage_limit = diag.udc_end_mean / 1.5;
diag.msc_mod_depth_mean = 1.5 * diag.msc_vcmd_mag_end_mean / diag.udc_end_mean;
diag.msc_mod_depth_max = 1.5 * diag.msc_vcmd_mag_end_max / diag.udc_end_mean;
diag.msc_mod_depth_signal_end_mean = localMean(msc_mod_depth_signal, tail);
diag.msc_mod_depth_signal_end_max = localMax(msc_mod_depth_signal, tail);
diag.msc_iq_pi_out_end_mean = localMean(msc_iq_pi_out, tail);
diag.msc_iq_pi_out_end_maxabs = localMaxAbs(msc_iq_pi_out, tail);
diag.pmeas_end_slope = localSlope(pmeas_out, tail);

% PCC three-phase active power reconstruction from line-line voltages.
[tp, ppcc] = localPccPowerFromLineLine(pcc_uab, pcc_ubc, pcc_ia, pcc_ib, pcc_ic);
diag.Ppcc_end_mean = localMeanRaw(tp, ppcc, tail);
diag.Ppcc_end_slope = localSlopeRaw(tp, ppcc, tail);
diag.Ppcc_target_error = diag.Ppcc_end_mean - P_wt_rated;
diag.Ppcc_target_error_pu = diag.Ppcc_target_error / P_wt_rated;
diag.pcc_ia_end_rms = localRms(pcc_ia, tail);
diag.pcc_ib_end_rms = localRms(pcc_ib, tail);
diag.pcc_ic_end_rms = localRms(pcc_ic, tail);
diag.pcc_uab_end_rms = localRms(pcc_uab, tail);
diag.pcc_ubc_end_rms = localRms(pcc_ubc, tail);
diag.pcc_uca_end_rms = localRms(pcc_uca, tail);
diag.three_phase_waveform_available_flag = all(isfinite([ ...
    diag.pcc_ia_end_rms, diag.pcc_ib_end_rms, diag.pcc_ic_end_rms, ...
    diag.pcc_uab_end_rms, diag.pcc_ubc_end_rms, diag.pcc_uca_end_rms]));
diag.missing_signals = missingSignals;
diag.missing_optional_signals = optionalSignals;
diag.observation_complete_flag = isempty(missingSignals);
diag.initial_state_loaded_flag = useInitialState;
diag.initial_state_start_time = initialStateStartTime;
if saveFinalState || saveOperatingPoint
    diag.final_state = localExtractFinalState(simOut, finalStateName);
end
if saveSeries
    diag.series = struct();
    diag.series.omega_g = localPackSeries(omega_g, inf, seriesMaxPoints);
    diag.series.omega_t = localPackSeries(omega_t, inf, seriesMaxPoints);
    diag.series.theta_tw = localPackSeries(theta_tw, inf, seriesMaxPoints);
    diag.series.T_sh = localPackSeries(T_sh, inf, seriesMaxPoints);
    diag.series.T_e = localPackSeries(T_e, inf, seriesMaxPoints);
    diag.series.T_aero = localPackSeries(T_aero, inf, seriesMaxPoints);
    diag.series.pmeas_out = localPackSeries(pmeas_out, inf, seriesMaxPoints);
diag.series.ppcc = localPackRaw(tp, ppcc, seriesMaxPoints);
diag.series.udc_meas = localPackSeries(udc_meas, inf, seriesMaxPoints);
diag.series.ud1ref_out = localPackSeries(ud1ref_out, inf, seriesMaxPoints);
diag.series.gsc_voltage_ref_out = localPackSeries(gsc_voltage_ref_out, inf, seriesMaxPoints);
diag.series.gsc_uod_ref_out = localPackSeries(gsc_uod_ref_out, inf, seriesMaxPoints);
diag.series.gsc_pcc_ud_out = localPackSeries(gsc_pcc_ud_out, inf, seriesMaxPoints);
diag.series.msc_iqref = localPackSeries(msc_iqref, inf, seriesMaxPoints);
    diag.series.msc_ud_fwd = localPackSeries(msc_ud_fwd, inf, seriesMaxPoints);
    diag.series.msc_uq_fwd = localPackSeries(msc_uq_fwd, inf, seriesMaxPoints);
    diag.series.msc_ud1ref = localPackSeries(msc_ud1ref, inf, seriesMaxPoints);
    diag.series.msc_uq1ref = localPackSeries(msc_uq1ref, inf, seriesMaxPoints);
    diag.series.msc_id = localPackSeries(msc_id, inf, seriesMaxPoints);
    diag.series.msc_iq = localPackSeries(msc_iq, inf, seriesMaxPoints);
    diag.series.msc_mod_depth_signal = localPackSeries(msc_mod_depth_signal, inf, seriesMaxPoints);
    diag.series.msc_vcmd_mag = localPackSeries(msc_vcmd_mag, inf, seriesMaxPoints);
    diag.series.msc_iq_pi_out = localPackSeries(msc_iq_pi_out, inf, seriesMaxPoints);
    diag.series.pcc_ia = localPackSeries(pcc_ia, threePhaseTail, seriesMaxPoints);
    diag.series.pcc_ib = localPackSeries(pcc_ib, threePhaseTail, seriesMaxPoints);
    diag.series.pcc_ic = localPackSeries(pcc_ic, threePhaseTail, seriesMaxPoints);
    diag.series.pcc_uab = localPackSeries(pcc_uab, threePhaseTail, seriesMaxPoints);
    diag.series.pcc_ubc = localPackSeries(pcc_ubc, threePhaseTail, seriesMaxPoints);
    diag.series.pcc_uca = localPackSeries(pcc_uca, threePhaseTail, seriesMaxPoints);
end

% Phase-1 gate check: mechanical first
diag.mech_settled_flag = ...
    abs(diag.omega_g_end_slope) < 1e-2 && ...
    abs(diag.omega_t_end_slope) < 1e-2 && ...
    abs(diag.theta_tw_end_slope) < 5e-4;

% Phase-2 gate check: delivery and DC behavior
diag.power_reached_1MW_flag = abs(diag.Ppcc_target_error_pu) < 0.05;
diag.dc_settled_flag = abs(diag.udc_end_slope) < 5;
% Baseline model gate: oscillations are allowed, but the simulation must
% remain bounded, avoid MSC current-reference saturation, and deliver power.
diag.dc_bounded_flag = isfinite(diag.udc_end_min) && isfinite(diag.udc_end_max) && ...
    diag.udc_end_min > 600 && diag.udc_end_max < 1600;
diag.msc_iqref_unsaturated_flag = diag.msc_iqref_end_maxabs < 0.95 * 1500;
diag.baseline_operational_flag = diag.three_phase_waveform_available_flag && ...
    diag.power_reached_1MW_flag && diag.dc_bounded_flag && diag.msc_iqref_unsaturated_flag;

% Root-cause hints (rule-based, first-pass)
cause = "undetermined";
if ~diag.observation_complete_flag
    cause = "observation_signals_missing";
elseif (~diag.initial_state_loaded_flag && ~diag.presyn_switch_happened) || diag.presyn_end_mean < 0.9
    cause = "breaker_sync_not_completed_or_sequence_issue";
elseif ~diag.dc_settled_flag
    cause = "dc_link_or_active_power_balance_not_settled";
elseif abs(diag.pmeas_end_slope) > 5e3
    cause = "power_loop_not_settled";
elseif ~diag.power_reached_1MW_flag
    cause = "grid_side_delivery_insufficient_check_voltage_current_loops";
elseif ~diag.mech_settled_flag
    cause = "mechanical_side_not_settled_first";
end
diag.primary_cause_hint = cause;

outDir = fullfile(root, 'Validation_Results');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
resultStem = 'no_disturbance_diagnosis';
if ~strcmp(mdl, 'Grid_Forming_PMSG')
    resultStem = ['no_disturbance_diagnosis_' mdl];
end
save(fullfile(outDir, [resultStem '.mat']), 'diag');

% Short text report
rep = fullfile(outDir, [resultStem '.md']);
fid = fopen(rep, 'w');
fprintf(fid, '# 无扰动稳态诊断结果\n\n');
fprintf(fid, '- 仿真时长: %.2f s\n', sim_stop_time_override);
fprintf(fid, '- 风速扰动: %.2f m/s (固定)\n\n', wind_step_mps_override);
fprintf(fid, '## 机械侧\n');
fprintf(fid, '- omega_g 末端斜率: %.6g rad/s^2\n', diag.omega_g_end_slope);
fprintf(fid, '- omega_t 末端斜率: %.6g rad/s^2\n', diag.omega_t_end_slope);
fprintf(fid, '- theta_tw 末端斜率: %.6g rad/s\n', diag.theta_tw_end_slope);
fprintf(fid, '- T_sh 末端斜率: %.6g N*m/s\n', diag.T_sh_end_slope);
fprintf(fid, '- 机械先稳判定: %d\n\n', diag.mech_settled_flag);
fprintf(fid, '## 送功与控制\n');
fprintf(fid, '- P_ref 末端均值: %.6g W\n', diag.pref_end_mean);
fprintf(fid, '- P_meas 末端均值: %.6g W\n', diag.pmeas_end_mean);
fprintf(fid, '- P_pcc 末端均值: %.6g W\n', diag.Ppcc_end_mean);
fprintf(fid, '- P_pcc 相对1MW误差: %.6g pu\n', diag.Ppcc_target_error_pu);
fprintf(fid, '- Udc 末端均值: %.6g V\n', diag.udc_end_mean);
fprintf(fid, '- Udc 末端斜率: %.6g V/s\n', diag.udc_end_slope);
fprintf(fid, '- Udc 末端范围: %.6g ~ %.6g V\n', diag.udc_end_min, diag.udc_end_max);
fprintf(fid, '- GSC 电压参考/幅值参考/PCC d轴末端均值: %.6g / %.6g / %.6g V\n', ...
    diag.gsc_voltage_ref_end_mean, diag.gsc_uod_ref_end_mean, diag.gsc_pcc_ud_end_mean);
fprintf(fid, '- GSC d轴电压指令末端均值/最大绝对值: %.6g / %.6g V\n', diag.gsc_ud1ref_end_mean, diag.gsc_ud1ref_end_maxabs);
fprintf(fid, '- MSC电压指令幅值末端均值/最大值: %.6g / %.6g V\n', diag.msc_vcmd_mag_end_mean, diag.msc_vcmd_mag_end_max);
fprintf(fid, '- MSC q轴电流PI输出末端均值/最大绝对值: %.6g / %.6g V\n', diag.msc_iq_pi_out_end_mean, diag.msc_iq_pi_out_end_maxabs);
fprintf(fid, '- PCC 三相电流 RMS: %.6g / %.6g / %.6g A\n', diag.pcc_ia_end_rms, diag.pcc_ib_end_rms, diag.pcc_ic_end_rms);
fprintf(fid, '- PCC 三相线电压 RMS: %.6g / %.6g / %.6g V\n', diag.pcc_uab_end_rms, diag.pcc_ubc_end_rms, diag.pcc_uca_end_rms);
fprintf(fid, '- PCC 三相波形可用判定: %d\n', diag.three_phase_waveform_available_flag);
fprintf(fid, '- PreSyn 末端均值: %.6g\n', diag.presyn_end_mean);
fprintf(fid, '- 1MW送功判定: %d\n', diag.power_reached_1MW_flag);
fprintf(fid, '- DC稳态判定: %d\n\n', diag.dc_settled_flag);
fprintf(fid, '- 基础模型有界运行判定: %d\n\n', diag.baseline_operational_flag);
fprintf(fid, '## 主要原因提示\n');
fprintf(fid, '- %s\n', diag.primary_cause_hint);
fprintf(fid, '\n## MSC diagnostic quantities\n');
fprintf(fid, '- Ud_fwd/Uq_fwd end mean: %.6g / %.6g V\n', diag.msc_ud_fwd_end_mean, diag.msc_uq_fwd_end_mean);
fprintf(fid, '- Ud1_ref/Uq1_ref end mean: %.6g / %.6g V\n', diag.msc_ud1ref_end_mean, diag.msc_uq1ref_end_mean);
fprintf(fid, '- Id/Iq end mean: %.6g / %.6g A\n', diag.msc_id_end_mean, diag.msc_iq_end_mean);
fprintf(fid, '- Iq_ref minus Iq tracking error mean/maxabs: %.6g / %.6g A\n', diag.msc_iq_tracking_error_end_mean, diag.msc_iq_tracking_error_end_maxabs);
fprintf(fid, '- Mod depth computed/signal mean: %.6g / %.6g\n', diag.msc_mod_depth_mean, diag.msc_mod_depth_signal_end_mean);
fprintf(fid, '- Mod depth computed/signal max: %.6g / %.6g\n', diag.msc_mod_depth_max, diag.msc_mod_depth_signal_end_max);
fclose(fid);

disp(diag);
fprintf('Saved: %s\n', rep);
end

function s = localSlope(ts, tailWin)
if isempty(ts.Time) || numel(ts.Time) < 2
    s = NaN;
    return;
end
t = ts.Time(:); y = ts.Data(:);
idx = t >= (t(end) - tailWin);
if nnz(idx) < 2
    s = NaN;
    return;
end
p = polyfit(t(idx), y(idx), 1);
s = p(1);
end

function m = localMean(ts, tailWin)
if isempty(ts.Time)
    m = NaN;
    return;
end
t = ts.Time(:); y = ts.Data(:);
idx = t >= (t(end) - tailWin);
m = mean(y(idx));
end

function m = localMaxAbs(ts, tailWin)
if isempty(ts.Time)
    m = NaN;
    return;
end
t = ts.Time(:); y = ts.Data(:);
idx = t >= (t(end) - tailWin);
m = max(abs(y(idx)));
end

function [t, p] = localPccPowerFromLineLine(uab, ubc, ia, ib, ic)
% Reconstruct phase voltages from line-line (assuming ua+ub+uc=0):
% ua=(2*uab+ubc)/3; ub=(-uab+ubc)/3; uc=(-uab-2*ubc)/3
if isempty(uab.Time) || isempty(ubc.Time) || isempty(ia.Time) || isempty(ib.Time) || isempty(ic.Time)
    t = zeros(0, 1);
    p = zeros(0, 1);
    return;
end
t = uab.Time(:);
uabv = uab.Data(:);
ubcv = ubc.Data(:);
iav = ia.Data(:);
ibv = ib.Data(:);
icv = ic.Data(:);
ua = (2*uabv + ubcv) / 3;
ub = (-uabv + ubcv) / 3;
uc = (-uabv - 2*ubcv) / 3;
p = ua .* iav + ub .* ibv + uc .* icv;
end

function m = localMeanRaw(t, y, tailWin)
if isempty(t)
    m = NaN;
    return;
end
idx = t >= (t(end) - tailWin);
m = mean(y(idx));
end

function s = localSlopeRaw(t, y, tailWin)
if numel(t) < 2
    s = NaN;
    return;
end
idx = t >= (t(end) - tailWin);
if nnz(idx) < 2
    s = NaN;
    return;
end
p = polyfit(t(idx), y(idx), 1);
s = p(1);
end

function [ts, missingSignals] = localGetSeries(simOut, name, missingSignals)
if isa(simOut, 'Simulink.SimulationOutput') && any(strcmp(who(simOut), name))
    ts = simOut.get(name);
elseif evalin('base', sprintf('exist(''%s'', ''var'')', name))
    ts = evalin('base', name);
else
    ts = timeseries(zeros(0, 1), zeros(0, 1));
    missingSignals(end + 1, 1) = string(name);
end
if isstruct(ts) && isfield(ts, 'time') && isfield(ts, 'signals') && ...
        isfield(ts.signals, 'values')
    ts = timeseries(ts.signals.values, ts.time);
end
end

function [ts, missingSignals] = localGetOptionalSeries(simOut, name, missingSignals)
[ts, missingSignals] = localGetSeries(simOut, name, missingSignals);
end

function [ts, missingSignals] = localGetSeriesWithFallback(simOut, names, missingSignals)
for k = 1:numel(names)
    name = names{k};
    if isa(simOut, 'Simulink.SimulationOutput') && any(strcmp(who(simOut), name))
        ts = simOut.get(name);
        ts = localNormalizeSeries(ts);
        return;
    elseif evalin('base', sprintf('exist(''%s'', ''var'')', name))
        ts = evalin('base', name);
        ts = localNormalizeSeries(ts);
        return;
    end
end
ts = timeseries(zeros(0, 1), zeros(0, 1));
missingSignals(end + 1, 1) = string(strjoin(names, '|'));
end

function ts = localNormalizeSeries(ts)
if isstruct(ts) && isfield(ts, 'time') && isfield(ts, 'signals') && ...
        isfield(ts.signals, 'values')
    ts = timeseries(ts.signals.values, ts.time);
end
end

function mag = localTimeseriesMagnitude(x, y)
if isempty(x) || isempty(y) || ~isprop(x, 'Time') || ~isprop(y, 'Time') || ...
        isempty(x.Time) || isempty(y.Time)
    mag = timeseries(zeros(0, 1), zeros(0, 1));
    return;
end
t = x.Time(:);
xv = x.Data(:);
yv = localInterpLike(y, t);
mag = timeseries(hypot(xv, yv), t);
end

function diffTs = localTimeseriesDifference(a, b)
if isempty(a) || isempty(b) || ~isprop(a, 'Time') || ~isprop(b, 'Time') || ...
        isempty(a.Time) || isempty(b.Time)
    diffTs = timeseries(zeros(0, 1), zeros(0, 1));
    return;
end
t = a.Time(:);
av = a.Data(:);
bv = localInterpLike(b, t);
diffTs = timeseries(av - bv, t);
end

function y = localInterpLike(ts, t)
tt = ts.Time(:);
yy = ts.Data(:);
if numel(tt) == numel(t) && max(abs(tt - t)) < 1e-12
    y = yy;
elseif numel(tt) >= 2
    y = interp1(tt, yy, t, 'linear', 'extrap');
else
    y = NaN(size(t));
end
end

function v = localSafeDiv(a, b)
if nargin < 2 || isempty(b) || ~isfinite(b) || b == 0
    v = NaN;
else
    v = a / b;
end
end

function out = ternary(cond, a, b)
if cond
    out = a;
else
    out = b;
end
end

function xFinal = localExtractFinalState(simOut, stateName)
xFinal = [];
try
    if isa(simOut, 'Simulink.SimulationOutput')
        if any(strcmp(who(simOut), stateName))
            xFinal = simOut.get(stateName);
            return;
        end
    end
catch
end
try
    xFinal = evalin('base', stateName);
catch
    xFinal = [];
end
end

function tf = localPresynSwitched(ts)
tf = false;
if isempty(ts) || ~isprop(ts, 'Time') || ~isprop(ts, 'Data')
    return;
end
t = ts.Time(:);
y = ts.Data(:);
if isempty(t) || isempty(y)
    return;
end
tf = any(y > 0.5) && any(y < 0.5);
end

function tSw = localPresynFirstSwitchTime(ts)
tSw = NaN;
if isempty(ts) || ~isprop(ts, 'Time') || ~isprop(ts, 'Data')
    return;
end
t = ts.Time(:);
y = ts.Data(:);
if numel(y) < 2
    return;
end
logic = (y > 0.5);
rise = (~logic(1:end-1)) & logic(2:end);
idxAll = find(rise);
if isempty(idxAll)
    return;
end
% Ignore very-early startup glitches; keep physically meaningful switch time.
idx = idxAll(find(t(idxAll + 1) > 1e-3, 1, 'first'));
if ~isempty(idx)
    tSw = t(idx + 1);
end
end

function s = localPackSeries(ts, tail, maxPoints)
s = struct('t', zeros(0, 1), 'y', zeros(0, 1));
if isempty(ts) || ~isprop(ts, 'Time') || ~isprop(ts, 'Data')
    return;
end
t = ts.Time(:);
y = ts.Data(:);
if isfinite(tail)
    idx = t >= max(t(end) - tail, t(1));
    t = t(idx);
    y = y(idx);
end
[s.t, s.y] = localThinSeries(t, y, maxPoints);
end

function s = localPackRaw(t, y, maxPoints)
s = struct('t', zeros(0, 1), 'y', zeros(0, 1));
[s.t, s.y] = localThinSeries(t(:), y(:), maxPoints);
end

function [t, y] = localThinSeries(t, y, maxPoints)
if isfinite(maxPoints) && maxPoints > 1 && numel(t) > maxPoints
    idx = unique(round(linspace(1, numel(t), maxPoints)));
    t = t(idx);
    y = y(idx);
end
end

function val = localRms(ts, tail)
val = NaN;
if isempty(ts) || ~isprop(ts, 'Time') || ~isprop(ts, 'Data')
    return;
end
t = ts.Time(:);
y = ts.Data(:);
if isempty(t) || isempty(y)
    return;
end
idx = t >= max(t(end) - tail, t(1));
if any(idx)
    val = sqrt(mean(y(idx).^2));
end
end

function val = localMin(ts, tail)
val = NaN;
if isempty(ts) || ~isprop(ts, 'Time') || ~isprop(ts, 'Data')
    return;
end
t = ts.Time(:);
y = ts.Data(:);
if isempty(t) || isempty(y)
    return;
end
idx = t >= max(t(end) - tail, t(1));
if any(idx)
    val = min(y(idx));
end
end

function val = localMax(ts, tail)
val = NaN;
if isempty(ts) || ~isprop(ts, 'Time') || ~isprop(ts, 'Data')
    return;
end
t = ts.Time(:);
y = ts.Data(:);
if isempty(t) || isempty(y)
    return;
end
idx = t >= max(t(end) - tail, t(1));
if any(idx)
    val = max(y(idx));
end
end
