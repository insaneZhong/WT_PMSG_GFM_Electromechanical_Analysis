function out = run_liu2024_5mw_experiment(options)
%RUN_LIU2024_5MW_EXPERIMENT Run the single 5 MW reproduction model.
arguments
    options.StopTime (1,1) double {mustBePositive} = 6
    options.DampingScale (1,1) double {mustBePositive} = 1
    options.PowerRamp (1,1) double {mustBePositive} = 0.10e6
    options.WindStep (1,1) double = 0
    options.WindStepTime (1,1) double {mustBeNonnegative} = 45
    options.SimulationMode (1,1) string = "accelerator"
end

root = fileparts(mfilename('fullpath'));
model = 'Grid_Forming_PMSG5MW_Liu2024_TwoMass';
assert(exist(fullfile(root,[model '.slx']),'file') == 4, ...
    'The Liu2024 5 MW model has not been generated.');

in = Simulink.SimulationInput(model);
in = in.setModelParameter('StopTime',num2str(options.StopTime,16), ...
    'SimulationMode',char(options.SimulationMode));

% The model InitFcn intentionally resolves experiment overrides from the
% base workspace. SimulationInput variables are installed too late for that
% callback in this legacy model, so stage-4 previously kept its defaults.
% Publish the four supported overrides before sim() and restore the user's
% previous workspace values when this function returns.
overrideNames = {'shaft_damping_scale_override', ...
    'stage4_power_ramp_Wps_override','wind_step_mps_override', ...
    'wind_step_time_override'};
overrideValues = {options.DampingScale,options.PowerRamp, ...
    options.WindStep,options.WindStepTime};
oldOverrides = cell(size(overrideNames));
hadOverrides = false(size(overrideNames));
for k = 1:numel(overrideNames)
    hadOverrides(k) = evalin('base',sprintf('exist(''%s'',''var'')', ...
        overrideNames{k})) ~= 0;
    if hadOverrides(k)
        oldOverrides{k} = evalin('base',overrideNames{k});
    end
    assignin('base',overrideNames{k},overrideValues{k});
end
overrideCleanup = onCleanup(@() restoreOverrides( ...
    overrideNames,hadOverrides,oldOverrides)); %#ok<NASGU>

old_folder = pwd;
folder_cleanup = onCleanup(@() cd(old_folder)); %#ok<NASGU>
cd(root);
out = sim(in);
end

function restoreOverrides(names,hadValues,oldValues)
for k = 1:numel(names)
    if hadValues(k)
        assignin('base',names{k},oldValues{k});
    else
        evalin('base',sprintf('clear(''%s'')',names{k}));
    end
end
end
