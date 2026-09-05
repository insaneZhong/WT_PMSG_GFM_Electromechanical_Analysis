function out = run_liu2024_5mw_experiment(options)
%RUN_LIU2024_5MW_EXPERIMENT Run the single 5 MW reproduction model.
arguments
    options.StopTime (1,1) double {mustBePositive} = 6
    options.DampingScale (1,1) double {mustBePositive} = 1
    options.PowerRamp (1,1) double {mustBePositive} = 0.10e6
    options.WindStep (1,1) double = 0
    options.WindStepTime (1,1) double {mustBeNonnegative} = 45
    options.Qref_var (1,1) double = 0
    options.SCR (1,1) double {mustBePositive} = 4
    options.GridVoltagePeak_V (1,1) double {mustBePositive} = 563.3821790809959
    options.GridFrequency_Hz (1,1) double {mustBePositive} = 50
    options.GridVariationEntity (1,1) string = "Amplitude"
    options.GridVariationType (1,1) string = "Step"
    options.GridVariationStep (1,1) double = 0
    options.GridVariationTiming (1,2) double = [1e9 1e9+1]
    options.SimulationMode (1,1) string = "accelerator"
end

root = fileparts(mfilename('fullpath'));
model = 'Grid_Forming_PMSG5MW_Liu2024_TwoMass';
p = Liu2024_5MW_Params();
assert(exist(fullfile(root,[model '.slx']),'file') == 4, ...
    'The Liu2024 5 MW model has not been generated.');

in = Simulink.SimulationInput(model);
in = in.setModelParameter('StopTime',num2str(options.StopTime,16), ...
    'SimulationMode',char(options.SimulationMode));
in = in.setBlockParameter([model '/MOTOR_CONTROL1'], ...
    'Qref',num2str(options.Qref_var,16));
in = in.setBlockParameter([model '/ProgrammableGridSource'], ...
    'VariationEntity',char(options.GridVariationEntity));
in = in.setBlockParameter([model '/ProgrammableGridSource'], ...
    'VariationType',char(options.GridVariationType));
in = in.setBlockParameter([model '/ProgrammableGridSource'], ...
    'VariationTypeAlt',char(options.GridVariationType));
in = in.setBlockParameter([model '/ProgrammableGridSource'], ...
    'VariationStep',num2str(options.GridVariationStep,16));
in = in.setBlockParameter([model '/ProgrammableGridSource'], ...
    'VariationTiming',mat2str(options.GridVariationTiming,16));
gridImpedanceScale = p.SCR/options.SCR;
for gridBranch = ["L3","L4","L5"]
    branchPath = char(string(model) + "/" + gridBranch);
    in = in.setBlockParameter(branchPath,'Resistance', ...
        num2str(p.Rg*gridImpedanceScale,16));
    in = in.setBlockParameter(branchPath,'Inductance', ...
        num2str(p.Lg*gridImpedanceScale,16));
end

% The model InitFcn intentionally resolves experiment overrides from the
% base workspace. SimulationInput variables are installed too late for that
% callback in this legacy model, so stage-4 previously kept its defaults.
% Publish the four supported overrides before sim() and restore the user's
% previous workspace values when this function returns.
overrideNames = {'shaft_damping_scale_override', ...
    'stage4_power_ramp_Wps_override','wind_step_mps_override', ...
    'wind_step_time_override','qref_var_override','scr_override', ...
    'grid_voltage_peak_V_override','grid_frequency_hz_override'};
overrideValues = {options.DampingScale,options.PowerRamp, ...
    options.WindStep,options.WindStepTime,options.Qref_var,options.SCR, ...
    options.GridVoltagePeak_V,options.GridFrequency_Hz};
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
