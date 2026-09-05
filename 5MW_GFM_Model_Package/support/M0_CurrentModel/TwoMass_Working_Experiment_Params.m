%% Runtime experiment parameters for the single two-mass working model
% Defaults describe a fixed normal 50 Hz grid.  The working model is the
% steady-operation model and intentionally contains no voltage-sag test.
% Prefer
% passing *_override variables through run_two_mass_working_experiment.m.

grid_voltage_peak_V = 563;
grid_frequency_hz = 50;

override_pairs = { ...
    'grid_voltage_peak_V_override', 'grid_voltage_peak_V'; ...
    'grid_frequency_hz_override',   'grid_frequency_hz'};

for override_index = 1:size(override_pairs, 1)
    override_name = override_pairs{override_index, 1};
    value_name = override_pairs{override_index, 2};
    if exist(override_name, 'var')
        eval([value_name ' = ' override_name ';']); %#ok<EVLDIR>
    elseif evalin('base', sprintf('exist(''%s'',''var'')', override_name))
        eval([value_name ' = evalin(''base'', override_name);']); %#ok<EVLDIR>
    end
end

assert(grid_voltage_peak_V > 0, 'Grid voltage peak must be positive.');
assert(grid_frequency_hz > 0, 'Grid frequency must be positive.');

working_experiment_names = { ...
    'grid_voltage_peak_V', 'grid_frequency_hz'};
for parameter_index = 1:numel(working_experiment_names)
    parameter_name = working_experiment_names{parameter_index};
    assignin('base', parameter_name, eval(parameter_name));
end
