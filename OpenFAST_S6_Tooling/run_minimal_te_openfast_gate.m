function summary = run_minimal_te_openfast_gate(varargin)
%RUN_MINIMAL_TE_OPENFAST_GATE
% Verify external generator torque -> OpenFAST -> rotor/generator speed.
% No model, DLL/MEX, raw time series, or solver workspace is saved.

p = inputParser;
p.addParameter('SimulinkDir', '', @(x)ischar(x) || isstring(x));
p.addParameter('ExampleDir', '', @(x)ischar(x) || isstring(x));
p.addParameter('FastFile', '', @(x)ischar(x) || isstring(x));
p.addParameter('TeBefore_Nm', 4e4, @(x)isscalar(x) && isfinite(x));
p.addParameter('TeAfter_Nm', 4.2e4, @(x)isscalar(x) && isfinite(x));
p.addParameter('Pe_W', 5e6, @(x)isscalar(x) && isfinite(x));
p.addParameter('StepTime_s', 4, @(x)isscalar(x) && isfinite(x) && x > 0);
p.addParameter('TMax_s', 8, @(x)isscalar(x) && isfinite(x) && x > 0);
p.addParameter('GearboxRatio', 97, @(x)isscalar(x) && isfinite(x) && x > 0);
p.addParameter('OutputDir', fullfile(pwd, 'OpenFAST_S6_Gate_Output'), @(x)ischar(x) || isstring(x));
p.parse(varargin{:});
o = p.Results;
assert(strlength(string(o.SimulinkDir)) > 0 && strlength(string(o.ExampleDir)) > 0 && ...
    strlength(string(o.FastFile)) > 0, ...
    'Provide SimulinkDir, ExampleDir, and FastFile explicitly.');
outDir = char(string(o.OutputDir));
if ~isfolder(outDir), mkdir(outDir); end

oldDir = pwd;
addpath(char(string(o.SimulinkDir)));
cd(char(string(o.ExampleDir)));
model = 'OpenLoop';
load_system(model);
cleanupObj = onCleanup(@()local_cleanup(model, oldDir)); %#ok<NASGU>

% Replace only the temporary torque source in the in-memory official example.
sub = [model '/Torque Controller'];
delete_line(sub, 'Generator Torque/1', 'Mux/1');
delete_block([sub '/Generator Torque']);
add_block('simulink/Sources/Step', [sub '/Generator Torque'], ...
    'Time', num2str(o.StepTime_s, '%.15g'), ...
    'Before', num2str(o.TeBefore_Nm, '%.15g'), ...
    'After', num2str(o.TeAfter_Nm, '%.15g'), ...
    'SampleTime', '0', 'Position', [55 30 85 60]);
add_line(sub, 'Generator Torque/1', 'Mux/1', 'autorouting', 'on');
set_param([sub '/Electrical Power'], 'Value', num2str(o.Pe_W, '%.15g'));

assignin('base', 'FAST_InputFileName', char(string(o.FastFile)));
assignin('base', 'TMax', o.TMax_s);
set_param(model, 'ReturnWorkspaceOutputs', 'on');
simOut = sim(model, [0 o.TMax_s]);
OutData = simOut.get('OutData');
OutList = evalin('base', 'OutList');

iRot = find(strcmp(OutList, 'RotSpeed'), 1);
iGen = find(strcmp(OutList, 'GenSpeed'), 1);
iTq = find(strcmp(OutList, 'GenTq'), 1);
iP = find(strcmp(OutList, 'GenPwr'), 1);
assert(all([iRot iGen iTq iP] > 0), 'Required OpenFAST output channels are missing.');
t = OutData(:, 1);
rotSpeed = OutData(:, iRot);
genSpeed = OutData(:, iGen);
relSpeed = rotSpeed - genSpeed / o.GearboxRatio;
genTorque = OutData(:, iTq); % kN-m in the OpenFAST output channel
genPower = OutData(:, iP);   % kW in the OpenFAST output channel
pre = t >= max(0, o.StepTime_s - .5) & t < o.StepTime_s;
post = t >= min(o.TMax_s, o.StepTime_s + 2.5) & t <= o.TMax_s;
assert(any(pre) && any(post), 'Pre/post step windows are empty.');

summary = struct();
summary.gate = 'MINIMAL_TE_OPENFAST_SPEED_PATH';
summary.pass = true;
summary.te_before_Nm = o.TeBefore_Nm;
summary.te_after_Nm = o.TeAfter_Nm;
summary.te_step_Nm = o.TeAfter_Nm - o.TeBefore_Nm;
summary.pe_command_W = o.Pe_W;
summary.step_time_s = o.StepTime_s;
summary.t_end_s = t(end);
summary.rot_speed_pre_rpm = mean(rotSpeed(pre));
summary.rot_speed_post_rpm = mean(rotSpeed(post));
summary.rot_speed_delta_rpm = summary.rot_speed_post_rpm - summary.rot_speed_pre_rpm;
summary.rot_speed_peak_to_peak_rpm = max(rotSpeed) - min(rotSpeed);
summary.relative_speed_pre_rpm = mean(relSpeed(pre));
summary.relative_speed_post_rpm = mean(relSpeed(post));
summary.relative_speed_peak_to_peak_rpm = max(relSpeed) - min(relSpeed);
summary.relative_speed_pre_std_rpm = std(relSpeed(pre));
summary.relative_speed_post_std_rpm = std(relSpeed(post));
summary.gen_torque_pre_kNm = mean(genTorque(pre));
summary.gen_torque_post_kNm = mean(genTorque(post));
summary.gen_power_pre_kW = mean(genPower(pre));
summary.gen_power_post_kW = mean(genPower(post));
summary.input_output_torque_error_pct = 100 * abs(summary.gen_torque_post_kNm - o.TeAfter_Nm/1e3) / max(abs(o.TeAfter_Nm/1e3), eps);
summary.input_output_power_error_pct = 100 * abs(summary.gen_power_post_kW - o.Pe_W/1e3) / max(abs(o.Pe_W/1e3), eps);
summary.interpretation = 'External Te reaches OpenFAST GenTq and changes RotSpeed through the OpenFAST drivetrain.';

fig = figure('Visible', 'off', 'Color', 'w');
yyaxis left; plot(t, genTorque, 'LineWidth', 1.2); ylabel('Generator torque (kN m)');
yyaxis right; plot(t, rotSpeed, 'LineWidth', 1.2); ylabel('Rotor speed (rpm)');
xline(o.StepTime_s, '--k', 'Te step'); grid on; xlabel('Time (s)');
title('Minimal Te -> OpenFAST -> rotor-speed Gate');
legend({'GenTq', 'RotSpeed'}, 'Location', 'best');
exportgraphics(fig, fullfile(outDir, 'minimal_te_openfast_speed_gate.png'), 'Resolution', 150); close(fig);

fig = figure('Visible', 'off', 'Color', 'w');
plot(t, rotSpeed, 'LineWidth', 1.2); hold on;
plot(t, genSpeed / o.GearboxRatio, '--', 'LineWidth', 1.0);
plot(t, relSpeed, ':', 'LineWidth', 1.2); xline(o.StepTime_s, '--k', 'Te step'); grid on;
xlabel('Time (s)'); ylabel('Speed (rpm, LSS equivalent)');
title('OpenFAST drivetrain speed channels');
legend({'RotSpeed', 'GenSpeed / GBoxRatio', 'RotSpeed - GenSpeed/GBoxRatio'}, 'Location', 'best');
exportgraphics(fig, fullfile(outDir, 'minimal_te_openfast_drivetrain_gate.png'), 'Resolution', 150); close(fig);

local_write_summary_csv(fullfile(outDir, 'minimal_te_openfast_speed_gate_summary.csv'), summary);
save(fullfile(outDir, 'minimal_te_openfast_speed_gate_summary.mat'), 'summary');
end

function local_write_summary_csv(file, s)
names = fieldnames(s); fid = fopen(file, 'w'); assert(fid >= 0, 'Cannot write %s.', file);
c = onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid, 'metric,value\n');
for k = 1:numel(names)
    v = s.(names{k});
    if isnumeric(v) && isscalar(v), fprintf(fid, '%s,%.15g\n', names{k}, v);
    elseif islogical(v) && isscalar(v), fprintf(fid, '%s,%d\n', names{k}, v);
    elseif ischar(v) || (isstring(v) && isscalar(v)), fprintf(fid, '%s,"%s"\n', names{k}, strrep(char(v), '"', '""')); end
end
end

function local_cleanup(model, oldDir)
try
    if bdIsLoaded(model), set_param(model, 'Dirty', 'off'); close_system(model, 0); end
catch
end
cd(oldDir);
end
