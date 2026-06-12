function report = check_vsg_parameter_consistency()
%CHECK_VSG_PARAMETER_CONSISTENCY Compare small-signal, MATLAB, and C parameters.
% This diagnostic is read-only. It is used before nonlinear tuning to avoid
% mistaking parameter inconsistency for a controller-stability problem.

rootDir = fileparts(mfilename('fullpath'));
oldDir = pwd;
cleanupObj = onCleanup(@() cd(oldDir)); %#ok<NASGU>
cd(rootDir);

use_ssm_same_object_params_override = true; %#ok<NASGU>
run(fullfile(rootDir, 'GFM_MWT_Nonlinear_Params.m'));

ssmPath = locate_ssm_parameters_mat();
ssm = load(ssmPath);

plantNames = { ...
    'P_wt_rated', ...
    'omega_m0', ...
    'v_w0', ...
    'J_t', ...
    'J_g', ...
    'K_sh', ...
    'D_sh', ...
    'D_t', ...
    'D_g', ...
    'T_aero0', ...
    'theta_tw0', ...
    'D_aero', ...
    'K_v_aero'};

rows = cell(numel(plantNames), 5);
for k = 1:numel(plantNames)
    name = plantNames{k};
    nonlinearValue = evalin('base', name);
    if isfield(ssm, name)
        smallSignalValue = ssm.(name);
        delta = nonlinearValue - smallSignalValue;
        status = localStatus(delta, nonlinearValue, smallSignalValue);
    else
        smallSignalValue = NaN;
        delta = NaN;
        status = "missing_in_small_signal_mat";
    end
    rows(k, :) = {name, nonlinearValue, smallSignalValue, delta, status};
end

plantTable = cell2table(rows, ...
    'VariableNames', {'Name', 'Nonlinear', 'SmallSignal', 'Delta', 'Status'});

macroFiles = {'motorcontrol.h', 'grid_forming_control_vsg.h'};
macroNames = {'GRID_UDC__C', 'MOTOR_POLE_PAIR', 'MOTOR_RS', 'MOTOR_LD', ...
    'MOTOR_LQ', 'MOTOR_FM_25_TEMPERATURE', 'MOTOR_JM', ...
    'CURRENT_LOOP_BANDWITH', 'CURRENT_LOOP_BANDWITH_ID'};

macroRows = {};
for f = 1:numel(macroFiles)
    filePath = fullfile(rootDir, macroFiles{f});
    text = fileread(filePath);
    for k = 1:numel(macroNames)
        macroName = macroNames{k};
        value = localReadMacro(text, macroName);
        if ~isnan(value)
            macroRows(end+1, :) = {macroFiles{f}, macroName, value}; %#ok<AGROW>
        end
    end
end

if isempty(macroRows)
    macroTable = table(string.empty, string.empty, [], ...
        'VariableNames', {'File', 'Macro', 'Value'});
else
    macroTable = cell2table(macroRows, ...
        'VariableNames', {'File', 'Macro', 'Value'});
end

J_eq = J_t * J_g / (J_t + J_g); %#ok<NODEF>
f_sh_Hz = sqrt(K_sh / J_eq) / (2*pi); %#ok<NODEF>
zeta_sh = D_sh / (2*sqrt(K_sh * J_eq)); %#ok<NODEF>
T_e0_from_power = P_wt_rated / omega_m0; %#ok<NODEF>

report = struct();
report.generatedAt = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
report.rootDir = rootDir;
report.ssmPath = ssmPath;
report.plantTable = plantTable;
report.macroTable = macroTable;
report.macroCompareTable = localBuildMacroCompareTable(macroTable, ssm);
report.derived = struct( ...
    'J_eq', J_eq, ...
    'f_sh_Hz', f_sh_Hz, ...
    'zeta_sh', zeta_sh, ...
    'T_e0_from_power', T_e0_from_power, ...
    'theta_tw0_from_power', T_e0_from_power / K_sh);

outDir = fullfile(rootDir, 'Validation_Results');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

outCsv = fullfile(outDir, 'vsg_parameter_consistency_latest.csv');
writetable(plantTable, outCsv);

outMacroCsv = fullfile(outDir, 'vsg_c_macro_consistency_latest.csv');
writetable(macroTable, outMacroCsv);

outMacroCompareCsv = fullfile(outDir, 'vsg_c_macro_vs_small_signal_latest.csv');
writetable(report.macroCompareTable, outMacroCompareCsv);

outMd = fullfile(outDir, 'VSG_Parameter_Consistency_Check_20260612.md');
localWriteMarkdown(outMd, report);

fprintf('Saved parameter consistency report:\n');
fprintf('  %s\n', outMd);
fprintf('  %s\n', outCsv);
fprintf('  %s\n', outMacroCsv);
fprintf('  %s\n', outMacroCompareCsv);
fprintf('Derived shaft mode: f_sh = %.6f Hz, zeta = %.6f\n', f_sh_Hz, zeta_sh);
end

function status = localStatus(delta, nonlinearValue, smallSignalValue)
tol = 1e-9 * max([1, abs(nonlinearValue), abs(smallSignalValue)]);
if isnan(delta)
    status = "missing";
elseif abs(delta) <= tol
    status = "match";
else
    status = "mismatch";
end
end

function value = localReadMacro(text, macroName)
expr = ['(?m)^\s*#define\s+', regexptranslate('escape', macroName), '\s+([0-9eE+\-\.]+)'];
tok = regexp(text, expr, 'tokens', 'once');
if isempty(tok)
    value = NaN;
else
    value = str2double(tok{1});
end
end

function compareTable = localBuildMacroCompareTable(macroTable, ssm)
map = { ...
    'GRID_UDC__C', 'C_dc', 'DC-link capacitance'; ...
    'MOTOR_POLE_PAIR', 'n_p', 'PMSG pole pairs'; ...
    'MOTOR_RS', 'R_s', 'PMSG stator resistance'; ...
    'MOTOR_LD', 'L_d', 'PMSG d-axis inductance'; ...
    'MOTOR_LQ', 'L_q', 'PMSG q-axis inductance'; ...
    'MOTOR_FM_25_TEMPERATURE', 'psi_f', 'PMSG PM flux linkage'; ...
    'MOTOR_JM', 'J_g', 'PMSG generator inertia'};

rows = {};
for k = 1:size(map, 1)
    macroName = map{k, 1};
    ssmName = map{k, 2};
    description = map{k, 3};
    idx = find(strcmp(macroTable.Macro, macroName), 1, 'first');
    if isempty(idx) || ~isfield(ssm, ssmName)
        continue;
    end
    macroValue = macroTable.Value(idx);
    ssmValue = ssm.(ssmName);
    delta = macroValue - ssmValue;
    if abs(ssmValue) > eps
        relPct = 100 * delta / ssmValue;
    else
        relPct = NaN;
    end
    status = localStatus(delta, macroValue, ssmValue);
    rows(end+1, :) = {macroName, ssmName, description, macroValue, ssmValue, delta, relPct, status}; %#ok<AGROW>
end

if isempty(rows)
    compareTable = table(string.empty, string.empty, string.empty, [], [], [], [], string.empty, ...
        'VariableNames', {'Macro', 'SmallSignalName', 'Description', 'MacroValue', ...
        'SmallSignalValue', 'Delta', 'RelativePercent', 'Status'});
else
    compareTable = cell2table(rows, ...
        'VariableNames', {'Macro', 'SmallSignalName', 'Description', 'MacroValue', ...
        'SmallSignalValue', 'Delta', 'RelativePercent', 'Status'});
end
end

function localWriteMarkdown(outMd, report)
fid = fopen(outMd, 'w');
cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>

fprintf(fid, '# VSG Nonlinear Parameter Consistency Check\n\n');
fprintf(fid, '- Generated at: %s\n', report.generatedAt);
fprintf(fid, '- Nonlinear directory: `%s`\n', report.rootDir);
fprintf(fid, '- Small-signal parameter file: `%s`\n\n', report.ssmPath);

fprintf(fid, '## 1. MATLAB Parameters vs Small-Signal MAT\n\n');
fprintf(fid, '| Parameter | Nonlinear MATLAB | Small-Signal MAT | Delta | Status |\n');
fprintf(fid, '|---|---:|---:|---:|---|\n');
T = report.plantTable;
for k = 1:height(T)
    fprintf(fid, '| `%s` | %.10g | %.10g | %.4g | %s |\n', ...
        T.Name{k}, T.Nonlinear(k), T.SmallSignal(k), T.Delta(k), string(T.Status(k)));
end

fprintf(fid, '\n## 2. Derived Shaft Quantities\n\n');
fprintf(fid, '- Equivalent inertia `J_eq = %.10g kg*m^2`\n', report.derived.J_eq);
fprintf(fid, '- Shaft natural frequency `f_sh = %.6f Hz`\n', report.derived.f_sh_Hz);
fprintf(fid, '- Shaft damping ratio `zeta_sh = %.6f`\n', report.derived.zeta_sh);
fprintf(fid, '- Steady torque from power/speed `T_e0 = %.10g N*m`\n', report.derived.T_e0_from_power);
fprintf(fid, '- Steady twist angle `theta_tw0 = %.10g rad`\n\n', report.derived.theta_tw0_from_power);

fprintf(fid, '## 3. C Header Macros\n\n');
fprintf(fid, '| File | Macro | Value |\n');
fprintf(fid, '|---|---|---:|\n');
M = report.macroTable;
for k = 1:height(M)
    fprintf(fid, '| `%s` | `%s` | %.10g |\n', M.File{k}, M.Macro{k}, M.Value(k));
end

fprintf(fid, '\n## 4. C Macros vs Small-Signal Parameters\n\n');
fprintf(fid, '| C Macro | Small-Signal Parameter | Meaning | C Value | Small-Signal Value | Relative Difference | Status |\n');
fprintf(fid, '|---|---|---|---:|---:|---:|---|\n');
C = report.macroCompareTable;
for k = 1:height(C)
    fprintf(fid, '| `%s` | `%s` | %s | %.10g | %.10g | %.4g%% | %s |\n', ...
        C.Macro{k}, C.SmallSignalName{k}, C.Description{k}, C.MacroValue(k), ...
        C.SmallSignalValue(k), C.RelativePercent(k), string(C.Status(k)));
end

fprintf(fid, '\n## 5. Diagnosis\n\n');
fprintf(fid, '- Mechanical MATLAB parameters match the small-signal MAT file.\n');
fprintf(fid, '- If `GRID_UDC__C` differs from small-signal `C_dc`, DC-link time scale and controller tuning are not comparable.\n');
fprintf(fid, '- PMSG resistance, inductance, flux linkage, pole pairs, and inertia should be kept aligned before interpreting torque/DC balance.\n');
fprintf(fid, '- If no-disturbance operation still fails to settle, inspect tail-window `T_aero - T_sh` and `T_sh + T_e` before expanding controller sweeps.\n');
end
