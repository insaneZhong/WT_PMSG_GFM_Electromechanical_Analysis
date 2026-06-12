%% Check small-signal DC-link parameters against the nonlinear Simulink model
% This script is read-only for the Simulink model. It documents whether the
% frozen small-signal baseline uses the same DC-link and PMSG parameters as
% the current nonlinear validation model.

close all
clear
clc

this_dir = fileparts(mfilename('fullpath'));
if isempty(this_dir)
    this_dir = pwd;
end
cd(this_dir);

run("Parameters.m");
this_dir = pwd; % Parameters.m clears workspace variables after saving Parameters.mat.
ssm = load("Parameters.mat");

nonlinear_dir = fullfile(this_dir, '..', '..', '..', '（1）构网型风电机组', ...
    '构网型风电机组', 'GFM_1MW _nonlinear');
nonlinear_dir = char(java.io.File(nonlinear_dir).getCanonicalPath());
mdl = fullfile(nonlinear_dir, 'Grid_FormingVSG_PMSG.mdl');
motor_h = fullfile(nonlinear_dir, 'motorcontrol.h');
gsc_h = fullfile(nonlinear_dir, 'grid_forming_control_vsg.h');

if exist(mdl, 'file') ~= 2
    error('Nonlinear model not found: %s', mdl);
end

load_system(mdl);
cleanupObj = onCleanup(@() close_system('Grid_FormingVSG_PMSG', 0)); %#ok<NASGU>

cdBlock = 'Grid_FormingVSG_PMSG/Cd';
nonlinear_Cdc = str2double(get_param(cdBlock, 'Capacitance'));
nonlinear_Cdc_initial_voltage = str2double(get_param(cdBlock, 'InitialVoltage'));

motorText = fileread(motor_h);
gscText = fileread(gsc_h);

macro_GRID_UDC_C_motor = localReadMacro(motorText, 'GRID_UDC__C');
macro_GRID_UDC_C_gsc = localReadMacro(gscText, 'GRID_UDC__C');
macro_MOTOR_LD = localReadMacro(motorText, 'MOTOR_LD');
macro_MOTOR_LQ = localReadMacro(motorText, 'MOTOR_LQ');
macro_MOTOR_RS = localReadMacro(motorText, 'MOTOR_RS');
macro_MOTOR_POLE_PAIR = localReadMacro(motorText, 'MOTOR_POLE_PAIR');
macro_MOTOR_FLUX = localReadMacro(motorText, 'MOTOR_FM_25_TEMPERATURE');
macro_MOTOR_JM = localReadMacro(motorText, 'MOTOR_JM');

rows = {
    'Vdc', 'DC-link operating voltage', ssm.Vdc, nonlinear_Cdc_initial_voltage, ssm.Vdc - nonlinear_Cdc_initial_voltage;
    'C_dc', 'DC-link capacitance', ssm.C_dc, nonlinear_Cdc, ssm.C_dc - nonlinear_Cdc;
    'C_dc_motor_macro', 'DC-link capacitance in motorcontrol.h', ssm.C_dc, macro_GRID_UDC_C_motor, ssm.C_dc - macro_GRID_UDC_C_motor;
    'C_dc_gsc_macro', 'DC-link capacitance in grid_forming_control_vsg.h', ssm.C_dc, macro_GRID_UDC_C_gsc, ssm.C_dc - macro_GRID_UDC_C_gsc;
    'L_d', 'PMSG d-axis inductance', ssm.L_d, macro_MOTOR_LD, ssm.L_d - macro_MOTOR_LD;
    'L_q', 'PMSG q-axis inductance', ssm.L_q, macro_MOTOR_LQ, ssm.L_q - macro_MOTOR_LQ;
    'R_s', 'PMSG stator resistance', ssm.R_s, macro_MOTOR_RS, ssm.R_s - macro_MOTOR_RS;
    'n_p', 'PMSG pole pairs', ssm.n_p, macro_MOTOR_POLE_PAIR, ssm.n_p - macro_MOTOR_POLE_PAIR;
    'psi_f', 'PMSG flux linkage', ssm.psi_f, macro_MOTOR_FLUX, ssm.psi_f - macro_MOTOR_FLUX;
    'J_g', 'PMSG generator inertia', ssm.J_g, macro_MOTOR_JM, ssm.J_g - macro_MOTOR_JM
    };

T = cell2table(rows, 'VariableNames', {'Parameter', 'Meaning', 'SmallSignal', 'Nonlinear', 'Delta'});
T.RelativePercent = 100 * T.Delta ./ max(abs(T.Nonlinear), eps);
T.Status = repmat("match", height(T), 1);
for k = 1:height(T)
    tol = 1e-9 * max([1, abs(T.SmallSignal(k)), abs(T.Nonlinear(k))]);
    if abs(T.Delta(k)) > tol
        T.Status(k) = "mismatch";
    end
end

out_dir = fullfile(this_dir, 'Reports');
if exist(out_dir, 'dir') ~= 7
    mkdir(out_dir);
end
csv_path = fullfile(out_dir, 'DC_Link_Alignment_With_Nonlinear_20260612.csv');
md_path = fullfile(out_dir, 'DC_Link_Alignment_With_Nonlinear_20260612.md');
writetable(T, csv_path);
localWriteReport(md_path, T, mdl, motor_h, gsc_h);

fprintf('Saved DC-link alignment report:\n  %s\n  %s\n', md_path, csv_path);
disp(T);

function value = localReadMacro(text, macroName)
expr = ['(?m)^\s*#define\s+', regexptranslate('escape', macroName), '\s+([0-9eE+\-\.]+)'];
tok = regexp(text, expr, 'tokens', 'once');
if isempty(tok)
    value = NaN;
else
    value = str2double(tok{1});
end
end

function localWriteReport(md_path, T, mdl, motor_h, gsc_h)
fid = fopen(md_path, 'w');
cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>

fprintf(fid, '# DC-Link Alignment Check With Nonlinear Model\n\n');
fprintf(fid, '- Nonlinear model: `%s`\n', mdl);
fprintf(fid, '- Motor header: `%s`\n', motor_h);
fprintf(fid, '- GSC header: `%s`\n\n', gsc_h);

fprintf(fid, '## Parameter Comparison\n\n');
fprintf(fid, '| Parameter | Meaning | Small-signal | Nonlinear | Delta | Relative Difference | Status |\n');
fprintf(fid, '|---|---|---:|---:|---:|---:|---|\n');
for k = 1:height(T)
    fprintf(fid, '| `%s` | %s | %.10g | %.10g | %.10g | %.4g%% | %s |\n', ...
        T.Parameter{k}, T.Meaning{k}, T.SmallSignal(k), T.Nonlinear(k), ...
        T.Delta(k), T.RelativePercent(k), string(T.Status(k)));
end

fprintf(fid, '\n## Interpretation\n\n');
fprintf(fid, '- The frozen small-signal baseline is reproducible, but its DC-link voltage/capacitance should not be claimed as fully aligned with the current nonlinear validation model until mismatches are resolved.\n');
fprintf(fid, '- Nonlinear `Cd` and controller macros both use the physical DC capacitance shown in the table.\n');
fprintf(fid, '- If the nonlinear object is retained, rerun the small-signal baseline with matched `Vdc` and `C_dc` before making final cross-domain damping claims.\n');
end
