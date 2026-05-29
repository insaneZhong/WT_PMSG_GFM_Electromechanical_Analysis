function rec = export_schemeA_c_tuning_from_small_signal()
% Export Scheme-A tuning hints from small-signal Parameters.mat
% to nonlinear C controller parameter suggestions.

root = fileparts(mfilename('fullpath'));
ssmMat = locate_ssm_parameters_mat();
s = load(ssmMat);

rec = struct();

% Machine-side current PI
rec.MOTOR_ID_KP = s.k_pm;
rec.MOTOR_ID_KI = s.k_im * 0.00025; % Ts-integrated form in C
rec.MOTOR_IQ_KP = rec.MOTOR_ID_KP;
rec.MOTOR_IQ_KI = rec.MOTOR_ID_KI;

% Grid-side current PI
rec.GSC_ID_KP = s.k_pi;
rec.GSC_ID_KI = s.k_ii * 0.00025;
rec.GSC_IQ_KP = rec.GSC_ID_KP;
rec.GSC_IQ_KI = rec.GSC_ID_KI;

% Grid-side voltage PI
rec.GSC_V_KP = s.k_pv;
rec.GSC_V_KI = s.k_iv * 0.00025;

% DC-link/active-power loop
if isfield(s, 'k_pdc_gwt') && isfield(s, 'k_idc_gwt')
    rec.GSC_P_KP = s.k_pdc_gwt;
    rec.GSC_P_KI = s.k_idc_gwt * 0.00025;
else
    rec.GSC_P_KP = s.k_pdc;
    rec.GSC_P_KI = s.k_idc * 0.00025;
end

% VSG-equivalent mapping
rec.VSG_H = s.h;
rec.VSG_MP = s.mp;
rec.VSG_W0 = s.wn;
rec.VSG_M_equiv = 2 * s.h * s.wn;
rec.VSG_D_equiv = 1 / s.mp;

outDir = fullfile(root, 'Validation_Results');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

names = fieldnames(rec);
vals = zeros(numel(names),1);
for i = 1:numel(names)
    vals(i) = rec.(names{i});
end
T = table(names, vals, 'VariableNames', {'Key','Value'});
writetable(T, fullfile(outDir, 'schemeA_c_tuning_from_small_signal.csv'));

md = fullfile(outDir, 'schemeA_c_tuning_from_small_signal.md');
fid = fopen(md, 'w');
fprintf(fid, '# 方案A：小信号到C控制器参数映射（首版）\n\n');
fprintf(fid, '- Source MAT: `%s`\n\n', ssmMat);
fprintf(fid, '| Key | Suggested value |\n|---|---:|\n');
for i = 1:numel(names)
    fprintf(fid, '| %s | %.8g |\n', names{i}, vals(i));
end
fprintf(fid, '\n> Note: `*_KI` values already include first-pass Ts conversion for the C discrete realization.\n');
fclose(fid);

disp(T);
fprintf('Saved: %s\n', md);
end

