function T = tune_gfm_only_stability()
% GFM-only stability tuning (no topology comparison).
% Target: find a parameter set that is stable in no-disturbance run.

root = fileparts(mfilename('fullpath'));
old = pwd; cleanup = onCleanup(@() cd(old)); cd(root);

hdrPath = fullfile(root, 'grid_forming_control.h');
hdrOrig = fileread(hdrPath);
restoreHdr = onCleanup(@() restore_file(hdrPath, hdrOrig)); %#ok<NASGU>

% Fixed operating point (same-object consistency)
vdcRef = 1200;
vacRef = 563;
pref = 1.0e6;
qref = 0;

% Coarse candidates (first pass, 6 s)
signList   = [1, -1];
tSyncList  = [0.2, 0.5, 1.0, 2.0];
kppList    = [5e-7, 1e-6, 2e-6];
kipList    = [1e-5, 2e-5, 4e-5];

rows = [];
caseId = 0;
for sgn = signList
    for tSync = tSyncList
        for kpp = kppList
            for kip = kipList
                caseId = caseId + 1;
                patch_header(hdrPath, tSync, kpp, kip, sgn);
                mex_rebuild(root);
                d = run_no_disturbance_diagnosis(false, 6.0, 0.0, struct( ...
                    'VdcRef_V', vdcRef, 'VacRef_V', vacRef, ...
                    'Pref_W', pref, 'Qref_var', qref, ...
                    'SkipMexCompile', true, ...
                    'UseNumericSfunParams', true));
                rows = [rows; pack_row(caseId, sgn, tSync, kpp, kip, d)]; %#ok<AGROW>
            end
        end
    end
end

T = array2table(rows, 'VariableNames', var_names());
T.score = score_fn(T);
T = sortrows(T, 'score');

% Re-check top 5 with 10 s
topN = min(5, height(T));
longRows = [];
for i = 1:topN
    patch_header(hdrPath, T.tSync(i), T.kpp(i), T.kip(i), T.pfSign(i));
    mex_rebuild(root);
    d10 = run_no_disturbance_diagnosis(false, 10.0, 0.0, struct( ...
        'VdcRef_V', vdcRef, 'VacRef_V', vacRef, ...
        'Pref_W', pref, 'Qref_var', qref, ...
        'SkipMexCompile', true, ...
        'UseNumericSfunParams', true));
    longRows = [longRows; pack_row(i, T.pfSign(i), T.tSync(i), T.kpp(i), T.kip(i), d10)]; %#ok<AGROW>
end
T10 = array2table(longRows, 'VariableNames', var_names());
T10.score = score_fn(T10);
T10 = sortrows(T10, 'score');

outDir = fullfile(root, 'Validation_Results');
if ~exist(outDir, 'dir'), mkdir(outDir); end
writetable(T, fullfile(outDir, 'tune_gfm_only_stability_6s.csv'));
writetable(T10, fullfile(outDir, 'tune_gfm_only_stability_10s_top5.csv'));

disp(T10);
fprintf('Saved: %s\n', fullfile(outDir, 'tune_gfm_only_stability_6s.csv'));
fprintf('Saved: %s\n', fullfile(outDir, 'tune_gfm_only_stability_10s_top5.csv'));
end

function names = var_names()
names = {'caseId','pfSign','tSync','kpp','kip', ...
    'Ppcc','PerrPu','Udc','UdcSlope','OmegaGSlope','OmegaTSlope','ThetaTwSlope','TshSlope'};
end

function r = pack_row(caseId, pfSign, tSync, kpp, kip, d)
r = [caseId, pfSign, tSync, kpp, kip, ...
    d.Ppcc_end_mean, d.Ppcc_target_error_pu, d.udc_end_mean, d.udc_end_slope, ...
    d.omega_g_end_slope, d.omega_t_end_slope, d.theta_tw_end_slope, d.T_sh_end_slope];
end

function s = score_fn(T)
% lower is better
s = abs(T.PerrPu) ...
    + abs(T.Udc - 1200)/1200 ...
    + abs(T.UdcSlope)/60 ...
    + abs(T.OmegaGSlope)*20 ...
    + abs(T.OmegaTSlope)*20 ...
    + abs(T.ThetaTwSlope)*500 ...
    + abs(T.TshSlope)/1e6;
end

function patch_header(hdrPath, tSync, kpp, kip, pfSign)
txt = fileread(hdrPath);
txt = regexprep(txt, '#define\s+PRESYN_SWITCH_TIME\s+[0-9\.eE+\-]+', ...
    sprintf('#define   PRESYN_SWITCH_TIME                    %.6g', tSync), 'once');
txt = regexprep(txt, '#define\s+GSI_PLOOP_KP\s+[0-9\.eE+\-]+f?', ...
    sprintf('#define   GSI_PLOOP_KP                        %.8g', kpp), 'once');
txt = regexprep(txt, '#define\s+GSI_PLOOP_KI\s+[0-9\.eE+\-]+f?', ...
    sprintf('#define   GSI_PLOOP_KI                        %.8g', kip), 'once');
txt = regexprep(txt, '#define\s+GSI_PF_LOOP_SIGN\s+[\-0-9\.eE+f]+', ...
    sprintf('#define   GSI_PF_LOOP_SIGN                    %.1ff', pfSign), 'once');
restore_file(hdrPath, txt);
end

function mex_rebuild(root)
setenv('MW_MINGW64_LOC', 'C:\mingw64');
evalin('base', 'clear mex');
evalin('base', 'bdclose(''all'')');
evalin('base', sprintf('cd(''%s''); mex main.c svpwm.c motorcontrol.c grid_forming_control.c;', strrep(root,'\','\\')));
end

function restore_file(path, txt)
fid = fopen(path, 'w');
fwrite(fid, txt);
fclose(fid);
end
