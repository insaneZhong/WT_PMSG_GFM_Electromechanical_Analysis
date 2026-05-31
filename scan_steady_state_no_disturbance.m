function results = scan_steady_state_no_disturbance()
% Scan k_pdc/k_idc and D_sh for no-disturbance steady-state convergence.
% Criterion: minimize end-window trends of omega_g and T_sh.

root = fileparts(mfilename('fullpath'));
oldFolder = pwd;
cleanup = onCleanup(@() cd(oldFolder));
cd(root);

run('GFM_MWT_Nonlinear_Params.m');
mdl = 'Grid_Forming_PMSG';
load_system(mdl);

% Baseline from current nonlinear parameter script.
k_pdc_list = [0.35, 0.50, 0.70];
k_idc_list = [20, 50, 80];
Dsh_scale_list = [0.8, 1.2];

simStop = 6.0;      % s, fast first-pass scan
tailWin = 0.8;      % s, end-window for trend checks

results = table();
row = 0;

for ikp = 1:numel(k_pdc_list)
    for iki = 1:numel(k_idc_list)
        for ids = 1:numel(Dsh_scale_list)
            row = row + 1;
            k_pdc = k_pdc_list(ikp);
            k_idc = k_idc_list(iki);
            D_sh_try = D_sh * Dsh_scale_list(ids);

            assignin('base', 'k_pdc', k_pdc);
            assignin('base', 'k_idc', k_idc);
            assignin('base', 'D_sh', D_sh_try);
            assignin('base', 'wind_step_mps', 0.0);

            in = Simulink.SimulationInput(mdl);
            in = in.setModelParameter('StopTime', num2str(simStop), ...
                'ReturnWorkspaceOutputs', 'on', ...
                'SimulationMode', 'normal');

            try
                out = sim(in);
                if ~isfield(out, 'omega_g') || ~isfield(out, 'omega_t') || ~isfield(out, 'T_sh')
                    error('Missing omega_g/omega_t/T_sh outputs.');
                end

                og = out.omega_g;
                ot = out.omega_t;
                tsh = out.T_sh;

                idx_og = og.Time >= (og.Time(end) - tailWin);
                idx_ot = ot.Time >= (ot.Time(end) - tailWin);
                idx_ts = tsh.Time >= (tsh.Time(end) - tailWin);

                p_og = polyfit(og.Time(idx_og), og.Data(idx_og), 1);
                p_ot = polyfit(ot.Time(idx_ot), ot.Data(idx_ot), 1);
                p_ts = polyfit(tsh.Time(idx_ts), tsh.Data(idx_ts), 1);

                omega_g_slope = p_og(1);
                omega_t_slope = p_ot(1);
                Tsh_slope = p_ts(1);

                score = abs(omega_g_slope) + abs(omega_t_slope) + 1e-6 * abs(Tsh_slope);
                status = "ok";
                errmsg = "";
            catch ME
                omega_g_slope = NaN;
                omega_t_slope = NaN;
                Tsh_slope = NaN;
                score = Inf;
                status = "failed";
                errmsg = string(ME.message);
            end

            results(row, :) = table( ...
                k_pdc, k_idc, D_sh_try, Dsh_scale_list(ids), ...
                omega_g_slope, omega_t_slope, Tsh_slope, score, status, errmsg, ...
                'VariableNames', {'k_pdc','k_idc','D_sh','D_sh_scale', ...
                                  'omega_g_slope','omega_t_slope','Tsh_slope','score','status','errmsg'});
            fprintf('[%02d/%02d] kp=%.3g ki=%.3g Dshx=%.2f -> score=%.6g status=%s\n', ...
                row, numel(k_pdc_list)*numel(k_idc_list)*numel(Dsh_scale_list), ...
                k_pdc, k_idc, Dsh_scale_list(ids), score, status);
        end
    end
end

results = sortrows(results, 'score', 'ascend');

outDir = fullfile(root, 'Validation_Results');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
writetable(results, fullfile(outDir, 'steady_state_scan_results.csv'));
save(fullfile(outDir, 'steady_state_scan_results.mat'), 'results', ...
    'k_pdc_list', 'k_idc_list', 'Dsh_scale_list', 'simStop', 'tailWin');

disp('Top 10 parameter sets:');
disp(results(1:min(10,height(results)), :));

close_system(mdl, 0);
end
