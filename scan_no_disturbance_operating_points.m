function T = scan_no_disturbance_operating_points()
% Scan no-disturbance operating points to find feasible steady candidates:
% - positive Pmeas/Ppcc
% - smaller Udc slope
% - better mechanical end slopes

root = fileparts(mfilename('fullpath'));
old = pwd;
cleanup = onCleanup(@() cd(old));
cd(root);

mdl = 'Grid_Forming_PMSG';
load_system(mdl);
ctrlBlk = [mdl '/MOTOR_CONTROL1'];
brkBlk = [mdl '/Three-Phase Breaker'];

origCtrl = get_param(ctrlBlk, 'MaskValues');
origBrk = get_param(brkBlk, 'MaskValues');

prefList = [5e5, 1.0e6];
vdcList = [1200, 1250];
breakerModes = {'sync', 'forced_closed'};

rows = {};
r = 0;

for ib = 1:numel(breakerModes)
    for iv = 1:numel(vdcList)
        for ip = 1:numel(prefList)
            r = r + 1;

            ctrlVals = origCtrl;
            ctrlVals{1} = num2str(prefList(ip));
            ctrlVals{4} = num2str(vdcList(iv));
            set_param(ctrlBlk, 'MaskValues', ctrlVals);

            brkVals = origBrk;
            if strcmp(breakerModes{ib}, 'forced_closed')
                brkVals{1} = 'closed'; % InitialState
                brkVals{6} = 'off';    % External
            else
                brkVals{1} = 'open';
                brkVals{6} = 'on';
                brkVals{5} = '[1/60 5/60]'; % SwitchTimes
            end
            set_param(brkBlk, 'MaskValues', brkVals);

            d = run_no_disturbance_diagnosis(false, 1.5, 0.0);

            % Score: prefer positive power, low Udc drift, low mech slopes
            score = ...
                2.0 * abs(d.Ppcc_target_error_pu) + ...
                0.02 * abs(d.udc_end_slope) + ...
                5.0 * abs(d.omega_g_end_slope) + ...
                0.5 * abs(d.theta_tw_end_slope);

            rows(r,:) = { ...
                breakerModes{ib}, prefList(ip), vdcList(iv), ...
                d.Ppcc_end_mean, d.pmeas_end_mean, d.udc_end_mean, ...
                d.Ppcc_target_error_pu, d.udc_end_slope, ...
                d.omega_g_end_slope, d.theta_tw_end_slope, ...
                d.primary_cause_hint, score}; %#ok<AGROW>
        end
    end
end

set_param(ctrlBlk, 'MaskValues', origCtrl);
set_param(brkBlk, 'MaskValues', origBrk);

T = cell2table(rows, 'VariableNames', { ...
    'breaker_mode','Pref_W','VdcRef_V', ...
    'Ppcc_W','Pmeas_W','UdcMean_V', ...
    'PpccErr_pu','UdcSlope_Vps','OmegaGSlope','ThetaTwSlope', ...
    'CauseHint','Score'});
T = sortrows(T, 'Score', 'ascend');

outDir = fullfile(root, 'Validation_Results');
if ~exist(outDir, 'dir'), mkdir(outDir); end
writetable(T, fullfile(outDir, 'scan_no_disturbance_operating_points.csv'));
save(fullfile(outDir, 'scan_no_disturbance_operating_points.mat'), 'T');

disp(T(1:min(12,height(T)),:));
end
