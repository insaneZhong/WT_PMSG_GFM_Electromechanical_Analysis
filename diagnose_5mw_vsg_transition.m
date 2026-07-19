function transition = diagnose_5mw_vsg_transition(stopTime,simulationMode)
%DIAGNOSE_5MW_VSG_TRANSITION Half-second cross sections around VSG handover.
arguments
    stopTime (1,1) double {mustBeGreaterThan(stopTime,15)} = 21
    simulationMode (1,1) string = "accelerator"
end

root = fileparts(mfilename('fullpath'));
p = Liu2024_5MW_Params();
out = run_liu2024_5mw_experiment(StopTime=stopTime, ...
    PowerRamp=p.pref_ramp_slope, ...
    SimulationMode=simulationMode);
diagTs = out.get('msc_diagnostic_vector');
t = diagTs.Time(:);
D = squeeze(diagTs.Data);
if size(D,1) ~= numel(t), D = D.'; end
udcTs = out.get('stage4_Udc');
udc = interp1(udcTs.Time(:),squeeze(udcTs.Data),t,'linear','extrap');

edges = (14.5:0.5:stopTime).';
n = numel(edges)-1;
transition = table('Size',[n 9], ...
    'VariableTypes',repmat("double",1,9), ...
    'VariableNames',{'t_start_s','t_end_s','P_ref_W','P_pcc_W', ...
    'Q_pcc_var','Udc_V','w_ref_Hz','theta_ref_rad','gsc_current_ref_A'});
for k = 1:n
    idx = t >= edges(k) & t < edges(k+1);
    transition{k,:} = [edges(k),edges(k+1),mean(D(idx,13)), ...
        mean(D(idx,14)),mean(D(idx,19)),mean(udc(idx)), ...
        mean(D(idx,15))/(2*pi),circMean(D(idx,16)), ...
        max(hypot(D(idx,23),D(idx,27)))];
end
writetable(transition,fullfile(root,'Validation_Results', ...
    '5MW_GFM_Characterization','vsg_transition_cross_sections.csv'));
disp(transition);
end

function a = circMean(theta)
a = atan2(mean(sin(theta)),mean(cos(theta)));
end
