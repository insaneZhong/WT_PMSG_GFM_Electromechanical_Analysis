function results = run_5mw_gfm_characterization(mode, caseNames, simulationMode)
%RUN_5MW_GFM_CHARACTERIZATION Parameterized P-f, Q-V and SCR validation.
% All cases use Grid_Forming_PMSG5MW_Liu2024_TwoMass.slx.  No model copies
% are created and only compact CSV/MAT summaries are retained.

arguments
    mode (1,1) string {mustBeMember(mode,["screen","full"])} = "screen"
    caseNames string = strings(0,1)
    simulationMode (1,1) string ...
        {mustBeMember(simulationMode,["normal","accelerator"])} = "normal"
end

root = fileparts(mfilename('fullpath'));
outputDir = fullfile(root,'Validation_Results','5MW_GFM_Characterization');
if ~isfolder(outputDir)
    mkdir(outputDir);
end
p = Liu2024_5MW_Params();

if mode == "screen"
    stopTime = 30;
    eventTime = 25;
else
    stopTime = 60;
    eventTime = 45;
end

cases = defineCases(p,eventTime);
if ~isempty(caseNames)
    keep = ismember(string({cases.name}),caseNames);
    unknown = setdiff(caseNames,string({cases.name}));
    assert(isempty(unknown),'Unknown case name(s): %s',strjoin(unknown,', '));
    cases = cases(keep);
end
assert(~isempty(cases),'No characterization cases selected.');

rows = repmat(emptyRow(),numel(cases),1);
for k = 1:numel(cases)
    c = cases(k);
    fprintf('Running 5 MW GFM %s case %d/%d: %s\n', ...
        mode,k,numel(cases),c.name);
    try
        out = run_liu2024_5mw_experiment( ...
            'StopTime',stopTime,'PowerRamp',p.pref_ramp_slope, ...
            'Qref_var',c.qref_var,'SCR',c.scr, ...
            'GridVoltagePeak_V',p.V_phase_peak, ...
            'GridFrequency_Hz',c.grid_frequency_hz, ...
            'GridVariationEntity',c.variation_entity, ...
            'GridVariationType','Step', ...
            'GridVariationStep',c.variation_step, ...
            'GridVariationTiming',[eventTime 1e9], ...
            'SimulationMode',simulationMode);
        rows(k) = summarizeCase(out,c,p,mode,eventTime,stopTime);
    catch ME
        rows(k) = emptyRow();
        rows(k).case_name = string(c.name);
        rows(k).category = string(c.category);
        rows(k).mode = mode;
        rows(k).status = "ERROR";
        rows(k).error_message = string(ME.message);
        warning('GFM5MW:CaseFailed','Case %s failed: %s',c.name,ME.message);
    end
    caseResult = applyDirectionChecks(struct2table(rows(k)));
    writetable(caseResult,fullfile(outputDir,sprintf('%s_%s.csv', ...
        char(caseResult.case_name),char(mode))));
    updateAccumulatedResults(caseResult,outputDir,mode);
    results = struct2table(rows(1:k));
    results = applyDirectionChecks(results);
    writetable(results,fullfile(outputDir, ...
        sprintf('gfm_characterization_%s.csv',mode)));
    save(fullfile(outputDir,sprintf('gfm_characterization_%s.mat',mode)), ...
        'results','cases','mode','eventTime','stopTime','simulationMode');
    resetCaseRuntime();
end

results = applyDirectionChecks(struct2table(rows));
writetable(results,fullfile(outputDir, ...
    sprintf('gfm_characterization_%s.csv',mode)));
writetable(results(results.category=="P-f",:), ...
    fullfile(outputDir,sprintf('pf_characterization_%s.csv',mode)));
writetable(results(results.category=="Q-V",:), ...
    fullfile(outputDir,sprintf('qv_characterization_%s.csv',mode)));
writetable(results(results.category=="SCR",:), ...
    fullfile(outputDir,sprintf('scr_characterization_%s.csv',mode)));
save(fullfile(outputDir,sprintf('gfm_characterization_%s.mat',mode)), ...
    'results','cases','mode','eventTime','stopTime','simulationMode');

fprintf('5 MW GFM %s summary: %d/%d cases passed available checks.\n', ...
    mode,sum(results.all_available_checks_pass),height(results));
end

function cases = defineCases(p,eventTime) %#ok<INUSD>
base = struct('name','','category','','scr',p.SCR,'qref_var',0, ...
    'grid_frequency_hz',p.f_base,'variation_entity','Amplitude', ...
    'variation_step',0);
cases = repmat(base,9,1);
cases(1) = setCase(base,'baseline','Baseline');
cases(2) = setCase(base,'pf_fminus_0p002','P-f', ...
    'variation_entity','Frequency','variation_step',-0.002);
cases(3) = setCase(base,'pf_fplus_0p002','P-f', ...
    'variation_entity','Frequency','variation_step',0.002);
cases(4) = setCase(base,'qv_qminus_0p1','Q-V','qref_var',-0.1*p.S_base);
cases(5) = setCase(base,'qv_qplus_0p1','Q-V','qref_var',0.1*p.S_base);
cases(6) = setCase(base,'qv_vminus_5pct','Q-V', ...
    'variation_entity','Amplitude','variation_step',-0.05);
cases(7) = setCase(base,'qv_vplus_5pct','Q-V', ...
    'variation_entity','Amplitude','variation_step',0.05);
cases(8) = setCase(base,'scr8','SCR','scr',8);
cases(9) = setCase(base,'scr2','SCR','scr',2);
end

function c = setCase(c,name,category,varargin)
c.name = name;
c.category = category;
for k = 1:2:numel(varargin)
    c.(varargin{k}) = varargin{k+1};
end
end

function row = summarizeCase(out,c,p,mode,eventTime,stopTime)
diagTs = out.get('msc_diagnostic_vector');
t = diagTs.Time(:);
D = squeeze(diagTs.Data);
if size(D,1) ~= numel(t) && size(D,2) == numel(t)
    D = D.';
end
preIdx = t >= eventTime-2 & t < eventTime;
postIdx = t >= stopTime-2;
assert(any(preIdx) && any(postIdx),'Pre/post analysis windows are empty.');

udc = sampleSeries(out.get('stage4_Udc'),t);
ppcc = sampleSeries(out.get('stage4_Ppcc'),t);
omegaT = sampleSeries(out.get('tm_omega_t'),t);
omegaG = sampleSeries(out.get('tm_omega_g'),t);
theta = sampleSeries(out.get('tm_theta_tw'),t);

q = D(:,19);
wVsg = D(:,15);
pccVoltage = hypot(D(:,22),D(:,26));
pccRelativeAngle = unwrap(atan2(D(:,26),D(:,22)));
gscMod = 1.5*hypot(D(:,17),D(:,18))./max(udc,1);
mscMod = abs(D(:,37));
gscIRef = hypot(D(:,23),D(:,27));
gscI = hypot(D(:,24),D(:,30));
mscIqRef = abs(D(:,31));
mscIq = abs(D(:,33));

row = emptyRow();
row.case_name = string(c.name);
row.category = string(c.category);
row.mode = mode;
row.status = "OK";
row.scr = c.scr;
row.qref_var = c.qref_var;
row.frequency_step_hz = double(strcmp(c.variation_entity,'Frequency'))*c.variation_step;
row.voltage_step_pu = double(strcmp(c.variation_entity,'Amplitude'))*c.variation_step;
row.P_pre_W = mean(ppcc(preIdx));
row.P_post_W = mean(ppcc(postIdx));
row.delta_P_W = row.P_post_W-row.P_pre_W;
row.Q_pre_var = mean(q(preIdx));
row.Q_post_var = mean(q(postIdx));
row.delta_Q_var = row.Q_post_var-row.Q_pre_var;
row.Udc_pre_V = mean(udc(preIdx));
row.Udc_post_V = mean(udc(postIdx));
row.PCC_voltage_pre_V = mean(pccVoltage(preIdx));
row.PCC_voltage_post_V = mean(pccVoltage(postIdx));
row.VSG_frequency_pre_Hz = mean(wVsg(preIdx))/(2*pi);
row.VSG_frequency_post_Hz = mean(wVsg(postIdx))/(2*pi);
gridPlusPre = (mean(wVsg(preIdx))+ ...
    signalSlope(t(preIdx),pccRelativeAngle(preIdx)))/(2*pi);
gridPlusPost = (mean(wVsg(postIdx))+ ...
    signalSlope(t(postIdx),pccRelativeAngle(postIdx)))/(2*pi);
gridMinusPre = (mean(wVsg(preIdx))- ...
    signalSlope(t(preIdx),pccRelativeAngle(preIdx)))/(2*pi);
gridMinusPost = (mean(wVsg(postIdx))- ...
    signalSlope(t(postIdx),pccRelativeAngle(postIdx)))/(2*pi);
expectedGridFrequency = p.f_base + row.frequency_step_hz;
if abs(gridPlusPost-expectedGridFrequency) <= ...
        abs(gridMinusPost-expectedGridFrequency)
    row.grid_frequency_est_pre_Hz = gridPlusPre;
    row.grid_frequency_est_post_Hz = gridPlusPost;
else
    row.grid_frequency_est_pre_Hz = gridMinusPre;
    row.grid_frequency_est_post_Hz = gridMinusPost;
end
row.omega_t_post_radps = mean(omegaT(postIdx));
row.omega_g_post_radps = mean(omegaG(postIdx));
row.P_slope_post_Wps = signalSlope(t(postIdx),ppcc(postIdx));
row.Udc_slope_post_Vps = signalSlope(t(postIdx),udc(postIdx));
row.omega_t_slope_post_radps2 = signalSlope(t(postIdx),omegaT(postIdx));
row.omega_g_slope_post_radps2 = signalSlope(t(postIdx),omegaG(postIdx));
row.theta_slope_post_radps = signalSlope(t(postIdx),theta(postIdx));
row.gsc_modulation_max = max(gscMod(postIdx));
row.msc_modulation_max = max(mscMod(postIdx));
row.gsc_current_ref_max_A = max(gscIRef(postIdx));
row.gsc_current_max_A = max(gscI(postIdx));
row.msc_iq_ref_max_A = max(mscIqRef(postIdx));
row.msc_iq_max_A = max(mscIq(postIdx));
row.gsc_limit_duty = mean(gscMod(postIdx)>=0.995*p.gsc_modulation_limit | ...
    gscIRef(postIdx)>=0.995*p.gsc_current_vector_limit);
row.msc_limit_duty = mean(mscIqRef(postIdx)>=0.995*p.motor_current_limit);

    finiteValues = [row.scr,row.qref_var,row.frequency_step_hz, ...
        row.voltage_step_pu,row.P_pre_W,row.P_post_W,row.delta_P_W, ...
        row.Q_pre_var,row.Q_post_var,row.delta_Q_var,row.Udc_pre_V, ...
        row.Udc_post_V,row.PCC_voltage_pre_V,row.PCC_voltage_post_V, ...
        row.VSG_frequency_pre_Hz,row.VSG_frequency_post_Hz, ...
        row.grid_frequency_est_pre_Hz,row.grid_frequency_est_post_Hz, ...
        row.omega_t_post_radps,row.omega_g_post_radps, ...
        row.P_slope_post_Wps,row.Udc_slope_post_Vps, ...
        row.omega_t_slope_post_radps2,row.omega_g_slope_post_radps2, ...
        row.theta_slope_post_radps,row.gsc_modulation_max, ...
        row.msc_modulation_max,row.gsc_current_ref_max_A, ...
        row.gsc_current_max_A,row.msc_iq_ref_max_A,row.msc_iq_max_A, ...
        row.gsc_limit_duty,row.msc_limit_duty];
    row.numeric_finite_pass = all(isfinite(finiteValues));
row.limit_pass = row.gsc_modulation_max < p.gsc_modulation_limit && ...
    row.msc_modulation_max < 0.90 && ...
    row.gsc_current_max_A <= p.grid_current_limit && ...
    row.msc_iq_max_A <= p.motor_current_limit && ...
    row.gsc_limit_duty == 0 && row.msc_limit_duty == 0;
row.dynamic_stability_pass = abs(row.Udc_post_V-p.Vdc)/p.Vdc < 0.10 && ...
    abs(row.omega_t_slope_post_radps2) < 0.02 && ...
    abs(row.omega_g_slope_post_radps2) < 0.02 && ...
    abs(row.theta_slope_post_radps) < 1e-3;
if mode == "full"
    row.dynamic_stability_pass = row.dynamic_stability_pass && ...
        abs(row.Udc_post_V-p.Vdc)/p.Vdc < 0.02 && ...
        abs(row.Udc_slope_post_Vps) < 5 && ...
        abs(row.P_slope_post_Wps) < 5e3 && ...
        abs(row.omega_t_slope_post_radps2) < 0.01 && ...
        abs(row.omega_g_slope_post_radps2) < 0.01 && ...
        abs(row.theta_slope_post_radps) < 5e-4;
end
row.response_direction_pass = NaN;
row.all_available_checks_pass = row.numeric_finite_pass && ...
    row.limit_pass && row.dynamic_stability_pass;
end

function results = applyDirectionChecks(results)
if isempty(results)
    return
end
baselineIndex = find(results.case_name=="baseline" & results.status=="OK",1);
for k = 1:height(results)
    if results.status(k) ~= "OK"
        results.response_direction_pass(k) = 0;
        results.all_available_checks_pass(k) = 0;
        continue
    end
    if results.frequency_step_hz(k) ~= 0
        actualFrequencyChange = results.grid_frequency_est_post_Hz(k)- ...
            results.grid_frequency_est_pre_Hz(k);
        results.response_direction_pass(k) = ...
            results.delta_P_W(k)*actualFrequencyChange < 0;
    elseif results.voltage_step_pu(k) ~= 0
        results.response_direction_pass(k) = ...
            results.delta_Q_var(k)*results.voltage_step_pu(k) < 0;
    elseif results.qref_var(k) ~= 0 && ~isempty(baselineIndex)
        results.response_direction_pass(k) = ...
            (results.Q_post_var(k)-results.Q_post_var(baselineIndex))* ...
            results.qref_var(k) > 0;
    else
        results.response_direction_pass(k) = NaN;
    end
    if ~isnan(results.response_direction_pass(k))
        results.all_available_checks_pass(k) = ...
            results.all_available_checks_pass(k) && ...
            logical(results.response_direction_pass(k));
    end
end
end

function row = emptyRow()
row = struct( ...
    'case_name',"",'category',"",'mode',"",'status',"PENDING", ...
    'scr',NaN,'qref_var',NaN,'frequency_step_hz',NaN, ...
    'voltage_step_pu',NaN,'P_pre_W',NaN,'P_post_W',NaN,'delta_P_W',NaN, ...
    'Q_pre_var',NaN,'Q_post_var',NaN,'delta_Q_var',NaN, ...
    'Udc_pre_V',NaN,'Udc_post_V',NaN, ...
    'PCC_voltage_pre_V',NaN,'PCC_voltage_post_V',NaN, ...
    'VSG_frequency_pre_Hz',NaN,'VSG_frequency_post_Hz',NaN, ...
    'grid_frequency_est_pre_Hz',NaN,'grid_frequency_est_post_Hz',NaN, ...
    'omega_t_post_radps',NaN,'omega_g_post_radps',NaN, ...
    'P_slope_post_Wps',NaN,'Udc_slope_post_Vps',NaN, ...
    'omega_t_slope_post_radps2',NaN,'omega_g_slope_post_radps2',NaN, ...
    'theta_slope_post_radps',NaN,'gsc_modulation_max',NaN, ...
    'msc_modulation_max',NaN,'gsc_current_ref_max_A',NaN, ...
    'gsc_current_max_A',NaN,'msc_iq_ref_max_A',NaN,'msc_iq_max_A',NaN, ...
    'gsc_limit_duty',NaN,'msc_limit_duty',NaN, ...
    'numeric_finite_pass',false,'limit_pass',false, ...
    'dynamic_stability_pass',false,'response_direction_pass',NaN, ...
    'all_available_checks_pass',false,'error_message',"");
end

function y = sampleSeries(ts,t)
sourceT = ts.Time(:);
sourceY = squeeze(ts.Data);
sourceY = sourceY(:);
[sourceT,uniqueIndex] = unique(sourceT,'stable');
sourceY = sourceY(uniqueIndex);
y = interp1(sourceT,sourceY,t,'linear','extrap');
end

function value = signalSlope(t,y)
fitValue = polyfit(t-t(1),y,1);
value = fitValue(1);
end

function resetCaseRuntime()
model = 'Grid_Forming_PMSG5MW_Liu2024_TwoMass';
if bdIsLoaded(model)
    close_system(model,0);
end
clear('main_legacy_liu2024_5mw_stable');
end

function updateAccumulatedResults(caseResult,outputDir,mode)
file = fullfile(outputDir,sprintf( ...
    'gfm_characterization_%s_accumulated.csv',mode));
if isfile(file)
    accumulated = readtable(file,'TextType','string');
    accumulated(accumulated.case_name==caseResult.case_name,:) = [];
    accumulated = [accumulated; caseResult]; %#ok<AGROW>
else
    accumulated = caseResult;
end
accumulated = sortrows(accumulated,'case_name');
writetable(accumulated,file);
end
