function files = plot_5mw_gfm_characterization()
%PLOT_5MW_GFM_CHARACTERIZATION Create compact baseline and response figures.
root = fileparts(mfilename('fullpath'));
outDir = fullfile(root,'Validation_Results','5MW_GFM_Characterization');
loaded = load(fullfile(root,'Validation_Results', ...
    'liu2024_5mw_active_run.mat'),'series');
s = loaded.series;

fig = figure('Visible','off','Color','w','Position',[100 100 1050 720]);
tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile; plot(s.stage4_Ppcc.Time,s.stage4_Ppcc.Data/1e6,'LineWidth',1); ...
    yline(5,'--'); grid on; ylabel('P_{PCC} (MW)'); xlabel('t (s)');
nexttile; plot(s.stage4_Udc.Time,s.stage4_Udc.Data,'LineWidth',1); ...
    yline(1500,'--'); grid on; ylabel('U_{dc} (V)'); xlabel('t (s)');
nexttile; plot(s.tm_omega_t.Time,s.tm_omega_t.Data,'LineWidth',1); hold on; ...
    plot(s.tm_omega_g.Time,s.tm_omega_g.Data,'LineWidth',1); grid on; ...
    ylabel('\omega (rad/s)'); xlabel('t (s)'); legend('\omega_t','\omega_g','Location','best');
nexttile; plot(s.tm_theta_tw.Time,s.tm_theta_tw.Data,'LineWidth',1); grid on; ...
    ylabel('\theta_{tw} (rad)'); xlabel('t (s)');
title(tl,'5 MW GFM PMSG: 60 s rated baseline');
files.baseline = fullfile(outDir,'rated_baseline_60s.png');
exportgraphics(fig,files.baseline,'Resolution',180); close(fig);

T = readtable(fullfile(outDir,'verified_screen_summary.csv'),'TextType','string');
fig = figure('Visible','off','Color','w','Position',[100 100 1050 480]);
tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
pf = T(T.category=="P-f",:);
pfLabels = replace(replace(pf.perturbation,"_Hz"," Hz"),"_"," ");
nexttile; bar(categorical(pfLabels),pf.P_post_MW-pf.P_pre_MW); ...
    grid on; ylabel('\DeltaP (MW)'); title('P-f response');
qv = T(T.category=="Q-V",:);
qvLabels = replace(replace(qv.perturbation,"_percent_grid_voltage","% grid voltage"),"_"," ");
nexttile; bar(categorical(qvLabels),qv.Q_post_Mvar-qv.Q_pre_Mvar); ...
    grid on; ylabel('\DeltaQ (Mvar)'); title('Q-V response');
files.responses = fullfile(outDir,'pf_qv_screen_responses.png');
exportgraphics(fig,files.responses,'Resolution',180); close(fig);
disp(files);
end
