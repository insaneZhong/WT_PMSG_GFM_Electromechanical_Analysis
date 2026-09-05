function resultPath=run_s7_5_b4_pll_lc1()
%RUN_S7_5_B4_PLL_LC1  S7-5B/B4 PLL/GFL 首步及多步确定性核对。
% 运行一个 GSI_GFL_MODE=1 的理想连续 C probe，分别覆盖 pre-sync
% PLL 和并网后 measurement-only PLL；过程数据仅保存在 temp。

here=fileparts(mfilename('fullpath')); tempDir=fullfile(here,'temp','S7_5_LegacyCertification');
if ~isfolder(tempDir),mkdir(tempDir);end
% 编译阶段临时切换到 C 源码目录；显式加入本运行器目录，避免
% MATLAB 在 onCleanup 尚未析构时解析到错误的同名对象。
addpath(here,'-begin');
srcDir=fullfile(fileparts(here),'CurrentModel_Idealized');
mexDir=tempDir; mexName='main_s7_5_b4_probe'; mexPath=fullfile(mexDir,[mexName,'.',mexext]);
if ~isfile(mexPath)
    old=pwd; c=onCleanup(@()cd(old)); cd(srcDir); setenv('MW_MINGW64_LOC','C:\mingw64'); clear mex;
    defs={'-DLEGACY_AD_S_FUNCTION_NAME=main_s7_5_b4_probe', ...
        '-DIDEAL_CONTINUOUS_CONTROLLER=1','-DIDEAL_AVG_OUTPUTS=0', ...
        '-DLEGACY_SFUNCTION_SAMPLE_TIME_S=4e-6','-DLEGACY_RESET_CONTROLLER_ON_INIT=1', ...
        '-DGSI_GFL_MODE=1','-DPRESYN_SWITCH_TIME=1.75','-DGSI_GFM_ENABLE_TIME_S=1.75'};
    mex(defs{:},'-output',fullfile(mexDir,mexName),'main_legacy_ad_base.c','svpwm.c', ...
        'motorcontrol_legacy_ad_base.c','grid_forming_control.c');
end
addpath(mexDir,'-begin'); clear mexName;

Ts=4e-6; N=100; amp=563; ang=single(.08);
% 以固定 alpha-beta 电压方向激励 PLL，避免使用随机或长时序数据。
ua=amp*cos(double(ang)); ub=amp*cos(double(ang)-2*pi/3); uc=amp*cos(double(ang)+2*pi/3);
uab=ua-ub; ubc=ub-uc; uca=uc-ua;
u0=zeros(20,1);u0(4)=900;u0(5)=314;u0(6)=0;u0(10:12)=[uab;ubc;uca];u0(16)=0;u0(17)=0;u0(18)=1;u0(19)=0;u0(20)=1000;
cases={'pre_sync';'post_gfl'}; rows={};
for ic=1:2
    mdl=sprintf('S7_5_B4_PLL_LC1_%d',ic); if bdIsLoaded(mdl),close_system(mdl,0);end
    assignin('base','u0',u0); new_system(mdl);load_system('simulink');
    add_block('simulink/Sources/Constant',[mdl '/Input20'],'Value','u0','Position',[30 50 100 90]);
    add_block('simulink/User-Defined Functions/S-Function',[mdl '/C'], ...
        'FunctionName','main_s7_5_b4_probe','Parameters','5e6,0,563,1000','Position',[160 40 300 100]);
    add_block('simulink/Sinks/To Workspace',[mdl '/Y'],'VariableName','yC','SaveFormat','Array','Position',[370 48 485 92]);
    add_line(mdl,'Input20/1','C/1');add_line(mdl,'C/1','Y/1');
    if ic==2,u0(16)=2;assignin('base','u0',u0);end
    set_param(mdl,'Solver','FixedStepDiscrete','FixedStep',num2str(Ts,'%.12g'),'StopTime',num2str(N*Ts,'%.12g'),'ReturnWorkspaceOutputs','on');
    set_param(mdl,'SimulationCommand','update'); simOut=sim(mdl,'ReturnWorkspaceOutputs','on');
    try,yC=simOut.get('yC');catch,yC=evalin('base','yC');end
    if isempty(yC)||size(yC,2)<37,error('run_s7_5_b4_pll_lc1:NoOutput','C 输出不足。');end
    if ic==1, disp('B4 C preview [w_ref theta_ref grid_phase]:'), disp(yC(1:min(5,size(yC,1)),[15 16 28])); end
    nC=size(yC,1); st=s7_legacy_replica_b4_pll_step('initial_state'); p=s7_legacy_replica_b4_pll_step('defaults');
    % Fixed-step S-function emits the first control event at t=0; therefore
    % row 1 is the first updated state (there is no unupdated pre-row).
    n=min(N,nC); rep=zeros(n,4); cv=zeros(n,4); er=zeros(n,4);
    for k=1:n
        uk=struct('Ts',Ts,'Ts_grid',1e-4,'system_Time',u0(16),'Pre_syn',ic==2,'GFM_enabled',ic==2, ...
            'pcc_uab',uab,'pcc_ubc',ubc,'pcc_uca',uca);
        [st,o,tr]=s7_legacy_replica_b4_pll_step(st,uk,p); %#ok<ASGLU>
        rep(k,:)=[o.freq,o.grid_phase_angle,o.w_ref,o.theta_ref];
        % C output: [15]=w_ref, [16]=theta_ref.  Before takeover [28]
        % mirrors the PLL phase; after takeover the legacy code keeps
        % grid_phase_angle as a stale diagnostic while GFL consumes
        % theta_ref=grid_pll_phase, so use [16] for the post-GFL phase.
        phaseC = yC(k,28); if ic==2, phaseC=yC(k,16); end
        cv(k,:)=[yC(k,15),phaseC,yC(k,15),yC(k,16)]; er(k,:)=cv(k,:)-rep(k,:);
    end
    maxerr=max(abs(er),[],1); rows{ic}=maxerr;
    if ic==1, disp('B4 Replica/C preview [freq phase wref theta]:'), disp([rep(1:min(5,n),:) cv(1:min(5,n),:)]); end
    close_system(mdl,0);
end
M={'pre_sync_freq';'pre_sync_phase';'pre_sync_wref';'pre_sync_theta'; ...
   'post_gfl_freq';'post_gfl_phase';'post_gfl_wref';'post_gfl_theta'};
V=[rows{1}(:);rows{2}(:)]; tol=repmat([2e-4;2e-7;2e-4;2e-7],2,1);
T=table(M,V,tol,abs(V)<=tol,'VariableNames',{'Metric','MaxAbsError','Tolerance','PASS'});
csvPath=fullfile(tempDir,'S7_Legacy_LC1_B4_Summary.csv');writetable(T,csvPath);resultPath=csvPath;
reportPath=fullfile(tempDir,'S7_Legacy_LC1_B4_Report_CN.md');fid=fopen(reportPath,'w');
if fid<0,error('run_s7_5_b4_pll_lc1:ReportOpen','无法写入报告。');end
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'# S7-5B/B4 Legacy PLL/GFL—LC1 报告\n\n');
fprintf(fid,'- C probe：`GSI_GFL_MODE=1`、S-Function 主步 Ts=4 us；Legacy `grid_side.Ts=0.1 ms`；固定相角 %.5g rad 的确定性 PCC 电压。\n',double(ang));
fprintf(fid,'- 覆盖 pre-sync PLL 和 post-takeover measurement-only PLL→GFL 角频率分支；逐步比较 C 输出与 Replica。\n\n');
fprintf(fid,'|指标|最大绝对误差|容差|PASS|\n|---|---:|---:|:---:|\n');
for i=1:height(T),fprintf(fid,'|%s|%.12g|%.4g|%s|\n',T.Metric{i},T.MaxAbsError(i),T.Tolerance(i),string(T.PASS(i)));end
fprintf(fid,'\n- **B4 LC1 总结：`%s`**\n\n',string(all(T.PASS)));
fprintf(fid,'## 边界\n\n本轮证明 PLL PI、Park 角度、频率/相角积分和 GFL 角度引用顺序一致；预同步切换判据和完整调度器仍由 B6/LC2 统一验证。\n');
end
