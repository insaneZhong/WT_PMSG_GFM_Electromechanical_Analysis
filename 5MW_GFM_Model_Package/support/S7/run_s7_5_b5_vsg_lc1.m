function resultPath=run_s7_5_b5_vsg_lc1()
%RUN_S7_5_B5_VSG_LC1  严格 VSG C probe 与 Replica 多步核对。

here=fileparts(mfilename('fullpath'));tempDir=fullfile(here,'temp','S7_5_LegacyCertification');
if ~isfolder(tempDir),mkdir(tempDir);end
addpath(here,'-begin');srcDir=fullfile(fileparts(here),'CurrentModel_Idealized');
mexName='main_s7_5_b5_probe';mexPath=fullfile(tempDir,[mexName,'.',mexext]);
if ~isfile(mexPath)
    old=pwd;c=onCleanup(@()cd(old));cd(srcDir);setenv('MW_MINGW64_LOC','C:\mingw64');clear mex;
    defs={'-DLEGACY_AD_S_FUNCTION_NAME=main_s7_5_b5_probe','-DIDEAL_CONTINUOUS_CONTROLLER=1', ...
        '-DIDEAL_AVG_OUTPUTS=0','-DLEGACY_SFUNCTION_SAMPLE_TIME_S=4e-6', ...
        '-DLEGACY_RESET_CONTROLLER_ON_INIT=1','-DENABLE_VSG_EQUIV_WREF=1', ...
        '-DGSI_GFL_MODE=0','-DPRESYN_SWITCH_TIME=0','-DGSI_GFM_ENABLE_TIME_S=0'};
    mex(defs{:},'-output',fullfile(tempDir,mexName),'main_legacy_ad_base.c','svpwm.c', ...
        'motorcontrol_legacy_ad_base.c','grid_forming_control.c');
end
addpath(tempDir,'-begin');

Ts=4e-6;N=100;V=563;ang=.02;ua=V*cos(ang);ub=V*cos(ang-2*pi/3);uc=V*cos(ang+2*pi/3);
uab=ua-ub;ubc=ub-uc;uca=uc-ua;
u0=zeros(20,1);u0(4)=900;u0(5)=314;u0(6)=0;u0(10:12)=[uab;ubc;uca];
u0(13:15)=[10;-5;-5];u0(16)=1;u0(17)=0;u0(18)=1;u0(19)=0;u0(20)=1000;
mdl='S7_5_B5_VSG_LC1';if bdIsLoaded(mdl),close_system(mdl,0);end;assignin('base','u0',u0);new_system(mdl);load_system('simulink');
add_block('simulink/Sources/Constant',[mdl '/Input20'],'Value','u0','Position',[30 50 100 90]);
add_block('simulink/User-Defined Functions/S-Function',[mdl '/C'],'FunctionName','main_s7_5_b5_probe','Parameters','1e6,0,563,1000','Position',[160 40 300 100]);
add_block('simulink/Sinks/To Workspace',[mdl '/Y'],'VariableName','yC','SaveFormat','Array','Position',[370 48 485 92]);
add_line(mdl,'Input20/1','C/1');add_line(mdl,'C/1','Y/1');
set_param(mdl,'Solver','FixedStepDiscrete','FixedStep',num2str(Ts,'%.12g'),'StopTime',num2str(N*Ts,'%.12g'),'ReturnWorkspaceOutputs','on');
set_param(mdl,'SimulationCommand','update');simOut=sim(mdl,'ReturnWorkspaceOutputs','on');
try,yC=simOut.get('yC');catch,yC=evalin('base','yC');end
if isempty(yC)||size(yC,2)<37,error('run_s7_5_b5_vsg_lc1:NoOutput','C 输出不足。');end
st=s7_legacy_replica_b5_vsg_step('initial_state');p=s7_legacy_replica_b5_vsg_step('defaults');
n=min(N,size(yC,1));rep=zeros(n,4);cv=zeros(n,4);er=zeros(n,4);
for k=1:n
    u=struct('Ts_grid',1e-4,'P_ref',1e6,'pcc_uab',uab,'pcc_ubc',ubc,'pcc_uca',uca, ...
        'pcc_Ia',10,'pcc_Ib',-5,'pcc_Ic',-5);
    [st,o,tr]=s7_legacy_replica_b5_vsg_step(st,u,p); %#ok<ASGLU>
    rep(k,:)=[o.w_ref,o.theta_ref,o.P,o.P_filter];
    % C output: [15] w_ref, [16] theta_ref, [14] P.
    cv(k,:)=[yC(k,15),yC(k,16),yC(k,14),NaN];er(k,1:3)=cv(k,1:3)-rep(k,1:3);
end
maxerr=max(abs(er(:,1:3)),[],1);pass=(maxerr(1)<=2e-3)&&(maxerr(2)<=2e-6)&&(maxerr(3)<=2e-2);
names={'w_ref';'theta_ref';'P_pcc';'all_finite'};values=[maxerr(:);double(all(isfinite(rep(:))))];tol=[2e-3;2e-6;2e-2;.5];
passCol=[maxerr(1)<=tol(1);maxerr(2)<=tol(2);maxerr(3)<=tol(3);values(4)>0];
T=table(names,values,tol,passCol,'VariableNames',{'Metric','MaxAbsError','Tolerance','PASS'});
csvPath=fullfile(tempDir,'S7_Legacy_LC1_B5_Summary.csv');writetable(T,csvPath);resultPath=csvPath;close_system(mdl,0);
reportPath=fullfile(tempDir,'S7_Legacy_LC1_B5_Report_CN.md');fid=fopen(reportPath,'w');
if fid<0,error('run_s7_5_b5_vsg_lc1:ReportOpen','无法写入报告。');end;cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'# S7-5B/B5 Legacy VSG—LC1 报告\n\n');
fprintf(fid,'- 严格 VSG：`ENABLE_VSG_EQUIV_WREF=1`；C 主步 4 us，但 `grid_side.Ts=0.1 ms`。\n');
fprintf(fid,'- 复制：P/Q 测量、Pref 斜率限制、VSG 摆动方程、w_vsg 先更新后积分 theta。\n\n');
fprintf(fid,'|指标|最大绝对误差|容差|PASS|\n|---|---:|---:|:---:|\n');
for i=1:height(T),fprintf(fid,'|%s|%.12g|%.4g|%s|\n',T.Metric{i},T.MaxAbsError(i),T.Tolerance(i),string(T.PASS(i)));end
fprintf(fid,'\n- **B5 LC1 总结：`%s`**\n\n',string(pass));
fprintf(fid,'## 边界\n\n当前测试使用固定 PCC 电压/电流和未饱和小功率输入；VSG 与完整 GSC PI、调度器的组合由 LC2 验证。\n');
end
