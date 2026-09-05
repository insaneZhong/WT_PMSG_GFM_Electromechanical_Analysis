function resultPath = run_s7_5_b2_gsc_lc1()
%RUN_S7_5_B2_GSC_LC1  S7-5B/B2 固定角度 GSC 单步交叉核对。
% 使用生产版 Legacy C-S-Function 的首个控制事件；PLL/P-f 只作为外部
% 产生 theta_fixed、w_ref 的输入，B2 专门核对 GSC 的 P/Q、Q-V、两级 PI、
% 解耦和 dq->alpha-beta。输出仅写入 temp/S7_5_LegacyCertification。

here = fileparts(mfilename('fullpath'));
tempDir = fullfile(here,'temp','S7_5_LegacyCertification');
if ~isfolder(tempDir), mkdir(tempDir); end
mexDir = fullfile(here,'temp','S7_5_LegacyCertification');
mexPath = fullfile(mexDir,'main_s7_5_b2_probe.mexw64');
if ~isfile(mexPath), error('run_s7_5_b2_gsc_lc1:MissingMex','找不到 %s',mexPath); end
addpath(mexDir,'-begin'); clear main_s7_5_b2_probe;

u0 = zeros(20,1);
u0(1:3) = [0;0;0];
u0(4) = 900; u0(5) = 100; u0(6) = 0.01;
u0(7:9) = [10;-5;-5];
u0(10:12) = [844.5;0;-844.5];
u0(13:15) = [100;-50;-50];
u0(16) = 3; u0(17) = 0; u0(18) = 1; u0(19) = 0; u0(20) = 1000;
assignin('base','u0',u0);

mdl='S7_5_B2_GSC_LC1_Runtime';
if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl); load_system('simulink');
add_block('simulink/Sources/Constant',[mdl '/Input20'],'Value','u0','Position',[30 50 100 90]);
add_block('simulink/User-Defined Functions/S-Function',[mdl '/LegacyC'], ...
    'FunctionName','main_s7_5_b2_probe','Parameters','5e6,0,563,1000','Position',[160 40 300 100]);
add_block('simulink/Sinks/To Workspace',[mdl '/Y'],'VariableName','yC','SaveFormat','Array','Position',[370 48 485 92]);
add_line(mdl,'Input20/1','LegacyC/1'); add_line(mdl,'LegacyC/1','Y/1');
set_param(mdl,'Solver','FixedStepDiscrete','FixedStep','1e-6','StopTime','1e-6', ...
    'ReturnWorkspaceOutputs','on');
set_param(mdl,'SimulationCommand','update');
simOut=sim(mdl,'ReturnWorkspaceOutputs','on');
try, yC=simOut.get('yC'); catch, yC=evalin('base','yC'); end
if isempty(yC) || size(yC,2)<37, error('run_s7_5_b2_gsc_lc1:NoOutput','C 输出不足 37 通道。'); end
kEvent=find(abs(yC(:,32))>1e-7,1,'first');
if isempty(kEvent), error('run_s7_5_b2_gsc_lc1:NoEvent','未检测到首个控制事件。'); end

theta_fixed=double(yC(kEvent,16));
w_ref=double(yC(kEvent,15));
disp('S7-5B/B2 C diagnostic columns 13:30:'); disp(yC(kEvent,13:30));
u=struct('Ts',4e-6,'theta_fixed',theta_fixed,'theta_voltage',0.05,'theta_current',0.05,'w_ref',w_ref, ...
    'Udc',900,'P_ref',5e6,'Q_ref',0,'Pre_syn',true,'GFM_enabled',true, ...
    'pcc_uab',844.5,'pcc_ubc',0,'pcc_uca',-844.5, ...
    'Ia1',10,'Ib1',-5,'Ic1',-5,'pcc_Ia',100,'pcc_Ib',-50,'pcc_Ic',-50);
s=s7_legacy_replica_b2_gsc_step('initial_state');
p=s7_legacy_replica_b2_gsc_step('defaults');
[~,o,tr]=s7_legacy_replica_b2_gsc_step(s,u,p);

% C 通道为 0-based [13,18,16,17,19,20,21,22,23,24,25,26,29]。
names={'P';'Q';'Ud1_ref';'Uq1_ref';'voltage_ref';'U_od_ref';'pcc_u_d'; ...
    'Id_ref';'Id';'U_oq_ref';'pcc_u_q';'Iq_ref';'Iq'};
cCols=[14 19 17 18 20 21 22 23 24 25 26 27 30];
rVals=[o.P;o.Q;o.Ud1_ref;o.Uq1_ref;o.voltage_ref;o.U_od_ref;o.pcc_ud; ...
    o.Id_ref;o.Id;o.U_oq_ref;o.pcc_uq;o.Iq_ref;o.Iq];
cVals=double(yC(kEvent,cCols)).';
tol=[1e-2;1e-2;1e-3;1e-3;1e-3;1e-3;1e-3;1e-3;1e-3;1e-3;1e-3;1e-3;1e-3];
err=cVals-rVals; pass=abs(err)<=tol;
T=table(names,cVals,rVals,err,abs(err),tol,pass, ...
    'VariableNames',{'Signal','C_FirstEvent','Replica_FixedAngle','SignedError', ...
    'AbsoluteError','Tolerance','PASS'});
csvPath=fullfile(tempDir,'S7_Legacy_LC1_B2_Summary.csv');
writetable(T,csvPath); resultPath=csvPath;
overall=all(pass)&&tr.all_limits_inactive;
reportPath=fullfile(tempDir,'S7_Legacy_LC1_B2_Report_CN.md'); fid=fopen(reportPath,'w');
if fid<0,error('run_s7_5_b2_gsc_lc1:ReportOpen','无法写入报告。');end
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'# S7-5B/B2 Legacy GSC Replica—LC1 单步报告\n\n');
fprintf(fid,'- C 首事件索引：%d；固定步长 1 us；theta/w_ref 由 C 诊断通道冻结。\n',kEvent);
fprintf(fid,'- `theta_fixed=%.12g rad`，`w_ref=%.12g rad/s`；PLL/VSG 状态不在 B2 内部复制。\n\n',theta_fixed,w_ref);
fprintf(fid,'## 复制范围\n\nP/Q、20 Hz 双线性低通、Q-V、电压 PI、电流 PI、Ls 交叉解耦和 dq→alpha-beta。C 默认电流向量/调制限幅关闭。\n\n');
fprintf(fid,'## 逐项结果\n\n|信号|C 首事件|Replica|绝对误差|容差|PASS|\n|---|---:|---:|---:|---:|:---:|\n');
for i=1:height(T)
    fprintf(fid,'|%s|%.12g|%.12g|%.4g|%.4g|%s|\n',T.Signal{i},T.C_FirstEvent(i),T.Replica_FixedAngle(i),T.AbsoluteError(i),T.Tolerance(i),string(T.PASS(i)));
end
fprintf(fid,'\n- B2 Replica 限幅状态：`%s`\n',string(tr.all_limits_inactive));
fprintf(fid,'- **B2 LC1 总结：`%s`**\n\n',string(overall));
fprintf(fid,'## 边界\n\n本轮把 PLL/P-f/VSG 角度作为冻结输入，只证明 GSC 运算核首事件一致；GSC 多事件调度、滤波状态连续性、PLL/VSG 内部状态以及完整 LC2 尚未通过。\n');
close_system(mdl,0);
end
