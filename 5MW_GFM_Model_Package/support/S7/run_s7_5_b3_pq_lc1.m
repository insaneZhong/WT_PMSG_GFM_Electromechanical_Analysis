function resultPath = run_s7_5_b3_pq_lc1()
%RUN_S7_5_B3_PQ_LC1  S7-5B/B3 P/Q 低通离散更新确定性核对。
% 该测试不写入长时序；只在内存中逐步比较 Replica 与 C 源码方程
% 的 single 精度重放，并把摘要写入 temp/S7_5_LegacyCertification。

here = fileparts(mfilename('fullpath'));
tempDir = fullfile(here,'temp','S7_5_LegacyCertification');
if ~isfolder(tempDir), mkdir(tempDir); end

N = 1000;
k = (1:N).';
P = single(4.0e6 + 0.8e6*sin(2*pi*k/137) + 0.15e6*(mod(k,29)==0));
Q = single(0.25e6*cos(2*pi*k/83) - 0.08e6*sin(2*pi*k/31));

% Replica 序列。
st = s7_legacy_replica_b3_pq_filter_step('initial_state');
p = s7_legacy_replica_b3_pq_filter_step('defaults');
repP=zeros(N,1,'single'); repQ=zeros(N,1,'single');
for n=1:N
    [st,o] = s7_legacy_replica_b3_pq_filter_step(st,struct('P',P(n),'Q',Q(n)),p);
    repP(n)=o.P_filter; repQ(n)=o.Q_filter;
end

% 逐字按 motor_low_pass_filter() 公式重放（独立变量，避免复用 Replica 状态）。
[srcP,srcQ] = local_replay(P,Q);
errP=double(repP)-double(srcP); errQ=double(repQ)-double(srcQ);
maxErr=max([max(abs(errP)),max(abs(errQ))]);
finiteOK=all(isfinite(repP)) && all(isfinite(repQ));
pass=finiteOK && maxErr <= 2e-6;

names={'P_filter';'Q_filter';'all_finite';'max_abs_error'};
values=[max(abs(errP));max(abs(errQ));double(finiteOK);maxErr];
tol=[2e-6;2e-6;0.5;2e-6];
passCol=[max(abs(errP))<=tol(1);max(abs(errQ))<=tol(2);finiteOK;maxErr<=tol(4)];
T=table(names,values,tol,passCol,'VariableNames',{'Metric','Value','Tolerance','PASS'});
csvPath=fullfile(tempDir,'S7_Legacy_LC1_B3_Summary.csv'); writetable(T,csvPath); resultPath=csvPath;
reportPath=fullfile(tempDir,'S7_Legacy_LC1_B3_Report_CN.md'); fid=fopen(reportPath,'w');
if fid<0,error('run_s7_5_b3_pq_lc1:ReportOpen','无法写入报告。');end
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'# S7-5B/B3 Legacy P/Q Filter—LC1 报告\n\n');
fprintf(fid,'- 测试长度：%d 步；输入为确定性 P/Q 序列；滤波器 `Ts=0.00025 s`、截止频率 20 Hz。\n',N);
fprintf(fid,'- 比较对象：Replica 与 `grid_forming_control.c::motor_low_pass_filter()` 的逐字 single 精度方程重放。\n\n');
fprintf(fid,'|指标|结果|容差|PASS|\n|---|---:|---:|:---:|\n');
for i=1:height(T), fprintf(fid,'|%s|%.12g|%.4g|%s|\n',T.Metric{i},T.Value(i),T.Tolerance(i),string(T.PASS(i))); end
fprintf(fid,'\n- **B3 LC1 总结：`%s`**\n\n',string(pass));
fprintf(fid,'## 边界\n\n本测试确认离散滤波方程、历史状态和更新顺序一致；C-S-Function 对外未导出 P/Q 滤波内部状态，故 C 二进制的多步内部状态仍在 LC2 完整控制器测试中通过 `w_ref/voltage_ref` 间接核对。\n');
end

function [yP,yQ] = local_replay(P,Q)
N=numel(P); yP=zeros(N,1,'single'); yQ=zeros(N,1,'single');
oP=single(0); oQ=single(0); xP=single(0); xQ=single(0);
Ts=single(0.00025); fc=single(20); pc=single(pi);
a0=single(1)+Ts*pc*fc; a1=Ts*pc*fc-single(1); b=Ts*pc*fc;
for n=1:N
    yP(n)=single((b*P(n)+b*xP-a1*oP)/a0); xP=P(n); oP=yP(n);
    yQ(n)=single((b*Q(n)+b*xQ-a1*oQ)/a0); xQ=Q(n); oQ=yQ(n);
end
end
