function resultPath=run_s7_5_b6_scheduler_lc1()
%RUN_S7_5_B6_SCHEDULER_LC1  控制中断/多速率 PWM 计时器确定性核对。
% 采用 C 源码调度规则的独立伪输入重放，不保存逐步原始序列。
here=fileparts(mfilename('fullpath'));tempDir=fullfile(here,'temp','S7_5_LegacyCertification');
if ~isfolder(tempDir),mkdir(tempDir);end;addpath(here,'-begin');
p=s7_legacy_replica_b6_scheduler_step('defaults');st=s7_legacy_replica_b6_scheduler_step('initial_state');
N=650;events=[];idx1=[];idx2=[];
% 插入零长度 PWM 段，检验 C 中 while(period==0) 的跳过和末段保护。
zeroP=[3 0 4 0 5 6 5 4 3];p.SegmentPeriods1=zeroP;p.SegmentPeriods2=zeroP;
p.SegmentStates1=0:numel(zeroP)-1;p.SegmentStates2=p.SegmentStates1;
for k=1:N
    [st,o]=s7_legacy_replica_b6_scheduler_step(st,p);
    if o.control_event,events(end+1)=k;end %#ok<AGROW>
    idx1(end+1)=o.pwm_index1;idx2(end+1)=o.pwm_index2; %#ok<AGROW>
end
expected=101:101:606;eventErr=events(:)-expected(:);eventPass=numel(events)==numel(expected)&&all(eventErr==0);
% 在序列中，零段索引为 2 和 4 不应停留；首次调用的 index=1 是合法的。
zeroSkipPass=(~any(idx1==2))&&(~any(idx1==4));finitePass=all(isfinite([events idx1 idx2]));
names={'event_count';'event_index_max_abs_error';'zero_segment_skip';'finite';'overall'};
values=[numel(events);max([0;abs(eventErr(:))]);double(zeroSkipPass);double(finitePass);double(eventPass&&zeroSkipPass&&finitePass)];
tol=[0;0;.5;.5;.5];pass=[numel(events)==numel(expected);eventPass;zeroSkipPass;finitePass;eventPass&&zeroSkipPass&&finitePass];
T=table(names,values,tol,pass,'VariableNames',{'Metric','Value','Tolerance','PASS'});
csvPath=fullfile(tempDir,'S7_Legacy_LC1_B6_Summary.csv');writetable(T,csvPath);resultPath=csvPath;
reportPath=fullfile(tempDir,'S7_Legacy_LC1_B6_Report_CN.md');fid=fopen(reportPath,'w');
if fid<0,error('run_s7_5_b6_scheduler_lc1:ReportOpen','无法写入报告。');end;cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'# S7-5B/B6 Legacy Scheduler—LC1 报告\n\n');
fprintf(fid,'- 主步计时：1 us；初始 `ControlTimerPeriod=100`；C 使用 `Counter++` 后 `Counter>Period` 触发，因此首事件为第101步。\n');
fprintf(fid,'- PWM 段计时按 C 的 `++`、`>`、零段循环和末段保护顺序重放；不生成长时序文件。\n\n');
fprintf(fid,'|指标|结果|容差|PASS|\n|---|---:|---:|:---:|\n');
for i=1:height(T),fprintf(fid,'|%s|%.12g|%.4g|%s|\n',T.Metric{i},T.Value(i),T.Tolerance(i),string(T.PASS(i)));end
fprintf(fid,'\n- 期望控制事件步：`101, 202, 303, 404, 505, 606`；实际事件步：`%s`。\n',strjoin(string(events),', '));
fprintf(fid,'- **B6 LC1 总结：`%s`**\n\n',string(all(T.PASS)));
fprintf(fid,'## 边界\n\n本轮验证的是调度器计数规则和多速率段更新逻辑；完整 C 控制器在每个事件中的状态连续性由 LC2 统一核对。\n');
end
