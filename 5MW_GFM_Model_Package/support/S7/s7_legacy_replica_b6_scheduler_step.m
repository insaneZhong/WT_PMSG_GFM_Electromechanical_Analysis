function [state,out] = s7_legacy_replica_b6_scheduler_step(state,p)
%S7_LEGACY_REPLICA_B6_SCHEDULER_STEP
% S7-5B/B6：复制 main_legacy_ad_base.c 的控制中断与 FPGA PWM
% 分段计时器更新顺序。每次调用代表一个 S-Function 主步。
if nargin==1 && (ischar(state)||(isstring(state)&&isscalar(state)))
    mode=lower(char(state));
    switch mode
        case 'defaults',state=local_defaults();
        case 'initial_state',state=local_initial_state();
        otherwise,error('s7_legacy_replica_b6_scheduler_step:UnknownMode','未知模式 %s。',mode);
    end
    out=[];return
end
if nargin<2||isempty(p),p=local_defaults();end
if isempty(state)||~isstruct(state),state=local_initial_state();end
state=local_complete(state);
% 对应 C：ControlTimerCounter++; if (Counter > Period) { ... }。
state.ControlTimerFlag=false;state.ControlTimerCounter=state.ControlTimerCounter+1;
if state.ControlTimerCounter>state.ControlTimerPeriod
    state.ControlTimerCounter=0;state.ControlTimerFlag=true;state.ControlTimerIntruptIndex=state.ControlTimerIntruptIndex+1;
    state.FPGAPWMTimerCounter1=0;state.FPGAPWMTimerCounter2=0;
    state.FPGAPWMTimerIntruptIndex1=0;state.FPGAPWMTimerIntruptIndex2=0;
    state.ControlTimerPeriod=state.PwmVectorPeriodTicks;
end
% C 在每个主步末尾先 ++，再与当前段计时值比较；越过零段时循环前进。
[state.FPGAPWMTimerCounter1,state.FPGAPWMTimerIntruptIndex1,state.PwmState1]=local_pwm_tick( ...
    state.FPGAPWMTimerCounter1,state.FPGAPWMTimerIntruptIndex1,state.PwmState1,p.SegmentPeriods1,p.SegmentStates1);
[state.FPGAPWMTimerCounter2,state.FPGAPWMTimerIntruptIndex2,state.PwmState2]=local_pwm_tick( ...
    state.FPGAPWMTimerCounter2,state.FPGAPWMTimerIntruptIndex2,state.PwmState2,p.SegmentPeriods2,p.SegmentStates2);
out=struct('control_event',state.ControlTimerFlag,'control_counter',state.ControlTimerCounter, ...
    'control_period',state.ControlTimerPeriod,'pwm_index1',state.FPGAPWMTimerIntruptIndex1, ...
    'pwm_index2',state.FPGAPWMTimerIntruptIndex2,'pwm_state1',state.PwmState1,'pwm_state2',state.PwmState2);
end
function p=local_defaults()
p=struct('ControlTimerPeriod',100,'PwmVectorPeriodTicks',100, ...
    'SegmentPeriods1',[3 4 5 6 5 4 3],'SegmentPeriods2',[3 4 5 6 5 4 3], ...
    'SegmentStates1',0:6,'SegmentStates2',0:6);
end
function st=local_initial_state()
st=struct('ControlTimerCounter',0,'ControlTimerPeriod',100,'ControlTimerIntruptIndex',0, ...
    'FPGAPWMTimerCounter1',0,'FPGAPWMTimerCounter2',0,'FPGAPWMTimerIntruptIndex1',0, ...
    'FPGAPWMTimerIntruptIndex2',0,'PwmVectorPeriodTicks',100,'PwmState1',0,'PwmState2',0);
end
function st=local_complete(st)
z=local_initial_state();f=fieldnames(z);
for k=1:numel(f),if ~isfield(st,f{k}),st.(f{k})=z.(f{k});end,end
end
function [cnt,idx,sv]=local_pwm_tick(cnt,idx,sv,periods,states)
cnt=cnt+1;idx=min(max(idx,1),numel(periods));sv=states(idx);
if cnt>periods(idx)
    cnt=0;
    while true
        idx=idx+1;
        if idx>=numel(periods),idx=numel(periods);break;end
        if periods(idx)~=0,break;end
    end
    sv=states(idx);
end
end
