function R = run_s7_legacy_command_probe(stopTime)
%RUN_S7_LEGACY_COMMAND_PROBE
% 读取 S7 Legacy 平均输出的四个 alpha-beta 电压指令，仅保存摘要。
% 该脚本在内存中临时增加 From/To Workspace，不保存模型，也不保存原始时序。

if nargin < 1 || isempty(stopTime), stopTime = 1e-3; end
here = fileparts(mfilename('fullpath'));
mdl = 'S7_Legacy_Average_Plant';
modelFile = fullfile(here,[mdl '.slx']);
assert(isfile(modelFile),'找不到模型副本：%s',modelFile);
load_system(modelFile);
cleanup = onCleanup(@() close_if_loaded(mdl)); %#ok<NASGU>

probeNames = {'S7Probe_MSC_Ualpha','S7Probe_MSC_Ubeta', ...
    'S7Probe_GSC_Ualpha','S7Probe_GSC_Ubeta'};
tags = {'Ideal_MSC_Ualpha','Ideal_MSC_Ubeta', ...
    'Ideal_GSC_Ualpha','Ideal_GSC_Ubeta'};
vars = {'s7ProbeMscUa','s7ProbeMscUb','s7ProbeGscUa','s7ProbeGscUb'};
created = {};
for k=1:numel(tags)
    f=[mdl '/' probeNames{k} '_From'];
    t=[mdl '/' probeNames{k} '_To'];
    add_block('simulink/Signal Routing/From',f,'GotoTag',tags{k}, ...
        'Position',[110 80+90*(k-1) 210 110+90*(k-1)]);
    add_block('simulink/Sinks/To Workspace',t,'VariableName',vars{k}, ...
        'SaveFormat','Structure With Time', ...
        'Position',[260 80+90*(k-1) 390 110+90*(k-1)]);
    add_line(mdl,[probeNames{k} '_From/1'],[probeNames{k} '_To/1'],'autorouting','on');
    created(end+1:end+2)={f,t}; %#ok<AGROW>
end

set_param(mdl,'StopTime',num2str(stopTime,'%.17g'), ...
    'ReturnWorkspaceOutputs','off');
set_param(mdl,'SimulationCommand','update');
R=struct('status','FAIL','stopTime',stopTime,'commands',struct(), ...
    'message','','output_complete',false);
try
    sim(mdl);
    for k=1:numel(vars)
        if evalin('base',sprintf('exist(''%s'',''var'')',vars{k}))
            s=evalin('base',vars{k});
            if isa(s,'timeseries')
                y=s.Data(:); t=s.Time(:);
            elseif isstruct(s) && isfield(s,'time') && isfield(s,'signals')
                y=s.signals.values(:); t=s.time(:);
            else
                y=[]; t=[];
            end
            if ~isempty(y)
                R.commands.(vars{k})=struct('first',y(1), ...
                    'last',y(end),'min',min(y),'max',max(y), ...
                    'span',max(y)-min(y),'samples',numel(y), ...
                    'time_last',t(end));
            end
        end
    end
    R.status='PASS';
    R.output_complete=(numel(fieldnames(R.commands))==numel(vars));
    R.message='short command probe completed';
catch ME
    R.message=ME.message;
end

outDir=fullfile(here,'temp','S7_5_LegacyPlant');
if ~isfolder(outDir), mkdir(outDir); end
fid=fopen(fullfile(outDir,'latest_command_probe_summary.txt'),'w','n','UTF-8');
if fid>0
    fprintf(fid,'status=%s\nstopTime=%.17g\noutput_complete=%d\nmessage=%s\n', ...
        R.status,R.stopTime,R.output_complete,R.message);
    fns=fieldnames(R.commands);
    for k=1:numel(fns)
        q=R.commands.(fns{k});
        fprintf(fid,'%s first=%.9g last=%.9g min=%.9g max=%.9g span=%.9g samples=%d\n', ...
            fns{k},q.first,q.last,q.min,q.max,q.span,q.samples);
    end
    fclose(fid);
end
% 临时块只存在于内存；不保存 S7 模型。
for k=numel(created):-1:1
    try, delete_block(created{k}); catch, end
end
clear(vars{:});
end

function close_if_loaded(mdl)
if bdIsLoaded(mdl), close_system(mdl,0); end
end
