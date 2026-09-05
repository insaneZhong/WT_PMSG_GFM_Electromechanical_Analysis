function R = run_s7_legacy_hotstart_probe(stopTime)
%RUN_S7_LEGACY_HOTSTART_PROBE
% 临时读取热启动模型在首个样本的实际测量面，仅写一份小型摘要。
% 本函数不保存模型、不保存完整时序；临时 From/To Workspace 块在退出
% 前删除。用于核对热启动状态映射，不改变任何控制方程。
if nargin < 1 || isempty(stopTime), stopTime = 1e-6; end
here = fileparts(mfilename('fullpath'));
mdl = 'S7_Legacy_HotStart_Average_Plant';
load_system(fullfile(here,[mdl '.slx']));
cleanupObj = onCleanup(@() close_if_loaded(mdl)); %#ok<NASGU>
tags = {'PCC_uab','PCC_ubc','PCC_uca','ian','ibn','icn', ...
    'Pcc_ia','Pcc_ib','Pcc_ic','Legacy_GSC_Ualpha','Legacy_GSC_Ubeta'};
vars = strcat('s7HotProbe_',matlab.lang.makeValidName(tags));
created = {};
for k=1:numel(tags)
    fn = sprintf('%s/HotProbeFrom%02d',mdl,k);
    tn = sprintf('%s/HotProbeTo%02d',mdl,k);
    add_block('simulink/Signal Routing/From',fn,'GotoTag',tags{k}, ...
        'Position',[80 60+45*(k-1) 170 85+45*(k-1)]);
    add_block('simulink/Sinks/To Workspace',tn,'VariableName',vars{k}, ...
        'SaveFormat','Structure With Time', ...
        'Position',[220 60+45*(k-1) 380 85+45*(k-1)]);
    add_line(mdl,sprintf('HotProbeFrom%02d/1',k),sprintf('HotProbeTo%02d/1',k), ...
        'autorouting','on');
    created(end+1:end+2)={fn,tn}; %#ok<AGROW>
end
set_param(mdl,'StopTime',num2str(stopTime,'%.17g'),'ReturnWorkspaceOutputs','off');
R=struct('status','FAIL','stopTime',stopTime,'signals',struct(),'message','');
try
    set_param(mdl,'SimulationCommand','update');
    sim(mdl);
    for k=1:numel(vars)
        if evalin('base',sprintf('exist(''%s'',''var'')',vars{k}))
            s=evalin('base',vars{k});
            if isstruct(s) && isfield(s,'signals')
                y=double(s.signals.values(:));
            else
                y=[];
            end
            if ~isempty(y)
                R.signals.(tags{k})=struct('first',y(1),'last',y(end), ...
                    'min',min(y),'max',max(y),'samples',numel(y));
            end
        end
    end
    R.status='PASS'; R.message='probe completed';
catch ME
    R.message=sprintf('%s: %s',ME.identifier,ME.message);
end
outDir=fullfile(here,'temp','S7_5_LegacyPlant');
if ~isfolder(outDir), mkdir(outDir); end
fid=fopen(fullfile(outDir,'latest_hotstart_probe_summary.txt'),'w','n','UTF-8');
if fid>0
    fprintf(fid,'status=%s\nstopTime=%.17g\nmessage=%s\n',R.status,R.stopTime,R.message);
    fns=fieldnames(R.signals);
    for k=1:numel(fns)
        q=R.signals.(fns{k});
        fprintf(fid,'%s first=%.12g last=%.12g min=%.12g max=%.12g samples=%d\n', ...
            fns{k},q.first,q.last,q.min,q.max,q.samples);
    end
    fclose(fid);
end
for k=numel(created):-1:1
    try, delete_block(created{k}); catch, end
end
for k=1:numel(vars)
    try, evalin('base',sprintf('clear %s',vars{k})); catch, end
end
end

function close_if_loaded(mdl)
if bdIsLoaded(mdl), close_system(mdl,0); end
end
