function result = run_s7_legacy_average_smoke(stopTime)
%RUN_S7_LEGACY_AVERAGE_SMOKE
% S7-5C2 的最短物理接口烟雾测试。仅验证模型能否更新并从 Legacy
% 平均电压指令驱动物理 plant；不保存完整 SimulationOutput 或时序。
if nargin < 1, stopTime = 0.005; end
here = fileparts(mfilename('fullpath'));
mdl = 'S7_Legacy_Average_Plant';
result = struct('status','FAIL','stopTime',stopTime,'message','');
logPath = fullfile(here,'temp','S7_5_LegacyPlant','latest_smoke_result.txt');
if ~isfolder(fileparts(logPath)), mkdir(fileparts(logPath)); end
fid = fopen(logPath,'w');
if fid < 0, error('S7:SmokeLog','无法创建烟雾测试日志：%s',logPath); end
cleanupLog = onCleanup(@() fclose(fid)); %#ok<NASGU>
try
    modelPath = fullfile(here,[mdl '.slx']);
    load_system(modelPath);
    cleanupModel = onCleanup(@() close_model(mdl)); %#ok<NASGU>
    set_param(mdl,'StopTime',num2str(stopTime,'%.15g'), ...
        'ReturnWorkspaceOutputs','off','SaveOutput','off','SaveTime','off');
    set_param(mdl,'SimulationCommand','update');
    sim(mdl,'StopTime',num2str(stopTime,'%.15g'),'SimulationMode','normal');
    result.status = 'PASS';
    result.message = 'update + short normal simulation completed';
catch ME
    result.status = 'FAIL';
    result.message = sprintf('%s: %s',ME.identifier,ME.message);
end
fprintf(fid,'status=%s\nstopTime=%.15g\nmessage=%s\n', ...
    result.status,result.stopTime,result.message);
fprintf('S7 smoke %s: %s\n',result.status,result.message);
end

function close_model(mdl)
if bdIsLoaded(mdl), close_system(mdl,0); end
end
