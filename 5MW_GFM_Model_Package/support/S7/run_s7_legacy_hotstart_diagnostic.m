function report = run_s7_legacy_hotstart_diagnostic(stopTimes)
%RUN_S7_LEGACY_HOTSTART_DIAGNOSTIC
% C2-R4/R5 热启动闭环短时验收。默认测试 1/10/100 ms，
% 每次从同一个 Legacy 热启动配置重新开始，只保存摘要，不保存时序。
if nargin < 1 || isempty(stopTimes), stopTimes = [1e-3 1e-2 1e-1]; end
stopTimes = stopTimes(:).';
here = fileparts(mfilename('fullpath'));
mdl = 'S7_Legacy_HotStart_Average_Plant';
modelPath = fullfile(here,[mdl '.slx']);
report = struct('status','FAIL','rows',table(),'message','');
names = {'ideal_Pmsc_ac','ideal_Pgsc_ac','ideal_Udc_state', ...
    'stage4_Ppcc','stage4_Udc','tm_T_e','tm_T_sh','tm_delta_omega_sh', ...
    'tm_omega_g','tm_omega_t'};
rows = cell(numel(stopTimes),1);
try
    addpath(fullfile(here,'temp','S7_5_LegacyPlant'));
    load_system(modelPath);
    cleanupObj = onCleanup(@() close_if_loaded(mdl)); %#ok<NASGU>
    for k = 1:numel(stopTimes)
        Tstop = stopTimes(k);
        row = struct('StopTime_s',Tstop,'RunStatus','FAIL', ...
            'OutputComplete',false,'UdcFirst_V',NaN,'UdcLast_V',NaN, ...
            'UdcTailMean_V',NaN,'UdcDrift_V',NaN,'PmscTail_MW',NaN, ...
            'PgscTail_MW',NaN,'PsumTail_MW',NaN,'PpccTail_MW',NaN, ...
            'TeTail_MNm',NaN,'TshTail_MNm',NaN,'OmegaShTail_radps',NaN, ...
            'PmscSpan_MW',NaN,'PgscSpan_MW',NaN,'TeSpan_MNm',NaN, ...
            'PhysicalGate','FAIL','Message','');
        try
            set_param(mdl,'StopTime',num2str(Tstop,'%.15g'), ...
                'ReturnWorkspaceOutputs','on','SaveOutput','off','SaveTime','off');
            set_param(mdl,'SimulationCommand','update');
            simOut = sim(mdl,'StopTime',num2str(Tstop,'%.15g'), ...
                'SimulationMode','normal','ReturnWorkspaceOutputs','on');
            row.RunStatus = 'PASS';
            S = struct();
            for n = 1:numel(names), S.(names{n}) = read_signal(simOut,names{n}); end
            row.OutputComplete = all(cellfun(@(n) S.(n).available && S.(n).finite,names));
            u=S.ideal_Udc_state; pm=S.ideal_Pmsc_ac; pg=S.ideal_Pgsc_ac;
            pp=S.stage4_Ppcc; te=S.tm_T_e; tsh=S.tm_T_sh; oms=S.tm_delta_omega_sh;
            row.UdcFirst_V=u.first; row.UdcLast_V=u.last; row.UdcTailMean_V=u.tailMean;
            row.UdcDrift_V=u.last-u.first; row.PmscTail_MW=pm.tailMean/1e6;
            row.PgscTail_MW=pg.tailMean/1e6; row.PsumTail_MW=(pm.tailMean+pg.tailMean)/1e6;
            row.PpccTail_MW=pp.tailMean/1e6; row.TeTail_MNm=te.tailMean/1e6;
            row.TshTail_MNm=tsh.tailMean/1e6; row.OmegaShTail_radps=oms.tailMean;
            row.PmscSpan_MW=(pm.max-pm.min)/1e6; row.PgscSpan_MW=(pg.max-pg.min)/1e6;
            row.TeSpan_MNm=(te.max-te.min)/1e6;
            % 热启动物理闭环门槛：Udc 接近 1500 V 且两交流端口功率代数和接近零。
            pGate=isfinite(row.PsumTail_MW) && abs(row.PsumTail_MW)<0.25;
            uGate=isfinite(row.UdcTailMean_V) && abs(row.UdcTailMean_V-1500)<10 && abs(row.UdcDrift_V)<10;
            row.PhysicalGate=ternary(row.OutputComplete && pGate && uGate,'PASS','FAIL');
        catch MEk
            row.Message=sprintf('%s: %s',MEk.identifier,MEk.message);
        end
        rows{k}=row;
    end
    report.status=ternary(all(cellfun(@(r) strcmp(r.RunStatus,'PASS'),rows)),'PASS','FAIL');
    report.message='hot-start drift runs completed';
catch ME
    report.status='FAIL'; report.message=sprintf('%s: %s',ME.identifier,ME.message);
end
T=rows_to_table(rows); report.rows=T;
csvPath=fullfile(here,'S7_Legacy_HotStart_Drift_Summary.csv');
writetable(T,csvPath);
fprintf('S7 Legacy hot-start diagnostic %s: %s\n',report.status,report.message);
disp(T);
end

function s=read_signal(simOut,name)
s=struct('available',false,'finite',false,'n',0,'first',NaN,'last',NaN, ...
    'tailMean',NaN,'tailStd',NaN,'min',NaN,'max',NaN);
try
    ts=simOut.get(name);
    if isa(ts,'timeseries'), d=double(ts.Data(:));
    elseif isnumeric(ts), d=double(ts(:)); else, d=[]; end
    if ~isempty(d)
        s.available=true; s.finite=all(isfinite(d)); s.n=numel(d);
        s.first=d(1); s.last=d(end); idx=max(1,numel(d)-100):numel(d);
        s.tailMean=mean(d(idx)); s.tailStd=std(d(idx)); s.min=min(d); s.max=max(d);
    end
catch
end
end

function T=rows_to_table(rows)
fields=fieldnames(rows{1}); data=cell(numel(rows),numel(fields));
for i=1:numel(rows), for j=1:numel(fields), data{i,j}=rows{i}.(fields{j}); end, end
T=cell2table(data,'VariableNames',fields);
end
function y=ternary(c,a,b), if c,y=a;else,y=b;end,end
function close_if_loaded(mdl), if bdIsLoaded(mdl), close_system(mdl,0); end, end
