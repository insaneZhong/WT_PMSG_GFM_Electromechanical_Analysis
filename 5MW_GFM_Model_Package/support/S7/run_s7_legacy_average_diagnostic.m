function result = run_s7_legacy_average_diagnostic(stopTime)
%RUN_S7_LEGACY_AVERAGE_DIAGNOSTIC
% C2 的短时物理兼容性诊断：在复制模型的原有初始化下运行 Legacy
% 平均包装器，内存中计算末段统计，并只写一行摘要。它不是固定点求解。
if nargin < 1 || isempty(stopTime), stopTime = 0.2; end
here = fileparts(mfilename('fullpath'));
mdl = 'S7_Legacy_Average_Plant';
modelPath = fullfile(here,[mdl '.slx']);
result = struct('status','FAIL','stopTime',stopTime,'message','');
names = {'stage4_Ppcc','stage4_Udc','tm_T_e','tm_T_sh', ...
    'tm_delta_omega_sh','tm_omega_g','tm_omega_t','stage4_PreSyn', ...
    'stage4_Pref_raw','ideal_Pgsc_ac','ideal_Pmsc_ac','ideal_Udc_state'};
try
    addpath(fullfile(here,'temp','S7_5_LegacyPlant'));
    load_system(modelPath);
    cleanupObj = onCleanup(@() close_if_loaded(mdl)); %#ok<NASGU>
    set_param(mdl,'StopTime',num2str(stopTime,'%.15g'), ...
        'ReturnWorkspaceOutputs','on','SaveOutput','off','SaveTime','off');
    set_param(mdl,'SimulationCommand','update');
    simOut = sim(mdl,'StopTime',num2str(stopTime,'%.15g'), ...
        'SimulationMode','normal','ReturnWorkspaceOutputs','on');
    result.status = 'PASS';
    result.message = 'short physical plant run completed';
    for k=1:numel(names)
        n=names{k}; s=struct('available',false,'finite',false,'n',0, ...
            'first',NaN,'last',NaN,'tailMean',NaN,'tailStd',NaN,'min',NaN,'max',NaN);
        try
            ts=simOut.get(n);
            if isa(ts,'timeseries')
                d=double(ts.Data(:));
                s.available=~isempty(d); s.finite=all(isfinite(d)); s.n=numel(d);
                s.first=d(1); s.last=d(end); idx=max(1,numel(d)-100):numel(d);
                s.tailMean=mean(d(idx)); s.tailStd=std(d(idx)); s.min=min(d); s.max=max(d);
            end
        catch
        end
        result.signals.(n)=s;
    end
    result.output_complete = all(cellfun(@(n) result.signals.(n).available && ...
        result.signals.(n).finite, names));
    result.u_dc_tail_V = result.signals.stage4_Udc.tailMean;
    result.p_pcc_tail_W = result.signals.stage4_Ppcc.tailMean;
    result.t_e_tail_Nm = result.signals.tm_T_e.tailMean;
    result.t_sh_tail_Nm = result.signals.tm_T_sh.tailMean;
    result.omega_sh_tail_radps = result.signals.tm_delta_omega_sh.tailMean;
    result.u_dc_drift_V = result.signals.stage4_Udc.last - result.signals.stage4_Udc.first;
    result.p_pcc_span_W = result.signals.stage4_Ppcc.max - result.signals.stage4_Ppcc.min;
    result.t_e_span_Nm = result.signals.tm_T_e.max - result.signals.tm_T_e.min;
catch ME
    result.status='FAIL'; result.message=sprintf('%s: %s',ME.identifier,ME.message);
end

csvPath=fullfile(here,'S7_Legacy_Progressive_Closure.csv');
fid=fopen(csvPath,'w','n','UTF-8');
if fid>0
    c=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'Stage,Status,StopTime_s,OutputComplete,UdcTail_V,UdcDrift_V,PpccTail_W,PpccSpan_W,TeTail_Nm,TeSpan_Nm,OmegaShTail_radps,Message\n');
    fprintf(fid,'C2_short_physical_compatibility,%s,%.15g,%s,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"%s"\n', ...
        result.status,result.stopTime,ternary(isfield(result,'output_complete')&&result.output_complete,'true','false'), ...
        get_num(result,'u_dc_tail_V'),get_num(result,'u_dc_drift_V'),get_num(result,'p_pcc_tail_W'), ...
        get_num(result,'p_pcc_span_W'),get_num(result,'t_e_tail_Nm'),get_num(result,'t_e_span_Nm'), ...
        get_num(result,'omega_sh_tail_radps'),csv_escape(result.message));
end
fprintf('S7 C2 diagnostic %s: %s\n',result.status,result.message);
end

function v=get_num(s,n), if isfield(s,n), v=s.(n); else, v=NaN; end, end
function y=ternary(c,a,b), if c,y=a;else,y=b;end,end
function y=csv_escape(x), y=strrep(strrep(strrep(x,'"','""'),char(13),' '),char(10),' '); end
function close_if_loaded(mdl), if bdIsLoaded(mdl), close_system(mdl,0); end, end
