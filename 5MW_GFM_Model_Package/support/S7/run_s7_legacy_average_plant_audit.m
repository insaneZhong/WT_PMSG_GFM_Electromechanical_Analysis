function audit = run_s7_legacy_average_plant_audit()
%RUN_S7_LEGACY_AVERAGE_PLANT_AUDIT
% C0/C1 的集中式接口审计。只保存一行/一项摘要，不保存完整仿真输出。

here = fileparts(mfilename('fullpath'));
mdl = 'S7_Legacy_Average_Plant';
modelPath = fullfile(here,[mdl '.slx']);
mexPath = fullfile(here,'temp','S7_5_LegacyPlant',['main_s7_legacy_avg.',mexext]);
csvPath = fullfile(here,'S7_Legacy_OpenLoop_Plant_Check.csv');
reportPath = fullfile(here,'S7_Legacy_PhysicalClosure_Gate.md');

checks = struct('name',{},'status',{},'observed',{},'criterion',{},'notes',{});
checks(end+1) = item('C0_model_copy',isfile(modelPath),modelPath,'file exists','副本存在且不覆盖 M0');
checks(end+1) = item('C1_mex',isfile(mexPath),mexPath,'file exists','隔离 MEX，不覆盖生产 MEX');

try
    addpath(fileparts(mexPath));
    load_system(modelPath);
    cleanupObj = onCleanup(@() close_if_loaded(mdl)); %#ok<NASGU>
    sub = [mdl '/MOTOR_CONTROL1'];
    names = get_param(sub,'Blocks');
    bt = cellfun(@(n)get_param([sub '/' n],'BlockType'),names,'UniformOutput',false);
    inCount = sum(strcmp(bt,'Inport'));
    outCount = sum(strcmp(bt,'Outport'));
    checks(end+1) = item('C1_input_port_count',inCount==20,inCount,'20','保留父模型端口编号');
    checks(end+1) = item('C1_output_port_count',outCount==18,outCount,'18','保留父模型诊断端口编号');
    hasLegacy = any(strcmp(names,'LegacyC'));
    hasMux = any(strcmp(names,'LegacyInputMux'));
    hasDemux = any(strcmp(names,'LegacyOutputDemux'));
    checks(end+1) = item('C1_legacy_wrapper',hasLegacy && hasMux && hasDemux, ...
        sprintf('LegacyC=%d;Mux=%d;Demux=%d',hasLegacy,hasMux,hasDemux), ...
        'LegacyC + 20路Mux + 41路Demux','包装器结构存在');
    oldIdeal = any(strcmp(bt,'MATLABSystem')) || any(contains(names,'IdealCtrl'));
    checks(end+1) = item('C1_old_controller_removed',~oldIdeal,~oldIdeal, ...
        'no IdealCtrlRHS inside wrapper','只替换控制器实现层');
    if hasLegacy
        lp = [sub '/LegacyC'];
        fn = get_param(lp,'FunctionName');
        pp = get_param(lp,'Parameters');
        checks(end+1) = item('C1_sfunction_name',strcmp(fn,'main_s7_legacy_avg'),fn, ...
            'main_s7_legacy_avg','使用隔离平均输出 MEX');
        checks(end+1) = item('C1_sfunction_parameters',strcmp(strrep(pp,' ',''),'5e6,0,563,1500'),pp, ...
            '5e6,0,563,1500','四参数接口');
    end

    % 20 路父模型输入连接检查。输入中既有 From 信号，也有
    % Sum/Product/Switch 等派生信号，因此不能只数 From 块；这里
    % 直接按 MOTOR_CONTROL1 边界端口句柄检查顶层所有连线。
    subH = get_param(sub,'Handle');
    ph = get_param(sub,'PortHandles');
    inHandles = ph.Inport;
    connectedPorts = [];
    lines = find_system(mdl,'FindAll','on','Type','Line');
    for k = 1:numel(lines)
        try
            d = get_param(lines(k),'DstBlockHandle');
            dp = get_param(lines(k),'DstPortHandle');
            if isempty(d) || d~=subH || isempty(dp) || dp<=0, continue; end
            [tf,idx] = ismember(dp,inHandles);
            if tf, connectedPorts(end+1)=idx; end %#ok<AGROW>
        catch
            % 分支线的句柄在更新期间可能暂时失效，忽略后继续审计。
        end
    end
    connectedPorts = unique(connectedPorts);
    checks(end+1) = item('C1_parent_input_connections',numel(connectedPorts)==20, ...
        sprintf('%d unique ports [%s]',numel(connectedPorts),num2str(connectedPorts)), ...
        '20 unique ports','父模型原有 From 连接未断开');

    expectedTags = {'Ideal_MSC_Ualpha','Ideal_MSC_Ubeta','Ideal_GSC_Ualpha','Ideal_GSC_Ubeta'};
    gotos = find_system(sub,'FindAll','on','Type','Block','BlockType','Goto');
    tags = cell(1,numel(gotos));
    for k=1:numel(gotos), tags{k}=get_param(gotos(k),'GotoTag'); end
    tagOK = all(ismember(expectedTags,tags));
    checks(end+1) = item('C1_average_command_tags',tagOK,strjoin(tags,','), ...
        strjoin(expectedTags,','),'四个平均电压指令标签已接入顶层平均 VSC');

    set_param(mdl,'SimulationCommand','update');
    checks(end+1) = item('C1_model_update',true,'UPDATE_OK','no update error', ...
        'Simulink 端口和 S-Function 接口更新通过');
catch ME
    checks(end+1) = item('C1_model_update',false,[ME.identifier ': ' ME.message], ...
        'no update error','当前副本仍未通过接口更新');
end

smokeLog = fullfile(here,'temp','S7_5_LegacyPlant','latest_smoke_result.txt');
smokeText = '';
if isfile(smokeLog), smokeText = fileread(smokeLog); end
smokePass = contains(smokeText,'status=PASS');
checks(end+1) = item('C1_short_smoke',smokePass,smokeText,'status=PASS', ...
    '仅为短时接口烟雾测试，不等同于固定点或物理闭合');

audit = checks;
write_audit_csv(csvPath,checks);
write_gate_report(reportPath,checks,modelPath,mexPath);
fprintf('S7 C0/C1 audit written: %s\n',csvPath);
end

function s = item(name,status,observed,criterion,notes)
s = struct('name',name,'status',logical(status),'observed',to_text(observed), ...
    'criterion',to_text(criterion),'notes',to_text(notes));
end

function t = to_text(v)
if ischar(v), t=v; elseif isstring(v), t=char(v); elseif isnumeric(v), t=mat2str(v); elseif islogical(v), t=mat2str(v); else, t=char(string(v)); end
end

function write_audit_csv(path,checks)
fid=fopen(path,'w','n','UTF-8');
if fid<0, error('S7:AuditWrite','无法写入 %s',path); end
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'Check,Status,Observed,Criterion,Notes\n');
for k=1:numel(checks)
    fprintf(fid,'%s,%s,"%s","%s","%s"\n',checks(k).name, ...
        ternary(checks(k).status,'PASS','FAIL'),csv_escape(checks(k).observed), ...
        csv_escape(checks(k).criterion),csv_escape(checks(k).notes));
end
end

function write_gate_report(path,checks,modelPath,mexPath)
fid=fopen(path,'w','n','UTF-8');
if fid<0, error('S7:ReportWrite','无法写入 %s',path); end
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
allPass=all([checks.status]);
fprintf(fid,'# S7-5C2 Legacy Controller–Average Plant Gate\n\n');
fprintf(fid,'- 模型副本：`%s`\n- 隔离 MEX：`%s`\n- 当前总状态：**%s**\n\n', ...
    modelPath,mexPath,ternary(allPass,'C1 PASS（仅接口）','C1 FAIL'));
fprintf(fid,'## 审计项目\n\n|项目|状态|观测|判据|\n|---|---|---|---|\n');
for k=1:numel(checks)
    fprintf(fid,'|%s|%s|%s|%s|\n',checks(k).name, ...
        ternary(checks(k).status,'PASS','FAIL'),checks(k).observed,checks(k).criterion);
end
fprintf(fid,'\n## 边界\n\n');
fprintf(fid,'本报告只证明 C0/C1 的副本、接口和极短仿真可解析；它**不**证明共同周期固定点、功率/转矩能量闭合，也不证明 Legacy 闭环稳定。C2 开环兼容、C3 逐环闭合、C4/C5 固定点和 C6 扰动 Gate 仍需按计划逐级执行。\n');
end

function y=ternary(cond,a,b), if cond, y=a; else, y=b; end, end
function y=csv_escape(x), y=strrep(strrep(strrep(x,'"','""'),char(13),' '),char(10),' '); end
function close_if_loaded(mdl), if bdIsLoaded(mdl), close_system(mdl,0); end, end
