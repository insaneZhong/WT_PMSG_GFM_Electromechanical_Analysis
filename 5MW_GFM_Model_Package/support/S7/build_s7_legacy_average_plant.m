function modelPath = build_s7_legacy_average_plant(doUpdate, doSave)
%BUILD_S7_LEGACY_AVERAGE_PLANT
% 建立 S7-5C2 隔离 Legacy_Average_Plant。
%
% 该脚本只修改复制出来的 S7_Legacy_Average_Plant.slx：
%   - 原 M0/理想连续模型不修改；
%   - MOTOR_CONTROL1 内部替换为 Legacy C 平均输出包装器；
%   - 20 个输入和 18 个外部诊断输出保持原端口编号；
%   - 端口 38--41 的 alpha-beta 电压指令接入两个平均 VSC；
%   - 暂不进行长时仿真或固定点求解，先完成 C0/C1 接口审计。
%
% modelPath = build_s7_legacy_average_plant(true,true)
% modelPath = build_s7_legacy_average_plant(false,false)

if nargin < 1 || isempty(doUpdate), doUpdate = true; end
if nargin < 2 || isempty(doSave),   doSave = true;  end

here = fileparts(mfilename('fullpath'));
modelPath = fullfile(here, 'S7_Legacy_Average_Plant.slx');
if ~isfile(modelPath)
    error('S7:MissingCopy', '找不到隔离模型副本：%s', modelPath);
end

mexDir = fullfile(here, 'temp', 'S7_5_LegacyPlant');
if ~isfolder(mexDir)
    mkdir(mexDir);
end
addpath(mexDir);

mdl = 'S7_Legacy_Average_Plant';
load_system(modelPath);
cleanupObj = onCleanup(@() close_if_loaded(mdl)); %#ok<NASGU>

% 保存原 InitFcn 的参数初始化。复制模型后，原回调中的
% fileparts(get_param(bdroot,'FileName')) 已不再指向参数脚本目录，
% 因此只在副本内把这个目录表达式替换成 CurrentModel_Idealized；
% 原模型回调文本完全不改。
oldInit = get_param(mdl, 'InitFcn');
sourceDir = fullfile(fileparts(here), 'CurrentModel_Idealized');
oldExpr = 'fileparts(get_param(bdroot,''FileName''))';
initBody = strrep(oldInit, oldExpr, 's7SourceDir');
mexLine = sprintf("addpath('%s');", strrep(mexDir, '\\', '/'));
sourceLine = sprintf("s7SourceDir='%s';", strrep(sourceDir, '\\', '/'));
if isempty(initBody)
    newInit = sprintf('%s\n%s', sourceLine, mexLine);
else
    newInit = sprintf('%s\n%s\n%s', sourceLine, initBody, mexLine);
end
set_param(mdl, 'InitFcn', newInit);

sub = [mdl '/MOTOR_CONTROL1'];
replace_controller_contents(sub);

if doUpdate
    % update 只检查端口、Goto/From 和 S-Function 接口，不执行长时仿真。
    set_param(mdl, 'SimulationCommand', 'update');
end
if doSave
    save_system(mdl, modelPath);
end

fprintf('S7 Legacy average plant ready: %s\n', modelPath);
end

function replace_controller_contents(sub)
% 删除 M0 的显式连续控制器，只保留同端口号的 Legacy 包装器。
% M0 的 MOTOR_CONTROL1 带有只用于旧连续控制器参数的空掩码；
% 包装器不再使用这些掩码变量。关闭掩码可以避免 Simulink 在
% 重建内部端口时把掩码端口连接误判为目标子系统端口冲突。
try
    set_param(sub, 'Mask', 'off');
catch ME
    error('S7:MaskRemoval', '无法关闭旧控制器掩码：%s', ME.message);
end
% `LineHandles` 只返回子系统边界端口的连接，不能清理内部线；
% 必须使用 `Lines` 返回的每条内部线句柄。
oldLines = get_param(sub, 'Lines');
for k = numel(oldLines):-1:1
    try
        delete_line(oldLines(k).Handle);
    catch ME
        error('S7:DeleteControllerLine', '删除旧控制器内部连线失败：%s', ME.message);
    end
end

oldBlocks = get_param(sub, 'Blocks');
for k = 1:numel(oldBlocks)
    % 保留原有 Inport/Outport：它们与父模型边界线相连，删除后外部
    % From/Outport 连接会被 Simulink 自动断开。只删除内部控制实现。
    blockPath = [sub '/' oldBlocks{k}];
    if any(strcmp(get_param(blockPath, 'BlockType'), {'Inport','Outport'}))
        continue;
    end
    try
        delete_block(blockPath);
    catch ME
        error('S7:DeleteController', '删除旧 MOTOR_CONTROL1 子块失败 %s: %s', ...
            oldBlocks{k}, ME.message);
    end
end

% 20 个输入：编号与原 MOTOR_CONTROL1 完全一致。端口块沿用 M0，
% 以保留父模型已有的 From -> MOTOR_CONTROL1 外部连线。
inNames = {'Ima','Imb','Imc','Udc','We','Position','Iga','Igb','Igc', ...
    'Uab','Ubc','Uca','Igga','Iggb','Iggc','clockTime','omega_rel_ad', ...
    'P_ref_MPPT','ActiveDampingScale','Vdc_ref_profile'};
add_block('simulink/Signal Routing/Mux', [sub '/LegacyInputMux'], ...
    'Inputs', '20', 'Position', [155 230 185 770]);
for k = 1:numel(inNames)
    add_line(sub, [inNames{k} '/1'], sprintf('LegacyInputMux/%d',k), ...
        'autorouting', 'on');
end

add_block('simulink/User-Defined Functions/S-Function', [sub '/LegacyC'], ...
    'FunctionName', 'main_s7_legacy_avg', ...
    'Parameters', '5e6,0,563,1500', ...
    'Position', [245 425 405 475]);
add_line(sub, 'LegacyInputMux/1', 'LegacyC/1', 'autorouting', 'on');

add_block('simulink/Signal Routing/Demux', [sub '/LegacyOutputDemux'], ...
    'Outputs', '41', 'Position', [465 55 500 875]);
add_line(sub, 'LegacyC/1', 'LegacyOutputDemux/1', 'autorouting', 'on');

% 18 个外部诊断端口：Legacy 向量的第 13--30 项。端口块沿用 M0，
% 因此父模型原有输出连接保持有效。
for k = 1:18
    outName = sprintf('Out%d', 11+k);
    y = 45 + (k-1)*42;
    add_line(sub, sprintf('LegacyOutputDemux/%d',12+k), [outName '/1'], ...
        'autorouting', 'on');
end

% 四个平均交流电压指令，保持原有 Goto 标签，直接接入顶层平均 VSC。
gotoInfo = { ...
    'Legacy_MSC_Ualpha','Ideal_MSC_Ualpha',38, ...
    'Legacy_MSC_Ubeta', 'Ideal_MSC_Ubeta', 39, ...
    'Legacy_GSC_Ualpha','Ideal_GSC_Ualpha',40, ...
    'Legacy_GSC_Ubeta', 'Ideal_GSC_Ubeta', 41};
for k = 1:4
    base = (k-1)*3 + 1;
    add_block('simulink/Signal Routing/Goto', [sub '/' gotoInfo{base}], ...
        'GotoTag', gotoInfo{base+1}, 'TagVisibility', 'global', ...
        'Position', [575 250+(k-1)*95 660 280+(k-1)*95]);
    add_line(sub, sprintf('LegacyOutputDemux/%d',gotoInfo{base+2}), ...
        [gotoInfo{base} '/1'], 'autorouting', 'on');
end

% 原模型的两个启动延时块仍读取 Pulse1/Pulse2；用零源保持接口可解析，
% 但不把它们接入任何物理控制路径。
for k = 1:2
    cname = sprintf('Pulse%dZero', k);
    gname = sprintf('Pulse%dSource', k);
    add_block('simulink/Sources/Constant', [sub '/' cname], ...
        'Value', '0', 'Position', [575 700+(k-1)*60 625 730+(k-1)*60]);
    add_block('simulink/Signal Routing/Goto', [sub '/' gname], ...
        'GotoTag', sprintf('Pulse%d',k), 'TagVisibility', 'global', ...
        'Position', [660 700+(k-1)*60 745 730+(k-1)*60]);
    add_line(sub, [cname '/1'], [gname '/1'], 'autorouting', 'on');
end

% 未使用的诊断端口由 Terminator 吸收，避免把它们误接到控制路径。
unused = [1:12 31:37];
for k = 1:numel(unused)
    tname = sprintf('Unused%02dTerm', unused(k));
    y = 920 + (k-1)*22;
    add_block('simulink/Sinks/Terminator', [sub '/' tname], ...
        'Position', [575 y 605 y+14]);
    add_line(sub, sprintf('LegacyOutputDemux/%d',unused(k)), [tname '/1'], ...
        'autorouting', 'on');
end
end

function close_if_loaded(mdl)
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
end
