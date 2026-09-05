function modelPath = build_s7_legacy_hotstart_plant(doUpdate, doSave)
%BUILD_S7_LEGACY_HOTSTART_PLANT
% 建立与冷启动 S7 副本分离的 Legacy 热启动模型。
%
% 只在 S7_Legacy_HotStart_Average_Plant.slx 上操作：
%   1) LegacyC 使用带 32 元素状态向量的隔离热启动 MEX；
%   2) InitFcn 载入由 M0 工作点反求的热启动配置；
%   3) 仅在该副本中关闭依赖电感的重复初值；
%   4) 不修改 M0、冷启动 S7 模型或生产控制源。

if nargin < 1 || isempty(doUpdate), doUpdate = true; end
if nargin < 2 || isempty(doSave),   doSave = true;  end

here = fileparts(mfilename('fullpath'));
modelPath = fullfile(here,'S7_Legacy_HotStart_Average_Plant.slx');
if ~isfile(modelPath)
    error('S7:MissingHotCopy','找不到热启动模型副本：%s',modelPath);
end

hotDir = fullfile(here,'temp','S7_5_LegacyPlant');
cfgFile = fullfile(hotDir,'S7_Legacy_HotStart_Config.mat');
assert(isfile(cfgFile),'请先运行 prepare_s7_legacy_hotstart 生成 %s',cfgFile);
addpath(hotDir);

mdl = 'S7_Legacy_HotStart_Average_Plant';
load_system(modelPath);
cleanupObj = onCleanup(@() close_if_loaded(mdl)); %#ok<NASGU>

% 副本 InitFcn 复用冷启动 S7 副本的参数初始化。每次都从冷启动副本
% 读取基准回调，避免重复调用本构建器后不断叠加 sourceLine/cfgLine。
coldMdl = 'S7_Legacy_Average_Plant';
coldPath = fullfile(here,[coldMdl '.slx']);
assert(isfile(coldPath),'找不到冷启动基准副本：%s',coldPath);
load_system(coldPath);
oldInit = get_param(coldMdl,'InitFcn');
if bdIsLoaded(coldMdl), close_system(coldMdl,0); end
sourceDir = fullfile(fileparts(here),'CurrentModel_Idealized');
oldExpr = 'fileparts(get_param(bdroot,''FileName''))';
initBody = strrep(oldInit,oldExpr,'s7SourceDir');
mexLine = sprintf("addpath('%s');",strrep(hotDir,'\\','/'));
cfgLine = sprintf(['hotCfgFile=''%s''; ' ...
    'hotCfg=load(hotCfgFile,''hotstart_vector'',''hotstart_vref_V''); ' ...
    'S7_LegacyHotStartVector=hotCfg.hotstart_vector; ' ...
    'S7_LegacyHotStartVref=hotCfg.hotstart_vref_V;'], ...
    strrep(cfgFile,'\\','/'));
% The retained SPS damping branches are oriented from the capacitor node
% toward the reference node.  Their branch-current sign is therefore the
% opposite of the positive converter-to-grid current used by OP.if/OP.ig.
% Apply this correction only in the isolated hot-start plant; the M0 source
% initialization and the cold Legacy copy remain untouched.
hotLclSignLine = ['IdealCfIabc0=-IdealCfIabc0; ' ...
    '% S7 hot-start SPS damping-branch orientation correction'];
% The existing SPS initialization uses the capacitor-branch state as an
% algebraic dependent state.  With the original ICs the first measured PCC
% voltage is about [131.779 -534.241 402.461] V (abc), whereas the
% source-aligned equilibrium requires [140.941 -536.280 395.339] V.  Apply
% this fixed, balanced correction only to the hot copy so its first plant
% sample is on the same voltage measurement surface as the controller.
hotLclVoltageLine = ['IdealCfVabc0=IdealCfVabc0+' ...
    '[9.162; -2.039; -7.122]; ' ...
    '% S7 hot-start PCC measurement-plane correction'];
if isempty(initBody)
    newInit = sprintf("s7SourceDir='%s';\n%s\n%s\n%s\n%s", ...
        strrep(sourceDir,'\\','/'),hotLclSignLine,hotLclVoltageLine,mexLine,cfgLine);
else
    newInit = sprintf('%s\n%s\n%s\n%s\n%s',initBody,hotLclSignLine, ...
        hotLclVoltageLine,mexLine,cfgLine);
end
set_param(mdl,'InitFcn',newInit);

% 替换 Legacy S-function 的名称和第五个热启动状态参数。
legacyBlock = [mdl '/MOTOR_CONTROL1/LegacyC'];
set_param(legacyBlock,'FunctionName','main_s7_legacy_hot', ...
    'Parameters','-5e6,0,S7_LegacyHotStartVref,1500,S7_LegacyHotStartVector');

% Powergui 不应再把 L1/L3 依赖电感当作独立状态初始化；
% 仅修改热启动副本，避免与外部 plant 初值重复定义。
disable_dependent_inductor_ic(mdl);

set_param(mdl,'LoadInitialState','off');
if doUpdate
    set_param(mdl,'SimulationCommand','update');
end
if doSave
    save_system(mdl,modelPath);
end
fprintf('S7 Legacy hot-start plant ready: %s\n',modelPath);
end

function disable_dependent_inductor_ic(mdl)
% 仅处理警告中出现的依赖电感；找不到时不改变其它块。
targets = find_system(mdl,'LookUnderMasks','all','FollowLinks','on', ...
    'Name','L1');
targets = [targets; find_system(mdl,'LookUnderMasks','all','FollowLinks','on', ...
    'Name','L3')]; %#ok<AGROW>
for k = 1:numel(targets)
    b = targets{k};
    try
        op = get_param(b,'ObjectParameters');
        if isfield(op,'Setx0'),         set_param(b,'Setx0','off');       end
        if isfield(op,'InitialCurrent'),set_param(b,'InitialCurrent','[]');end
        if isfield(op,'InitialCondition'),set_param(b,'InitialCondition','[]');end
    catch ME
        warning('S7:InductorIC','无法修改 %s 的初值参数：%s',b,ME.message);
    end
end
end

function close_if_loaded(mdl)
if bdIsLoaded(mdl)
    close_system(mdl,0);
end
end
