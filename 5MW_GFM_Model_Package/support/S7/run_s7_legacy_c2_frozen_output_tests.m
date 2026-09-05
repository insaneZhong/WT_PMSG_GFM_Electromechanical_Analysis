function T = run_s7_legacy_c2_frozen_output_tests(stopTime)
%RUN_S7_LEGACY_C2_FROZEN_OUTPUT_TESTS
% C2 的冻结输出/方向测试。LegacyC 被临时旁路，四路平均电压指令取
% 同源 M0 工作点命令，再逐一翻转 MSC/GSC 符号。模型只在内存中修改，
% 不保存模型和原始时序；最终只写一张摘要 CSV。

if nargin < 1 || isempty(stopTime), stopTime = 5e-3; end
here = fileparts(mfilename('fullpath'));
mdl = 'S7_Legacy_Average_Plant';
modelFile = fullfile(here,[mdl '.slx']);
assert(isfile(modelFile),'找不到模型副本：%s',modelFile);
sourceDir = fullfile(fileparts(here),'CurrentModel_Idealized');
addpath(sourceDir);

% 从与 M0 相同的源对齐工作点计算实际 alpha-beta 电压命令。
OP = solve_currentmodel_source_aligned_equilibrium();
p = OP.pvec(:); c = OP.controller_x0(:);
dq2abc = @(d,q,th) [d*cos(th)-q*sin(th); ...
    -0.5*(d*cos(th)-q*sin(th))+sqrt(3)/2*(d*sin(th)+q*cos(th)); ...
    -0.5*(d*cos(th)-q*sin(th))-sqrt(3)/2*(d*sin(th)+q*cos(th))];
imabc = dq2abc(OP.pmsg_id0,OP.pmsg_iq0,0);
thetaGrid = -pi/2;
ifabc = dq2abc(OP.if_grid_dq_A(1),OP.if_grid_dq_A(2),thetaGrid);
igabc = dq2abc(OP.ig_grid_dq_A(1),OP.ig_grid_dq_A(2),thetaGrid);
vpabc = dq2abc(OP.vnode_grid_dq_V(1),OP.vnode_grid_dq_V(2),thetaGrid);
u = [imabc(:);1500;OP.omega0;0;ifabc(:); ...
    vpabc(1)-vpabc(2);vpabc(2)-vpabc(3);vpabc(3)-vpabc(1);igabc(:); ...
    0;0;OP.P_gsc_W;0;1500];
z = currentmodel_continuous_controller_io([u;c;p]);
cmd0 = z(11+[38 39 40 41]);

load_system(modelFile);
cleanupObj = onCleanup(@() close_if_loaded(mdl)); %#ok<NASGU>
sub = [mdl '/MOTOR_CONTROL1'];
legacy = [sub '/LegacyC']; demux = [sub '/LegacyOutputDemux'];
oldLine = get_param(get_param(legacy,'PortHandles').Outport(1),'Line');
if oldLine > 0, delete_line(oldLine); end
set_param(legacy,'Commented','on');
frozen = [sub '/FrozenLegacyVector'];
add_block('simulink/Sources/Constant',frozen,'Value',mat2str([zeros(37,1);cmd0],17), ...
    'Position',[245 530 405 570]);
add_line(sub,'FrozenLegacyVector/1','LegacyOutputDemux/1','autorouting','on');

cases = { ...
    'baseline_same_sign', 1, 1, '同源 M0 alpha-beta 命令';
    'msc_sign_flip',     -1, 1, '仅翻转 MSC 命令';
    'gsc_sign_flip',      1,-1, '仅翻转 GSC 命令';
    'both_sign_flip',    -1,-1, '同时翻转 MSC/GSC 命令'};
rows = cell(size(cases,1),18);
for k=1:size(cases,1)
    name=cases{k,1}; sm=cases{k,2}; sg=cases{k,3}; note=cases{k,4};
    vec=[zeros(37,1);sm*cmd0(1);sm*cmd0(2);sg*cmd0(3);sg*cmd0(4)];
    set_param(frozen,'Value',mat2str(vec,17));
    set_param(mdl,'StopTime',num2str(stopTime,'%.17g'), ...
        'ReturnWorkspaceOutputs','on','SaveOutput','off','SaveTime','off');
    row={name,'FAIL',stopTime,0,NaN,NaN,NaN,NaN,NaN,NaN, ...
        NaN,NaN,NaN,NaN,NaN,NaN,'',note};
    try
        set_param(mdl,'SimulationCommand','update');
        simOut=sim(mdl,'StopTime',num2str(stopTime,'%.17g'), ...
            'SimulationMode','normal','ReturnWorkspaceOutputs','on');
        row{2}='PASS';
        [udc,okU]=summarySignal(simOut,'stage4_Udc');
        [te,okTe]=summarySignal(simOut,'tm_T_e');
        [tsh,okTsh]=summarySignal(simOut,'tm_T_sh'); %#ok<ASGLU>
        [pcc,okP]=summarySignal(simOut,'stage4_Ppcc');
        [pmsc,okPmsc]=summarySignal(simOut,'ideal_Pmsc_ac');
        [pgsc,okPgsc]=summarySignal(simOut,'ideal_Pgsc_ac');
        [udcs,okUdcs]=summarySignal(simOut,'ideal_Udc_state');
        row{4}=double(okU&&okTe&&okTsh&&okP);
        if okU, row{5}=udc.tail; row{6}=udc.last-udc.first; end
        if okP, row{7}=pcc.tail; row{8}=pcc.max-pcc.min; end
        if okTe, row{9}=te.tail; row{10}=te.max-te.min; end
        if okPmsc, row{11}=pmsc.tail; row{13}=pmsc.max-pmsc.min; end
        if okPgsc, row{12}=pgsc.tail; row{14}=pgsc.max-pgsc.min; end
        if okUdcs, row{15}=udcs.tail; row{16}=udcs.last-udcs.first; end
        row{17}=sprintf('all_outputs=%d',row{4});
    catch ME
        row{11}=strrep(strrep(ME.message,char(13),' '),char(10),' ');
    end
    rows(k,:)=row;
end

T=cell2table(rows,'VariableNames',{'Case','Status','StopTime_s','OutputsComplete', ...
    'UdcTail_V','UdcDrift_V','PpccTail_W','PpccSpan_W','TeTail_Nm','TeSpan_Nm', ...
    'PmscTail_W','PgscTail_W','PmscSpan_W','PgscSpan_W','UdcStateTail_V', ...
    'UdcStateDrift_V','Message','Notes'});
outFile=fullfile(here,'S7_Legacy_C2_Frozen_Output_Tests.csv');
writetable(T,outFile);
fprintf('S7 C2 frozen-output tests written: %s\n',outFile);
end

function [r,ok]=summarySignal(simOut,name)
r=struct('tail',NaN,'last',NaN,'first',NaN,'min',NaN,'max',NaN); ok=false;
try
    s=simOut.get(name);
    if ~isa(s,'timeseries'), return; end
    y=double(s.Data(:));
    if isempty(y)||any(~isfinite(y)), return; end
    idx=max(1,numel(y)-100):numel(y);
    r.first=y(1); r.last=y(end); r.tail=mean(y(idx));
    r.min=min(y); r.max=max(y); ok=true;
catch
end
end

function close_if_loaded(mdl)
if bdIsLoaded(mdl), close_system(mdl,0); end
end
