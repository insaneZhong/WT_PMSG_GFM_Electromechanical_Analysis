function contract = establish_output_alignment_contract(writeReport)
%ESTABLISH_OUTPUT_ALIGNMENT_CONTRACT
% 为“当前模型理想化副本—M0同源小信号模型”建立唯一输出对齐合同。
% 本函数只检查名称、单位、测量面和待验证的正负号，不保存原始时序。

if nargin < 1, writeReport = true; end
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(root);
mdl = 'Grid_Forming_PMSG5MW_TwoMass_Idealized';

% 目标端是已经通过同源非线性—SSM验收的 M0 输出定义。
m0Names = m0_output_names();
m0Required = ["P_PCC_W","Udc_V","Tgen_Nm","Tshaft_Nm", ...
    "omega_rel_radps","omega_vsg_radps"];

% 第一列为 M0 输出；第二列为当前副本 ToWorkspace 变量；第三列只用于
% 记录待验证的物理符号，绝不能在未通过功率/转矩单元测试前用于改图。
currentNames = ["stage4_Ppcc","stage4_Udc","tm_T_e","tm_T_sh", ...
    "tm_delta_omega_sh","gfm_omega_vsg"];
units = ["W","V","N*m","N*m","rad/s","rad/s"];
measurementPlanes = [ ...
    "PCC export power (not GSC AC-port power)", ...
    "unique DC-link state", ...
    "generator electromagnetic/braking torque", ...
    "two-mass shaft torque", ...
    "omega_t - omega_g", ...
    "absolute VSG angular frequency"];
signStatus = [ ...
    "verify: export convention differs in current model", ...
    "fixed: positive voltage", ...
    "verify: must match Jg*dwg=Tshaft-Tgen", ...
    "verify: must match Ksh*theta+Dsh*(wt-wg)", ...
    "fixed: measured name and short-run data confirm wt-wg", ...
    "fixed: absolute angular frequency"];

assert(all(ismember(m0Required,m0Names)), ...
    'M0 输出定义缺少论文对齐必需量。');

wasLoaded = bdIsLoaded(mdl);
if ~wasLoaded
    load_system(fullfile(here,[mdl '.slx']));
end
cleanupObj = onCleanup(@() closeIfNeeded(mdl,wasLoaded)); %#ok<NASGU>
blocks = find_system(mdl,'LookUnderMasks','all','FollowLinks','on', ...
    'BlockType','ToWorkspace');
actual = strings(numel(blocks),1);
for k=1:numel(blocks)
    actual(k) = string(get_param(blocks{k},'VariableName'));
end
available = ismember(currentNames,actual);

contract = table(m0Required.',currentNames.',units.',measurementPlanes.', ...
    signStatus.',available.', ...
    'VariableNames',{'M0_output','CurrentModel_variable','Unit', ...
    'Measurement_plane','Sign_requirement','Available'});

if writeReport
    report = fullfile(here,'04_Output_Alignment_Contract_CN.md');
    fid = fopen(report,'w','n','UTF-8');
    assert(fid>0,'无法写入输出对齐合同。');
    c = onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'# 当前理想化副本与M0小信号模型：输出对齐合同\n\n');
    fprintf(fid,'本合同规定后续所有非线性—小信号比较只使用下列六个输出。\n\n');
    fprintf(fid,'| M0输出 | 当前模型变量 | 单位 | 测量面 | 符号要求 | 已存在 |\n|---|---|---|---|---|---|\n');
    for k=1:height(contract)
        fprintf(fid,'| `%s` | `%s` | %s | %s | %s | %s |\n', ...
            contract.M0_output(k),contract.CurrentModel_variable(k), ...
            contract.Unit(k),contract.Measurement_plane(k), ...
            contract.Sign_requirement(k),ternary(contract.Available(k),'yes','no'));
    end
    fprintf(fid,'\n## 对齐验收顺序\n\n');
    fprintf(fid,'1. 在稳定平衡点完成端口 abc/dq 功率与 DC-link 能量方向测试；\n');
    fprintf(fid,'2. 用两质量方程确认 `Tgen`、`Tshaft` 的正方向；\n');
    fprintf(fid,'3. 对同一 dPref 小阶跃比较六个输出的频率、阻尼、峰值与相位；\n');
    fprintf(fid,'4. 仅当六项均通过后，才将该副本用于复转矩和论文参数扫描。\n');
end
end

function y = ternary(condition,a,b)
if condition, y=a; else, y=b; end
end

function closeIfNeeded(mdl,wasLoaded)
if ~wasLoaded && bdIsLoaded(mdl)
    close_system(mdl,0);
end
end
