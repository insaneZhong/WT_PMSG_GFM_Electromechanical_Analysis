function report = audit_current_copy()
% 审计当前5 MW开关模型副本，不修改模型。
% 输出：00_Copy_and_Structure_Audit_CN.md（精简汇总，不保存高频波形）。
mdl = 'Grid_Forming_PMSG5MW_Liu2024_TwoMass';
here = fileparts(mfilename('fullpath'));
addpath(here);
load_system(fullfile(here,[mdl '.slx']));

blks = find_system(mdl,'LookUnderMasks','all','FollowLinks','on','Type','Block');
rows = cell(numel(blks),8);
for k = 1:numel(blks)
    b = blks{k};
    rows{k,1}=b;
    rows{k,2}=safeget(b,'BlockType');
    rows{k,3}=safeget(b,'MaskType');
    rows{k,4}=safeget(b,'SampleTime');
    rows{k,5}=safeget(b,'FunctionName');
    rows{k,6}=safeget(b,'DelayLength');
    rows{k,7}=safeget(b,'TimeDelay');
    rows{k,8}=safeget(b,'Name');
end

isText = @(x) ischar(x) || isstring(x);
hit = false(size(rows,1),1);
for k=1:size(rows,1)
    blob = lower(strjoin(string(rows(k,:)),' '));
    hit(k)=contains(blob,'s-function') || contains(blob,'discrete') || ...
        contains(blob,'delay') || contains(blob,'unit delay') || contains(blob,'zero-order') || ...
        contains(blob,'pwm') || contains(blob,'universal bridge') || contains(blob,'limiter') || ...
        contains(blob,'saturation') || contains(blob,'switch');
end

report = struct();
report.model = get_param(mdl,'FileName');
report.allBlocks = rows;
report.flagged = rows(hit,:);
report.nBlocks = numel(blks);
report.nFlagged = nnz(hit);
report.time = char(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));

out = fullfile(here,'00_Copy_and_Structure_Audit_CN.md');
fid=fopen(out,'w','n','UTF-8');
cleanup=onCleanup(@() fclose(fid));
fprintf(fid,'# 当前5 MW模型副本结构审计\n\n');
fprintf(fid,'- 模型：`%s.slx`\n- 审计时间：%s\n- 总块数：%d\n- 命中候选离散/延迟/桥/限幅/开关块：%d\n\n',mdl,report.time,report.nBlocks,report.nFlagged);
fprintf(fid,'## 审计结论\n\n');
fprintf(fid,'本轮只读审计，未修改副本，也未修改原始模型。后续理想化只在本目录副本进行。\n\n');
fprintf(fid,'## 必须替换或旁路的候选块\n\n');
fprintf(fid,'|路径|BlockType|MaskType|SampleTime|FunctionName|DelayLength|TimeDelay|名称|\n|---|---|---|---|---|---|---|---|\n');
for k=find(hit).'
    args=cellfun(@esc,rows(k,:),'UniformOutput',false);
    fprintf(fid,'|%s|%s|%s|%s|%s|%s|%s|%s|\n',args{:});
end
fprintf(fid,'\n## 理想化边界\n\n');
fprintf(fid,'保留：两质量轴系、PMSG电流动态、MSC/GSC连续PI状态、DC-link能量、P/Q滤波、VSG惯量/功角、LCL和电网。\n\n');
fprintf(fid,'删除或替换：控制采样调度、PWM/SVPWM、数字延迟、PI限幅/抗积分饱和、参考斜率限制、PLL/预同步/GFM接管、主动阻尼，以及PMSG内部与外部两质量轴系重复的离散机械积分。\n\n');
fprintf(fid,'## 下一步\n\n');
fprintf(fid,'1. 复制完成并确认原始模型不变；2. 替换控制器S-Function为透明连续状态实现；3. 用理想连续三相受控电压源替换两个物理桥；4. 统一MSC/GSC交流端口功率面并重建唯一DC-link能量状态；5. 编译、求平衡点并检查极点。\n');
close_system(mdl,0);
end

function y=safeget(b,p)
try, y=get_param(b,p); catch, y=''; end
if iscell(y), y=strjoin(string(y),','); end
if isnumeric(y), y=mat2str(y); end
if isempty(y), y=''; end
end

function y=esc(x)
y=char(string(x));
y=strrep(y,'|','\\|');
y=strrep(y,newline,' ');
end
