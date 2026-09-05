function status = run_emt_representative_gate()
%RUN_EMT_REPRESENTATIVE_GATE
% 记录当前开关 EMT 代表工况的独立验证门状态。
%
% 本程序只保存紧凑的验收结果，不保存 SimulationOutput、完整时序或
% solver history。它把本轮实际执行的 EMT 检查与已有历史 EMT 摘要放在
% 同一张表中，明确区分“可用于论文的证据”和“尚未通过的尝试”。

root = fileparts(mfilename('fullpath'));
outCsv = fullfile(root,'EMT_Representative_Gate_Status.csv');
outMd  = fullfile(root,'EMT_Representative_Gate_Report_CN.md');

% M0 共同工作点（来自 ThreeControl_Summary.csv）。
m0f = 2.4942097109;
m0z = 0.0298105190;

caseName = [
    "当前模型冷启动 6 s";
    "HotEMT_Validated 冷启动 6 s";
    "历史独立 EMT 摘要（4 MW）";
    "M0 连续平均基准（参照）" ];
modelName = [
    "Grid_Forming_PMSG5MW_Liu2024_TwoMass";
    "Grid_Forming_PMSG5MW_Liu2024_TwoMass_HotEMT_Validated";
    "Grid_Forming_PMSG5MW_Liu2024_TwoMass.slx";
    "M0 continuous average" ];
startMode = ["cold";"cold";"cold";"equilibrium"];
workpointStatus = ["not_settled";"not_settled";"noncommon_4MW";"strict_common"];
P_MW = [1.88068;1.74020;4.00000;5.00000];
Udc_V = [1484.33;1497.37;NaN;1500.00];
f_Hz = [NaN;NaN;2.7179718019;m0f];
zeta = [NaN;NaN;0.0189219101;m0z];
preconditionPass = [false;false;false;true];
independentGate = [false;false;false;true];
reason = [
    "6 s 后仍处于启动调理段，P/Udc 未达到稳态，不能施加独立小扰动";
    "热模型冷启动仍未达到共同稳态，不能作为 M0 对照";
    "历史数据的事件前 Udc/P 波动和限幅门失败，且运行点为 4 MW";
    "共同平衡点、全部极点和连续非线性小扰动对齐均通过" ];

status = table(caseName,modelName,startMode,workpointStatus,P_MW,Udc_V, ...
    f_Hz,zeta,preconditionPass,independentGate,reason);
writetable(status,outCsv);

fid = fopen(outMd,'w','n','UTF-8');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'# 5 MW 开关 EMT 代表工况 Gate B 状态\n\n');
fprintf(fid,'生成时间：%s\n\n',char(datetime('now','Format','yyyy-MM-dd HH:mm:ss')));
fprintf(fid,'## 当前 M0 参照\n\n');
fprintf(fid,'- 轴系模态：`f_tor=%.9f Hz`，`zeta_tor=%.9f`。\n',m0f,m0z);
fprintf(fid,'- 本报告只保留汇总，不保存高频开关时序。\n\n');
fprintf(fid,'## 本轮实际检查\n\n');
fprintf(fid,'|工况|运行点状态|P(MW)|Udc(V)|频率(Hz)|阻尼比|前置稳态门|Gate B|\n');
fprintf(fid,'|---|---|---:|---:|---:|---:|---|---|\n');
for k=1:height(status)
    if isnan(status.f_Hz(k)), fs='—'; else, fs=sprintf('%.6f',status.f_Hz(k)); end
    if isnan(status.zeta(k)), zs='—'; else, zs=sprintf('%.6f',status.zeta(k)); end
    fprintf(fid,'|%s|%s|%.5g|%.5g|%s|%s|%s|%s|\n', ...
        status.caseName(k),status.workpointStatus(k),status.P_MW(k), ...
        status.Udc_V(k),fs,zs,passText(status.preconditionPass(k)), ...
        passText(status.independentGate(k)));
end
fprintf(fid,'\n## 结论\n\n');
fprintf(fid,'当前 EMT Gate B **尚未通过**。原因不是 M0 不稳定，而是可用 EMT 快照与当前模型的校验和/端口结构已漂移，旧快照不能直接加载；本轮冷启动 6 s 也未达到稳态。历史 4 MW EMT 摘要的事件前稳态门和限幅门均失败，不能用来证明 5 MW M0—EMT 对应。\n\n');
fprintf(fid,'因此暂不报告 EMT 阻尼误差或跨模型排序。下一步必须在当前 EMT 模型版本上重新生成被接受的 ModelOperatingPoint（或从当前冷启动继续到稳态），再用同一工作点施加机械侧与电气侧小扰动。\n');
end

function s = passText(v)
if v, s='PASS'; else, s='FAIL'; end
end
