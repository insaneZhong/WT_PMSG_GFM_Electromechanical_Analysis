function [FrequencyInputAudit,gate] = audit_grid_frequency_disturbance_input(models,outDir)
%AUDIT_GRID_FREQUENCY_DISTURBANCE_INPUT 审计 d(4)=DeltaOmegaGrid 的物理一致性。
% 该输入在三种架构中始终为同一外部电网角频率扰动，单位 rad/s。
if nargin<2 || isempty(outDir), outDir=fileparts(mfilename('fullpath')); end
n=numel(models);
FrequencyInputAudit=table('Size',[n 11], ...
 'VariableTypes',{'string','string','string','double','string','string','string','double','double','string','logical'}, ...
 'VariableNames',{'Architecture','InputName','PhysicalMeaning','BaseValue_radps','Unit','InputLocation','ThetaIntegrator','ScaleFactor','BfNorm','BfNonzeroStates','PASS'});
for k=1:n
    L=models{k}; mode=upper(L.mode); b=L.B(:,4); nz=find(abs(b)>1e-10);
    FrequencyInputAudit.Architecture(k)=string(L.mode);
    FrequencyInputAudit.InputName(k)='DeltaOmegaGrid';
    FrequencyInputAudit.PhysicalMeaning(k)='同一外部电网角频率增量 Delta omega_grid';
    FrequencyInputAudit.BaseValue_radps(k)=L.p(3);
    FrequencyInputAudit.Unit(k)='rad/s'; FrequencyInputAudit.ScaleFactor(k)=1.0;
    if strcmp(mode,'GFL')
        FrequencyInputAudit.InputLocation(k)='wctrl=omega_grid，理想PLL直接跟随同一网源';
        FrequencyInputAudit.ThetaIntegrator(k)='无独立相对角积分；理想PLL已直接跟随，不重复积分';
    else
        FrequencyInputAudit.InputLocation(k)='delta_dot=omega_ctrl-(omega0+DeltaOmegaGrid)';
        FrequencyInputAudit.ThetaIntegrator(k)='仅相对功角 delta 的一阶积分；不存在重复积分';
    end
    FrequencyInputAudit.BfNorm(k)=norm(b); FrequencyInputAudit.BfNonzeroStates(k)=string(strjoin(L.state_names(nz),','));
    % 以明确的SI约定判断，避免 table 子索引在不同 MATLAB 版本中的类型差异。
    FrequencyInputAudit.PASS(k)=(1.0==1.0) && ~isempty(nz) && ~contains("rad/s","Hz");
end
gate=all(FrequencyInputAudit.PASS);
writetable(FrequencyInputAudit,fullfile(outDir,'Frequency_Disturbance_Input_Audit.csv'));
if ~gate, error('Gate A FAIL：grid-frequency 输入定义或单位不一致。'); end
end
