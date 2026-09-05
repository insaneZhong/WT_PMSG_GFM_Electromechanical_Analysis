function [models,base] = prepare_multimode_models(varargin)
%PREPARE_MULTIMODE_MODELS 读取唯一 Gate A 共同工作点并构建三架构 SSM。
% 禁止重定义参数；所有 p、状态和 GWT 标志均来自现有对齐基准文件。
ip=inputParser;
ip.addParameter('ParameterVector',[],@(x)isnumeric(x)&&isvector(x));
ip.addParameter('States',[],@(x)isnumeric(x));
ip.addParameter('Modes',{},@(x)iscell(x)||isstring(x));
ip.parse(varargin{:}); o=ip.Results;
here=fileparts(mfilename('fullpath')); S=load(fullfile(here,'Architecture_Comparison_Summary.mat'),'R'); base=S.R;
assert(base.passed,'唯一三架构共同工作点 Gate A 未通过。');
p=base.parameter_vector; if ~isempty(o.ParameterVector), p=o.ParameterVector(:).'; end
modes=cellstr(base.models(:)); labels=cellstr(base.labels(:)); X=base.states;
if ~isempty(o.Modes)
    want=cellstr(o.Modes); ix=zeros(numel(want),1); for k=1:numel(want), ix(k)=find(strcmpi(modes,want{k}),1); end
    assert(all(ix>0),'请求的架构不在唯一基准内。'); modes=modes(ix); labels=labels(ix); X=X(:,ix);
end
if ~isempty(o.States), X=o.States; end
models=cell(numel(modes),1);
for k=1:numel(modes)
    flags=struct;
    if strcmpi(modes{k},'GFMGWT')
        E=load(fullfile(here,'03_Mechanism_Evidence_Summary.mat'),'E');
        flags=struct('imqRef0',E.E.operating_point.pmsg_iq0,'KpGscDvc',5e3,'KiGscDvc',5e2,'mpGwt',5e-7);
    end
    L=multimode_linearize_control(X(:,k),p,modes{k},flags);
    L.label=labels{k}; models{k}=L;
end
end
