function [state,out] = s7_legacy_replica_b3_pq_filter_step(state,u,p)
%S7_LEGACY_REPLICA_B3_PQ_FILTER_STEP
% S7-5B/B3：Legacy C 的 P/Q 一阶低通离散更新复制器。
%
% 对应 grid_forming_control.c: motor_low_pass_filter()。C 源码把
% Ts_frequcncy 固定为 0.00025 s，并采用双线性离散式；这里保持
% single 精度、同一更新顺序和同一历史输入/输出状态。

if nargin == 1 && (ischar(state) || (isstring(state) && isscalar(state)))
    mode = lower(char(state));
    switch mode
        case 'defaults', state = local_defaults();
        case 'initial_state', state = local_initial_state();
        otherwise, error('s7_legacy_replica_b3_pq_filter_step:UnknownMode', ...
                '未知模式 %s。',mode);
    end
    out = []; return;
end
if nargin < 2 || isempty(u), error('s7_legacy_replica_b3_pq_filter_step:MissingInput','缺少输入。'); end
if nargin < 3 || isempty(p), p = local_defaults(); end
if isempty(state) || ~isstruct(state), state = local_initial_state(); end
state = local_complete_state(state);

P = single(local_get(u,'P',0));
Q = single(local_get(u,'Q',0));
fcP = single(local_get(p,'P_cutoff_Hz',20));
fcQ = single(local_get(p,'Q_cutoff_Hz',20));
state.P = local_filter_step(state.P,P,fcP);
state.Q = local_filter_step(state.Q,Q,fcQ);
out = struct('P_filter',state.P.out,'Q_filter',state.Q.out, ...
    'P_input',P,'Q_input',Q,'Ts_filter',single(0.00025));
end

function p = local_defaults()
p = struct('P_cutoff_Hz',20,'Q_cutoff_Hz',20);
end
function st = local_initial_state()
z = struct('out',single(0),'Ui_n_1',single(0),'fs_cutoff',single(20));
st = struct('P',z,'Q',z);
end
function st = local_complete_state(st)
z = local_initial_state();
if ~isfield(st,'P'), st.P=z.P; end
if ~isfield(st,'Q'), st.Q=z.Q; end
st.P = local_complete_filter(st.P); st.Q = local_complete_filter(st.Q);
end
function st = local_complete_filter(st)
z = struct('out',single(0),'Ui_n_1',single(0),'fs_cutoff',single(20));
f = fieldnames(z);
for k=1:numel(f), if ~isfield(st,f{k}), st.(f{k})=z.(f{k}); end, end
end
function st = local_filter_step(st,x,fc)
st = local_complete_filter(st);
% 与 C 完全对应：a0=1+Ts*pi*fc，a1=Ts*pi*fc-1，b0=b1=Ts*pi*fc。
Ts = single(0.00025); pi_c = single(pi);
a0 = single(1) + Ts*pi_c*fc;
a1 = Ts*pi_c*fc - single(1);
b = Ts*pi_c*fc;
st.out = single((b*x + b*st.Ui_n_1 - a1*st.out)/a0);
st.Ui_n_1 = x;
st.fs_cutoff = fc;
end
function v = local_get(s,n,d)
if isstruct(s) && isfield(s,n) && ~isempty(s.(n)), v=s.(n); else, v=d; end
end
