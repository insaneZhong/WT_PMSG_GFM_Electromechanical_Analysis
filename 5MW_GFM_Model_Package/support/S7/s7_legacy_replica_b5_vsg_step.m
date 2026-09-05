function [state,out,trace] = s7_legacy_replica_b5_vsg_step(state,u,p)
%S7_LEGACY_REPLICA_B5_VSG_STEP
% S7-5B/B5：严格 VSG 分支的 P/Q 测量、Pref 斜率、摆动方程和角度
% 更新复制器。该函数只复制控制状态，不含 PLL 或功率级。

if nargin==1 && (ischar(state)||(isstring(state)&&isscalar(state)))
    mode=lower(char(state));
    switch mode
        case 'defaults',state=local_defaults();
        case 'initial_state',state=local_initial_state();
        otherwise,error('s7_legacy_replica_b5_vsg_step:UnknownMode','未知模式 %s。',mode);
    end
    out=[];trace=[];return
end
if nargin<2||isempty(u),error('s7_legacy_replica_b5_vsg_step:MissingInput','缺少输入。');end
if nargin<3||isempty(p),p=local_defaults();end
if isempty(state)||~isstruct(state),state=local_initial_state();end
state=local_complete(state);

Ts=single(local_get(u,'Ts_grid',p.GridControllerTs));
theta_old=state.theta;
[ua,ub,uc]=local_line_to_phase(single(local_get(u,'pcc_uab',0)), ...
    single(local_get(u,'pcc_ubc',0)),single(local_get(u,'pcc_uca',0)));
[ua0,ub0]=local_clarke(ua,ub,uc);
[pcc_ud,pcc_uq]=local_park(ua0,ub0,theta_old);
[ia,ib,ic]=local_abc(u,'pcc_Ia','pcc_Ib','pcc_Ic');
[ia0,ib0]=local_clarke(ia,ib,ic);
[pcc_id,pcc_iq]=local_park(ia0,ib0,theta_old);
P=single(1.5)*(pcc_id*pcc_ud+pcc_iq*pcc_uq);
Q=single(1.5)*(pcc_id*pcc_uq-pcc_iq*pcc_ud);
state.Pf=local_lpf(state.Pf,P,p.PFilterCutoff);
state.Qf=local_lpf(state.Qf,Q,p.QFilterCutoff);

pref=single(local_get(u,'P_ref',0));
dPmax=single(p.PrefRampSlope)*Ts; deltaP=pref-state.PrefRampOut;
if deltaP>dPmax,state.PrefRampOut=state.PrefRampOut+dPmax;
elseif deltaP<-dPmax,state.PrefRampOut=state.PrefRampOut-dPmax;
else,state.PrefRampOut=pref;end

p_err=state.PrefRampOut-state.Pf.out;
delta_w=state.w_vsg-state.w_sync;
dw=(single(p.VSGPowerErrorSign)*p_err-delta_w/single(p.VSG_mp))* ...
    single(p.VSG_w0)/(single(2)*single(p.VSG_H)*single(p.VSG_Sbase));
state.w_vsg=single(state.w_vsg+Ts*dw);
state.theta=local_wrap(state.theta+Ts*state.w_vsg);
out=struct('P',P,'Q',Q,'P_filter',state.Pf.out,'Q_filter',state.Qf.out, ...
    'P_ref_ramped',state.PrefRampOut,'pcc_ud',pcc_ud,'pcc_uq',pcc_uq, ...
    'w_ref',state.w_vsg,'theta_ref',state.theta,'dw',dw);
trace=struct('finite',all(isfinite([double(P),double(Q),double(state.Pf.out), ...
    double(state.w_vsg),double(state.theta)])));
end

function p=local_defaults()
p=struct('GridControllerTs',single(1e-4),'PFilterCutoff',single(20), ...
    'QFilterCutoff',single(20),'PrefRampSlope',single(1e6), ...
    'VSGPowerErrorSign',single(1),'VSG_H',single(10), ...
    'VSG_mp',single(1.57e-6),'VSG_Sbase',single(1e6),'VSG_w0',single(314));
end
function st=local_initial_state()
z=struct('out',single(0),'Ui_n_1',single(0),'fs_cutoff',single(20));
st=struct('theta',single(.05),'w_vsg',single(314),'w_sync',single(314), ...
    'Pf',z,'Qf',z,'PrefRampOut',single(0));
end
function st=local_complete(st)
z=local_initial_state();f=fieldnames(z);for k=1:numel(f),if ~isfield(st,f{k}),st.(f{k})=z.(f{k});end,end
st.Pf=local_filter_complete(st.Pf);st.Qf=local_filter_complete(st.Qf);
end
function st=local_filter_complete(st)
z=struct('out',single(0),'Ui_n_1',single(0),'fs_cutoff',single(20));f=fieldnames(z);
for k=1:numel(f),if ~isfield(st,f{k}),st.(f{k})=z.(f{k});end,end
end
function st=local_lpf(st,x,fc)
st=local_filter_complete(st);Ts=single(.00025);pc=single(pi);
a0=single(1)+Ts*pc*fc;a1=Ts*pc*fc-single(1);b=Ts*pc*fc;
st.out=single((b*x+b*st.Ui_n_1-a1*st.out)/a0);st.Ui_n_1=x;st.fs_cutoff=fc;
end
function y=local_wrap(x)
% Match the C expression: the macro is a double literal, so subtraction is
% performed in double precision and then assigned back to float.
y=single(x);tw=single(6.2831853);
while y>tw,y=single(double(y)-6.2831853);end
while y<0,y=single(double(y)+6.2831853);end
end
function [a,b,c]=local_line_to_phase(uab,ubc,uca)
a=single(-1/3)*(uca-uab);b=single(-1/3)*(uab-ubc);c=single(-1/3)*(ubc-uca);
end
function [alpha,beta]=local_clarke(a,b,c)
alpha=single(2/3)*(a-single(.5)*b-single(.5)*c);beta=single(2/3)*single(1.732)*(b-c)/single(2);
end
function [d,q]=local_park(alpha,beta,theta)
d=alpha*single(cos(theta))+beta*single(sin(theta));q=beta*single(cos(theta))-alpha*single(sin(theta));
end
function [a,b,c]=local_abc(u,fa,fb,fc)
a=single(local_get(u,fa,0));b=single(local_get(u,fb,0));c=single(local_get(u,fc,0));
end
function v=local_get(s,n,d)
if isstruct(s)&&isfield(s,n)&&~isempty(s.(n)),v=s.(n);else,v=d;end
end
