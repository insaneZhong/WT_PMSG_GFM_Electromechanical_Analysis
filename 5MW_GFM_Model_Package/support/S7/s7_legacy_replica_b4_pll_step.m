function [state,out,trace] = s7_legacy_replica_b4_pll_step(state,u,p)
%S7_LEGACY_REPLICA_B4_PLL_STEP
% S7-5B/B4：Legacy PLL/GFL 角度分支复制器。
% 复制 grid_forming_control.c 中的 Park(PLL)->PLL PI->freq/phase 更新，
% 以及并网后 GFL 分支使用 measurement-only PLL 的状态更新。

if nargin == 1 && (ischar(state) || (isstring(state) && isscalar(state)))
    mode=lower(char(state));
    switch mode
        case 'defaults', state=local_defaults();
        case 'initial_state', state=local_initial_state();
        otherwise, error('s7_legacy_replica_b4_pll_step:UnknownMode','未知模式 %s。',mode);
    end
    out=[]; trace=[]; return;
end
if nargin<2 || isempty(u), error('s7_legacy_replica_b4_pll_step:MissingInput','缺少输入。'); end
if nargin<3 || isempty(p), p=local_defaults(); end
if isempty(state)||~isstruct(state),state=local_initial_state();end
state=local_complete(state);

% grid_side.Ts is initialized from GRID_SIDE_INV_DEFAULTS (0.0001 s) in
% the legacy C code and is not overwritten by main.c's S-function sample
% time.  Keep both clocks explicit so this replica does not accidentally
% replace the controller update interval with the Simulink major step.
Ts=single(local_get(u,'Ts_grid',local_get(p,'GridControllerTs',1e-4)));
time=single(local_get(u,'system_Time',0));
presyn=logical(local_get(u,'Pre_syn',time>=single(p.PresynSwitchTime)));
gfm=logical(local_get(u,'GFM_enabled',time>=single(p.GfmEnableTime)));

[ua,ub,uc]=local_line_to_phase(single(local_get(u,'pcc_uab',0)), ...
    single(local_get(u,'pcc_ubc',0)),single(local_get(u,'pcc_uca',0)));
[alpha,beta]=local_clarke(ua,ub,uc);

% grid_forming_control.c: pre-sync PLL uses grid_phase_angle as its own
% feedback angle; post-takeover GFL uses the same measurement-only PLL.
if ~presyn
    [~,uq]=local_park(alpha,beta,state.phase);
    state.pll.Ref=uq; state.pll.Fdb=single(0);
    state.pll=local_pi2_step(state.pll,p.PLL, p.PLL_OutMax,p.PLL_OutMin);
    state.freq=single(p.NominalOmega)+state.pll.Out;
    state.phase=local_wrap(state.phase+Ts*state.freq);
elseif gfm
    [~,uq]=local_park(alpha,beta,state.phase);
    state.pll.Ref=uq; state.pll.Fdb=single(0);
    state.pll=local_pi2_step(state.pll,p.PLL,p.PLL_OutMax,p.PLL_OutMin);
    state.freq=single(p.NominalOmega)+state.pll.Out;
    state.freq=min(max(state.freq,single(p.NominalOmega)-single(p.PLLFreqLimit)), ...
        single(p.NominalOmega)+single(p.PLLFreqLimit));
    state.phase=local_wrap(state.phase+Ts*state.freq);
else
    state.freq=single(p.NominalOmega);
    state.phase=local_wrap(state.phase+Ts*state.freq);
end

out=struct('freq',state.freq,'grid_phase_angle',state.phase, ...
    'w_ref',state.freq,'theta_ref',state.phase,'pcc_uq_at_pll',state.pll.Ref, ...
    'Pre_syn',double(presyn),'GFM_enabled',double(gfm));
trace=struct('finite',all(isfinite([double(state.freq),double(state.phase), ...
    double(state.pll.Out),double(state.pll.Ui)])));
end

function p=local_defaults()
p=struct('NominalOmega',single(314),'PresynSwitchTime',single(1.75), ...
    'GfmEnableTime',single(1.75),'PLL_OutMax',single(400), ...
    'PLL_OutMin',single(-400),'PLLFreqLimit',single(4*pi), ...
    'GridControllerTs',single(1e-4));
p.PLL=struct('Kp',single(1),'Ki',single(.001),'Kc',single(0), ...
    'Kd',single(0));
end
function st=local_initial_state()
st=struct('phase',single(0),'freq',single(314),'pll',local_pi());
end
function st=local_complete(st)
z=local_initial_state(); if ~isfield(st,'phase'),st.phase=z.phase;end
if ~isfield(st,'freq'),st.freq=z.freq;end; if ~isfield(st,'pll'),st.pll=z.pll;end
st.pll=local_complete_pi(st.pll);
end
function z=local_pi()
z=struct('Ref',single(0),'Fdb',single(0),'Kp',single(0),'Ki',single(0), ...
    'Kc',single(0),'Kd',single(0),'Ui',single(0),'Up',single(0), ...
    'Up_old',single(0),'Ud',single(0),'SatErr',single(0),'Out',single(0), ...
    'OutPreSat',single(0),'Error',single(0));
end
function z=local_complete_pi(z)
d=local_pi(); f=fieldnames(d); for k=1:numel(f),if ~isfield(z,f{k}),z.(f{k})=d.(f{k});end,end
end
function z=local_pi2_step(z,par,omax,omin)
z=local_complete_pi(z); e=single(z.Ref-z.Fdb); up=single(par.Kp)*e;
ui=single(z.Ui+single(par.Ki)*up+single(par.Kc)*z.SatErr);
ud=single(par.Kd)*(up-z.Up_old); pre=single(up+ui+ud);
out=min(max(pre,omin),omax);
z.Error=e;z.Up=up;z.Ui=ui;z.Ud=ud;z.OutPreSat=pre;z.Out=out;z.SatErr=single(out-pre);z.Up_old=up;
end
function y=local_wrap(x)
y=single(x); tw=single(6.2831853);
while y>tw,y=single(double(y)-6.2831853);end
while y<0,y=single(double(y)+6.2831853);end
end
function [a,b,c]=local_line_to_phase(uab,ubc,uca)
a=single(-1/3)*(uca-uab);b=single(-1/3)*(uab-ubc);c=single(-1/3)*(ubc-uca);
end
function [alpha,beta]=local_clarke(a,b,c)
alpha=single(2/3)*(a-single(.5)*b-single(.5)*c);
beta=single(2/3)*single(1.732)*(b-c)/single(2);
end
function [d,q]=local_park(alpha,beta,theta)
d=alpha*single(cos(theta))+beta*single(sin(theta));q=beta*single(cos(theta))-alpha*single(sin(theta));
end
function v=local_get(s,n,d)
if isstruct(s)&&isfield(s,n)&&~isempty(s.(n)),v=s.(n);else,v=d;end
end
