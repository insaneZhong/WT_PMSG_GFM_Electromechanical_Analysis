function [x,meta] = solve_multimode_control_equilibrium(xSeed,p,mode,flags)
%SOLVE_MULTIMODE_CONTROL_EQUILIBRIUM 在不改变控制结构的情况下求23状态物理平衡点。
% GFL 的 omega_vsg/delta 是理想PLL坐标占位状态，不作为独立平衡方程求解。
if nargin<4 || isempty(flags), flags=struct; end
sx=[1;1;1;1e4;1e4;1e4;1e4;1e4;1e3;5e6;5e6;1;1;1e4;1e4;1e4;1e4;1e4;1e4;1e3;1e3;1e4;1e4];
sr=[1;1;1;1e4;1e4;1e4;1e4;1e4;1e6;5e8;5e8;1;1;1e4;1e4;1e4;1e4;1e6;1e6;1e6;1e6;1e6;1e6];
opts=optimoptions('fsolve','Display','off','Algorithm','levenberg-marquardt','FunctionTolerance',1e-11,'StepTolerance',1e-11,'OptimalityTolerance',1e-11,'MaxIterations',3000,'MaxFunctionEvaluations',50000);
if strcmpi(mode,'GFL')
    active=[1:11 14:23]; x=xSeed; z0=x(active)./sx(active);
    [z,fval,exitflag]=fsolve(@(z)localReduced(z,xSeed,active,p,mode,flags,sx,sr),z0,opts); x(active)=sx(active).*z;
else
    [z,fval,exitflag]=fsolve(@(z)source_aligned_rhs_control(sx.*z,p,mode,zeros(4,1),flags)./sr,xSeed./sx,opts); x=sx.*z;
end
dx=source_aligned_rhs_control(x,p,mode,zeros(4,1),flags);
meta=struct('exitflag',exitflag,'residual_norm',norm(fval,inf),'max_abs_dx',max(abs(dx)),'pass',exitflag>0 && norm(fval,inf)<1e-8);
if ~meta.pass, error('Equilibrium solve failed for %s: exit=%g, residual=%.3g',mode,exitflag,meta.residual_norm); end
end

function r=localReduced(z,xBase,active,p,mode,flags,sx,sr)
x=xBase; x(active)=sx(active).*z; dx=source_aligned_rhs_control(x,p,mode,zeros(4,1),flags); r=dx(active)./sr(active);
end
