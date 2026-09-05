function [t,x,y] = multimode_simulate_linear_step(L,dStep,stepTime_s,stopTime_s,numPoints)
%MULTIMODE_SIMULATE_LINEAR_STEP 在零初值增量模型上施加常值阶跃。
if nargin<5, numPoints=5001; end
t=linspace(0,stopTime_s,round(numPoints)).'; n=numel(L.x0);
iStep=max(2,min(numel(t)-1,round(stepTime_s/stopTime_s*(numel(t)-1))+1));
opt=odeset('RelTol',1e-8,'AbsTol',1e-9,'MaxStep',0.01);
[t1,x1]=ode15s(@(~,z)L.A*z,t(1:iStep),zeros(n,1),opt);
[t2,x2]=ode15s(@(~,z)L.A*z+L.B*dStep,t(iStep:end),x1(end,:).',opt);
t=[t1;t2(2:end)]; x=[x1;x2(2:end,:)]; y=x*L.C.';
for k=1:numel(t)
    if t(k)>=stepTime_s, y(k,:)=y(k,:)+(L.D*dStep).'; end
end
end
