function V = validate_multimode_representative_nonlinear()
%VALIDATE_MULTIMODE_REPRESENTATIVE_NONLINEAR
% 仅对少量代表点比较完整SSM、最小多模态重构和理想连续非线性模型。
% 所有时序仅保留在内存中供主程序绘图和写报告，不输出单独的时序数据文件。
[baseModels,base]=prepare_multimode_models(); p0=base.parameter_vector; xGFL=baseModels{1}.x0; xMWT=baseModels{3}.x0;
scr0=4; cases=struct('Name',{},'Mode',{},'p',{},'xSeed',{},'NeedSolve',{});
cases(end+1)=struct('Name','Baseline GFL','Mode','GFL','p',p0,'xSeed',xGFL,'NeedSolve',false);
cases(end+1)=struct('Name','Baseline GFM-MWT','Mode','VSG','p',p0,'xSeed',xMWT,'NeedSolve',false);
p=p0; p(9)=p0(9)*(scr0/6); p(10)=p0(10)*(scr0/6); cases(end+1)=struct('Name','High-Gamma representative (SCR=6)','Mode','VSG','p',p,'xSeed',xMWT,'NeedSolve',true);
p=p0; p(33)=p0(33)*.5; cases(end+1)=struct('Name','Low-Gamma representative (H=0.5H0)','Mode','VSG','p',p,'xSeed',xMWT,'NeedSolve',false);
p=p0; p(9)=p0(9)*(scr0/10); p(10)=p0(10)*(scr0/10); cases(end+1)=struct('Name','High-SCR representative (SCR=10)','Mode','VSG','p',p,'xSeed',xMWT,'NeedSolve',true);
p=p0; p(9)=p0(9)*(scr0/2); p(10)=p0(10)*(scr0/2); cases(end+1)=struct('Name','Low-SCR representative (SCR=2)','Mode','VSG','p',p,'xSeed',xMWT,'NeedSolve',true);

% 为保证此处属于严格线性区，频率阶跃取 0.005 Hz（而非用于残差归一化的 0.05 Hz）。
d=zeros(4,1); d(4)=2*pi*.005; stepTime=.10; stopTime=10; nPts=5001; outputs={'omega_sh','T_sh'};
V=struct('case',{},'mode',{},'output',{},'t',{},'nl',{},'ssm',{},'minimal',{},'nrmse_full',{},'corr_full',{},'peak_full',{},'nrmse_min',{},'corr_min',{},'peak_min',{},'f_tor',{});
for c=1:numel(cases)
    C=cases(c); flags=struct;
    if C.NeedSolve, [x,~]=solve_multimode_control_equilibrium(C.xSeed,C.p,C.Mode,flags); else, x=C.xSeed; end
    L=multimode_linearize_control(x,C.p,C.Mode,flags); MD=multimode_modal_data(L.A,L.state_names);
    [t,~,yss]=multimode_simulate_linear_step(L,d,stepTime,stopTime,nPts); yn=localNonlinearStep(x,C.p,C.Mode,flags,d,stepTime,t);
    it=multimode_pick_torsion_mode(MD); ftor=abs(imag(MD.lambda(it)))/(2*pi);
    for o=1:numel(outputs)
        iy=find(strcmp(L.output_names,outputs{o}),1); full=yss(:,iy); nl=yn(:,iy); minimal=localMinimal(MD,L,iy,d,t,stepTime);
        [n1,c1,p1]=localMetrics(nl,full); [n2,c2,p2]=localMetrics(nl,minimal);
        V(end+1)=struct('case',C.Name,'mode',C.Mode,'output',outputs{o},'t',t,'nl',nl,'ssm',full,'minimal',minimal, ...
            'nrmse_full',n1,'corr_full',c1,'peak_full',p1,'nrmse_min',n2,'corr_min',c2,'peak_min',p2,'f_tor',ftor); %#ok<AGROW>
    end
end
end

function Y=localNonlinearStep(x0,p,mode,flags,d,tStep,t)
i=max(2,min(numel(t)-1,find(t>=tStep,1))); opt=odeset('RelTol',1e-8,'AbsTol',1e-9,'MaxStep',.01);
[t1,x1]=ode15s(@(~,x)source_aligned_rhs_control(x,p,mode,zeros(4,1),flags),t(1:i),x0,opt);
[t2,x2]=ode15s(@(~,x)source_aligned_rhs_control(x,p,mode,d,flags),t(i:end),x1(end,:).',opt);
tt=[t1;t2(2:end)]; xx=[x1;x2(2:end,:)];
y0=source_aligned_internal_outputs_control(x0,p,mode,zeros(4,1),flags); Y=zeros(numel(tt),numel(y0));
for k=1:numel(tt)
    dd=zeros(4,1); if tt(k)>=tStep, dd=d; end
    Y(k,:)=source_aligned_internal_outputs_control(xx(k,:).',p,mode,dd,flags).'-y0.';
end
end

function y=localMinimal(MD,L,iy,d,t,tStep)
cand=find((imag(MD.lambda)>1e-8) | (abs(imag(MD.lambda))<=1e-8 & abs(MD.lambda)>1e-6)); R=zeros(numel(cand),1); score=zeros(numel(cand),1);
for q=1:numel(cand)
    k=cand(q); R(q)=L.C(iy,:)*MD.V(:,k)*(MD.W(:,k)'*L.B*d);
    score(q)=iffLocal(imag(MD.lambda(k))>1e-8,2,1)*abs(R(q)/MD.lambda(k));
end
[~,ord]=sort(score,'descend'); qmax=min(5,numel(ord)); y=localReconstruct(t,tStep,MD.lambda(cand(ord(1:qmax))),R(ord(1:qmax)));
end

function y=localReconstruct(t,tStep,lam,R)
tau=max(t-tStep,0); y=zeros(size(t)); ix=t>=tStep;
for q=1:numel(lam)
    term=R(q)/lam(q).*(exp(lam(q)*tau(ix))-1);
    if imag(lam(q))>1e-8, y(ix)=y(ix)+2*real(term); else, y(ix)=y(ix)+real(term); end
end
end

function [n,c,p]=localMetrics(a,b)
n=norm(a-b)/max(norm(a-mean(a)),eps); aa=a-mean(a); bb=b-mean(b); c=(aa'*bb)/max(norm(aa)*norm(bb),eps); p=100*abs(max(abs(a))-max(abs(b)))/max(max(abs(a)),eps);
end
function v=iffLocal(c,a,b), if c, v=a; else, v=b; end, end
