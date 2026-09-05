function R=run_joint_physical_alignment_validation(varargin)
%RUN_JOINT_PHYSICAL_ALIGNMENT_VALIDATION
% 联合物理工作点与同源小信号模型六输出对齐主程序。
% 只维护一个 SLX，不保存原始时序、SimulationOutput 或图片。

ip=inputParser;
ip.addParameter('StopTime_s',8,@(x)isnumeric(x)&&isscalar(x)&&x>2);
ip.addParameter('StepTime_s',1,@(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('DeltaPref_W',25e3,@(x)isnumeric(x)&&isscalar(x));
ip.addParameter('SaveSummary',true,@(x)islogical(x)&&isscalar(x));
ip.parse(varargin{:}); o=ip.Results;
assert(o.StepTime_s<o.StopTime_s,'Step time must precede StopTime.');

here=fileparts(mfilename('fullpath'));
addpath(fullfile(here,'temp'));

% 1) 物理模型：powergui 9状态、两个受控源、连续GSC/VSG、MSC、
% DC-link和两质量轴系共享同一个5 MW周期工作点。
J=solve_joint_physical_periodic_operating_point();

% 2) 同源23状态平均小信号模型。该模型与非线性副本使用同一组
% 控制器、PMSG、DC-link、轴系及LCL方程；线性化不含限幅/PWM/PLL。
O=solve_currentmodel_source_aligned_equilibrium();
[x0,p]=stateVector(O);
f0=source_aligned_rhs(x0,p);
[A,B,C,D]=linearizeModel(x0,p);
ev=eig(A);
[shaftEig,shaftHz,shaftZeta]=findShaftMode(ev);

% 3) 物理非线性副本做成对试验：相同初态的零扰动轨迹与扰动轨迹
% 相减，排除极小周期初始化偏差，只保留DeltaPref引起的增量响应。
S0=probe_short_joint_transient(o.StopTime_s,J.x0,J.pvec, ...
    J.source_aligned,1e6,0);
S1=probe_short_joint_transient(o.StopTime_s,J.x0,J.pvec, ...
    J.source_aligned,o.StepTime_s,o.DeltaPref_W);
t=(0:1e-3:o.StopTime_s).';
yN=extractSix(S1,t)-extractSix(S0,t);

% 4) 线性模型施加完全相同的有功参考阶跃。
u=o.DeltaPref_W*double(t>=o.StepTime_s);
sys=ss(A,B,C,D);
yL=lsim(sys,u,t,zeros(size(A,1),1));

% 5) 在扰动后比较六个论文输出，不用开关纹波或绝对工作点抬高误差。
idx=t>=o.StepTime_s;
names={'P_pcc_W','Udc_V','Tgen_Nm','Tshaft_Nm', ...
    'omega_rel_radps','omega_vsg_radps'};
nrmse=zeros(6,1); peakError=zeros(6,1); finalError=zeros(6,1); corrcoef6=zeros(6,1);
for k=1:6
    a=yL(idx,k); b=yN(idx,k); e=b-a;
    den=max(norm(a),1e-12);
    nrmse(k)=norm(e)/den;
    peakError(k)=abs(max(abs(b))-max(abs(a)))/max(max(abs(a)),1e-12);
    finalError(k)=abs(b(end)-a(end))/max(max(abs(a)),1e-12);
    cc=corrcoef(a,b);
    if all(size(cc)==[2 2]), corrcoef6(k)=cc(1,2); else, corrcoef6(k)=NaN; end
end

R=struct();
R.model=J.model;
R.joint_power=struct('Pmsc_W',J.source_aligned.P_msc_W, ...
    'Pgsc_W',J.source_aligned.P_gsc_W, ...
    'Ppcc_W',J.source_aligned.P_pcc_measurement_W, ...
    'Qpcc_var',J.source_aligned.Q_pcc_measurement_var, ...
    'dc_power_mismatch_W',J.source_aligned.energy_residual_W, ...
    'phasor_residual_inf',J.phasor_residual_norm);
R.small_signal=struct('max_rhs_abs',max(abs(f0)), ...
    'max_real_eigenvalue',max(real(ev)),'shaft_eigenvalue',shaftEig, ...
    'shaft_frequency_Hz',shaftHz,'shaft_damping_ratio',shaftZeta);
R.metrics=table(names.',nrmse,peakError,finalError,corrcoef6, ...
    'VariableNames',{'Signal','NRMSE','PeakError','FinalError','Correlation'});
linMin=min(yL(idx,:),[],1).'; linMax=max(yL(idx,:),[],1).'; linFinal=yL(end,:).';
nlMin=min(yN(idx,:),[],1).'; nlMax=max(yN(idx,:),[],1).'; nlFinal=yN(end,:).';
R.response_summary=table(names.',linMin,linMax,linFinal,nlMin,nlMax,nlFinal, ...
    'VariableNames',{'Signal','LinearMin','LinearMax','LinearFinal', ...
    'NonlinearMin','NonlinearMax','NonlinearFinal'});
R.settings=o;
R.pass=all(isfinite(nrmse)) && max(nrmse)<0.20 && ...
    max(peakError)<0.20 && max(real(ev))<0;

if o.SaveSummary
    validation_results=R; %#ok<NASGU>
    xInitial=J.x0; %#ok<NASGU>
    IdealCtrlPVec=J.pvec; %#ok<NASGU>
    operating_point_summary=R.joint_power; %#ok<NASGU>
    save(fullfile(here,'02_Joint_Physical_Alignment_Summary.mat'), ...
        'xInitial','IdealCtrlPVec','operating_point_summary', ...
        'validation_results');
end

fprintf('\n=== 5 MW联合物理工作点/小信号六输出对齐 ===\n');
fprintf('PMSC=%.9f MW, PGSC=%.9f MW, PPCC=%.9f MW, Q=%.6f var\n', ...
    R.joint_power.Pmsc_W/1e6,R.joint_power.Pgsc_W/1e6, ...
    R.joint_power.Ppcc_W/1e6,R.joint_power.Qpcc_var);
fprintf('DC mismatch=%.3e W, phasor residual=%.3e\n', ...
    R.joint_power.dc_power_mismatch_W,R.joint_power.phasor_residual_inf);
fprintf('SSM maxRe=%.6g 1/s, shaft=%.6f Hz, zeta=%.4f%%\n', ...
    R.small_signal.max_real_eigenvalue,shaftHz,100*shaftZeta);
disp(R.metrics);
disp(R.response_summary);
fprintf('OVERALL=%s\n',string(R.pass));

if o.SaveSummary
    writeReport(fullfile(here,'02_Idealization_Validation_Report_CN.md'),R);
end
end

function [x,p]=stateVector(O)
c=O.controller_x0;
x=[O.theta_tw0;O.omega0;O.omega0;O.pmsg_id0;O.pmsg_iq0; ...
    c(1:3);O.pvec(2);c(4:11);O.if_grid_dq_A; ...
    O.vcap_grid_dq_V;O.ig_grid_dq_A];
p=O.pvec(:);
end

function [A,B,C,D]=linearizeModel(x,p)
n=numel(x); ny=6; A=zeros(n); C=zeros(ny,n);
for k=1:n
    h=1e-6*max(abs(x(k)),1);
    xp=x; xm=x; xp(k)=xp(k)+h; xm(k)=xm(k)-h;
    A(:,k)=(source_aligned_rhs(xp,p)-source_aligned_rhs(xm,p))/(2*h);
    C(:,k)=(source_aligned_outputs(xp,p)-source_aligned_outputs(xm,p))/(2*h);
end
hp=10;
pp=p; pm=p; pp(37)=pp(37)+hp; pm(37)=pm(37)-hp;
B=(source_aligned_rhs(x,pp)-source_aligned_rhs(x,pm))/(2*hp);
D=(source_aligned_outputs(x,pp)-source_aligned_outputs(x,pm))/(2*hp);
end

function y=extractSix(S,t)
z=interpSignal(S.CtrlZ,t);
P=z(:,25); wv=z(:,26);
Udc=interpSignal(S.Udc,t);
Tgen=interpSignal(S.Tgen,t);
Tsh=interpSignal(S.Tsh,t);
wt=interpSignal(S.omega_t,t); wg=interpSignal(S.omega_g,t);
y=[P,Udc,Tgen,Tsh,wt-wg,wv];
end

function y=interpSignal(s,t)
[tu,ia]=unique(s.t(:),'stable');
v=s.y; v=v(ia,:);
y=interp1(tu,v,t,'linear','extrap');
end

function [lam,f,zeta]=findShaftMode(ev)
cand=ev(imag(ev)>0 & abs(imag(ev))/(2*pi)>1 & abs(imag(ev))/(2*pi)<5);
assert(~isempty(cand),'No 1--5 Hz shaft-mode candidate found.');
[~,k]=max(abs(imag(cand)));
lam=cand(k); f=abs(imag(lam))/(2*pi); zeta=-real(lam)/abs(lam);
end

function writeReport(file,R)
fid=fopen(file,'w','n','UTF-8'); assert(fid>0,'Cannot write report.');
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'# 5 MW理想连续非线性模型与小信号模型联合验证\n\n');
fprintf(fid,'- 唯一模型：`%s.slx`\n',R.model);
fprintf(fid,'- 工作点：PCC %.9f MW，Q %.6f var；MSC/GSC端口功率均为 %.9f MW。\n', ...
    R.joint_power.Ppcc_W/1e6,R.joint_power.Qpcc_var,R.joint_power.Pgsc_W/1e6);
fprintf(fid,'- DC功率残差：%.3e W；SPS周期相量残差：%.3e。\n', ...
    R.joint_power.dc_power_mismatch_W,R.joint_power.phasor_residual_inf);
fprintf(fid,'- 小信号最大特征值实部：%.6g 1/s。\n',R.small_signal.max_real_eigenvalue);
fprintf(fid,'- 轴系模态：%.6f Hz，阻尼比 %.4f%%。\n', ...
    R.small_signal.shaft_frequency_Hz,100*R.small_signal.shaft_damping_ratio);
fprintf(fid,'- 扰动：t=%.3f s施加DeltaPref=%g W；非线性结果采用同初态“扰动-零扰动”增量。\n\n', ...
    R.settings.StepTime_s,R.settings.DeltaPref_W);
fprintf(fid,'|信号|NRMSE|峰值误差|末值误差|相关系数|\n|---|---:|---:|---:|---:|\n');
for k=1:height(R.metrics)
    fprintf(fid,'|%s|%.6g|%.6g|%.6g|%.6g|\n',R.metrics.Signal{k}, ...
        R.metrics.NRMSE(k),R.metrics.PeakError(k), ...
        R.metrics.FinalError(k),R.metrics.Correlation(k));
end
fprintf(fid,'\n**总体结果：%s。**\n',string(R.pass));
end
