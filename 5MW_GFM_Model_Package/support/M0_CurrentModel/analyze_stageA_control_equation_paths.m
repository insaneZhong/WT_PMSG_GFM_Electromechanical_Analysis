function R = analyze_stageA_control_equation_paths(outDir)
%ANALYZE_STAGEA_CONTROL_EQUATION_PATHS 阶段A：控制方程通道的结构性判别。
% 此程序只分析当前已对齐的理想连续模型，不修改plant、控制器或工作点。
% 为避免DC-link能量闭环把“Udc作为原因”和“Udc作为状态”混在一起，分析时：
%   1) 从状态矩阵中移除Udc能量积分状态；
%   2) 把Udc列作为一个外部规定的小扰动输入；
%   3) 保留其余控制器和物理状态的完整动态。
% 长期输出严格限制为一张CSV和一份中文报告。

if nargin<1 || isempty(outDir), outDir=fileparts(mfilename('fullpath')); end
[models,base]=prepare_multimode_models(); %#ok<ASGLU>
paths={'iq_MSC_ref','P_GSC','P_PCC'};
nRows=numel(models)*numel(paths);
T=table('Size',[nRows 19], ...
    'VariableTypes',{'string','string','string','double','double','double','double','double','double','double','double','double','double','logical','logical','string','logical','double','string'}, ...
    'VariableNames',{'Architecture','Mode','Path','Frequency_Hz', ...
    'DirectAnalyticDerivative','DirectNumericDerivative','AnalyticControllerReal','AnalyticControllerImag', ...
    'FrequencyResponseReal','FrequencyResponseImag','FrequencyResponseMagnitude', ...
    'OffPointMinusMagnitude','OffPointPlusMagnitude','AnalyticPathExists','JacobianReachable', ...
    'Classification','ConsistencyPASS','RelativeError','Equation'});

rr=0; allPass=true; details=cell(numel(models),1);
for a=1:numel(models)
    L=models{a}; p=L.p; flags=L.flags; mode=upper(string(L.mode));
    M=multimode_modal_data(L.A,L.state_names); it=multimode_pick_torsion_mode(M);
    wtor=abs(imag(M.lambda(it))); ftor=wtor/(2*pi);
    iU=9; keep=setdiff(1:size(L.A,1),iU);
    Ar=L.A(keep,keep); Bu=L.A(keep,iU);

    % 用同一套方程在Udc0正负0.2%处重新求局部Jacobian；不将偏移点冒充平衡点。
    dU=0.002*p(2); xm=L.x0; xp=L.x0; xm(iU)=xm(iU)-dU; xp(iU)=xp(iU)+dU;
    Lm=multimode_linearize_control(xm,p,L.mode,flags);
    Lp=multimode_linearize_control(xp,p,L.mode,flags);

    Kpdc=p(25); Kidc=p(26); H=p(33); Sb=p(1); w0=p(3); sP=p(41);
    isGwt=(mode=="GFMGWT");
    if isGwt
        KpG=getFlagValueLocal(flags,'KpGscDvc',5e3);
        KiG=getFlagValueLocal(flags,'KiGscDvc',5e2);
        dIqDirect=0; Giq=0;
        gscInjection=[KpG; -KiG; w0*sP*KpG/(2*H*Sb)];
        eqIq="iq*_MSC = iq*0（MSC-DVC旁路）";
        eqGrid="e_dc=Udc0-Udc; Pctrl=Pref-Kp_gdc*e_dc-xi_gdc";
    else
        dIqDirect=-Kpdc; Giq=-(Kpdc+Kidc/(1i*wtor));
        gscInjection=[0;0;0];
        eqIq="iq*_MSC=Kp_dc(Udc0-Udc)+xi_dc; dxi_dc/dt=Ki_dc(Udc0-Udc)";
        eqGrid="GSC交流控制方程不含Udc；理想受控电压源不受DC电压调制";
    end

    for q=1:numel(paths)
        rr=rr+1; out=paths{q}; iy=find(strcmp(L.output_names,out),1);
        assert(~isempty(iy),'缺少输出 %s。',out);
        [G,reachable]=prescribedUdcResponse(L,iy,keep,wtor);
        Gm=prescribedUdcResponse(Lm,iy,keep,wtor);
        Gp=prescribedUdcResponse(Lp,iy,keep,wtor);
        directNum=L.C(iy,iU);

        if strcmp(out,'iq_MSC_ref')
            directAna=dIqDirect; Gana=Giq; pathExists=~isGwt;
            relErr=abs(G-Gana)/max([abs(Gana),abs(G),1e-12]);
            pass=abs(directNum-directAna)<=1e-7*max([1,abs(directAna)]) && relErr<=1e-5;
            equation=eqIq;
        else
            directAna=0; Gana=complex(NaN,NaN); pathExists=isGwt;
            relErr=NaN; equation=eqGrid;
            if pathExists
                % 解析方程明确存在Udc注入，频响和Jacobian都必须显示可达。
                pass=all(isfinite(gscInjection)) && any(abs(gscInjection)>0) && reachable && abs(G)>1e-9;
            else
                % 解析方程无Udc到GSC/PCC的有向边；数值频响必须保持机器零。
                pass=~reachable && max([abs(G),abs(Gm),abs(Gp),abs(directNum)])<1e-9;
            end
        end

        cls=classifyPath(pathExists,reachable,G,Gm,Gp);
        allPass=allPass && pass;
        T.Architecture(rr)=string(L.label); T.Mode(rr)=mode; T.Path(rr)="Udc -> "+string(out);
        T.Frequency_Hz(rr)=ftor; T.DirectAnalyticDerivative(rr)=directAna; T.DirectNumericDerivative(rr)=directNum;
        T.AnalyticControllerReal(rr)=real(Gana); T.AnalyticControllerImag(rr)=imag(Gana);
        T.FrequencyResponseReal(rr)=real(G); T.FrequencyResponseImag(rr)=imag(G); T.FrequencyResponseMagnitude(rr)=abs(G);
        T.OffPointMinusMagnitude(rr)=abs(Gm); T.OffPointPlusMagnitude(rr)=abs(Gp);
        T.AnalyticPathExists(rr)=pathExists; T.JacobianReachable(rr)=reachable;
        T.Classification(rr)=cls; T.ConsistencyPASS(rr)=pass; T.RelativeError(rr)=relErr; T.Equation(rr)=equation;
    end
    details{a}=struct('label',L.label,'wtor',wtor,'ftor',ftor,'gscInjection',gscInjection);
end

T=T(1:rr,:);
csvFile=fullfile(outDir,'StageA_ControlEquation_Path_Classification.csv');
reportFile=fullfile(outDir,'StageA_ControlEquation_Path_Report_CN.md');
writetable(T,csvFile,'Encoding','UTF-8');
writeReport(reportFile,T,details,allPass);

R=struct('passed',allPass,'summary',T,'csv_file',csvFile,'report_file',reportFile);
if ~allPass
    bad=T(~T.ConsistencyPASS,:);
    warning('StageA:ConsistencyMismatch','阶段A解析偏导与频响不一致，已停止。失败行数=%d。',height(bad));
end
end

function [G,reachable]=prescribedUdcResponse(L,iy,keep,w)
iU=9; A=L.A(keep,keep); B=L.A(keep,iU); C=L.C(iy,keep); D=L.C(iy,iU);
G=C*((1i*w*eye(size(A))-A)\B)+D;
reachable=directedReachable(A,B,C,D);
end

function tf=directedReachable(A,B,C,D)
if abs(D)>1e-12, tf=true; return; end
n=size(A,1); active=significantVector(B); seen=active;
adj=false(n,n);
for j=1:n
    scale=max(abs(A(:,j))); if scale>0, adj(:,j)=abs(A(:,j))>max(1e-12,1e-10*scale); end
end
for k=1:n
    active=any(adj(:,active),2); new=active & ~seen; seen=seen|active;
    if ~any(new), break; end
end
out=significantVector(C.'); tf=any(seen & out);
end

function m=significantVector(v)
v=abs(v(:)); scale=max(v); if scale==0, m=false(size(v)); else, m=v>max(1e-12,1e-10*scale); end
end

function cls=classifyPath(pathExists,reachable,G,Gm,Gp)
mag=[abs(G),abs(Gm),abs(Gp)];
if ~pathExists && ~reachable && max(mag)<1e-9
    cls="STRUCTURAL_ZERO";
elseif pathExists && abs(G)<1e-8 && max(mag(2:3))>=100*max(abs(G),1e-14)
    cls="OPERATING_POINT_CANCELLATION";
elseif pathExists && reachable && abs(G)<1e-6
    cls="WEAK_COUPLING";
elseif pathExists && reachable
    cls="ACTIVE_COUPLING";
else
    cls="INCONSISTENT";
end
end

function writeReport(file,T,details,allPass)
fid=fopen(file,'w','n','UTF-8'); assert(fid>0,'无法写入报告。'); c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'# 阶段A：GWT/MWT/GFL控制方程通道结构判别\n\n');
fprintf(fid,'## 1. 范围与方法\n\n');
fprintf(fid,'本报告只使用已对齐的理想连续非线性模型及其同源23状态SSM。没有引入EMT、PWM、离散PI、采样、延迟、限流、LVRT或MPPT/Pitch动态，也没有修改公共plant和共同工作点。\n\n');
fprintf(fid,'为单独识别由直流电压出发的控制通道，计算时将 $U_{dc}$ 能量积分状态从状态矩阵中移除，并把其Jacobian列作为外部规定输入。由此得到\n\n');
fprintf(fid,'$$G_{y,U_{dc}}(j\\omega)=C_r(j\\omega I-A_r)^{-1}B_{U_{dc}}+D_{y,U_{dc}}.$$\n\n');
fprintf(fid,'同时在 $U_{dc0}\\pm0.2\\%%$ 的局部点重复线性化。该偏移只用于排除工作点抵消，不作为新平衡点。\n\n');

fprintf(fid,'## 2. 实际控制方程\n\n');
fprintf(fid,'- **GFM-GWT：** MSC采用固定转矩/电流参考，$i_{q,MSC}^*=i_{q0}^*$，因此 $\\partial i_q^*/\\partial U_{dc}=0$，且不存在积分动态通道。DC-link误差进入GSC-DVC：$P_{ctrl}=P_{ref}-K_{p,gdc}(U_{dc0}-U_{dc})-\\xi_{gdc}$。\n');
fprintf(fid,'- **GFM-MWT与GFL：** MSC-DVC为 $i_q^*=K_{p,dc}(U_{dc0}-U_{dc})+\\xi_{dc}$、$\\dot\\xi_{dc}=K_{i,dc}(U_{dc0}-U_{dc})$，故 $G_{i_q^*,U_{dc}}=-(K_{p,dc}+K_{i,dc}/s)$。\n');
fprintf(fid,'- **当前理想MWT与GFL的网侧：** GSC交流控制和理想受控电压源不含 $U_{dc}$；DC-link只通过MSC-DVC闭合。因此在规定 $U_{dc}$ 的方向性测试中，$U_{dc}\\to P_{GSC}/P_{PCC}$ 没有有向控制边。\n\n');

fprintf(fid,'## 3. 数值结果\n\n');
fprintf(fid,'| 架构 | 路径 | $f_{tor}$ (Hz) | 直接偏导(解析/数值) | $|G(j\\omega_{tor})|$ | -0.2%% / +0.2%% | 可达 | 分类 | 一致 |\n');
fprintf(fid,'|---|---|---:|---:|---:|---:|:---:|---|:---:|\n');
for k=1:height(T)
    fprintf(fid,'| %s | %s | %.6f | %.6g / %.6g | %.6g | %.6g / %.6g | %s | %s | %s |\n', ...
        T.Architecture(k),T.Path(k),T.Frequency_Hz(k),T.DirectAnalyticDerivative(k),T.DirectNumericDerivative(k), ...
        T.FrequencyResponseMagnitude(k),T.OffPointMinusMagnitude(k),T.OffPointPlusMagnitude(k),yesno(T.JacobianReachable(k)),T.Classification(k),yesno(T.ConsistencyPASS(k)));
end

fprintf(fid,'\n## 4. 判定\n\n');
if allPass
    fprintf(fid,'**阶段A一致性门：PASS。** 解析偏导、Jacobian有向可达性、轴系频率处频响和正负0.2%%局部检查相互一致。\n\n');
else
    fprintf(fid,'**阶段A一致性门：FAIL。** 已按要求停止后续阶段；请检查CSV中 `ConsistencyPASS=false` 的行。\n\n');
end
for a=1:numel(details)
    fprintf(fid,'- %s 的轴系评价频率：%.6f Hz。\n',details{a}.label,details{a}.ftor);
end
fprintf(fid,'\n最终分类含义：\n\n');
fprintf(fid,'- `STRUCTURAL_ZERO`：实际控制方程不存在有向边，Jacobian不可达，基准与偏移点频响均为机器零。\n');
fprintf(fid,'- `WEAK_COUPLING`：方程和Jacobian存在通道，但轴系频率处增益很小。\n');
fprintf(fid,'- `OPERATING_POINT_CANCELLATION`：方程存在通道，基准点近零，但偏移后显著恢复。\n');
fprintf(fid,'- `ACTIVE_COUPLING`：解析与数值均显示有限动态通道；它不是三类“近似零”之一。\n\n');
fprintf(fid,'## 5. 阶段A结论\n\n');
fprintf(fid,'当前模型的双向传播差异首先来自**DC-link调节责任的结构分配**：GWT把直流调节放在GSC侧，因此切断了 $U_{dc}\\to i_{q,MSC}^*$，但保留 $U_{dc}\\to P_{GSC}/P_{PCC}$；MWT与GFL把直流调节放在MSC侧，因此保留前一通道，并在当前理想网侧实现中切断后一通道。若表中不存在 `WEAK_COUPLING` 或 `OPERATING_POINT_CANCELLATION`，则不能把这些机器零解释成参数过小或单一工作点偶然抵消。\n\n');
fprintf(fid,'本报告到阶段A为止，不继续构造归一化双向矩阵、$\\alpha_{dc}$ 或SCR/H/DVC统一扫描。\n');
end

function s=yesno(v), if v, s='是'; else, s='否'; end, end
function v=getFlagValueLocal(s,name,default), if isfield(s,name)&&~isempty(s.(name)), v=double(s.(name)); else, v=default; end, end
