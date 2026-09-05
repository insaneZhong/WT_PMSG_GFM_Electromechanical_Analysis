function R = analyze_bidirectional_disturbance_transfer(outDir)
%ANALYZE_BIDIRECTIONAL_DISTURBANCE_TRANSFER
% 在同一个23状态SSM中量化 Grid->Machine 与 Machine->Grid 的方向依赖扰动传递。
% 输入/输出并非共轭能量端口，因此不使用“非互易性”表述。
if nargin<1 || isempty(outDir), outDir=fileparts(mfilename('fullpath')); end
[models,base]=prepare_multimode_models(); p=base.parameter_vector; nM=numel(models);
defs=struct('Direction',{'GRID_TO_MACHINE','GRID_TO_MACHINE','MACHINE_TO_GRID','MACHINE_TO_GRID'}, ...
    'Disturbance',{'Grid angle','Grid frequency','Mechanical torque','Equivalent aerodynamic power'}, ...
    'InputIndex',{3,4,1,2},'Outputs',{{'omega_sh','T_sh'},{'omega_sh','T_sh'}, {'omega_sh','T_sh','omega_g','T_e','P_MSC','Udc','P_GSC','P_PCC','delta','omega_v'}, {'omega_sh','T_sh','omega_g','T_e','P_MSC','Udc','P_GSC','P_PCC','delta','omega_v'}});
rows=nM*sum(cellfun(@numel,{defs.Outputs})); rr=0;
T=table('Size',[rows 10], ...
    'VariableTypes',{'string','string','string','string','double','double','double','double','double','string'}, ...
    'VariableNames',{'Architecture','Direction','Disturbance','Output','Frequency_Hz','Magnitude','Phase_deg','TorsionalResidue','RelativeGain','Observability'});
cache=struct;
for a=1:nM
    L=models{a}; M=multimode_modal_data(L.A,L.state_names); it=multimode_pick_torsion_mode(M); f=abs(imag(M.lambda(it)))/(2*pi); w=2*pi*f;
    for d=1:numel(defs)
        def=defs(d); for o=1:numel(def.Outputs)
            out=def.Outputs{o}; iy=find(strcmp(L.output_names,out),1); assert(~isempty(iy),'缺少双向分析输出 %s。',out);
            h=L.C(iy,:)*((1i*w*eye(size(L.A))-L.A)\L.B(:,def.InputIndex))+L.D(iy,def.InputIndex);
            r=L.C(iy,:)*M.V(:,it)*(M.W(:,it)'*L.B(:,def.InputIndex));
            rr=rr+1; T.Architecture(rr)=string(L.label); T.Direction(rr)=string(def.Direction); T.Disturbance(rr)=string(def.Disturbance); T.Output(rr)=string(out); T.Frequency_Hz(rr)=f; T.Magnitude(rr)=abs(h); T.Phase_deg(rr)=rad2deg(angle(h)); T.TorsionalResidue(rr)=abs(r); T.RelativeGain(rr)=NaN;
            if def.Direction=="MACHINE_TO_GRID" && out=="P_PCC"
                T.Observability(rr)=iffLocal(abs(r)>1e-8,"TORSIONAL_MODE_OBSERVABLE","NO_NUMERICAL_OBSERVABILITY");
            else, T.Observability(rr)="NOT_APPLICABLE"; end
        end
    end
    cache.(localField(L.mode))=L;
end
T=T(1:rr,:);
% 相对增益以同输出、同扰动的GFL为统一参考；GFL参考本身取1。
for r=1:height(T)
    ref=find(T.Architecture=="GFL" & T.Direction==T.Direction(r) & T.Disturbance==T.Disturbance(r) & T.Output==T.Output(r),1);
    if ~isempty(ref), T.RelativeGain(r)=T.Magnitude(r)/max(T.Magnitude(ref),eps); end
end
writetable(T,fullfile(outDir,'Bidirectional_Disturbance_Transfer_Summary.csv'));
Mtx=localMatrix(T); writetable(Mtx,fullfile(outDir,'Bidirectional_Coupling_Matrix_Summary.csv'));
R=struct('summary',T,'matrix',Mtx,'models',{models},'baseline_parameters',p);
end

function M=localMatrix(T)
arch=unique(T.Architecture,'stable'); M=table('Size',[numel(arch) 7], ...
 'VariableTypes',{'string','double','double','double','double','double','double'}, ...
 'VariableNames',{'Architecture','Gmm_Tm_to_omega_sh','Gmg_angle_to_omega_sh','Gmg_frequency_to_omega_sh','Ggm_Tm_to_P_PCC','Ggm_Paero_to_P_PCC','Ggg_frequency_to_P_PCC'});
for k=1:numel(arch)
    M.Architecture(k)=arch(k); M.Gmm_Tm_to_omega_sh(k)=localMag(T,arch(k),"Mechanical torque","omega_sh");
    M.Gmg_angle_to_omega_sh(k)=localMag(T,arch(k),"Grid angle","omega_sh"); M.Gmg_frequency_to_omega_sh(k)=localMag(T,arch(k),"Grid frequency","omega_sh");
    M.Ggm_Tm_to_P_PCC(k)=localMag(T,arch(k),"Mechanical torque","P_PCC"); M.Ggm_Paero_to_P_PCC(k)=localMag(T,arch(k),"Equivalent aerodynamic power","P_PCC");
    M.Ggg_frequency_to_P_PCC(k)=localMag(T,arch(k),"Grid frequency","P_PCC");
end
end
function x=localMag(T,a,d,o), ix=find(T.Architecture==a & T.Disturbance==d & T.Output==o,1); if isempty(ix), x=NaN; else, x=T.Magnitude(ix); end, end
function f=localField(s), f=regexprep(lower(char(s)),'[^a-z0-9]','_'); end
function x=iffLocal(c,a,b), if c, x=a; else, x=b; end, end
