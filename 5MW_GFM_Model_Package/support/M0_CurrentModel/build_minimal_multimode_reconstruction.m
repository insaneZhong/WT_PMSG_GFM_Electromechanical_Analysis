function [T,dominant,details,gate] = build_minimal_multimode_reconstruction(models,outDir)
%BUILD_MINIMAL_MULTIMODE_RECONSTRUCTION
% 从全SSM阶跃响应中找出满足 5% NRMSE / 0.98相关度的最小模态集合。
if nargin<2 || isempty(outDir), outDir=fileparts(mfilename('fullpath')); end
distNames={'Grid angle','Grid frequency'}; inIdx=[3 4]; amps=[deg2rad(0.2),2*pi*0.05];
outNames={'omega_sh','T_sh'}; stepTime=0.10; stopTime=10; nPts=5001;
maxRows=numel(models)*numel(inIdx)*numel(outNames)*23; rr=0;
T=table('Size',[maxRows 9], ...
 'VariableTypes',{'string','string','string','double','string','double','double','double','string'}, ...
 'VariableNames',{'Architecture','Disturbance','Output','NumModePairs','IncludedModeIDs','NRMSE','Correlation','PeakError_pct','Status'});
dominant=table('Size',[numel(models)*numel(inIdx)*numel(outNames) 9], ...
 'VariableTypes',{'string','string','string','double','string','double','double','double','string'}, ...
 'VariableNames',{'Architecture','Disturbance','Output','NumModePairs','IncludedModeIDs','NRMSE','Correlation','PeakError_pct','Status'}); dr=0;
details=struct('architecture',{},'disturbance',{},'output',{},'t',{},'full',{},'minimal',{},'selected',{},'lambda',{},'residue',{},'physical_class',{});
for a=1:numel(models)
    L=models{a}; MD=multimode_modal_data(L.A,L.state_names); cand=localCandidates(MD.lambda);
    for d=1:numel(inIdx)
        u=zeros(4,1); u(inIdx(d))=amps(d); [t,~,Y]=multimode_simulate_linear_step(L,u,stepTime,stopTime,nPts);
        for o=1:numel(outNames)
            iy=find(strcmp(L.output_names,outNames{o}),1); yFull=Y(:,iy); dr=dr+1;
            if max(abs(yFull))<1e-12
                rr=rr+1; T(rr,:)={string(L.label),string(distNames{d}),string(outNames{o}),0,"",0,1,0,"NO_EXCITATION"};
                dominant(dr,:)=T(rr,:); details(end+1)=localDetail(L,distNames{d},outNames{o},t,yFull,zeros(size(yFull)),[],[],[],{}); %#ok<AGROW>
                continue;
            end
            R=zeros(numel(cand),1); score=zeros(numel(cand),1);
            for q=1:numel(cand)
                k=cand(q); R(q)=L.C(iy,:)*MD.V(:,k)*(MD.W(:,k)'*L.B*u); score(q)=localPairFactor(MD.lambda(k))*abs(R(q)/MD.lambda(k));
            end
            [~,ord]=sort(score,'descend'); selected=[]; yMin=zeros(size(t)); hit=false; bestRow=0;
            for q=1:numel(ord)
                selected(end+1)=cand(ord(q)); %#ok<AGROW>
                yMin=localStepReconstruct(t,stepTime,MD.lambda(selected),R(ord(1:q)),0);
                [nrmse,corrv,peak]=localMetrics(yFull,yMin);
                status=iffLocal(nrmse<0.05 && corrv>0.98,"PASS","CONTINUE");
                rr=rr+1; T(rr,:)={string(L.label),string(distNames{d}),string(outNames{o}),q,string(localIDs(selected)),nrmse,corrv,peak,status}; bestRow=rr;
                if status=="PASS", hit=true; break; end
            end
            if ~hit, T.Status(bestRow)="REVIEW"; end
            dominant(dr,:)=T(bestRow,:);
            details(end+1)=localDetail(L,distNames{d},outNames{o},t,yFull,yMin,selected,MD.lambda(selected),R(ord(1:numel(selected))),MD.physical_class(selected)); %#ok<AGROW>
        end
    end
end
T=T(1:rr,:); writetable(T,fullfile(outDir,'Minimal_Multimode_Reconstruction.csv')); writetable(dominant,fullfile(outDir,'Minimal_Dominant_Mode_Set.csv'));
gate=all(dominant.Status=="PASS" | dominant.Status=="NO_EXCITATION");
end

function y=localStepReconstruct(t,tStep,lam,R,direct)
tau=max(t-tStep,0); y=zeros(size(t)); ix=t>=tStep;
for q=1:numel(lam)
    term=R(q)/lam(q).*(exp(lam(q).*tau(ix))-1);
    if imag(lam(q))>1e-8, y(ix)=y(ix)+2*real(term); else, y(ix)=y(ix)+real(term); end
end
y(ix)=y(ix)+direct;
end
function [nrmse,corrv,peak]=localMetrics(a,b)
nrmse=norm(a-b)/max(norm(a-mean(a)),eps); aa=a-mean(a); bb=b-mean(b); corrv=(aa'*bb)/max(norm(aa)*norm(bb),eps); peak=100*abs(max(abs(a))-max(abs(b)))/max(max(abs(a)),eps);
end
function s=localIDs(ix), s=strjoin(arrayfun(@(k)sprintf('M%02d',k),ix,'UniformOutput',false),','); end
function q=localDetail(L,d,o,t,full,minimal,sel,lam,R,physicalClass), q=struct('architecture',L.label,'disturbance',d,'output',o,'t',t,'full',full,'minimal',minimal,'selected',sel,'lambda',lam,'residue',R,'physical_class',{physicalClass}); end
function cand=localCandidates(lam), cand=find((imag(lam)>1e-8) | (abs(imag(lam))<=1e-8 & abs(lam)>1e-6)); end
function f=localPairFactor(lam), if imag(lam)>1e-8, f=2; else, f=1; end, end
function v=iffLocal(c,a,b), if c, v=a; else, v=b; end, end
