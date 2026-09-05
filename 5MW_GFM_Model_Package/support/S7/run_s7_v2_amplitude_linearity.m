function R=run_s7_v2_amplitude_linearity(varargin)
%RUN_S7_V2_AMPLITUDE_LINEARITY  D1机械扰动幅值线性验收（仅保存摘要）。
ip=inputParser;ip.addParameter('StopTime',0.2,@(x)isnumeric(x)&&isscalar(x)&&x>0);ip.addParameter('StepTime',0.02,@(x)isnumeric(x)&&isscalar(x)&&x>=0);ip.parse(varargin{:});o=ip.Results;
here=fileparts(mfilename('fullpath'));idealDir=fileparts(here);addpath(idealDir);addpath(here);
S=load(fullfile(idealDir,'M0_5MW_Aligned_Workpoint_and_SSM.mat'),'params','operating_point');[pvec,~]=m0_pack_parameters(S.params,S.operating_point);x0=S.operating_point.x0(:);u0=zeros(6,1);cmd=s7a_discrete_average_core('commands',x0,u0,pvec);z0=[x0;cmd;cmd];y0=s7a_discrete_average_core('output',z0,u0,pvec);Ts=S.params.controller_Ts_s;Tm0=S.params.Tm0_Nm;
build_s7a_discrete_average_model('Ts',Ts,'Delay',Ts,'Compile',true);
mags=[0.00025 0.0005 0.001];rows=struct([]);for k=1:numel(mags)
    N=round(o.StopTime/Ts)+1;t=(0:N-1)'*Ts;u=zeros(N,6);ii=find(t>=o.StepTime,1);u(ii:end,1)=mags(k)*Tm0;
    in=Simulink.SimulationInput('S7A_DiscreteAvg_5MW');in=in.setModelParameter('StopTime',num2str(o.StopTime,'%.15g'));in=in.setExternalInput(timeseries(u,t));so=sim(in);ds=so.yout;try,v=ds.getElement(1).Values;catch,v=ds{1}.Values;end;y=v.Data.';idx=t>=o.StepTime;yTe=y(7,idx)-y0(7);yW=y(11,idx)-y0(11);rr=struct('amplitude_pu',mags(k),'Te_peak_Nm',max(abs(yTe)),'omegaRel_peak_radps',max(abs(yW)),'Te_peak_per_pu_Nm',max(abs(yTe))/mags(k),'omegaRel_peak_per_pu',max(abs(yW))/mags(k));if isempty(rows),rows=rr;else,rows(end+1)=rr;end
end
base=rows(2);for k=1:numel(rows),rows(k).Te_slope_error=rows(k).Te_peak_per_pu_Nm/base.Te_peak_per_pu_Nm-1;rows(k).omegaRel_slope_error=rows(k).omegaRel_peak_per_pu/base.omegaRel_peak_per_pu-1;rows(k).status='PASS_IF_ABS_SLOPE_ERROR_LT_5_PERCENT';end
csv=fullfile(here,'S7_V2_Amplitude_Linearity.csv');writeCsv(csv,rows);maxErr=max(abs([[rows.Te_slope_error] [rows.omegaRel_slope_error]]),[],'all');R=struct('csv',csv,'max_abs_slope_error',maxErr,'pass',maxErr<0.05);fprintf('V2幅值线性验收：max error=%.6g, pass=%d\n',maxErr,R.pass);
end
function writeCsv(path,S),fn=fieldnames(S);fid=fopen(path,'w');c=onCleanup(@()fclose(fid));for j=1:numel(fn),if j>1,fprintf(fid,',');end,fprintf(fid,'%s',fn{j});end,fprintf(fid,'\n');for i=1:numel(S),for j=1:numel(fn),if j>1,fprintf(fid,',');end,fprintf(fid,'%.15g',S(i).(fn{j}));end,fprintf(fid,'\n');end,end
