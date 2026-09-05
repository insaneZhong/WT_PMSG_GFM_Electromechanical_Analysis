function result = audit_currentmodel_dc_power_sign(stopTime)
%AUDIT_CURRENTMODEL_DC_POWER_SIGN Gate-B in-memory port-power audit.
% The report holds only end/tail summary values, never raw waveforms.
if nargin<1, stopTime=0.005; end
here=fileparts(mfilename('fullpath'));
install_power_energy_audit_signals();
mdl='Grid_Forming_PMSG5MW_TwoMass_Idealized';
in=Simulink.SimulationInput(mdl);
in=in.setModelParameter('StopTime',num2str(stopTime,'%.15g'), ...
    'ReturnWorkspaceOutputs','on');
out=sim(in);
pm=out.get('ideal_Pmsc_ac'); pg=out.get('ideal_Pgsc_ac'); ud=out.get('ideal_Udc_state');
t=double(ud.Time(:)); U=double(ud.Data(:));
Pmsc=interp1(double(pm.Time(:)),double(pm.Data(:)),t,'linear','extrap');
Pgsc=interp1(double(pg.Time(:)),double(pg.Data(:)),t,'linear','extrap');
dUdt=gradient(U,t);
n=numel(t); idx=max(1,n-min(200,n-1)):n;
result=struct();
result.stopTime_s=stopTime;
result.Pmsc_tail_W=mean(Pmsc(idx));
result.Pgsc_tail_W=mean(Pgsc(idx));
result.Udc_tail_V=mean(U(idx));
result.dUdt_tail_Vps=mean(dUdt(idx));
result.energy_rhs_tail_W=0.3*result.Udc_tail_V*result.dUdt_tail_Vps;
result.power_difference_tail_W=result.Pmsc_tail_W-result.Pgsc_tail_W;
result.relative_energy_residual=abs(result.energy_rhs_tail_W-result.power_difference_tail_W)/ ...
    max(1e3,max(abs([result.energy_rhs_tail_W,result.power_difference_tail_W])));
result.note='Cold-start diagnostic only; it is not a stability or operating-point pass.';
fid=fopen(fullfile(here,'06_GateB_PortPower_Audit_CN.md'),'w','n','UTF-8');
fprintf(fid,'# Gate B：理想VSC端口功率与DC-link符号审计\n\n');
fprintf(fid,'- 运行方式：冷启动短时诊断 %.6g s（非平衡点，不用于模态结论）。\n',stopTime);
fprintf(fid,'- MSC交流端口功率：%.6g W\n- GSC交流端口功率：%.6g W\n',result.Pmsc_tail_W,result.Pgsc_tail_W);
fprintf(fid,'- Udc：%.6g V；dUdc/dt：%.6g V/s\n',result.Udc_tail_V,result.dUdt_tail_Vps);
fprintf(fid,'- 0.3 Udc dUdc/dt：%.6g W；Pmsc-Pgsc：%.6g W；归一残差：%.6g\n', ...
    result.energy_rhs_tail_W,result.power_difference_tail_W,result.relative_energy_residual);
fprintf(fid,'\n%s\n',result.note);
fclose(fid);
end
