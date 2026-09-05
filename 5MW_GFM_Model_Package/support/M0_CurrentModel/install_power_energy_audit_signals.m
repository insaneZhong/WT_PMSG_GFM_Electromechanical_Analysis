function install_power_energy_audit_signals()
%INSTALL_POWER_ENERGY_AUDIT_SIGNALS Adds three scalar diagnostic taps only.
% They are in-memory ToWorkspace logs; no raw MAT/CSV data is written.
here=fileparts(mfilename('fullpath'));
mdl='Grid_Forming_PMSG5MW_TwoMass_Idealized';
load_system(fullfile(here,[mdl '.slx']));
items={ ...
    'Ideal_Pmsc_AcPort','ideal_Pmsc_ac',[1120 320 1215 340]; ...
    'Ideal_Pgsc_AcPort','ideal_Pgsc_ac',[1120 360 1215 380]; ...
    'Ideal_Udc_State','ideal_Udc_state',[1120 400 1215 420]};
src={'Ideal_MSC_AverageVSC/1','Ideal_GSC_AverageVSC/1','Ideal_DC_UdcState/1'};
for k=1:3
    b=[mdl '/' items{k,1}];
    if isempty(find_system(mdl,'SearchDepth',1,'Name',items{k,1}))
        add_block('simulink/Sinks/To Workspace',b, ...
            'VariableName',items{k,2},'SaveFormat','Timeseries', ...
            'MaxDataPoints','5000', ...
            'Position',items{k,3});
        add_line(mdl,src{k},[items{k,1} '/1'],'autorouting','on');
    end
end
save_system(mdl,fullfile(here,[mdl '.slx']));
close_system(mdl,0);
end
