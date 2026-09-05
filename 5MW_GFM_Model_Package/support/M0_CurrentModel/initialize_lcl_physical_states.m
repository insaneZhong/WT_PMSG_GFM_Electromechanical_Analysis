function initialize_lcl_physical_states()
%INITIALIZE_LCL_PHYSICAL_STATES Transfers the M0 LCL equilibrium to the
% retained physical three-phase branches of the single idealized copy.
here=fileparts(mfilename('fullpath'));
run(fullfile(here,'initialize_currentmodel_continuous_controller.m'));
mdl='Grid_Forming_PMSG5MW_TwoMass_Idealized';
load_system(fullfile(here,[mdl '.slx']));
lf={'L1','L2','L6'}; lg={'L3','L4','L5'}; cf={'L7','L8','L9'};
for k=1:3
    % Specialized Power Systems branches ignore InitialCurrent unless
    % SetiL0 is enabled.  M0 equilibrium values must therefore enable
    % the block initial-condition flags explicitly.
    set_param([mdl '/' lf{k}],'SetiL0','on','InitialCurrent', ...
        sprintf('IdealLfIabc0(%d)',k));
    set_param([mdl '/' lg{k}],'SetiL0','on','InitialCurrent', ...
        sprintf('IdealLgIabc0(%d)',k));
    % Damped shunt branch retains both inductor-current and capacitor-
    % voltage states, hence enable both flags before assigning M0 values.
    set_param([mdl '/' cf{k}],'SetiL0','on','Setx0','on','InitialCurrent', ...
        sprintf('IdealCfIabc0(%d)',k));
    set_param([mdl '/' cf{k}],'InitialVoltage', ...
        sprintf('IdealCfVabc0(%d)',k));
end
save_system(mdl,fullfile(here,[mdl '.slx']));
close_system(mdl,0);
end
