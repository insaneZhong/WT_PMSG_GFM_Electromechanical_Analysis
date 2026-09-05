function report=idealize_pmsg_dynamic_blocks()
%IDEALIZE_PMSG_DYNAMIC_BLOCKS Replaces the four local PMSG sample integrators
% with continuous Integrators in the one maintained idealized model.
% External two-mass speed input remains unchanged.
here=fileparts(mfilename('fullpath'));
mdl='Grid_Forming_PMSG5MW_TwoMass_Idealized';
load_system(fullfile(here,[mdl '.slx']));
targets={ ...
    [mdl '/PMSM1/elemodel3/iq,id/id/Discrete-Time Integrator'],'IdealPmsgId0'; ...
    [mdl '/PMSM1/elemodel3/iq,id/iq/Discrete-Time Integrator'],'IdealPmsgIq0'; ...
    [mdl '/PMSM1/Mechanical model/Discrete-Time Integrator'],'0'; ...
    [mdl '/PMSM1/Mechanical model/Discrete-Time Integrator1'],'0'};
for k=1:size(targets,1)
    old=targets{k,1};
    if ~isempty(find_system(old,'SearchDepth',0)) && ...
            strcmp(get_param(old,'BlockType'),'DiscreteIntegrator')
        replaceOneDiscreteIntegrator(old,targets{k,2});
    end
end

function replaceOneDiscreteIntegrator(old,initialCondition)
% Capture only the direct signal endpoints, then replace the block in place.
[parent,name]=fileparts(old);
pos=get_param(old,'Position');
orient=get_param(old,'Orientation');
lh=get_param(old,'LineHandles');
srcName=''; srcPort=[]; dstName={}; dstPort=[];
if lh.Inport~=-1
    srcName=getfullname(get_param(lh.Inport,'SrcBlockHandle'));
    srcPort=get_param(get_param(lh.Inport,'SrcPortHandle'),'PortNumber');
end
if lh.Outport~=-1
    dBlocks=get_param(lh.Outport,'DstBlockHandle');
    dPorts=get_param(lh.Outport,'DstPortHandle');
    for q=1:numel(dBlocks)
        dstName{end+1}=getfullname(dBlocks(q)); %#ok<AGROW>
        dstPort(end+1)=get_param(dPorts(q),'PortNumber'); %#ok<AGROW>
    end
end
if lh.Inport~=-1, delete_line(lh.Inport); end
if lh.Outport~=-1, delete_line(lh.Outport); end
delete_block(old);
add_block('simulink/Continuous/Integrator',old, ...
    'Position',pos,'Orientation',orient,'InitialCondition',initialCondition);
if ~isempty(srcName)
    srcLocal=erase(srcName,[parent '/']);
    add_line(parent,[srcLocal '/' num2str(srcPort)], ...
        [name '/1'],'autorouting','on');
end
for q=1:numel(dstName)
    dstLocal=erase(dstName{q},[parent '/']);
    add_line(parent,[name '/1'], ...
        [dstLocal '/' num2str(dstPort(q))],'autorouting','on');
end
end
set_param(mdl,'SimulationCommand','update');
save_system(mdl,fullfile(here,[mdl '.slx']));
close_system(mdl,0);
report=struct('model',fullfile(here,[mdl '.slx']), ...
    'replacedDiscreteIntegratorCount',size(targets,1));
end
