function compile_vsg_sfunction(varargin)
%COMPILE_VSG_SFUNCTION Compile the VSG-aligned controller S-Function.
%  This script builds main_vsg.mexw64 for Grid_FormingVSG_PMSG.mdl.
%  It keeps the legacy main.mexw64 untouched, so the original P-f PI
%  baseline model and this VSG-aligned model can be tested separately.
%
%  Optional name-value pairs:
%    VSG_H   - virtual inertia coefficient H
%    VSG_MP  - active power/frequency droop coefficient
%    MotorFF - MSC active-power-to-Iq feedforward gain, A/W
%    MotorKp - MSC DC-voltage PI proportional gain
%    MotorKi - MSC DC-voltage PI integral gain
%    MotorKc - MSC DC-voltage PI anti-windup gain
%    MotorVoltLimit - MSC current-loop PI voltage limit, V
%    DvcType - MSC-DVC structure selector: 1 = Type a, 3 = Type c.
%    MotorCurrentKp/Ki - MSC dq current-loop PI gains
%    MotorIqPiSign - diagnostic sign for MSC q-axis PI voltage output.
%    MotorIqFeedbackSign - diagnostic sign for MSC q-axis current feedback.
%    GridCurrentKp/Ki - GSC dq current-loop PI gains
%    GridVoltageKp/Ki - GSC capacitor-voltage-loop PI gains
%    GridPowerKp/Ki - legacy GSC P-f loop PI gains. These only affect
%        builds with ENABLE_VSG_EQUIV_WREF disabled, but are kept as a
%        named entry for comparisons with the original P-f controller.
%    PowerLpfCutoff - PCC active/reactive power measurement LPF cutoff, Hz.
%    PrefRampSlope - GSC active-power reference ramp slope, W/s
%    DcCapF - DC-link capacitance macro GRID_UDC__C, F
%    MotorLd/MotorLq - PMSG dq inductance macros, H

root = fileparts(mfilename('fullpath'));
oldDir = pwd;
cleanupObj = onCleanup(@() cd(oldDir));

cd(root);
clear mex;

cfg = struct('VSG_H', [], 'VSG_MP', [], 'MotorFF', [], 'MotorKp', [], 'MotorKi', [], ...
    'MotorKc', [], 'MotorVoltLimit', [], 'DvcType', [], ...
    'MotorCurrentKp', [], 'MotorCurrentKi', [], 'MotorCurrentKc', [], ...
    'MotorIqPiSign', [], 'MotorIqFeedbackSign', [], ...
    'GridCurrentKp', [], 'GridCurrentKi', [], 'GridCurrentKc', [], ...
    'GridVoltageKp', [], 'GridVoltageKi', [], ...
    'GridPowerKp', [], 'GridPowerKi', [], ...
    'PowerLpfCutoff', [], 'PrefRampSlope', [], ...
    'DcCapF', [], 'MotorLd', [], 'MotorLq', []);
if mod(nargin, 2) ~= 0
    error('Use name-value pairs, e.g. compile_vsg_sfunction(''VSG_H'', 200).');
end
for k = 1:2:nargin
    cfg.(varargin{k}) = varargin{k+1};
end

defs = {};
if ~isempty(cfg.VSG_H)
    defs{end+1} = sprintf('-DVSG_EQUIV_H=%.12ef', cfg.VSG_H); %#ok<AGROW>
end
if ~isempty(cfg.VSG_MP)
    defs{end+1} = sprintf('-DVSG_EQUIV_MP=%.12ef', cfg.VSG_MP); %#ok<AGROW>
end
if ~isempty(cfg.MotorFF)
    defs{end+1} = sprintf('-DMOTOR_IQ_POWER_FF_A_PER_W=%.12ef', cfg.MotorFF); %#ok<AGROW>
end
if ~isempty(cfg.DcCapF)
    defs{end+1} = sprintf('-DGRID_UDC__C=%.12ef', cfg.DcCapF); %#ok<AGROW>
end
if ~isempty(cfg.MotorLd)
    defs{end+1} = sprintf('-DMOTOR_LD=%.12ef', cfg.MotorLd); %#ok<AGROW>
end
if ~isempty(cfg.MotorLq)
    defs{end+1} = sprintf('-DMOTOR_LQ=%.12ef', cfg.MotorLq); %#ok<AGROW>
end
if ~isempty(cfg.MotorKp)
    defs{end+1} = sprintf('-DMOTOR_PWM_SPEED_KP=%.12ef', cfg.MotorKp); %#ok<AGROW>
end
if ~isempty(cfg.MotorKi)
    defs{end+1} = sprintf('-DMOTOR_PWM_SPEED_KI=%.12ef', cfg.MotorKi); %#ok<AGROW>
end
if ~isempty(cfg.MotorKc)
    defs{end+1} = sprintf('-DMOTOR_PWM_SPEED_KC=%.12ef', cfg.MotorKc); %#ok<AGROW>
end
if ~isempty(cfg.MotorVoltLimit)
    defs{end+1} = sprintf('-DMOTOR_PI_ID_OUT_MAX=%.12ef', cfg.MotorVoltLimit); %#ok<AGROW>
    defs{end+1} = sprintf('-DMOTOR_PI_ID_OUT_MIN=-%.12ef', cfg.MotorVoltLimit); %#ok<AGROW>
    defs{end+1} = sprintf('-DMOTOR_PI_IQ_OUT_MAX=%.12ef', cfg.MotorVoltLimit); %#ok<AGROW>
    defs{end+1} = sprintf('-DMOTOR_PI_IQ_OUT_MIN=-%.12ef', cfg.MotorVoltLimit); %#ok<AGROW>
end
if ~isempty(cfg.MotorCurrentKp)
    defs{end+1} = sprintf('-DMOTOR_ID_KP=%.12ef', cfg.MotorCurrentKp); %#ok<AGROW>
    defs{end+1} = sprintf('-DMOTOR_IQ_KP=%.12ef', cfg.MotorCurrentKp); %#ok<AGROW>
end
if ~isempty(cfg.MotorCurrentKi)
    defs{end+1} = sprintf('-DMOTOR_ID_KI=%.12ef', cfg.MotorCurrentKi); %#ok<AGROW>
    defs{end+1} = sprintf('-DMOTOR_IQ_KI=%.12ef', cfg.MotorCurrentKi); %#ok<AGROW>
end
if ~isempty(cfg.MotorCurrentKc)
    defs{end+1} = sprintf('-DMOTOR_ID_KC=%.12ef', cfg.MotorCurrentKc); %#ok<AGROW>
    defs{end+1} = sprintf('-DMOTOR_IQ_KC=%.12ef', cfg.MotorCurrentKc); %#ok<AGROW>
end
if ~isempty(cfg.MotorIqPiSign)
    if ~ismember(cfg.MotorIqPiSign, [-1, 1])
        error('MotorIqPiSign must be +1 or -1.');
    end
    defs{end+1} = sprintf('-DMOTOR_IQ_PI_OUTPUT_SIGN=%d', cfg.MotorIqPiSign); %#ok<AGROW>
end
if ~isempty(cfg.MotorIqFeedbackSign)
    if ~ismember(cfg.MotorIqFeedbackSign, [-1, 1])
        error('MotorIqFeedbackSign must be +1 or -1.');
    end
    defs{end+1} = sprintf('-DMOTOR_IQ_FEEDBACK_SIGN=%d', cfg.MotorIqFeedbackSign); %#ok<AGROW>
end
if ~isempty(cfg.GridCurrentKp)
    defs{end+1} = sprintf('-DCURRENT_ID_KP=%.12ef', cfg.GridCurrentKp); %#ok<AGROW>
    defs{end+1} = sprintf('-DCURRENT_IQ_KP=%.12ef', cfg.GridCurrentKp); %#ok<AGROW>
end
if ~isempty(cfg.GridCurrentKi)
    defs{end+1} = sprintf('-DCURRENT_ID_KI=%.12ef', cfg.GridCurrentKi); %#ok<AGROW>
    defs{end+1} = sprintf('-DCURRENT_IQ_KI=%.12ef', cfg.GridCurrentKi); %#ok<AGROW>
end
if ~isempty(cfg.GridCurrentKc)
    defs{end+1} = sprintf('-DCURRENT_ID_KC=%.12ef', cfg.GridCurrentKc); %#ok<AGROW>
    defs{end+1} = sprintf('-DCURRENT_IQ_KC=%.12ef', cfg.GridCurrentKc); %#ok<AGROW>
end
if ~isempty(cfg.GridVoltageKp)
    defs{end+1} = sprintf('-DGSI_V_LOOP_KP=%.12ef', cfg.GridVoltageKp); %#ok<AGROW>
end
if ~isempty(cfg.GridVoltageKi)
    defs{end+1} = sprintf('-DGSI_V_LOOP_KI=%.12ef', cfg.GridVoltageKi); %#ok<AGROW>
end
if ~isempty(cfg.GridPowerKp)
    defs{end+1} = sprintf('-DGSI_PLOOP_KP=%.12ef', cfg.GridPowerKp); %#ok<AGROW>
end
if ~isempty(cfg.GridPowerKi)
    defs{end+1} = sprintf('-DGSI_PLOOP_KI=%.12ef', cfg.GridPowerKi); %#ok<AGROW>
end
if ~isempty(cfg.PowerLpfCutoff)
    defs{end+1} = sprintf('-DGSI_POWER_LPF_CUTOFF=%.12ef', cfg.PowerLpfCutoff); %#ok<AGROW>
end
if ~isempty(cfg.PrefRampSlope)
    defs{end+1} = sprintf('-DGSI_PREF_RAMP_SLOPE=%.12ef', cfg.PrefRampSlope); %#ok<AGROW>
end
if ~isempty(cfg.DvcType)
    if ~ismember(cfg.DvcType, [1, 3])
        error('DvcType must be 1 for Type a or 3 for Type c.');
    end
    defs{end+1} = sprintf('-DMOTOR_MSC_DVC_TYPE=%d', cfg.DvcType); %#ok<AGROW>
end

mex(defs{:}, ...
    'main_vsg.c', ...
    'svpwm.c', ...
    'motorcontrol.c', ...
    'grid_forming_control_vsg.c');

fprintf('VSG S-Function compiled: %s\n', fullfile(root, ['main_vsg.' mexext]));
end
