function results = sweep_GFM_MWT_Drivetrain_Frequency()
% Scan provisional shaft and turbine inertia values to locate the nonlinear
% mechanical response band before designing an APCAD passband.

root = fileparts(mfilename('fullpath'));
oldFolder = pwd;
cleanup = onCleanup(@() cd(oldFolder));
cd(root);

run('GFM_MWT_Nonlinear_Params.m');
mdl = 'Grid_Forming_PMSG';
load_system(mdl);

K_scale = [0.5 0.75 1.0 1.25 1.5];
Jt_scale = [0.5 1.0 1.5 2.0];
rows = [];

for i = 1:numel(K_scale)
    for j = 1:numel(Jt_scale)
        in = Simulink.SimulationInput(mdl);
        in = in.setVariable('K_sh', K_sh*K_scale(i));
        in = in.setVariable('J_t', J_t*Jt_scale(j));
        in = in.setModelParameter('StopTime', '2.50', ...
            'ReturnWorkspaceOutputs', 'on', 'SimulationMode', 'normal');
        out = sim(in);
        sig = out.T_sh;
        mask = sig.Time >= frequency_estimation_start;
        [freq, mag] = estimatePeak(sig.Time(mask), sig.Data(mask));
        rows = [rows; K_scale(i), Jt_scale(j), freq, mag]; %#ok<AGROW>
    end
end

results = array2table(rows, 'VariableNames', ...
    {'K_sh_scale','J_t_scale','ObservedFrequency_Hz','PeakMagnitude'});
resultDir = fullfile(root, 'Validation_Results');
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end
writetable(results, fullfile(resultDir, 'drivetrain_frequency_sweep.csv'));
disp(results);
close_system(mdl, 0);
end

function [freqPeak, magPeak] = estimatePeak(t, x)
x = detrend(x(:));
t = t(:);
fs = 1/median(diff(t));
n = numel(x);
win = 0.5 - 0.5*cos(2*pi*(0:n-1)'/(n-1));
mag = abs(fft(x .* win));
freq = (0:n-1)'*fs/n;
band = freq > 0.1 & freq < 20;
[magPeak, idx] = max(mag(band));
chosen = freq(band);
freqPeak = chosen(idx);
end
