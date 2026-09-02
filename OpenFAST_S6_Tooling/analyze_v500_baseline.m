function R = analyze_v500_baseline(caseName)
%ANALYZE_V500_BASELINE Audit only the new v5.0.0 double-precision flexible
% OpenFAST periodic linearization.  This is deliberately plant-only: no GFM
% controller is connected until the open-loop flexible baseline is stable.

if nargin < 1 || strlength(string(caseName)) == 0
    caseName = '5MW_Land_Linear_Aero_CalcSteady_v500';
end
caseName = char(string(caseName));
here = fileparts(mfilename('fullpath'));
caseDir = fullfile(here,'runtime',caseName);
toolboxDir = fullfile(here,'matlab-toolbox');
files = arrayfun(@(k)fullfile(caseDir, ...
    sprintf('%s.%d.lin',caseName,k)),1:3, ...
    'UniformOutput',false);
assert(all(cellfun(@(f)exist(f,'file')==2,files)), ...
    'Missing new v5.0.0 periodic .lin files.');

oldPath = path;
cleaner = onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(genpath(toolboxDir));
[mbc,md] = fx_mbc3(files);
assert(mbc.performedTransformation, ...
    'The official MBC transformation did not complete.');

Aavg = real(mbc.AvgA);
stateNames = string(mbc.DescStates(:));
[V,D] = eig(Aavg);
lam = diag(D);
[avgMaxReal,iAvg] = max(real(lam));
[~,iAvgDom] = max(abs(V(:,iAvg)));

az = md.Azimuth(:);
if max(abs(az)) > 2*pi + sqrt(eps)
    az = deg2rad(az);
end
omega = mean(md.Omega);
[azSort,order] = sort(mod(az,2*pi));
dt = diff([azSort;azSort(1)+2*pi])/omega;
Phi = eye(size(Aavg));
for k = 1:numel(order)
    Phi = expm(real(mbc.A(:,:,order(k)))*dt(k))*Phi;
end
[VF,DF] = eig(Phi);
mu = diag(DF);
lamF = log(mu)/sum(dt);
[floquetMaxReal,iF] = max(real(lamF));
[~,iFDom] = max(abs(VF(:,iF)));

isAD = ~startsWith(stateNames,"ED ");
isED = ~isAD;
R = struct();
R.source = "OpenFAST v5.0.0 double, CalcSteady 8 m/s: " + string(caseName);
R.nstates = size(Aavg,1);
R.nED = nnz(isED);
R.nAD = nnz(isAD);
R.omega_radps = omega;
R.azimuth_deg = rad2deg(az(:)).';
R.avg_max_real = avgMaxReal;
R.avg_worst_frequency_hz = abs(imag(lam(iAvg)))/(2*pi);
R.avg_dominant_state = stateNames(iAvgDom);
R.floquet_max_real = floquetMaxReal;
R.floquet_worst_frequency_hz = abs(imag(lamF(iF)))/(2*pi);
R.floquet_multiplier = abs(mu(iF));
R.floquet_dominant_state = stateNames(iFDom);
R.avg_stable = avgMaxReal < 0;
R.floquet_stable = floquetMaxReal < 0;

fprintf('V500_NSTATES=%d ED=%d AD=%d\n',R.nstates,R.nED,R.nAD);
fprintf('V500_OMEGA=%.12g rad/s AZ=[%.8g %.8g %.8g] deg\n', ...
    R.omega_radps,R.azimuth_deg);
fprintf('V500_AVG_MAX_REAL=%.12g F=%.12gHz DOM=%s STABLE=%d\n', ...
    R.avg_max_real,R.avg_worst_frequency_hz,R.avg_dominant_state,R.avg_stable);
fprintf('V500_FLOQUET_MAX_REAL=%.12g F=%.12gHz MU=%.12g DOM=%s STABLE=%d\n', ...
    R.floquet_max_real,R.floquet_worst_frequency_hz,R.floquet_multiplier, ...
    R.floquet_dominant_state,R.floquet_stable);
end
