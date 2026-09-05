function Cp = wind_cp_5mw(lambda,beta_deg)
%WIND_CP_5MW Generic variable-speed turbine Cp(lambda,beta) characteristic.
% lambda is dimensionless and beta_deg is in degrees.

lambda = max(lambda,0.1);
beta_deg = max(beta_deg,0);
lambda_i = 1 ./ (1 ./ (lambda + 0.08.*beta_deg) - ...
    0.035 ./ (beta_deg.^3 + 1));
Cp = 0.5176 .* (116 ./ lambda_i - 0.4.*beta_deg - 5) .* ...
    exp(-21 ./ lambda_i) + 0.0068 .* lambda;
Cp = min(max(Cp,0),0.59);
end
