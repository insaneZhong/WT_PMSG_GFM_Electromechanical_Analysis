function lin = linearize_m0_equilibrium(P,OP)
%LINEARIZE_M0_EQUILIBRIUM 对同一非线性RHS作数值雅可比。
if nargin<2
    [OP,P] = solve_m0_equilibrium(P);
end
[pvec,~] = m0_pack_parameters(P,OP);
x0=OP.x0; u0=zeros(6,1);
n=numel(x0); m=numel(u0); ny=numel(m0_output_names());
A=zeros(n); B=zeros(n,m); C=zeros(ny,n); D=zeros(ny,m);
for k=1:n
    h=1e-6*max(abs(x0(k)),1);
    xp=x0; xm=x0; xp(k)=xp(k)+h; xm(k)=xm(k)-h;
    [fp,yp]=m0_nonlinear_dynamics(xp,u0,pvec);
    [fm,ym]=m0_nonlinear_dynamics(xm,u0,pvec);
    A(:,k)=(fp-fm)/(2*h); C(:,k)=(yp-ym)/(2*h);
end
for k=1:m
    switch k
        case 1, base=max(abs(OP.Tm0_Nm),1);
        case {2,3}, base=P.Sbase_W;
        case 4, base=P.omega0_radps;
        case 5, base=1;
        otherwise, base=P.Vdc_ref_V;
    end
    h=1e-6*base;
    up=u0; um=u0; up(k)=h; um(k)=-h;
    [fp,yp]=m0_nonlinear_dynamics(x0,up,pvec);
    [fm,ym]=m0_nonlinear_dynamics(x0,um,pvec);
    B(:,k)=(fp-fm)/(2*h); D(:,k)=(yp-ym)/(2*h);
end
lambda=eig(A);
freq=abs(imag(lambda))/(2*pi);
zeta=-real(lambda)./max(abs(lambda),eps);
lin=struct('A',A,'B',B,'C',C,'D',D,'eigenvalues',lambda, ...
    'frequency_Hz',freq,'damping_ratio',zeta, ...
    'max_real_part',max(real(lambda)),'state_names',m0_state_names(), ...
    'input_names',["dTm_Nm";"dPref_W";"dQref_var"; ...
        "dOmegaGrid_radps";"dVgrid_pu";"dVdcRef_V"], ...
    'output_names',m0_output_names());
end
