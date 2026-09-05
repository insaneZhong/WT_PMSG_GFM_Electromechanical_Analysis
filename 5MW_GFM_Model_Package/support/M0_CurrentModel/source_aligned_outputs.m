function y=source_aligned_outputs(x,p)
%SOURCE_ALIGNED_OUTPUTS Six paper-facing outputs of source_aligned_rhs.
% Order: Ppcc [W], Udc [V], Tgen [N m], Tsh [N m],
%        omega_t-omega_g [rad/s], omega_vsg [rad/s].
Rf=p(5); Cf=p(7); Rd=p(8); Kt=p(18); Ksh=p(21); Dsh=p(22);
theta=x(1); wt=x(2); wg=x(3); imq=x(5); Udc=x(9); wv=x(12);
ifd=x(18); ifq=x(19); vcd=x(20); vcq=x(21); igd=x(22); igq=x(23); %#ok<NASGU>
icapd=ifd-igd; icapq=ifq-igq;
vnodeD=vcd+Rd*ifd-(Rd+1e-4)*igd;
vnodeQ=vcq+Rd*ifq-(Rd+1e-4)*igq;
Ppcc=1.5*(vnodeD*igd+vnodeQ*igq);
Tgen=Kt*imq;
Tsh=Ksh*theta+Dsh*(wt-wg);
y=[Ppcc;Udc;Tgen;Tsh;wt-wg;wv];
end
