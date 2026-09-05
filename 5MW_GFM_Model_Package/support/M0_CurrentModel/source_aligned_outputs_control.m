function y=source_aligned_outputs_control(x,p,mode,d,flags)
%SOURCE_ALIGNED_OUTPUTS_CONTROL 公共控制模式输出。
if nargin<4 || isempty(d), d=zeros(4,1); end
if nargin<5 || isempty(flags), flags=struct; end
mode=upper(char(string(mode))); %#ok<NASGU>
Rd=p(8); Kt=p(18); Ksh=p(21); Dsh=p(22); w0=p(3); mp=p(34); Pref=p(37); Vdc0=p(2); Kpdc=p(25);
theta=x(1); wt=x(2); wg=x(3); imq=x(5); Udc=x(9); wv=x(12); Pf=x(10); xiDc=x(6);
ifd=x(18); ifq=x(19); vcd=x(20); vcq=x(21); igd=x(22); igq=x(23);
icapd=ifd-igd; icapq=ifq-igq; vnodeD=vcd+Rd*ifd-(Rd+1e-4)*igd; vnodeQ=vcq+Rd*ifq-(Rd+1e-4)*igq;
Ppcc=1.5*(vnodeD*igd+vnodeQ*igq); Tgen=Kt*imq; Tsh=Ksh*theta+Dsh*(wt-wg);
switch mode
    case 'VSG', wctrl=wv;
    case 'DROOP', wctrl=w0+mp*(Pref-Pf);
    case 'GFL', wctrl=w0+d(4);
    case 'GFMGWT', wctrl=wv;
    otherwise, error('Unsupported control mode: %s',mode);
end
y=[Ppcc;Udc;Tgen;Tsh;wt-wg;wctrl];
end

function v=getFlagValue(s,name,default)
if isfield(s,name) && ~isempty(s.(name)), v=double(s.(name)); else, v=default; end
end
