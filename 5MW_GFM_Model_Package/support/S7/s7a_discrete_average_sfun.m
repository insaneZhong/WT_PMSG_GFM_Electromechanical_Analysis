function [sys,x0,str,ts]=s7a_discrete_average_sfun(~,x,u,flag,pvec,z0,Ts,tau)
%S7A_DISCRETE_AVERAGE_SFUN  S7A参考数字平均模型的离散S-Function。
% 仅包含M0方程、采样/ZOH/Forward-Euler软件状态和可选计算延迟；
% 不包含PWM、开关器件、限幅、保护或旧C控制器状态。
switch flag
    case 0
        sizes=simsizes;
        sizes.NumContStates=0;
        sizes.NumDiscStates=31;       % 23 x状态 + 当前/上一拍命令各4
        sizes.NumOutputs=29;           % 与M0 y输出一致
        sizes.NumInputs=6;
        sizes.DirFeedthrough=1;
        sizes.NumSampleTimes=1;
        sys=simsizes(sizes);
        x0=z0(:); str=[]; ts=[Ts 0];
    case 2
        sys=s7a_discrete_average_core('step',x,u,pvec,Ts,tau);
        x0=[]; str=[]; ts=[];
    case 3
        sys=s7a_discrete_average_core('output',x,u,pvec);
        x0=[]; str=[]; ts=[];
    case {1,4,9}
        sys=[]; x0=[]; str=[]; ts=[];
    otherwise
        error('Unhandled S-function flag %d.',flag);
end
end
