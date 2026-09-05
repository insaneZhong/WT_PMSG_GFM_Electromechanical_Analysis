function names = m0_state_names()
%M0_STATE_NAMES 唯一状态顺序；模型、平衡点和线性化共同使用。
names = [ ...
    "theta_sh_rad"; ...       % 1  轴扭转角
    "omega_t_radps"; ...      % 2  风轮转速
    "omega_g_radps"; ...      % 3  发电机转速
    "i_m_d_A"; ...            % 4  PMSG d轴电流
    "i_m_q_gen_A"; ...        % 5  正发电制动q轴电流
    "xi_dvc_A"; ...           % 6  Type-A DVC积分输出
    "xi_m_id_V"; ...          % 7  MSC d轴电流PI积分输出
    "xi_m_iq_V"; ...          % 8  MSC q轴电流PI积分输出
    "Udc_V"; ...              % 9  直流母线电压
    "P_f_W"; ...              % 10 PCC有功滤波状态
    "Q_f_var"; ...            % 11 PCC无功滤波状态
    "omega_vsg_radps"; ...    % 12 VSG角频率
    "delta_v_rad"; ...        % 13 VSG相对电网功角
    "xi_g_vd_A"; ...          % 14 GSC电压环d轴积分输出
    "xi_g_vq_A"; ...          % 15 GSC电压环q轴积分输出
    "xi_g_id_V"; ...          % 16 GSC电流环d轴积分输出
    "xi_g_iq_V"; ...          % 17 GSC电流环q轴积分输出
    "i_f_d_A"; ...            % 18 变流器侧滤波电流（电网dq）
    "i_f_q_A"; ...            % 19
    "v_cf_d_V"; ...           % 20 滤波电容电压（电网dq）
    "v_cf_q_V"; ...           % 21
    "i_g_d_A"; ...            % 22 PCC流向电网电流（电网dq）
    "i_g_q_A"];               % 23
end
