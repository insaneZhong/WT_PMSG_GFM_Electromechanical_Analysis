//##########################################################################################################
//##########################################################################################################
//##########################################################################################################
//##########################################################################################################
//##########################################################################################################
//##########################################################################################################
//##########################################################################################################
//##########################################################################################################
//#  FunctionName:   网侧整流器控制
//#      Designer:   李梦迪
//#   Description:   目标是380V电网电压，得到直流母线电压恒定为600V,系统的开关频率为4k
//#       history:   2025-2-25
//##########################################################################################################
//##########################################################################################################
#include        "math.h" 
#include        "grid_forming_control.h"
#include        "motorcontrol.h"
#ifdef ENABLE_SCHEMEA_OVERRIDES
#include        "schemeA_tuning_overrides.h"
#endif
#ifndef GSI_LVRT_CURRENT_REF_AW_GAIN
#define GSI_LVRT_CURRENT_REF_AW_GAIN 0.0f
#endif
#ifndef GSI_LVRT_VOLTAGE_AW_GAIN
#define GSI_LVRT_VOLTAGE_AW_GAIN 0.0f
#endif
#ifndef GSI_LVRT_ACTIVE_CURRENT_PRIORITY
#define GSI_LVRT_ACTIVE_CURRENT_PRIORITY 0
#endif
#ifndef GSI_LVRT_FREEZE_CURRENT_PI_INTEGRAL
#define GSI_LVRT_FREEZE_CURRENT_PI_INTEGRAL 0
#endif
/* Optional startup-only bumpless voltage takeover.  The baseline remains
 * unchanged when this macro is zero; the experimental controller enables it
 * through compile-time overrides so the paper baseline is not modified. */
#ifndef GSI_BUMPLESS_TAKEOVER
#define GSI_BUMPLESS_TAKEOVER 0
#endif
#ifndef GSI_BUMPLESS_TAKEOVER_DURATION_S
#define GSI_BUMPLESS_TAKEOVER_DURATION_S 0.25f
#endif

void clack_transform(CLACK *v);
void park_transform(PARK *v);

extern void motor_slope_limit_calc(MOTOR_SLOPE_LIMIT *v);
extern void motor_PI2_calc(MOTOR_PI *v);
extern void motor_slope_limit_calc(MOTOR_SLOPE_LIMIT *v);

MOTOR_SLOPE_LIMIT  vloop_slope;
MOTOR_PI   d_loop_pi = CURRENT_PI_ID_DEFAULTS;
MOTOR_PI   q_loop_pi = CURRENT_PI_IQ_DEFAULTS;                           
MOTOR_PI   d_voltage_loop_pi = VOLTAGE_LOOP_PI_DEFAULTS;
MOTOR_PI   q_voltage_loop_pi = VOLTAGE_LOOP_PI_DEFAULTS;
MOTOR_PI   PLL_loop_pi = PLL_LOOP_PI_DEFAULTS;
MOTOR_PI   E_voltage_loop_pi = E_VOLTAGE_LOOP_PI_DEFAULTS;
MOTOR_PI   power_loop_pi = POWER_LOOP_PI_DEFAULTS;


CLACK clack_trans;
PARK  park_u,park_PLL;
GRID_SIDE_INV  grid_side =  GRID_SIDE_INV_DEFAULTS; 
extern float system_Time;
extern int legacy_lvrt_active;

MOTOR_LOW_PASS_FILTER    lpf,lpf1;
MOTOR_HIGH_PASS_FILTER   hpf;
MOTOR_BAND_PASS_FILTER   bandpf;
static float w_vsg_state = VSG_EQUIV_W0;
static float w_vsg_sync_anchor = VSG_EQUIV_W0;
static float grid_pll_phase = 0.0f;
static float grid_pll_freq = VSG_EQUIV_W0;
static int gsi_takeover_initialized = 0;
static float gsi_takeover_ud_ref = 0.0f;
static float gsi_takeover_uq_ref = 0.0f;

float grid_side_get_w_vsg_state(void)
{
    return w_vsg_state;
}

void grid_side_set_w_vsg_state(float value)
{
    w_vsg_state = value;
}

float grid_side_get_w_vsg_sync_anchor(void)
{
    return w_vsg_sync_anchor;
}

void grid_side_set_w_vsg_sync_anchor(float value)
{
    w_vsg_sync_anchor = value;
}

void motor_low_pass_filter(MOTOR_LOW_PASS_FILTER *v);
void motor_high_pass_filter(MOTOR_HIGH_PASS_FILTER *v);
void motor_band_pass_filter(MOTOR_BAND_PASS_FILTER *v);
void motor_band_pass_filter1(MOTOR_BAND_PASS_FILTER *v);

void grid_side_reset(void)
{
    MOTOR_SLOPE_LIMIT vloop_slope_defaults = {0};
    MOTOR_PI d_loop_pi_defaults = CURRENT_PI_ID_DEFAULTS;
    MOTOR_PI q_loop_pi_defaults = CURRENT_PI_IQ_DEFAULTS;
    MOTOR_PI d_voltage_loop_pi_defaults = VOLTAGE_LOOP_PI_DEFAULTS;
    MOTOR_PI q_voltage_loop_pi_defaults = VOLTAGE_LOOP_PI_DEFAULTS;
    MOTOR_PI PLL_loop_pi_defaults = PLL_LOOP_PI_DEFAULTS;
    MOTOR_PI E_voltage_loop_pi_defaults = E_VOLTAGE_LOOP_PI_DEFAULTS;
    MOTOR_PI power_loop_pi_defaults = POWER_LOOP_PI_DEFAULTS;
    CLACK clack_trans_defaults = {0};
    PARK park_u_defaults = {0};
    GRID_SIDE_INV grid_side_defaults = GRID_SIDE_INV_DEFAULTS;
    MOTOR_LOW_PASS_FILTER lpf_defaults = {0};
    MOTOR_HIGH_PASS_FILTER hpf_defaults = {0};
    MOTOR_BAND_PASS_FILTER bandpf_defaults = {0};

    vloop_slope = vloop_slope_defaults;
    d_loop_pi = d_loop_pi_defaults;
    q_loop_pi = q_loop_pi_defaults;
    d_voltage_loop_pi = d_voltage_loop_pi_defaults;
    q_voltage_loop_pi = q_voltage_loop_pi_defaults;
    PLL_loop_pi = PLL_loop_pi_defaults;
    E_voltage_loop_pi = E_voltage_loop_pi_defaults;
    power_loop_pi = power_loop_pi_defaults;
    clack_trans = clack_trans_defaults;
    park_u = park_u_defaults;
    park_PLL = park_u_defaults;
    grid_side = grid_side_defaults;
    lpf = lpf_defaults;
    lpf1 = lpf_defaults;
    hpf = hpf_defaults;
    bandpf = bandpf_defaults;
    w_vsg_state = VSG_EQUIV_W0;
    w_vsg_sync_anchor = VSG_EQUIV_W0;
    grid_pll_phase = 0.0f;
    grid_pll_freq = VSG_EQUIV_W0;
    gsi_takeover_initialized = 0;
    gsi_takeover_ud_ref = 0.0f;
    gsi_takeover_uq_ref = 0.0f;
}

//##########################################################################################################
//                             网侧整流器控制主程序
//##########################################################################################################
void grid_side_control(GRID_SIDE_INV *p)
{
    int gfm_enabled;
    float gsi_takeover_alpha = 1.0f;
    
    /* Deterministic pre-synchronization state update to avoid startup chattering. */
    p->val.Pre_syn = (system_Time >= PRESYN_SWITCH_TIME) ? 1 : 0;
    gfm_enabled = (system_Time >= GSI_GFM_ENABLE_TIME_S) ? 1 : 0;
#if GSI_BUMPLESS_TAKEOVER
    /* Blend only the converter voltage command during the short handover.
     * The pre-synchronization command is the already synchronized nominal
     * voltage; the closed-loop GFM command is introduced gradually. */
    if (gfm_enabled)
    {
        const float takeover_duration =
            (GSI_BUMPLESS_TAKEOVER_DURATION_S > 1.0e-6f) ?
            GSI_BUMPLESS_TAKEOVER_DURATION_S : 1.0e-6f;
        /* The blend starts when the GFM controller actually takes over.
         * Using PRESYN_SWITCH_TIME here makes the blend finish long before
         * GFM is enabled whenever PLL presynchronization is intentionally
         * separated from GFM takeover, so the supposed bumpless path becomes
         * an abrupt handover again. */
        const float elapsed = system_Time - GSI_GFM_ENABLE_TIME_S;
        gsi_takeover_alpha = elapsed / takeover_duration;
        if (gsi_takeover_alpha < 0.0f) gsi_takeover_alpha = 0.0f;
        if (gsi_takeover_alpha > 1.0f) gsi_takeover_alpha = 1.0f;
    }
    else
    {
        gsi_takeover_initialized = 0;
    }
#endif
    /* steady-state validation: keep active-power reference constant */

//##########################################################################################################
// 1.并网前先进行预充电阶段
// 2.为了减少并网过程中电流冲击，启用电网锁相环，实际上的并网角度是相差不大， 启动电角度为开管时锁相角度
// 3.初始调制的电网电压幅值，也是电网电压幅值
// 4. 预同步同步控制：        a.控制q轴电网电压分量U_gq到0（通过PI调节器），生成相位角θ_isyn，确保频率、相位和幅值与电网同步。           
//                            b.引入虚拟功率或角度差调节
//                            c.启用监控条件，频率误差0.1Hz,电压幅值1%以内，电网电压角差5度以内，才能并网。
//##########################################################################################################
//                             计算得到系统的反馈信息
//##########################################################################################################    
      clack_trans.a = -1.0/3.0 * (p->bak.pcc_uca - p->bak.pcc_uab);
      clack_trans.b = -1.0/3.0 * (p->bak.pcc_uab - p->bak.pcc_ubc);
      clack_trans.c = -1.0/3.0 * (p->bak.pcc_ubc - p->bak.pcc_uca);
          
      clack_transform(&clack_trans);
      
      park_u.ualpha =  clack_trans.alpha;
      park_u.ubeta  =  clack_trans.beta;
      park_u.thet   =  p->pf.thet_ref; 
      park_transform(&park_u);
            
      p->val.pcc_u_q = park_u.uq;
      p->val.pcc_u_d = park_u.ud;
//##########################################################################################################
//      首先是锁相环节，在启动前完成，锁相完成后，有一个锁相完成标志位
//##########################################################################################################   
      if(p->val.Pre_syn == 0)
      {
          park_PLL.ualpha =  clack_trans.alpha;
          park_PLL.ubeta  =  clack_trans.beta;
          park_PLL.thet   =  p->val.grid_phase_angle;
          park_transform(&park_PLL);

          PLL_loop_pi.Ref = park_PLL.uq;   //注意park_u.uq
          PLL_loop_pi.Fdb =  0;
          motor_PI2_calc(&PLL_loop_pi);
          p->val.freq = PLL_loop_pi.Out+314;
          grid_pll_freq = p->val.freq;

          p->val.grid_phase_angle = p->val.freq * p->Ts + p->val.grid_phase_angle;
          if( p->val.grid_phase_angle > MOTOR_2PI_RADIAN)
            { p->val.grid_phase_angle =  p->val.grid_phase_angle  -  MOTOR_2PI_RADIAN;}
         if( p->val.grid_phase_angle < 0)
            { p->val.grid_phase_angle =  p->val.grid_phase_angle + MOTOR_2PI_RADIAN;}       
          grid_pll_phase = p->val.grid_phase_angle;
      }   
      else if(!gfm_enabled)
      {
          /* After breaker closure, do not let the PLL follow a PCC voltage
           * that is already influenced by the converter itself.  Preserve
           * the synchronized angle and advance it at the known grid nominal
           * frequency until the VSG takes over. */
          p->val.freq = GSI_NOMINAL_OMEGA_RADPS;
          p->val.grid_phase_angle += p->Ts * p->val.freq;
          if (p->val.grid_phase_angle > MOTOR_2PI_RADIAN)
              p->val.grid_phase_angle -= MOTOR_2PI_RADIAN;
          if (p->val.grid_phase_angle < 0)
              p->val.grid_phase_angle += MOTOR_2PI_RADIAN;
          grid_pll_phase = p->val.grid_phase_angle;
          grid_pll_freq = p->val.freq;
      }
      else
      {
          /* Measurement-only PLL after GFM takeover.  p->pf.thet_ref and
           * p->pf.w_ref remain VSG states, so this cannot become GFL. */
          park_PLL.ualpha = clack_trans.alpha;
          park_PLL.ubeta = clack_trans.beta;
          park_PLL.thet = grid_pll_phase;
          park_transform(&park_PLL);
          PLL_loop_pi.Ref = park_PLL.uq;
          PLL_loop_pi.Fdb = 0;
          motor_PI2_calc(&PLL_loop_pi);
          grid_pll_freq = PLL_loop_pi.Out + GSI_NOMINAL_OMEGA_RADPS;
          if (grid_pll_freq > GSI_NOMINAL_OMEGA_RADPS + VSG_GRID_PLL_FREQ_LIMIT_RADPS)
              grid_pll_freq = GSI_NOMINAL_OMEGA_RADPS + VSG_GRID_PLL_FREQ_LIMIT_RADPS;
          if (grid_pll_freq < GSI_NOMINAL_OMEGA_RADPS - VSG_GRID_PLL_FREQ_LIMIT_RADPS)
              grid_pll_freq = GSI_NOMINAL_OMEGA_RADPS - VSG_GRID_PLL_FREQ_LIMIT_RADPS;
          grid_pll_phase += p->Ts * grid_pll_freq;
          if (grid_pll_phase > MOTOR_2PI_RADIAN)
              grid_pll_phase -= MOTOR_2PI_RADIAN;
          if (grid_pll_phase < 0)
              grid_pll_phase += MOTOR_2PI_RADIAN;
      }
////////////////////////////////////////////////////////////////////////////////////      
      clack_trans.a = p->bak.Ia1;
      clack_trans.b = p->bak.Ib1;
      clack_trans.c = p->bak.Ic1;        
      clack_transform(&clack_trans);
      
      p->bak.I_alfa  =  clack_trans.alpha;
      p->bak.I_beta  =  clack_trans.beta;
      
      park_u.ualpha =  p->bak.I_alfa;
      park_u.ubeta  =  p->bak.I_beta;
      park_u.thet = p->pf.thet_ref;
      park_transform(&park_u); 
      
	  p->val.Id    =  park_u.ud;
	  p->val.Iq    =  park_u.uq;  
      
      clack_trans.a = p->bak.pcc_Ia;
      clack_trans.b = p->bak.pcc_Ib;
      clack_trans.c = p->bak.pcc_Ic;        
      clack_transform(&clack_trans);
      
      p->bak.I_alfa  =  clack_trans.alpha;
      p->bak.I_beta  =  clack_trans.beta;
      
      park_u.ualpha =  p->bak.I_alfa;
      park_u.ubeta  =  p->bak.I_beta;
      park_u.thet = p->pf.thet_ref;
      park_transform(&park_u); 
      
	  p->val.pcc_Id    =  park_u.ud;
	  p->val.pcc_Iq    =  park_u.uq;  
           
      p->val.pcc_P_active_Power   = 1.5 * ( p->val.pcc_Id *  p->val.pcc_u_d  +  p->val.pcc_Iq *  p->val.pcc_u_q);
      p->val.pcc_Q_reactive_Power = 1.5 * ( p->val.pcc_Id *  p->val.pcc_u_q  -  p->val.pcc_Iq *  p->val.pcc_u_d);
                    
     lpf.Ui=p->val.pcc_P_active_Power;
     lpf.fs_cutoff=20;
     motor_low_pass_filter( &lpf );
     p->val.pcc_P_active_Power_filter = lpf.out;
    
     lpf1.Ui=p->val.pcc_Q_reactive_Power;
     lpf1.fs_cutoff=20;
     motor_low_pass_filter( &lpf1 );
     p->val.pcc_Q_reactive_Power_filter = lpf1.out;
//##########################################################################################################    
//                             P-f控制
//########################################################################################################## 
      float m1,m2,m4,m3=0;
      p->pf.we_set = GSI_NOMINAL_OMEGA_RADPS;
      if  (gfm_enabled)
      {
          vloop_slope.Ts    = p->Ts;
          vloop_slope.Init = 0;
          vloop_slope.In    = p->ref.P_active_power_ref;
          vloop_slope.Slope = GSI_PREF_RAMP_SLOPE;
          motor_slope_limit_calc(&vloop_slope);  

#if GSI_GFL_MODE
          /* Grid-following comparison branch: the PLL supplies angle and
           * frequency. Active/reactive power are regulated through current
           * references below; no swing equation or Q-V voltage source is
           * active in this branch. */
          p->pf.w_ref = grid_pll_freq;
          p->pf.we_set = grid_pll_freq;
          p->pf.thet_ref = grid_pll_phase;
#else
#if ENABLE_VSG_EQUIV_WREF
          {
              float p_err = vloop_slope.Out -
                  p->val.pcc_P_active_Power_filter;
              float transition_alpha;
#if VSG_GRID_SYNC_ENABLE
              /* Move only the VSG synchronization reference toward the
               * measurement.  Injecting the PLL-frequency error directly
               * into dw was tested and conflicts with this legacy model's
               * measured P-angle sign; the swing state itself must remain
               * governed by the VSG equation below. */
              w_vsg_sync_anchor += p->Ts*VSG_GRID_SYNC_KW*
                  (grid_pll_freq - w_vsg_sync_anchor);
#endif
              float delta_w = w_vsg_state - w_vsg_sync_anchor;
              float dw_startup = (VSG_STARTUP_POWER_ERROR_SIGN*p_err -
                  delta_w/VSG_STARTUP_MP) /
                  (2.0f*VSG_EQUIV_H*VSG_EQUIV_W0);
              /* SI operating swing equation:
               *   dw/dt = w0/(2*H*Sbase) * (DeltaP-DeltaW/mp)
               * Preserve the validated commissioning dynamics only during
               * cold start, then blend to the physical operating equation. */
              float dw_operating = (VSG_POWER_ERROR_SIGN*p_err -
                  delta_w/VSG_EQUIV_MP) *
                  VSG_EQUIV_W0 /
                  (2.0f*VSG_EQUIV_H*VSG_EQUIV_SBASE_W);
              if (VSG_DYNAMICS_TRANSITION_DURATION_S <= 0.0f)
              {
                  transition_alpha = 1.0f;
              }
              else
              {
                  transition_alpha = (system_Time -
                      VSG_DYNAMICS_TRANSITION_START_S) /
                      VSG_DYNAMICS_TRANSITION_DURATION_S;
                  if (transition_alpha < 0.0f) transition_alpha = 0.0f;
                  if (transition_alpha > 1.0f) transition_alpha = 1.0f;
              }
              float dw = (1.0f-transition_alpha)*dw_startup +
                  transition_alpha*dw_operating;
              w_vsg_state += p->Ts * dw;
              /* Keep the validated PWM/Park angle coordinate continuous.
               * Its measured incremental P-angle orientation is negative,
               * therefore the operating swing equation uses P-Pref through
               * VSG_POWER_ERROR_SIGN=-1 instead of mirroring frequency. */
              p->pf.w_ref = w_vsg_state;
          }
#else
          power_loop_pi.Ref =  vloop_slope.Out;
          power_loop_pi.Fdb =  p->val.pcc_P_active_Power_filter;
          motor_PI2_calc(&power_loop_pi) ;          
          p->pf.w_ref = GSI_NOMINAL_OMEGA_RADPS + GSI_PF_LOOP_SIGN * power_loop_pi.Out;
#endif
          
          if (p->pf.w_ref  > GSI_WREF_MAX )   p->pf.w_ref  = GSI_WREF_MAX;
          if (p->pf.w_ref  < GSI_WREF_MIN )   p->pf.w_ref  = GSI_WREF_MIN;  
          
          p->pf.we_set = p->pf.w_ref;
          p->pf.thet_ref = p->pf.thet_ref + p->Ts * p->pf.w_ref;
#endif
      }       
      if  (!gfm_enabled) // PLL tracking before GFM takeover
      {
              p->pf.w_ref = p->val.freq;
              w_vsg_state = p->val.freq;
              w_vsg_sync_anchor = p->val.freq;
              p->pf.thet_ref = p->val.grid_phase_angle;
#if !ENABLE_VSG_EQUIV_WREF
              /*
               * Bumpless transfer for the legacy P-to-frequency PI path.
               * Track the PLL frequency in the PI integral state while the
               * converter is in presynchronization.  Without this tracking,
               * the first enabled sample forces w_ref back to the hard-coded
               * nominal value (314 rad/s), causing a phase step whenever the
               * grid is not exactly at nominal frequency.
               */
              power_loop_pi.Ui =
                  (p->val.freq - VSG_EQUIV_W0)/GSI_PF_LOOP_SIGN;
              power_loop_pi.Out = power_loop_pi.Ui;
              power_loop_pi.OutPreSat = power_loop_pi.Ui;
              power_loop_pi.Up = 0.0f;
              power_loop_pi.Up_old = 0.0f;
              power_loop_pi.Ud = 0.0f;
              power_loop_pi.SatErr = 0.0f;
#endif
      }
                    
      if(p->pf.thet_ref>MOTOR_2PI_RADIAN)
      {p->pf.thet_ref = p->pf.thet_ref - MOTOR_2PI_RADIAN;}
       if(p->pf.thet_ref <0)
      {p->pf.thet_ref = p->pf.thet_ref + MOTOR_2PI_RADIAN;}
 //##########################################################################################################    
//                             交流电压幅值控制
//##########################################################################################################  
//       if  (p->val.Pre_syn == 0 )//锁相完成并网前
//       {
//              p->ref.voltage_ref  =  park_PLL.ud;
//       } 
       if  (gfm_enabled && !GSI_GFL_MODE)
       {
             p->ref.voltage_ref = GSI_NOMINAL_VOLTAGE_PHASE_PEAK_V +
                 GSI_QV_DROOP_V_PER_VAR *
                 (p->ref.Q_reactive_power_ref -
                  p->val.pcc_Q_reactive_Power_filter);
       }
       p->bak.CosPos = cos(p->pf.thet_ref);
       p->bak.SinPos = sin(p->pf.thet_ref);

     p->pf.E_voltage_amplitude = p->ref.voltage_ref;
     if (p->pf.E_voltage_amplitude  > GSI_E_VOLTAGE_MAX_V)   p->pf.E_voltage_amplitude = GSI_E_VOLTAGE_MAX_V;
     if (p->pf.E_voltage_amplitude  < 0)     p->pf.E_voltage_amplitude = 0;
     
	 p->pf.U_od_ref    =  p->pf.E_voltage_amplitude;
	 p->pf.U_oq_ref    =  0;
//##########################################################################################################    
//                           逆变器电压环和电流环控制
//##########################################################################################################   
    if (p->val.Pre_syn == 1)
    {
        float current_ref_magnitude;
        float voltage_ref_magnitude;
        float voltage_ref_limit;
        float vector_scale;
        float id_ref_unsat;
        float iq_ref_unsat;
        float ud_ref_unsat;
        float uq_ref_unsat;

#if GSI_GFL_MODE
        {
            float voltage_d_for_power = p->val.pcc_u_d;
            if (fabsf(voltage_d_for_power) < 0.20f*
                GSI_NOMINAL_VOLTAGE_PHASE_PEAK_V)
            {
                voltage_d_for_power =
                    (voltage_d_for_power >= 0.0f ? 1.0f : -1.0f)*
                    0.20f*GSI_NOMINAL_VOLTAGE_PHASE_PEAK_V;
            }
            /* With the implemented power convention and PLL d-axis:
             * P = 1.5*Ud*Id, Q = -1.5*Ud*Iq when Uq is near zero. */
            p->val.Id_ref = vloop_slope.Out/
                (1.5f*voltage_d_for_power);
            p->val.Iq_ref = -p->ref.Q_reactive_power_ref/
                (1.5f*voltage_d_for_power);
        }
#else
        d_voltage_loop_pi.Ref =  p->pf.U_od_ref;
        d_voltage_loop_pi.Fdb =  p->val.pcc_u_d;
        motor_PI2_calc(&d_voltage_loop_pi) ;

        p->val.Id_ref = d_voltage_loop_pi.Out  - GRID_FILTER__C * p->pf.w_ref * p->val.pcc_u_q;

        q_voltage_loop_pi.Ref =  p->pf.U_oq_ref;
        q_voltage_loop_pi.Fdb =  p->val.pcc_u_q;
        motor_PI2_calc(&q_voltage_loop_pi) ;

        p->val.Iq_ref = q_voltage_loop_pi.Out  + GRID_FILTER__C * p->pf.w_ref * p->val.pcc_u_d  ;
#endif

        if (GSI_CURRENT_VECTOR_LIMIT_A > 0.0f &&
            (legacy_lvrt_active || GSI_NORMAL_LIMITS_ENABLE))
        {
            current_ref_magnitude = sqrtf(
                p->val.Id_ref*p->val.Id_ref +
                p->val.Iq_ref*p->val.Iq_ref);
            if (current_ref_magnitude > GSI_CURRENT_VECTOR_LIMIT_A)
            {
                float iq_remaining_limit;
                id_ref_unsat = p->val.Id_ref;
                iq_ref_unsat = p->val.Iq_ref;
                if (GSI_LVRT_ACTIVE_CURRENT_PRIORITY)
                {
                    /* PCC voltage is d-oriented: preserve active current. */
                    if (fabsf(p->val.Id_ref) >= GSI_CURRENT_VECTOR_LIMIT_A)
                    {
                        p->val.Id_ref = (p->val.Id_ref >= 0.0f) ?
                            GSI_CURRENT_VECTOR_LIMIT_A :
                            -GSI_CURRENT_VECTOR_LIMIT_A;
                        p->val.Iq_ref = 0.0f;
                    }
                    else
                    {
                        iq_remaining_limit = sqrtf(
                            GSI_CURRENT_VECTOR_LIMIT_A*GSI_CURRENT_VECTOR_LIMIT_A -
                            p->val.Id_ref*p->val.Id_ref);
                        if (p->val.Iq_ref > iq_remaining_limit)
                            p->val.Iq_ref = iq_remaining_limit;
                        if (p->val.Iq_ref < -iq_remaining_limit)
                            p->val.Iq_ref = -iq_remaining_limit;
                    }
                }
                else
                {
                    vector_scale =
                        GSI_CURRENT_VECTOR_LIMIT_A/current_ref_magnitude;
                    p->val.Id_ref *= vector_scale;
                    p->val.Iq_ref *= vector_scale;
                }
                d_voltage_loop_pi.Ui += GSI_LVRT_CURRENT_REF_AW_GAIN *
                    (p->val.Id_ref - id_ref_unsat);
                q_voltage_loop_pi.Ui += GSI_LVRT_CURRENT_REF_AW_GAIN *
                    (p->val.Iq_ref - iq_ref_unsat);
            }
        }

        d_loop_pi.Ref      =  p->val.Id_ref; 
        d_loop_pi.Fdb      =  p->val.Id ;
        if (legacy_lvrt_active && GSI_LVRT_FREEZE_CURRENT_PI_INTEGRAL)
        {
            float ui_hold = d_loop_pi.Ui;
            motor_PI2_calc(&d_loop_pi);
            d_loop_pi.Ui = ui_hold;
        }
        else
        {
            motor_PI2_calc(&d_loop_pi);
        }
        p->val.Ud1_ref = d_loop_pi.Out  -
            p->pf.w_ref * GRID_FILTER__LS * p->val.Iq
#if GSI_GFL_MODE
            + p->val.pcc_u_d
#endif
            ;

        q_loop_pi.Ref      =  p->val.Iq_ref; 
        q_loop_pi.Fdb      =  p->val.Iq ;
        if (legacy_lvrt_active && GSI_LVRT_FREEZE_CURRENT_PI_INTEGRAL)
        {
            float ui_hold = q_loop_pi.Ui;
            motor_PI2_calc(&q_loop_pi);
            q_loop_pi.Ui = ui_hold;
        }
        else
        {
            motor_PI2_calc(&q_loop_pi);
        }

        p->val.Uq1_ref = q_loop_pi.Out +
            p->pf.w_ref * GRID_FILTER__LS * p->val.Id
#if GSI_GFL_MODE
            + p->val.pcc_u_q
#endif
            ;

        if (GSI_VOLTAGE_MODULATION_LIMIT > 0.0f &&
            (legacy_lvrt_active || GSI_NORMAL_LIMITS_ENABLE))
        {
            voltage_ref_magnitude = sqrtf(
                p->val.Ud1_ref*p->val.Ud1_ref +
                p->val.Uq1_ref*p->val.Uq1_ref);
            voltage_ref_limit = GSI_VOLTAGE_MODULATION_LIMIT *
                p->bak.Udc1/1.5f;
            if (voltage_ref_magnitude > voltage_ref_limit &&
                voltage_ref_magnitude > 1.0e-6f)
            {
                ud_ref_unsat = p->val.Ud1_ref;
                uq_ref_unsat = p->val.Uq1_ref;
                vector_scale = voltage_ref_limit/voltage_ref_magnitude;
                p->val.Ud1_ref *= vector_scale;
                p->val.Uq1_ref *= vector_scale;
                d_loop_pi.Ui += GSI_LVRT_VOLTAGE_AW_GAIN *
                    (p->val.Ud1_ref - ud_ref_unsat);
                q_loop_pi.Ui += GSI_LVRT_VOLTAGE_AW_GAIN *
                    (p->val.Uq1_ref - uq_ref_unsat);
            }
        }
    }
    else
    {
        p->val.Uq1_ref = 0;
        p->val.Ud1_ref = GSI_NOMINAL_VOLTAGE_PHASE_PEAK_V;
        p->bak.CosPos  =  cos( p->val.grid_phase_angle );
 	    p->bak.SinPos  =  sin( p->val.grid_phase_angle );
        q_loop_pi.Ui   = 0;
        d_loop_pi.Ui   = GSI_NOMINAL_VOLTAGE_PHASE_PEAK_V;
    }

#if GSI_BUMPLESS_TAKEOVER
    if (gfm_enabled && gsi_takeover_alpha < 1.0f)
    {
        if (!gsi_takeover_initialized)
        {
            gsi_takeover_ud_ref = GSI_NOMINAL_VOLTAGE_PHASE_PEAK_V;
            gsi_takeover_uq_ref = 0.0f;
            gsi_takeover_initialized = 1;
        }
        p->val.Ud1_ref = gsi_takeover_ud_ref +
            gsi_takeover_alpha * (p->val.Ud1_ref - gsi_takeover_ud_ref);
        p->val.Uq1_ref = gsi_takeover_uq_ref +
            gsi_takeover_alpha * (p->val.Uq1_ref - gsi_takeover_uq_ref);
    }
#endif
    
//##########################################################################################################
 	p->out.Us_alfa = (p->val.Ud1_ref * p->bak.CosPos)- (p->val.Uq1_ref * p->bak.SinPos);
	p->out.Us_beta = (p->val.Ud1_ref * p->bak.SinPos)+ (p->val.Uq1_ref * p->bak.CosPos);  
 //##########################################################################################################    

} 
void clack_transform(CLACK    *v)
{
      v->alpha = 2.0 /3.0 *( v->a - 0.5 * v->b -  0.5 * v->c);
      v->beta   =  2.0 /3.0 * SQRT3 /2.0 *(  v->b -  v->c );
}
void park_transform(PARK *v)
{
       v->ud = v->ualpha * cos(v->thet) + v->ubeta * sin(v->thet) ;
       v->uq = v->ubeta  * cos(v->thet) - v->ualpha * sin(v->thet) ;
}

//*********************一阶低通滤波器**************************************
//传递函数y=1/(1+S / (2*pi*fs))
//a0=1+pi*fs*Ts;a1=pi*fs*Ts-1；
//b0=pi*fs*Ts;b1=pi*fs*Ts;
//y(n)=1/a0*(b0*x(n)-b1*x(n-1)-a1*y(n-1));
void motor_low_pass_filter(MOTOR_LOW_PASS_FILTER *v)
{
     float a0,a1,b0,b1;
     v->Ts_frequcncy  =  0.00025;
     a0 = 1.0 + v->Ts_frequcncy * MOTOR_PI_RADIAN * v->fs_cutoff ;
     a1 =  v->Ts_frequcncy * MOTOR_PI_RADIAN * v->fs_cutoff -1.0 ;
     b0 = MOTOR_PI_RADIAN * v->fs_cutoff *  v->Ts_frequcncy;
     b1 = MOTOR_PI_RADIAN * v->fs_cutoff *  v->Ts_frequcncy;
     v->out = 1.0 / a0 * ( b0 *v->Ui + b1 * v->Ui_n_1 - a1 * v->out );
     v->Ui_n_1 = v->Ui;
}

void motor_high_pass_filter(MOTOR_HIGH_PASS_FILTER *v)
{
    float a0,a1,b0,b1;
    v->Ts_frequcncy  =  0.0002;
    a0 = 1.0 + v->Ts_frequcncy * MOTOR_PI_RADIAN * v->fs_cutoff ;
    a1 =  v->Ts_frequcncy * MOTOR_PI_RADIAN * v->fs_cutoff -1.0 ;
    b0 = 1.0;
    b1 = -1.0;
    v->out = 1.0 / a0 * ( b0 *v->Ui + b1 * v->Ui_n_1 - a1 * v->out );
    v->Ui_n_1 = v->Ui;
}
//*************************************************************************
//************************带通滤波器1***************************************
//y=kws/(s2+ws+w2)
void motor_band_pass_filter(MOTOR_BAND_PASS_FILTER *v)
{
     float a0,a1,a2,b0,b1,b2; 
     a0 =  4.0 + 2.0 * v->k_gain *  v->Ts_frequcncy * v->w_cutoff  + v->w_cutoff * v->w_cutoff * v->Ts_frequcncy * v->Ts_frequcncy; 
     a1 =  2.0 * v->w_cutoff * v->w_cutoff * v->Ts_frequcncy * v->Ts_frequcncy-8.0;  
     a2 =  4.0 - 2.0 * v->k_gain *  v->Ts_frequcncy * v->w_cutoff  + v->w_cutoff * v->w_cutoff * v->Ts_frequcncy * v->Ts_frequcncy; 
     b0 = 2.0 * v->k_gain *  v->Ts_frequcncy * v->w_cutoff;
     b1 = 0.0;
     b2 = -2.0 * v->k_gain *  v->Ts_frequcncy * v->w_cutoff;
     
     v->Uo = 1.0 / a0 * ( b0 *v->Ui + b1 * v->Ui_n_1 + b2 * v->Ui_n_2 - a1 * v->Uo_n_1 -a2 * v->Uo_n_2 );
     v->Ui_n_2 = v->Ui_n_1;
     v->Ui_n_1 = v->Ui;
     v->Uo_n_2 =v->Uo_n_1;
     v->Uo_n_1 =v->Uo;
}

void motor_band_pass_filter1(MOTOR_BAND_PASS_FILTER *v)
{
     float a0, a1, a2, b0, b1, b2; 
     a0 =  4.0 + 2.0 * v->k_gain *  v->Ts_frequcncy * v->w_cutoff  + v->w_cutoff * v->w_cutoff * v->Ts_frequcncy * v->Ts_frequcncy; 
     a1 =  v->w_cutoff * v->w_cutoff * v->Ts_frequcncy * v->Ts_frequcncy-8.0;  
     a2 =  4.0 - 2.0 * v->k_gain *  v->Ts_frequcncy * v->w_cutoff  + v->w_cutoff * v->w_cutoff * v->Ts_frequcncy * v->Ts_frequcncy;
     b0 = 4.0 * v->k_gain * v->w_cutoff;
     b1 = -8.0 * v->k_gain * v->w_cutoff;
     b2 = 4.0 * v->k_gain * v->w_cutoff;
     
     v->Uo = 1.0 / a0 * ( b0 *v->Ui + b1 * v->Ui_n_1 + b2 * v->Ui_n_2 - a1 * v->Uo_n_1 -a2 * v->Uo_n_2 );
     v->Ui_n_2 = v->Ui_n_1;
     v->Ui_n_1 = v->Ui;
     v->Uo_n_2 =v->Uo_n_1;
     v->Uo_n_1 =v->Uo;
}











