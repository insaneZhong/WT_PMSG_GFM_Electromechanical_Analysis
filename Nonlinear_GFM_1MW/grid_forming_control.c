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

MOTOR_LOW_PASS_FILTER    lpf,lpf1;
MOTOR_HIGH_PASS_FILTER   hpf;
MOTOR_BAND_PASS_FILTER   bandpf;
static float w_vsg_state = VSG_EQUIV_W0;

void motor_low_pass_filter(MOTOR_LOW_PASS_FILTER *v);
void motor_high_pass_filter(MOTOR_HIGH_PASS_FILTER *v);
void motor_band_pass_filter(MOTOR_BAND_PASS_FILTER *v);
void motor_band_pass_filter1(MOTOR_BAND_PASS_FILTER *v);

//##########################################################################################################
//                             网侧整流器控制主程序
//##########################################################################################################
void grid_side_control(GRID_SIDE_INV *p)
{	
    
    if (  system_Time < PRESYN_SWITCH_TIME     ) p->val.Pre_syn = 0;
    if (  system_Time > PRESYN_SWITCH_TIME     ) p->val.Pre_syn = 1;
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

          p->val.grid_phase_angle = p->val.freq * p->Ts + p->val.grid_phase_angle;
          if( p->val.grid_phase_angle > MOTOR_2PI_RADIAN)
            { p->val.grid_phase_angle =  p->val.grid_phase_angle  -  MOTOR_2PI_RADIAN;}
         if( p->val.grid_phase_angle < 0)
            { p->val.grid_phase_angle =  p->val.grid_phase_angle + MOTOR_2PI_RADIAN;}       
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
      p->pf.we_set = 314;
      if  (p->val.Pre_syn == 1 )
      {
          vloop_slope.Ts    = p->Ts;
          vloop_slope.Init = 0;
          vloop_slope.In    = p->ref.P_active_power_ref; 
          vloop_slope.Slope = 5000000;
          motor_slope_limit_calc(&vloop_slope);  

#if ENABLE_VSG_EQUIV_WREF
          {
              float p_err = vloop_slope.Out - p->val.pcc_P_active_Power_filter;
              float dw = (p_err - (w_vsg_state - VSG_EQUIV_W0)/VSG_EQUIV_MP) / (2.0f*VSG_EQUIV_H*VSG_EQUIV_W0);
              w_vsg_state += p->Ts * dw;
              p->pf.w_ref = w_vsg_state;
          }
#else
          power_loop_pi.Ref =  vloop_slope.Out;
          power_loop_pi.Fdb =  p->val.pcc_P_active_Power_filter;
          motor_PI2_calc(&power_loop_pi) ;          
          p->pf.w_ref = 314 - power_loop_pi.Out;
#endif
          
          if (p->pf.w_ref  > GSI_WREF_MAX )   p->pf.w_ref  = GSI_WREF_MAX;
          if (p->pf.w_ref  < GSI_WREF_MIN )   p->pf.w_ref  = GSI_WREF_MIN;  
          
          p->pf.we_set = p->pf.w_ref;
          p->pf.thet_ref = p->pf.thet_ref + p->Ts * p->pf.w_ref;
      }       
      if  (p->val.Pre_syn == 0 ) //锁相完成并网前
      {
              p->pf.w_ref = p->val.freq;
              w_vsg_state = p->val.freq;
              p->pf.thet_ref = p->val.grid_phase_angle;
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
       if  (p->val.Pre_syn == 1 )
       {
             p->ref.voltage_ref = 563  + 3.45/100000.0 *(p->ref.Q_reactive_power_ref - p->val.pcc_Q_reactive_Power_filter );
             p->bak.CosPos     =  cos( p->pf.thet_ref );
 	         p->bak.SinPos     =  sin( p->pf.thet_ref );
       }

     p->pf.E_voltage_amplitude = p->ref.voltage_ref;
     if (p->pf.E_voltage_amplitude  > 800)   p->pf.E_voltage_amplitude = 800;
     if (p->pf.E_voltage_amplitude  < 0)     p->pf.E_voltage_amplitude = 0;
     
	 p->pf.U_od_ref    =  p->pf.E_voltage_amplitude;
	 p->pf.U_oq_ref    =  0;
//##########################################################################################################    
//                           逆变器电压环和电流环控制
//##########################################################################################################   
    if (p->val.Pre_syn == 1 ) 
    {
        d_voltage_loop_pi.Ref =  p->pf.U_od_ref;
        d_voltage_loop_pi.Fdb =  p->val.pcc_u_d;
        motor_PI2_calc(&d_voltage_loop_pi) ;

        p->val.Id_ref = d_voltage_loop_pi.Out  - GRID_FILTER__C * p->pf.w_ref * p->val.pcc_u_q;

        d_loop_pi.Ref      =  p->val.Id_ref; 
        d_loop_pi.Fdb      =  p->val.Id ;
        motor_PI2_calc(&d_loop_pi);

        p->val.Ud1_ref = d_loop_pi.Out  - p->pf.w_ref * GRID_FILTER__LS * p->val.Iq ;

        q_voltage_loop_pi.Ref =  p->pf.U_oq_ref;
        q_voltage_loop_pi.Fdb =  p->val.pcc_u_q;
        motor_PI2_calc(&q_voltage_loop_pi) ;

        p->val.Iq_ref = q_voltage_loop_pi.Out  + GRID_FILTER__C * p->pf.w_ref * p->val.pcc_u_d  ;

        q_loop_pi.Ref      =  p->val.Iq_ref; 
        q_loop_pi.Fdb      =  p->val.Iq ;
        motor_PI2_calc(&q_loop_pi);

        p->val.Uq1_ref = q_loop_pi.Out + p->pf.w_ref * GRID_FILTER__LS * p->val.Id ;
    }
    else
    {
        p->val.Uq1_ref = 0;
        p->val.Ud1_ref = 563;
        p->bak.CosPos  =  cos( p->val.grid_phase_angle );
 	    p->bak.SinPos  =  sin( p->val.grid_phase_angle );
        q_loop_pi.Ui   = 0;
        d_loop_pi.Ui   = 563;
    }
    
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











