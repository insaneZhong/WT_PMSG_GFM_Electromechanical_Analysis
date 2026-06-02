#ifndef   __GRID_SIDE_RECTIFY_CONTROL__
#define   __GRID_SIDE_RECTIFY_CONTROL__
#define   SQRT3     1.732
#define   GRID_FILTER__LS                       0.00012       //   0.12mH
#define   GRID_FILTER__C                        0.000055     //  55uF
#define   GRID_LINE_impedance__L                0.0005    //0.5mH
#define   GRID__RS                              0.0002
#define   GRID_UDC__C                           0.03      
#ifndef PRESYN_SWITCH_TIME
#define   PRESYN_SWITCH_TIME                    0.5
#endif
/* Optional: enable VSG-equivalent frequency state update for strict
 * small-signal/nonlinear structure alignment.
 * 0: keep legacy P-loop -> w_ref PI implementation
 * 1: use 2H*w0*dw/dt = P_ref - P - (w-w0)/mp
 */
#define   ENABLE_VSG_EQUIV_WREF                0
#define   VSG_EQUIV_W0                         314.0
#define   VSG_EQUIV_H                          10.0
#define   VSG_EQUIV_MP                         1.57e-6
 
#define   V_REF_SLOPE_LIMIT_MAX                  1200      
#define   MOTOR_2PI_RADIAN  		             6.2831853    //弧度  360°
#define   PWM_2PI3_RADIAN                        2.0943950    //弧度 2pi/3

#define   CURRENT_LOOP_BANDWITH                  220   //Hz, mapped from small-signal model (scheme A)  //电流环的带宽一般是系统控制频率的0.1-0.2倍fs之间
#define   CURRENT_ID_KP                          0.16
#define   CURRENT_ID_KI                          0.0172917
#define   CURRENT_ID_KC                          0   
#define   CURRENT_ID_KD                          0
#define   CURRENT_PI_ID_OUT_MAX                  700     
#define   CURRENT_PI_ID_OUT_MIN                 -700     

#define   CURRENT_IQ_KP                          CURRENT_ID_KP
#define   CURRENT_IQ_KI                          CURRENT_ID_KI
#define   CURRENT_IQ_KC                          0 
#define   CURRENT_IQ_KD                          0
#define   CURRENT_PI_IQ_OUT_MAX                  700
#define   CURRENT_PI_IQ_OUT_MIN                 -700
#define    CURRENT_LIMIT_MAX           1500
#define   GSI_V_LOOP_BANDWITH         CURRENT_LOOP_BANDWITH/10   //Hz
/* Preserve the legacy effective voltage-loop gains while keeping the
 * grid-side loop independent from the machine-side DC loop symbols. */
#ifndef GSI_V_LOOP_KP
#define   GSI_V_LOOP_KP               1.1309733
#endif
#ifndef GSI_V_LOOP_KI
#define   GSI_V_LOOP_KI               0.0282743
#endif
#define   GSI_V_LOOP_KC               0.00001
#define   GSI_V_LOOP_KD               0.000001
#define   GSI_V_LOOP_OUT_MAX          CURRENT_LIMIT_MAX    //输出为电流
#define   GSI_V_LOOP_OUT_MIN         -CURRENT_LIMIT_MAX    //输出为电流
#define   GSI_WREF_MAX                        450
#define   GSI_WREF_MIN                        200
#define   GSI_PLOOP_OUT_MAX                   5
#define   GSI_PLOOP_OUT_MIN                  -5
#ifndef GSI_PLOOP_KP
#define   GSI_PLOOP_KP                        1e-6
#endif
#ifndef GSI_PLOOP_KI
#define   GSI_PLOOP_KI                        2e-5
#endif
#ifndef GSI_PREF_RAMP_SLOPE
#define   GSI_PREF_RAMP_SLOPE                 5000000.0f
#endif
#ifndef GSI_PF_LOOP_SIGN
/* For inverter power export: Pref-Pmeas > 0 should advance the internal
 * angle, so the legacy P-f loop uses a positive frequency correction. */
#define   GSI_PF_LOOP_SIGN                    1.0f
#endif

                                                                   
typedef     struct     {float	    Ts;
                        float	    In;                                                                 				       
                        float	    Slope;
                        float	    Init;   				                                                       
                        float       Out;                                                               
                       }VOLTAGE_SLOPE_LIMIT;
                         								                                 
#define VOLTAGE_SLOPE_LIMIT_DEFAULTS {0.0,\
				                    0.0,\
								    V_REF_SLOPE_LIMIT_MAX,\
								    0.0,\
									0.0,\
								   }	                                                                               
//**********************************************************
#define CURRENT_PI_ID_DEFAULTS    {0.0,0.0,  0.0, \
		                         CURRENT_ID_KP,  CURRENT_ID_KI,  CURRENT_ID_KC, CURRENT_ID_KD,\
		                         0.0, 0.0,0.0, 0.0, 0.0, 0.0, \
		                         CURRENT_PI_ID_OUT_MAX,  CURRENT_PI_ID_OUT_MIN, \
		                         0.0,0.0}
//**********************************************************
#define CURRENT_PI_IQ_DEFAULTS    {0.0, 0.0, 0.0, \
                                 CURRENT_IQ_KP, CURRENT_IQ_KI, CURRENT_IQ_KC,CURRENT_IQ_KD,\
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \
                                 CURRENT_PI_IQ_OUT_MAX, CURRENT_PI_IQ_OUT_MIN, \
                                 0.0,0.0}
//**********************************************************
#define VOLTAGE_LOOP_PI_DEFAULTS   {0.0, 0.0, 0.0, \
                             GSI_V_LOOP_KP, GSI_V_LOOP_KI,0, 0,\
                             0.0, 0.0, 0.0, 0.0,  \
                             0.0,0.0,CURRENT_LIMIT_MAX,  -CURRENT_LIMIT_MAX, 0.0, 0.0}
                                 //**********************************************************
#define PLL_LOOP_PI_DEFAULTS   {0.0, 0.0, 0.0, \
                               1, 0.001, 0.0, 0,\
                               0, 0, 0.0, 0.0, \
                               0.0, 0.0,  400, -400,0.0, 0.0}

 #define E_VOLTAGE_LOOP_PI_DEFAULTS   {0.0, 0.0, 0.0, \
                               0, 5.86, 0.0, 0,\
                               0, 0, 0.0, 0.0, \
                               0.0, 0.0,  1.15, 0,0.0, 0.0}    
 #define POWER_LOOP_PI_DEFAULTS   {0.0, 0.0, 0.0, GSI_PLOOP_KP, GSI_PLOOP_KI, 0.0, 0,\
                               0, 0, 0.0, 0.0, \
                               0.0, 0.0,  GSI_PLOOP_OUT_MAX, GSI_PLOOP_OUT_MIN,0.0, 0.0}    
//##########################################################################################################
//##########################################################################################################
struct  GRID_INV_PARAM_INIT       {float                  grid_filter_Ls       ; //参数 
							  	   float                  grid_filter_C        ; //参数 
					              };
#define  GRID_INV_PARAM_INIT_DEFAULTS  { GRID_FILTER__LS, GRID_UDC__C }
//##########################################################################################################
//##########################################################################################################
//****************电机给定***************************************************
struct   GSI_REF               {  float                P_active_power_ref                 ;  
                                float                  Q_reactive_power_ref              ;  //单位 N
								float                  voltage_ref                  ;  //单位 A
                                };

#define  GSI_REF_DEFAULTS         {0, 0,0 }

//##########################################################################################################
//##########################################################################################################
//****************电机反馈***************************************************
struct  GSI_BACK             {  float                  Ia1                 ; 	
							  	float                  Ib1                 ; 
                                float                  Ic1                 ; 
                                float                  pcc_Ia                 ; 	
							  	float                  pcc_Ib                 ; 
                                float                  pcc_Ic                 ; 
                                float                  pcc_uab             ; 	
							  	float                  pcc_ubc             ; 
                                float                  pcc_uca             ; 
                                float                  I_alfa               ; 
                                float                  I_beta              ;
                                float                  Is_Amplitude		   ; 
                                float                  Is_Phase		       ;                               								  	
							  	float                  RotorPos            ; 
								float                  CosPos              ; 
								float                  SinPos              ;
							  	float                  We                  ; 
							  	float                  Udc1                ; 
							   };
#define  GSI_BACK_DEFAULTS   {0.0}
//##########################################################################################################
//##########################################################################################################
//****************电机输出**************************************************
struct  GSI_OUT              {
                                float                  Us_alfa             ; //输出 	
						  		float                  Us_beta             ; //输出 
						    	float                  Us_Amplitude        ; //输出
						    	float                  Us_Phase            ; //输出
						    	unsigned short         MotorRunState       ; //输出
							   };
#define  GSI_OUT_DEFAULTS    {0.0,  0.0,  0.0,  0.0, 0}

//##########################################################################################################
//##########################################################################################################
//****************电机模型中间变量*******************************************
struct  GSI_MIDDLE_VARIABLE  {
                                int                    Pre_syn             ;                            
                                float                  u_a                 ;   
                                float                  u_b                 ;
                                float                  u_c                 ;
                                float                  u_alpha             ;
                                float                  u_beta              ;
                                float                  pcc_u_d             ;
                                float                  pcc_u_q             ;
                                float                  Id                  ; 	
						    	float                  Iq                  ; 
                                float                  pcc_Id              ; 	
						    	float                  pcc_Iq              ; 
                                float                  pcc_Q_reactive_Power; 	
						    	float                  pcc_P_active_Power  ; 
                                float                  pcc_Q_reactive_Power_filter; 	
						    	float                  pcc_P_active_Power_filter  ; 
                                float                  grid_phase_angle    ;
                                float                  Id_ref              ; 
						    	float                  Iq_ref              ;                              
								float                  Ud_fwd              ; 
								float                  Uq_fwd              ; 						
								float                  Ud1_ref             ; 
								float                  Uq1_ref			   ;	
                                float                  freq                ;
                                 int                   PLL_Flag;
					            };

#define GSI_MIDDLE_VARIABLE_DEFAULTS {0.0}
                                
                                
                                
struct P_F_CONTROL{             float                  J_virtual           ;//虚拟转子转动惯量    初始值取10
                                float                  Damping_coeff       ; //阻尼系数     初始值取2000    
                                float                  w_ref               ;//生成角频率参考值
                                float                  we_set              ;//设定的系统运行额定角频率  
                                float                  kp_voltage          ;    //0.01
                                float                  ki_voltage          ;    //0.2
                                float                  thet_ref            ;
                                float                  U_od_ref            ;
                                float                  U_oq_ref            ;
                                float                  E_ua                ;
                                float                  E_ub                ;
                                float                  E_uc                ;
                                float                  virtual_flux        ;
                                float                  E_voltage_amplitude ;
                     };
#define P_F_CONTROL_DEFAULTS   {10, 2000, 0, 50.0*MOTOR_2PI_RADIAN, \
                                0.01, 0.2, 0.05, 0.05}


//##########################################################################################################
//##########################################################################################################
typedef     struct         {float                           Ts ;                              
                            struct  GRID_INV_PARAM_INIT     par  ;
							struct  GSI_REF                 ref  ;	
							struct  GSI_BACK                bak  ;
							struct  GSI_OUT                 out  ;
							struct  GSI_MIDDLE_VARIABLE     val  ;
								    VOLTAGE_SLOPE_LIMIT 	voltage_slope_limit;
                            struct  P_F_CONTROL     pf;
							}GRID_SIDE_INV;	

#define GRID_SIDE_INV_DEFAULTS     {0.0001,\
                                    GRID_INV_PARAM_INIT_DEFAULTS,\
                                    GSI_REF_DEFAULTS, \
                                    GSI_BACK_DEFAULTS,\
                                    GSI_OUT_DEFAULTS, \
                                    GSI_MIDDLE_VARIABLE_DEFAULTS,\
                                    VOLTAGE_SLOPE_LIMIT_DEFAULTS,\
                                            P_F_CONTROL_DEFAULTS}

typedef struct CLACK_DEF{       float  alpha;
                                float  beta;
                                float  a;
                                float  b;
                                float  c;
}CLACK;
typedef struct PARK_DEF{     float  ualpha;
                             float  ubeta;
                             float  ud;
                             float  uq;
                             float  thet;
}PARK;

extern GRID_SIDE_INV  grid_side;
extern PARK  park_u;
extern CLACK clack_trans;

//****************************一阶低通滤波器************************************************************
//传递函数y=w/(w+s)  w为截止角频率，w=2*pi*f,f为截止频率
 typedef struct  MOTOR_LOW_PASS_FILTER_DEF { 
                                             float        w_cutoff;
                                             float        fs_cutoff;
                                             float        Ts_frequcncy;
                                             float        out;                     //输出信号
                                             float        Ui;                     //输入信号
                                             float        Ui_n_1;                 //上一次输入信号
                                           }  MOTOR_LOW_PASS_FILTER ; 
extern MOTOR_LOW_PASS_FILTER  lpf;

typedef struct  MOTOR_HIGH_PASS_FILTER_DEF { 
                                             float        w_cutoff;
                                             float        fs_cutoff;
                                             float        Ts_frequcncy;
                                             float        out;                     //输出信号
                                              float       Uo_n_1;                 //输出信号
                                             float        Ui;                     //输入信号
                                             float        Ui_n_1;                 //上一次输入信号
                                           }  MOTOR_HIGH_PASS_FILTER ; 
extern MOTOR_HIGH_PASS_FILTER  hpf;     

typedef struct  MOTOR_BAND_PASS_FILTER_DEF { 
                                             float        w_cutoff;
                                             float        fs_cutoff;
                                             float        k_gain;
                                             float        Ts_frequcncy;
                                             float        Uo;                     //输出信号
                                             float        Uo_n_1;                 //输出信号
                                             float        Uo_n_2;                 //输出信号
                                             float        Ui;                     //输入信号
                                             float        Ui_n_1;                 //上一次输入信号
                                             float        Ui_n_2;                 //上一次输入信号
                                           }  MOTOR_BAND_PASS_FILTER ; 
 extern MOTOR_BAND_PASS_FILTER  bandpf;  

void grid_side_control(GRID_SIDE_INV *p);
void grid_side_reset(void);

#endif






