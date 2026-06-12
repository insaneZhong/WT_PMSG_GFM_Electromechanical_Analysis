#ifndef   __MOTOR_CONT__
#define   __MOTOR_CONT__

#define   MOTOR_RS                       0.0122      
#ifndef MOTOR_LD
#define   MOTOR_LD                       0.00102
#endif
#ifndef MOTOR_LQ
#define   MOTOR_LQ                       0.00102
#endif
#define   MOTOR_FM_25_TEMPERATURE        8.64
#define   MOTOR_POLE_PAIR                20          
#define   MOTOR_JM                       183750  
#define   MOTOR_BW                       0.001
#define   System_frequcncy               0.00025

#ifndef GRID_UDC__C
#define   GRID_UDC__C                           0.03
#endif

#define   MOTOR_ID_LIMIT_MAX                         1500     
#define   MOTOR_IQ_LIMIT_MAX                         1500      
#define   MOTOR_SPEED_PROTECT                        100     

#define   MOTOR_ID_REF_SLOPE_LIMIT_MAX               1000      
#ifndef MOTOR_IQ_REF_SLOPE_LIMIT_MAX
#define   MOTOR_IQ_REF_SLOPE_LIMIT_MAX               1000
#endif
#define   MOTOR_CLOSE_SPEED_REF_SLOPE_LIMIT_MAX      100      

#ifndef CURRENT_LOOP_BANDWITH_ID
#define   CURRENT_LOOP_BANDWITH_ID             220    // mapped from small-signal model (scheme A)
#endif
#ifndef MOTOR_ID_KP
#define   MOTOR_ID_KP                          1.4
#endif
#ifndef MOTOR_ID_KI
#define   MOTOR_ID_KI                          0.00290476
#endif
#ifndef MOTOR_ID_KC
#define   MOTOR_ID_KC                          0.0001
#endif
#ifndef MOTOR_ID_KD
#define   MOTOR_ID_KD                          0.000001
#endif
#ifndef MOTOR_PI_ID_OUT_MAX
#define   MOTOR_PI_ID_OUT_MAX                  700
#endif
#ifndef MOTOR_PI_ID_OUT_MIN
#define   MOTOR_PI_ID_OUT_MIN                 -700
#endif

#ifndef MOTOR_IQ_KP
#define   MOTOR_IQ_KP                          1.4
#endif
#ifndef MOTOR_IQ_KI
#define   MOTOR_IQ_KI                          0.00290476
#endif
#ifndef MOTOR_IQ_KC
#define   MOTOR_IQ_KC                          0.0001
#endif
#ifndef MOTOR_IQ_KD
#define   MOTOR_IQ_KD                          0.000001
#endif
#ifndef MOTOR_PI_IQ_OUT_MAX
#define   MOTOR_PI_IQ_OUT_MAX                  700
#endif
#ifndef MOTOR_PI_IQ_OUT_MIN
#define   MOTOR_PI_IQ_OUT_MIN                 -700
#endif
#ifndef MOTOR_IQ_PI_OUTPUT_SIGN
#define   MOTOR_IQ_PI_OUTPUT_SIGN              1
#endif
#ifndef MOTOR_IQ_FEEDBACK_SIGN
#define   MOTOR_IQ_FEEDBACK_SIGN               1
#endif

#define   SPEED_LOOP_BANDWITH                  12   //Hz, reduced for no-disturbance steady-state convergence
#ifndef MOTOR_PWM_SPEED_KP
#define   MOTOR_PWM_SPEED_KP                  0.050
#endif
#ifndef MOTOR_PWM_SPEED_KI
#define   MOTOR_PWM_SPEED_KI                  0.00035
#endif
#ifndef MOTOR_PWM_SPEED_KC
#define	  MOTOR_PWM_SPEED_KC                   0.00001
#endif
#define	  MOTOR_PWM_SPEED_KD                   0.000001
#define   MOTOR_PWM_SPEED_PI_OUT_MAX           MOTOR_IQ_LIMIT_MAX    //输出为电流,
#define   MOTOR_PWM_SPEED_PI_OUT_MIN          -MOTOR_IQ_LIMIT_MAX    //输出为电流,
/* MSC-DVC structure selector.
 * Type a: DC-voltage feedback only, Iq_ref = -PI(Udc_ref - Udc).
 * Type c: Type a plus active-power feedforward,
 *         Iq_ref = -Kff*Pref - PI(Udc_ref - Udc).
 */
#define   MOTOR_MSC_DVC_TYPE_A                 1
#define   MOTOR_MSC_DVC_TYPE_C                 3
#ifndef MOTOR_MSC_DVC_TYPE
#define   MOTOR_MSC_DVC_TYPE                   MOTOR_MSC_DVC_TYPE_C
#endif

/* Type-c feedforward gain. It is ignored when MOTOR_MSC_DVC_TYPE is Type a. */
#ifndef MOTOR_IQ_POWER_FF_A_PER_W
#define   MOTOR_IQ_POWER_FF_A_PER_W           0.000225
#endif


#define   V_LOOP_BANDWITH                     12   // Hz, MSC DC-link loop
#define	  V_LOOP_KP                   GRID_UDC__C * V_LOOP_BANDWITH * 2 *3.1415926
#define   V_LOOP_KI                    V_LOOP_KP/ 0.01 *0.00025

#define   MOTOR_1PI6_RADIAN  		           0.5235988    //弧度  30°
#define   MOTOR_1PI3_RADIAN  		           1.0471976    //弧度  60°
#define   MOTOR_1PI2_RADIAN  		           1.5707963    //弧度  90°
#define   MOTOR_2PI3_RADIAN  		           2.0943951    //弧度  120°
#define   MOTOR_5PI6_RADIAN  		           2.6179939    //弧度  150°
#define   MOTOR_PI_RADIAN  		               3.1415927    //弧度  180°
#define   MOTOR_7PI6_RADIAN  		           3.6651914    //弧度  210°
#define   MOTOR_4PI3_RADIAN  		           4.1887902    //弧度  240°
#define   MOTOR_3PI2_RADIAN  		           4.7123890    //弧度  270°
#define   MOTOR_5PI3_RADIAN  		           5.2359878    //弧度  300°
#define   MOTOR_11PI6_RADIAN  		           5.7595865    //弧度  330°
#define   MOTOR_2PI_RADIAN  		           6.2831853    //弧度  360°
#define   MOTOR_4PI_RADIAN  		           12.566371    //弧度  720°
#define   MOTOR_SQRT3                          1.73205081   //根号3
#define   MOTOR_1SQRT3                         0.57735027   //根号3分之1
#define   MOTOR_1SQRT2                         0.70710678   //根号2分之1
#define   MOTOR_TWO_DIV_PI                     0.63661980   //PI分之2

                                                                   
typedef     struct     {float	    Ts;
                        float	    In;                                                                 				       
                        float	    Slope;
                        float	    Init;   				                                                       
                        float       Out; 
                        void (*cale)(); 
                        void (*reset)();                                                               
                       }MOTOR_SLOPE_LIMIT;

#define MOTOR_ID_SLOPE_LIMIT_DEFAULTS {0.0,\
									0.0,\
								    MOTOR_ID_REF_SLOPE_LIMIT_MAX,\
									0.0,\
								    0.0,\
								    (void (*)(int))motor_slope_limit_calc,\
								    (void (*)(int))motor_slope_limit_reset}
								          
#define MOTOR_IQ_SLOPE_LIMIT_DEFAULTS {0.0,\
				                    0.0,\
								    MOTOR_IQ_REF_SLOPE_LIMIT_MAX,\
								    0.0,\
									0.0,\
								    (void (*)(int))motor_slope_limit_calc,\
								    (void (*)(int))motor_slope_limit_reset}                                  
								                                 
#define MOTOR_SPEED_SLOPE_LIMIT_DEFAULTS {0.0,\
				                    0.0,\
								    MOTOR_CLOSE_SPEED_REF_SLOPE_LIMIT_MAX,\
								    0.0,\
									0.0,\
								    (void (*)(int))motor_slope_limit_calc,\
								    (void (*)(int))motor_slope_limit_reset}	                                                                         
void motor_slope_limit_calc(MOTOR_SLOPE_LIMIT *v);
void motor_slope_limit_reset(MOTOR_SLOPE_LIMIT *v);

typedef struct   {float  Ts;   	        /* Input: Reference current     */
                  float  Ref;   	    /* Input: Reference current     */
				  float  Fdb;   	    /* Input: Feedback current      */
								  
		          float  Kp;        	/* Parameter: Proportional gain */				
		          float  Ki;			/* Parameter: Integral gain     */
		          float  Kc;			/* Parameter: Integral correction gain  */
		          float  Kd;			/* Parameter: Integral correction gain  */
				  			
				  float  Ui;	        /* Variable: Integral output    */
				  float  Up;	        /* Variable: Integral output    */	
				  float  Up_old;	    /* Variable: Integral output    */	
				  float  Ud;	        /* Variable: Integral output    */	
				  			
				  float  SatErr;        /* Variable: Integral output    */
				  float  Error;         /* Variable: Integral output    */
				  float  OutMax;		/* Parameter: Maximum output    */
				  float  OutMin;		/* Parameter: Minimum output    */
				  float  OutPreSat;     /* Output: PI output            */
				  float  Out;         	/* Output: PI output            */
		 	 	  void  (*calc1)();	  
		 	 	  void  (*calc2)();	  	
				  void  (*reset)();	
				 }MOTOR_PI;	            
//**********************************************************
#define MOTOR_PI_ID_DEFAULTS    {0.0,\
                                 0.0, \
								 0.0, \
		                         MOTOR_ID_KP, \
		                         MOTOR_ID_KI, \
		                         MOTOR_ID_KC, \
								 MOTOR_ID_KD,\
		                         0.0, \
		                         0.0, \
		                         0.0, \
		                         0.0, \
		                         0.0, \
		                         0.0, \
		                         MOTOR_PI_ID_OUT_MAX, \
		                         MOTOR_PI_ID_OUT_MIN, \
		                         0.0, \
		                         0.0, \
								 (void (*)(int))motor_PI1_calc,\
								 (void (*)(int))motor_PI2_calc,\
								 (void (*)(int))motor_PI_reset}
//**********************************************************
#define MOTOR_PI_IQ_DEFAULTS    {0.0, \
                                 0.0, \
                                 0.0, \
                                 MOTOR_IQ_KP, \
                                 MOTOR_IQ_KI, \
                                 MOTOR_IQ_KC, \
                                 MOTOR_IQ_KD,\
                                 0.0, \
                                 0.0, \
                                 0.0, \
                                 0.0, \
                                 0.0, \
                                 0.0, \
                                 MOTOR_PI_IQ_OUT_MAX, \
                                 MOTOR_PI_IQ_OUT_MIN, \
                                 0.0, \
                                 0.0, \
                                 (void (*)(int))motor_PI1_calc,\
                                 (void (*)(int))motor_PI2_calc,\
                                 (void (*)(int))motor_PI_reset}
//**********************************************************
#define MOTOR_PI_PWM_SPEED_DEFAULTS   {0.0, \
				                 0.0, \
								 0.0, \
	                             MOTOR_PWM_SPEED_KP,\
	                             MOTOR_PWM_SPEED_KI,\
	                             MOTOR_PWM_SPEED_KC,\
								 MOTOR_PWM_SPEED_KD,\
		                         0.0, \
		                         0.0, \
		                         0.0, \
		                         0.0, \
		                         0.0, \
								 0.0, \
	                             MOTOR_PWM_SPEED_PI_OUT_MAX, \
	                             MOTOR_PWM_SPEED_PI_OUT_MIN, \
	                             0.0, \
	                             0.0, \
		                         (void (*)(int))motor_PI1_calc,\
								 (void (*)(int))motor_PI2_calc,\
								 (void (*)(int))motor_PI_reset}

void motor_PI1_calc(MOTOR_PI *v);
void motor_PI2_calc(MOTOR_PI *v);
void motor_PI_reset(MOTOR_PI *v);


//##########################################################################################################
//##########################################################################################################
struct  MOTOR_PARAM_INIT       {short                  Polar                 ; //参数 
							  	float                  Rs                    ; //参数 
							  	float                  Fm                    ; //参数
							  	float                  Ld                    ; //参数
							  	float                  Lq                    ; //参数
                                float                  Lls                   ;//定子漏感
					            float                  Jm                    ; //电机机械动惯量
					            };

#define  MOTOR_PARAM_INIT_DEFAULTS  {MOTOR_POLE_PAIR,\
                                     MOTOR_RS,\
                                     MOTOR_FM_25_TEMPERATURE,\
                                     MOTOR_LD,\
                                     MOTOR_LQ,\
                                     MOTOR_JM}
//##########################################################################################################
//##########################################################################################################
//****************电机给定***************************************************
struct   MOTOR_REF             {float  voltage_ref;
                                float  active_power_ref;
                                unsigned short dvc_enable;
							   };

#define  MOTOR_REF_DEFAULTS         {0, 0, 0}

//##########################################################################################################
//##########################################################################################################
//****************电机反馈***************************************************
struct  MOTOR_BACK             {float                  Ia1                  ; 	
							  	float                  Ib1                  ; 
                                float                  Ic1                  ; 	
								float                  Id1                  ; 	
						    	float                  Iq1                  ; 
                                float                  I_alfa             ; 
                                float                  I_beta             ; 
                                float                  Is_Amplitude		   ; 
                                float                  Is_Phase		       ;                               								  	
							  	float                  RotorPos            ; 
								float                  CosPos              ; 
								float                  SinPos              ;
							  	float                  We                  ; 
							  	float                  Udc1                 ; 
							   };
#define  MOTOR_BACK_DEFAULTS   {0.0,  0.0,  0.0,  0.0,  0.0,\
                                0.0,  0.0,  0.0,  0.0,  0.0,\
								0.0,  0.0,  0.0,  0.0}
//##########################################################################################################
//##########################################################################################################
//****************电机输出**************************************************
struct  MOTOR_OUT              {float                  Us_alfa             ; //输出 	
						  		float                  Us_beta             ; //输出 
                                float                  Us_a1               ; //输出 	
						  		float                  Us_b1               ; //输出 
                                float                  Us_c1               ; //输出 	
						    	float                  Us_Amplitude        ; //输出
						    	float                  Us_Phase            ; //输出
						    	unsigned short         MotorRunState       ; //输出
							   };
#define  MOTOR_OUT_DEFAULTS    {0.0,  0.0,  0.0,  0.0, 0.0,\
                                         0.0, 0.0, 0}

//##########################################################################################################
//##########################################################################################################
//****************电机模型中间变量*******************************************
struct  MOTOR_MIDDLE_VARIABLE  {float                  Id_mpta             ; 	
						    	float                  Iq_mpta             ; 
                                float                  Id_ref              ; 
						    	float                  Iq_ref              ; 
                                
								float                  Ud_fwd              ; 
								float                  Uq_fwd              ; 
								float                  Ud_pi               ; 
								float                  Uq_pi               ; 
                                
								float                  Ud1_ref              ; 
								float                  Uq1_ref			   ;
							    float                  Us_Amplitude        ; //中间变量  
								float                  Us_Phase            ; //中间变量	
                                float                  We_ref              ;
                                float                  RotorPos            ;
					            };

#define MOTOR_MIDDLE_VARIABLE_DEFAULTS {0.0,  0.0,  0.0,  0.0, \
                                        0.0,  0.0,  0.0,  0.0, \
                                        0.0,  0.0,  0.0,  0.0,  0.0,0.0}

//##########################################################################################################
//##########################################################################################################
//##########################################################################################################
//##########################################################################################################
//                   电机对象结构体
//##########################################################################################################
//##########################################################################################################
typedef     struct         {float                           Ts ;                              
						    unsigned short                  TaskIndex; 
						    unsigned short                  CalcNumbPerPwmSwitch;

                            struct  MOTOR_PARAM_INIT        par  ;
							struct  MOTOR_REF               ref  ;	
							struct  MOTOR_BACK              bak  ;
							struct  MOTOR_OUT               out  ;
							struct  MOTOR_MIDDLE_VARIABLE   val  ;
							        MOTOR_PI		        id_pi;
							        MOTOR_PI			    iq_pi;                                
							        MOTOR_PI			    pwm_speed_pi;
								    MOTOR_SLOPE_LIMIT       id_slope_limit;
								    MOTOR_SLOPE_LIMIT       iq_slope_limit;
								    MOTOR_SLOPE_LIMIT 	    speed_slope_limit;
							void   (*init)();
							void   (*control)();
							void   (*reset)();
							}MOTOR;	

#define MOTOR_DEFAULTS     {0.0,\
                            0,\
							1,\
							MOTOR_PARAM_INIT_DEFAULTS,\
							MOTOR_REF_DEFAULTS, \
							MOTOR_BACK_DEFAULTS,\
							MOTOR_OUT_DEFAULTS, \
							MOTOR_MIDDLE_VARIABLE_DEFAULTS,\
							MOTOR_PI_ID_DEFAULTS,\
							MOTOR_PI_IQ_DEFAULTS,\
							MOTOR_PI_PWM_SPEED_DEFAULTS,\
							MOTOR_ID_SLOPE_LIMIT_DEFAULTS,\
							MOTOR_IQ_SLOPE_LIMIT_DEFAULTS,\
                            MOTOR_SPEED_SLOPE_LIMIT_DEFAULTS,\
                            (void (*)(int))motor_init,\
							(void (*)(int))motor_control,\
							(void (*)(int))motor_reset}

//##########################################################################################################
//##########################################################################################################
//**********电机函数声明****************************************
void    motor_init(MOTOR *p);
void    motor_control(MOTOR *p);
void    motor_reset(MOTOR *p);

 extern MOTOR_SLOPE_LIMIT  vloop_slope;

#endif

//##########################################################################################################
//##########################################################################################################
//##########################################################################################################

