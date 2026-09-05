#ifndef   __MOTOR_CONT__
#define   __MOTOR_CONT__

#ifndef MOTOR_RS
#define   MOTOR_RS                       0.0122
#endif
#ifndef MOTOR_LD
#define   MOTOR_LD                       0.00102
#endif
#ifndef MOTOR_LQ
#define   MOTOR_LQ                       0.00102
#endif
#ifndef MOTOR_FM_25_TEMPERATURE
#define   MOTOR_FM_25_TEMPERATURE        8.64
#endif
#ifndef MOTOR_POLE_PAIR
#define   MOTOR_POLE_PAIR                20
#endif
#ifndef MOTOR_JM
#define   MOTOR_JM                       183750
#endif
#define   MOTOR_BW                       0.001
#define   System_frequcncy               0.00025

#ifndef GRID_UDC__C
#define   GRID_UDC__C                           0.03
#endif

#ifndef MOTOR_ID_LIMIT_MAX
#define   MOTOR_ID_LIMIT_MAX                         1500
#endif
#ifndef MOTOR_IQ_LIMIT_MAX
#define   MOTOR_IQ_LIMIT_MAX                         1500
#endif
#define   MOTOR_SPEED_PROTECT                        100     

#ifndef MOTOR_ID_REF_SLOPE_LIMIT_MAX
#define   MOTOR_ID_REF_SLOPE_LIMIT_MAX               1000
#endif
#ifndef MOTOR_IQ_REF_SLOPE_LIMIT_MAX
#define   MOTOR_IQ_REF_SLOPE_LIMIT_MAX               1000
#endif
#define   MOTOR_CLOSE_SPEED_REF_SLOPE_LIMIT_MAX      100      

#define   CURRENT_LOOP_BANDWITH_ID             220    // mapped from small-signal model (scheme A)
#ifndef MOTOR_ID_KP
#define   MOTOR_ID_KP                          1.4
#endif
#ifndef MOTOR_ID_KI
#define   MOTOR_ID_KI                          0.00290476
#endif
#define   MOTOR_ID_KC                          0.0001   
#define   MOTOR_ID_KD                          0.000001
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
#define   MOTOR_IQ_KC                          0.0001 
#define   MOTOR_IQ_KD                          0.000001
#ifndef MOTOR_PI_IQ_OUT_MAX
#define   MOTOR_PI_IQ_OUT_MAX                  700
#endif
#ifndef MOTOR_PI_IQ_OUT_MIN
#define   MOTOR_PI_IQ_OUT_MIN                 -700
#endif

#define   SPEED_LOOP_BANDWITH                  12   //Hz, reduced for no-disturbance steady-state convergence
#ifndef MOTOR_PWM_SPEED_KP
#define   MOTOR_PWM_SPEED_KP                   0.05
#endif
#ifndef MOTOR_PWM_SPEED_KI
#define   MOTOR_PWM_SPEED_KI                   0.0005
#endif
#ifndef MOTOR_PWM_SPEED_KC
#define	  MOTOR_PWM_SPEED_KC                   0.00001
#endif
#ifndef MOTOR_PWM_SPEED_KD
#define	  MOTOR_PWM_SPEED_KD                   0.000001
#endif
#ifndef MSC_DVC_ENABLE_TIME_S
#define   MSC_DVC_ENABLE_TIME_S                2.75
#endif
#ifndef MSC_DVC_NONNEGATIVE_INTEGRAL
#define   MSC_DVC_NONNEGATIVE_INTEGRAL         0
#endif
#ifndef MSC_CURRENT_PI_FREEZE_UNTIL_DVC
#define   MSC_CURRENT_PI_FREEZE_UNTIL_DVC      0
#endif
#define   MOTOR_PWM_SPEED_PI_OUT_MAX           MOTOR_IQ_LIMIT_MAX    //杈撳嚭涓虹數娴?
#define   MOTOR_PWM_SPEED_PI_OUT_MIN          -MOTOR_IQ_LIMIT_MAX    //杈撳嚭涓虹數娴?


#define   V_LOOP_BANDWITH                     5   // Hz, MSC DC-link loop
#define	  V_LOOP_KP                   GRID_UDC__C * V_LOOP_BANDWITH * 2 *3.1415926
#define   V_LOOP_KI                    V_LOOP_KP/ 0.01 *0.00025

#define   MOTOR_1PI6_RADIAN  		           0.5235988    //寮у害  30掳
#define   MOTOR_1PI3_RADIAN  		           1.0471976    //寮у害  60掳
#define   MOTOR_1PI2_RADIAN  		           1.5707963    //寮у害  90掳
#define   MOTOR_2PI3_RADIAN  		           2.0943951    //寮у害  120掳
#define   MOTOR_5PI6_RADIAN  		           2.6179939    //寮у害  150掳
#define   MOTOR_PI_RADIAN  		               3.1415927    //寮у害  180掳
#define   MOTOR_7PI6_RADIAN  		           3.6651914    //寮у害  210掳
#define   MOTOR_4PI3_RADIAN  		           4.1887902    //寮у害  240掳
#define   MOTOR_3PI2_RADIAN  		           4.7123890    //寮у害  270掳
#define   MOTOR_5PI3_RADIAN  		           5.2359878    //寮у害  300掳
#define   MOTOR_11PI6_RADIAN  		           5.7595865    //寮у害  330掳
#define   MOTOR_2PI_RADIAN  		           6.2831853    //寮у害  360掳
#define   MOTOR_4PI_RADIAN  		           12.566371    //寮у害  720掳
#define   MOTOR_SQRT3                          1.73205081   //鏍瑰彿3
#define   MOTOR_1SQRT3                         0.57735027   //鏍瑰彿3鍒嗕箣1
#define   MOTOR_1SQRT2                         0.70710678   //鏍瑰彿2鍒嗕箣1
#define   MOTOR_TWO_DIV_PI                     0.63661980   //PI鍒嗕箣2

                                                                   
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
struct  MOTOR_PARAM_INIT       {short                  Polar                 ; //鍙傛暟 
							  	float                  Rs                    ; //鍙傛暟 
							  	float                  Fm                    ; //鍙傛暟
							  	float                  Ld                    ; //鍙傛暟
							  	float                  Lq                    ; //鍙傛暟
                                float                  Lls                   ;//瀹氬瓙婕忔劅
					            float                  Jm                    ; //鐢垫満鏈烘鍔ㄦ儻閲?
					            };

#define  MOTOR_PARAM_INIT_DEFAULTS  {MOTOR_POLE_PAIR,\
                                     MOTOR_RS,\
                                     MOTOR_FM_25_TEMPERATURE,\
                                     MOTOR_LD,\
                                     MOTOR_LQ,\
                                     MOTOR_JM}
//##########################################################################################################
//##########################################################################################################
//****************鐢垫満缁欏畾***************************************************
struct   MOTOR_REF             {float  voltage_ref;						  							  	
							   };

#define  MOTOR_REF_DEFAULTS         {0}

//##########################################################################################################
//##########################################################################################################
//****************鐢垫満鍙嶉***************************************************
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
//****************鐢垫満杈撳嚭**************************************************
struct  MOTOR_OUT              {float                  Us_alfa             ; //杈撳嚭 	
						  		float                  Us_beta             ; //杈撳嚭 
                                float                  Us_a1               ; //杈撳嚭 	
						  		float                  Us_b1               ; //杈撳嚭 
                                float                  Us_c1               ; //杈撳嚭 	
						    	float                  Us_Amplitude        ; //杈撳嚭
						    	float                  Us_Phase            ; //杈撳嚭
						    	unsigned short         MotorRunState       ; //杈撳嚭
							   };
#define  MOTOR_OUT_DEFAULTS    {0.0,  0.0,  0.0,  0.0, 0.0,\
                                         0.0, 0.0, 0}

//##########################################################################################################
//##########################################################################################################
//****************鐢垫満妯″瀷涓棿鍙橀噺*******************************************
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
							    float                  Us_Amplitude        ; //涓棿鍙橀噺  
								float                  Us_Phase            ; //涓棿鍙橀噺	
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
//                   鐢垫満瀵硅薄缁撴瀯浣?
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
//**********鐢垫満鍑芥暟澹版槑****************************************
void    motor_init(MOTOR *p);
void    motor_control(MOTOR *p);
void    motor_reset(MOTOR *p);

 extern MOTOR_SLOPE_LIMIT  vloop_slope;

#endif

//##########################################################################################################
//##########################################################################################################
//##########################################################################################################

