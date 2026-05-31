//##########################################################################################################
//##########################################################################################################
//##########################################################################################################
//##########################################################################################################
//#           FilerName:   调制模块头文件                                                                  #
//#            Designer:   廖武                                                                        #
//#         Description:   
//#             history:   2020-3-7  V1.0                                                                 #
//##########################################################################################################
//##########################################################################################################
#ifndef     __SVPWM_H__
#define     __SVPWM_H__

//##########################################################################################################
#define   PWM_ASYNCH_SWITCH_FREQUENCE        2000         // 单位：Hz  异步调制开关频率3500Hz
#define   PWM_FPGA_COUNT_PER_S               1e6  // 单位：次数  1s钟对应的FPGA计数个数
#define   PWM_CONVERT_PLUSE_MIN_TIME         1e-6        // 单位：秒 逆变器Ua电压最窄脉冲时间
#define   PWM_CONVERT_DEAD_TIME              1.6e-6      // 单位：秒 逆变器死区时间16微秒  
#define   PWM_CONVERT_PLUSE_DELAY_TIME       4e-6        // 单位：秒 逆变器脉冲延时时间

//********* 脉冲状态************************
#define     	U0 				0xFFFF//0x0008//
#define 		U1 				0xEEEE//0x0009//
#define 		U2 				0xCCCC//0x000b//
#define 		U3 				0xDDDD//0x000a//
#define 		U4 				0x9999//0x000e//
#define 		U5 				0xBBBB//0x000c//
#define 		U6 				0xAAAA//0x000d//
#define 		U7 				0x8888//0x000f
#define 		U_BLANK         0x7777

//逆变器1的开关矢量
#define  SVPWM_TABLE   {{U0,U6,U1,U1,U2,U0,U0},\
						{U0,U1,U1,U2,U2,U0,U0},\
						{U0,U1,U2,U2,U3,U0,U0},\
						{U0,U2,U2,U3,U3,U0,U0},\
						{U0,U2,U3,U3,U4,U0,U0},\
						{U0,U3,U3,U4,U4,U0,U0},\
						{U0,U3,U4,U4,U5,U0,U0},\
						{U0,U4,U4,U5,U5,U0,U0},\
						{U0,U4,U5,U5,U6,U0,U0},\
						{U0,U5,U5,U6,U6,U0,U0},\
						{U0,U5,U6,U6,U1,U0,U0},\
						{U0,U6,U6,U1,U1,U0,U0}}

//逆变器2的开关矢量
#define  SVPWM_TABLE1   {{U6,U6,U0,U0,U1,U1,U0},\
						{U6,U1,U0,U0,U1,U2,U0},\
						{U1,U1,U0,U0,U2,U2,U0},\
						{U1,U2,U0,U0,U2,U3,U0},\
						{U2,U2,U0,U0,U3,U3,U0},\
						{U2,U3,U0,U0,U3,U4,U0},\
						{U3,U3,U0,U0,U4,U4,U0},\
						{U3,U4,U0,U0,U4,U5,U0},\
						{U4,U4,U0,U0,U5,U5,U0},\
						{U4,U5,U0,U0,U5,U6,U0},\
						{U5,U5,U0,U0,U6,U6,U0},\
						{U5,U6,U0,U0,U6,U1,U0}}


#define   PWM_1PI12_RADIAN         0.2617993    //弧度 1pi/12  15°
#define   PWM_2PI12_RADIAN         0.5235987    //弧度 2pi/12  30°
#define   PWM_1PI4_RADIAN          0.7853981    //弧度 3pi/12  45°
#define   PWM_1PI3_RADIAN          1.0471975    //弧度 4pi/12  60°
#define   PWM_5PI12_RADIAN         1.3089969
#define   PWM_7PI12_RADIAN         1.8325957
#define   PWM_3PI4_RADIAN          2.3561944
#define   PWM_11PI12_RADIAN        2.8797932
#define   PWM_13PI12_RADIAN        3.4033920
#define   PWM_5PI4_RADIAN          3.9269908
#define   PWM_17PI12_RADIAN        4.4505895
#define   PWM_19PI12_RADIAN        4.9741883
#define   PWM_21PI12_RADIAN        5.4977871
#define   PWM_23PI12_RADIAN        6.0213859

#define   PWM_PI_RADIAN            3.1415926    //弧度  pi
#define   PWM_1PI2_RADIAN          1.5707963    //弧度  pi/2
#define   PWM_3PI2_RADIAN          4.7123890    //弧度 3pi/2
#define   PWM_2PI_RADIAN           6.2831853    //弧度  2pi
#define   PWM_1PI3_RADIAN          1.0471975    //弧度 1pi/3
#define   PWM_2PI3_RADIAN          2.0943950    //弧度 2pi/3
#define   PWM_4PI3_RADIAN          4.1887901    //弧度 4pi/3
#define   PWM_5PI3_RADIAN          5.2359876    //弧度 5pi/3
#define   PWM_1PI6_RADIAN          0.5235987    //弧度 1pi/6
#define   PWM_2PI6_RADIAN          1.0471975    //弧度 2pi/6
#define   PWM_5PI6_RADIAN          2.6179939    //弧度 5pi/6
#define   PWM_7PI6_RADIAN          3.6651914    //弧度 7pi/6
#define   PWM_11PI6_RADIAN         5.7595865    //弧度 11pi/6
#define   PWM_1PI12_RADIAN         0.2617993    //弧度 1pi/12  15°
#define   PWM_2PI12_RADIAN         0.5235987    //弧度 2pi/12  30°
#define   PWM_3PI12_RADIAN         0.7853981    //弧度 3pi/12  45°
#define   PWM_4PI12_RADIAN         1.0471975    //弧度 4pi/12  60°
#define   PWM_1PI24_RADIAN         0.1308996    //弧度 1pi/24  7.5度
#define   PWM_1PI30_RADIAN         0.1047195    //弧度 1pi/30  6°
#define   PWM_1PI15_RADIAN         0.2094395    //弧度 1pi/15  12°
#define   PWM_2PI15_RADIAN         0.4188790    //弧度 2pi/15  24°
#define   PWM_3PI15_RADIAN         0.6283185    //弧度 3pi/15  36°
#define   PWM_4PI15_RADIAN         0.8377580    //弧度 4pi/15  48°
#define   PWM_5PI15_RADIAN         1.0471975    //弧度 5pi/15  60°
#define   PWM_6PI15_RADIAN         1.2566370    //弧度 6pi/15  72°
#define   PWM_1PI18_RADIAN         0.1745329    //弧度 1pi/18  10°
#define   PWM_1PI9_RADIAN          0.3490658    //弧度 1pi/9   20°
#define   PWM_2PI9_RADIAN          0.6981317    //弧度 2pi/9   40°
#define   PWM_3PI9_RADIAN          1.0471953    //弧度 3pi/9   60°
#define   PWM_4PI9_RADIAN          1.3962633    //弧度 4pi/9   80°
#define   PWM_17PI9_RADIAN         5.9341195    //弧度 17pi/9  340°
#define   PWM_1PI45_RADIAN         0.06981317   //弧度 pi/45

#define   PWM_1_DIV_1PI9_RADIAN    2.86478897   //1/(pi/9)
#define   PWM_1_DIV_1PI6_RADIAN    1.90985932   //1/(pi/6)
#define   PWM_1_DIV_1PI3_RADIAN    0.95492966   //1/(pi/3)

//*************************************************************
#define   PWM_SIN0                 0.0000000    
#define   PWM_SIN15                0.2588190    
#define   PWM_SIN30                0.5000000    
#define   PWM_SIN45                0.7071067    
#define   PWM_SIN60                0.8660254  
#define   PWM_SIN75                0.9659258    
#define   PWM_SIN90                1.0000000 
#define   PWM_COS0                 1.0000000   
#define   PWM_COS15                0.9659258   
#define   PWM_COS30                0.8660254   
#define   PWM_COS45                0.7071067   
#define   PWM_COS60                0.5000000   
#define   PWM_COS75                0.2588190    
#define   PWM_COS90                0.0000000    
#define   PWM_COS60                0.5000000    
#define   PWM_SIN30                0.5000000    
#define   PWM_COS40                0.7660444    
#define   PWM_COS80                0.1736481    
#define   PWM_SIN10                0.1736481    
#define   PWM_SIN50                0.7660444    
#define   PWM_COS120              -0.5000000
#define   PWM_COS180              -1.0000000
#define   PWM_COS240              -0.5000000
#define   PWM_COS300               0.5000000  
#define   PWM_SIN120               0.8660254
#define   PWM_SIN180               0
#define   PWM_SIN240              -0.8660254
#define   PWM_SIN300              -0.8660254
#define   PWM_TWO_DIV_PI           0.6366198    //2除以pi
#define   PWM_SQRT3                1.7320508    //根号3
#define   PWM_SQRT3_DIVI_2         0.8660254    //2分之根号3
#define   PWM_PI_DIVI_2            1.5707963    //pi除以2
#define   PWM_2SQRT3_DIVI_PI       1.1026578    //pi分之2倍根3

/*****************************************************************************************
 *定义开关表结构体
 *****************************************************************************************/
typedef struct  { //const unsigned short    StepNumb_Dirction; //代表矢量顺序
                  const unsigned short    Pwmstate[7];      //状态表格
                 }PWMSWICHTABLE;
typedef struct  { //const unsigned short    StepNumb_Dirction; //代表矢量顺序
                  const unsigned short    Pwmstate[7];      //状态表格
                }PWMSWICHTABLE1;
/*****************************************************************************************
 *定义时间相关参数
 *****************************************************************************************/
struct     PWM_TIME_PARAMETER {float    PWMSwitchFrequency;  //开关频率
                               float    FPGACountPerS;      //每秒钟对应的FPGA计数
                               float    ConvertDeadTime;         //逆变器死区时间计数
                               float    ConvertPluseMinTime;     //逆变器最小脉宽计数
                               float    ConvertPluseDelayTime;   //逆变器延时时间
                               };

#define  PWM_TIME_PARAMETER_DEFAULTS      {PWM_ASYNCH_SWITCH_FREQUENCE,\
                                           PWM_FPGA_COUNT_PER_S,\
										   PWM_CONVERT_DEAD_TIME,\
										   PWM_CONVERT_PLUSE_MIN_TIME,\
										   PWM_CONVERT_PLUSE_DELAY_TIME}
/*****************************************************************************************
 *  PWM输出
 *****************************************************************************************/
struct    PWM_OUT           {unsigned int           Timer1[7];              // 脉冲时长
                             unsigned short         State1[7];	          // 脉冲矢量
                             unsigned int           Timer2[7];       // 脉冲时长
                             unsigned short         State2[7];	          // 脉冲矢量
                             int                    control_mode;           //偶左奇右
                            };
#define  PWM_OUT_DEFAULTS    {{2000,2000,2000,2000,2000,2000,2000},\
                              {0,0,0,0,0,0,0},\
                               0}
/*****************************************************************************************
 *  PWM输入
 *****************************************************************************************/
struct   PWM_REF{float      U_alfa;               // Input:      alph-beta坐标系，U_alfa
                 float      U_beta;               // Input:      alph-beta坐标系，U_beta
                 float      U_z1;               // Input:      alph-beta坐标系，U_alfa
                 float      U_z2;               // Input:      alph-beta坐标系，U_beta
                 float      Us_Amplitude;         // Input:      极坐标 |Us|
                 float      Us_Phase;             // Input:      极坐标 相位角Us_Phase
                 float      Is_Phase;             // Input:      极坐标 相位角Is_Phase
                 float      Udc;                  // Input:      直流侧电压
                 float      We;                   // Input:      角速度
                 float      ModulationDepth;
				};
#define  PWM_REF_DEFAULTS         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}

//##########################################################################################################
//****************PWM中间变量*******************************************
struct  PWM_MIDDLE_VARIABLE    { int    SectorNumber;         // Variable    扇区编号
                                float    T1;
                                float    T2;
                                float    T3;
                                float    T4;
                                float    T0;           // Variable    单位：毫秒
                                float    Ta;
                                float    Tb;
                                float    Tc;
                                float    Td;
                                float    Ta1;
                                float    Tb1;
                                float    Tc1;
                                float    Td1;
                                float    ThetaCalc;
                                float    PwmVecterPeriod;
					            };

#define PWM_MIDDLE_VARIABLE_DEFAULTS {0.0,  0.0,  0.0,  0.0, 0.0,  0.0}
//##########################################################################################################
//##########################################################################################################
//##########################################################################################################
//##########################################################################################################
/*****************************************************************************************
 *                   定义PWM对象，
 *****************************************************************************************/
typedef struct 	 {short                          TaskIndex ;           // Input       任务编号
				  float                          Ts;                   // Input       采样时间
                  short                          RighOrLeft;
				  struct  PWM_TIME_PARAMETER     Par;            // parament    与频率设置相关
                  struct  PWM_REF                Ref;
	              struct  PWM_OUT                PwmOutTemp;           // OutPut           
                  struct  PWM_MIDDLE_VARIABLE    Val;
                  PWMSWICHTABLE                  *PwmTabPtr;
                  PWMSWICHTABLE1                 *PwmTabPtr1;
                  float                          Iu1;
                  float                          Iv1;
                  float                          Iw1;
                  float                          Iu2;
                  float                          Iv2;
                  float                          Iw2;                  
				  void (*init)(); 
				  void (*reset)();                  				  
				  void (*calc)();
				  }PWM;

#define PWM_DEFAULTS       {0,\
							0.0,\
                            0,\
                            PWM_TIME_PARAMETER_DEFAULTS,\
							PWM_REF_DEFAULTS,\
                            PWM_OUT_DEFAULTS,\
                            PWM_MIDDLE_VARIABLE_DEFAULTS,\
                            0,\
                            0,\
                            0.0,\
                            0.0,\
                            0.0,\
                            0.0,\
                            0.0,\
                            0.0,\
                            (void (*)(int))svpwm_init,\
                            (void (*)(int))svpwm_reset,\
   						    (void (*)(int))svpwm_calc,\
   						    }
//*********************************************************************************************
void      svpwm_init(PWM *p);
void      svpwm_calc(PWM *p);
void      svpwm_reset(PWM *p);
//##########################################################################################################
//##########################################################################################################
extern PWM svpwm1;
extern PWM svpwm2;
#endif

