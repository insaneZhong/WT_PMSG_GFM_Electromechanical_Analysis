/*
 * sfuntmpl_basic.c: Basic 'C' template for a level 2 S-function.
 *
 *  -------------------------------------------------------------------------
 *  | See matlabroot/simulink/src/sfuntmpl_doc.c for a more detailed template |
 *  -------------------------------------------------------------------------
 *
 * Copyright 1990-2002 The MathWorks, Inc.
 * $Revision: 1.27.4.2 $
 * You must specify the S_FUNCTION_NAME as the name of your S-function
 * (i.e. replace sfuntmpl_basic with the name of your S-function).
 */
#define  S_FUNCTION_NAME   main_vsg  // VSG副本专用S-Function名称
#define  S_FUNCTION_LEVEL  2
/* --------------------------------------------------------------------*/
/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include   "simstruc.h"
#include   <math.h>
#include   <string.h>
#include   "motorcontrol.h"
#include        "grid_forming_control_vsg.h"
#ifdef ENABLE_SCHEMEA_OVERRIDES
#include   "schemeA_tuning_overrides.h"
#endif
#include   "svpwm.h"
#define     Input(element)       (*uPtrs[element])  /* Pointer to Input Port0 */

int         FPGAPWMTimerCounter1       ;//PWM脉冲定时计数器
int         FPGAPWMTimerPeriod1        ; //PWM脉冲时间
int         FPGAPWMTimerIntruptIndex1  ;//脉冲段位置计数
int         PwmPulseSate1              ;//PWM脉冲状态
int         FPGAPWMTimerCounter2       ;//PWM脉冲定时计数器
int         FPGAPWMTimerPeriod2        ; //PWM脉冲时间
int         FPGAPWMTimerIntruptIndex2  ;//脉冲段位置计数
int         PwmPulseSate2              ;//PWM脉冲状态
int         ControlTimerCounter        ;//控制定时器计数
int         ControlTimerPeriod         ; //控制定时器周期
real_T      Ts_control                 ;//控制程序执行周期 单位us
int         ControlTimerEnable         ;//控制定时器使能
int         ControlTimerFlag           ;//控制定时器中断标志
int         ControlTimerIntruptIndex   ;//控制定时器中断计数
int         PWMSegmentNumber;

int         vector;
int         vector1;
PWM         svpwm1           = PWM_DEFAULTS;
PWM         svpwm2           = PWM_DEFAULTS;
MOTOR       motor           = MOTOR_DEFAULTS;   
float       system_Time=0;

extern MOTOR_PI   voltage_loop_pi;
extern MOTOR_PI   PLL_loop_pi;
extern MOTOR_SLOPE_LIMIT vloop_slope;
extern MOTOR_PI d_loop_pi;
extern MOTOR_PI q_loop_pi;
extern MOTOR_PI d_voltage_loop_pi;
extern MOTOR_PI q_voltage_loop_pi;
extern MOTOR_PI E_voltage_loop_pi;
extern MOTOR_PI power_loop_pi;
extern CLACK clack_trans;
extern PARK park_u;
extern PARK park_PLL;
extern MOTOR_LOW_PASS_FILTER lpf;
extern MOTOR_LOW_PASS_FILTER lpf1;
extern MOTOR_HIGH_PASS_FILTER hpf;
extern MOTOR_BAND_PASS_FILTER bandpf;

static void reset_controller_state(void)
{
    PWM svpwm1_defaults = PWM_DEFAULTS;
    PWM svpwm2_defaults = PWM_DEFAULTS;
    MOTOR motor_defaults = MOTOR_DEFAULTS;

    FPGAPWMTimerCounter1 = 0;
    FPGAPWMTimerPeriod1 = 10;
    FPGAPWMTimerIntruptIndex1 = 0;
    PwmPulseSate1 = 0;
    FPGAPWMTimerCounter2 = 0;
    FPGAPWMTimerPeriod2 = 10;
    FPGAPWMTimerIntruptIndex2 = 0;
    PwmPulseSate2 = 0;
    ControlTimerCounter = 0;
    ControlTimerPeriod = 100;
    ControlTimerEnable = 0;
    ControlTimerFlag = 0;
    ControlTimerIntruptIndex = 0;
    Ts_control = 0;
    PWMSegmentNumber = 7;
    vector = 0;
    vector1 = 0;
    system_Time = 0;

    svpwm1 = svpwm1_defaults;
    svpwm2 = svpwm2_defaults;
    motor = motor_defaults;
    svpwm1.init(&svpwm1);
    svpwm2.init(&svpwm2);
    motor.init(&motor);
    grid_side_reset();
}

static void rebind_controller_callbacks(void)
{
    PWM pwm_defaults = PWM_DEFAULTS;
    MOTOR motor_defaults = MOTOR_DEFAULTS;
    MOTOR_PI d_loop_defaults = CURRENT_PI_ID_DEFAULTS;
    MOTOR_PI q_loop_defaults = CURRENT_PI_IQ_DEFAULTS;
    MOTOR_PI d_voltage_loop_defaults = VOLTAGE_LOOP_PI_DEFAULTS;
    MOTOR_PI q_voltage_loop_defaults = VOLTAGE_LOOP_PI_DEFAULTS;
    MOTOR_PI pll_loop_defaults = PLL_LOOP_PI_DEFAULTS;
    MOTOR_PI e_voltage_loop_defaults = E_VOLTAGE_LOOP_PI_DEFAULTS;
    MOTOR_PI power_loop_defaults = POWER_LOOP_PI_DEFAULTS;

    svpwm1.init = pwm_defaults.init;
    svpwm1.reset = pwm_defaults.reset;
    svpwm1.calc = pwm_defaults.calc;
    svpwm2.init = pwm_defaults.init;
    svpwm2.reset = pwm_defaults.reset;
    svpwm2.calc = pwm_defaults.calc;

    motor.init = motor_defaults.init;
    motor.control = motor_defaults.control;
    motor.reset = motor_defaults.reset;
    motor.id_pi.calc1 = motor_defaults.id_pi.calc1;
    motor.id_pi.calc2 = motor_defaults.id_pi.calc2;
    motor.id_pi.reset = motor_defaults.id_pi.reset;
    motor.iq_pi.calc1 = motor_defaults.iq_pi.calc1;
    motor.iq_pi.calc2 = motor_defaults.iq_pi.calc2;
    motor.iq_pi.reset = motor_defaults.iq_pi.reset;
    motor.pwm_speed_pi.calc1 = motor_defaults.pwm_speed_pi.calc1;
    motor.pwm_speed_pi.calc2 = motor_defaults.pwm_speed_pi.calc2;
    motor.pwm_speed_pi.reset = motor_defaults.pwm_speed_pi.reset;
    motor.id_slope_limit.cale = motor_defaults.id_slope_limit.cale;
    motor.id_slope_limit.reset = motor_defaults.id_slope_limit.reset;
    motor.iq_slope_limit.cale = motor_defaults.iq_slope_limit.cale;
    motor.iq_slope_limit.reset = motor_defaults.iq_slope_limit.reset;
    motor.speed_slope_limit.cale = motor_defaults.speed_slope_limit.cale;
    motor.speed_slope_limit.reset = motor_defaults.speed_slope_limit.reset;

#define RELOAD_PI_PARAMS(target, defaults) \
    (target).Kp = (defaults).Kp; \
    (target).Ki = (defaults).Ki; \
    (target).Kc = (defaults).Kc; \
    (target).Kd = (defaults).Kd; \
    (target).OutMax = (defaults).OutMax; \
    (target).OutMin = (defaults).OutMin
    RELOAD_PI_PARAMS(motor.id_pi, motor_defaults.id_pi);
    RELOAD_PI_PARAMS(motor.iq_pi, motor_defaults.iq_pi);
    RELOAD_PI_PARAMS(motor.pwm_speed_pi, motor_defaults.pwm_speed_pi);
    RELOAD_PI_PARAMS(d_loop_pi, d_loop_defaults);
    RELOAD_PI_PARAMS(q_loop_pi, q_loop_defaults);
    RELOAD_PI_PARAMS(d_voltage_loop_pi, d_voltage_loop_defaults);
    RELOAD_PI_PARAMS(q_voltage_loop_pi, q_voltage_loop_defaults);
    RELOAD_PI_PARAMS(PLL_loop_pi, pll_loop_defaults);
    RELOAD_PI_PARAMS(E_voltage_loop_pi, e_voltage_loop_defaults);
    RELOAD_PI_PARAMS(power_loop_pi, power_loop_defaults);
#undef RELOAD_PI_PARAMS
}

#if defined(MATLAB_MEX_FILE)
static mxArray *snapshot_bytes(const void *source, size_t count)
{
    mxArray *value = mxCreateNumericMatrix(1, count, mxUINT8_CLASS, mxREAL);
    memcpy(mxGetData(value), source, count);
    return value;
}

static int restore_bytes(SimStruct *S, const mxArray *snapshot,
                         const char *name, void *target, size_t count)
{
    const mxArray *value = mxGetField(snapshot, 0, name);
    if (value == NULL || !mxIsUint8(value) || mxGetNumberOfElements(value) != count) {
        ssSetErrorStatus(S, "Invalid or incompatible GFM S-Function operating point.");
        return 0;
    }
    memcpy(target, mxGetData(value), count);
    return 1;
}
#endif

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    /* See sfuntmpl_doc.c for more details on the macros below */

    ssSetNumSFcnParams(S, 4);  //定义参数个数
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        return;
    }
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 16); //定义输入变量个数
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 30);//定义输出变量个数   
    ssSetNumSampleTimes(S, 1); 
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    /* Specify the sim state compliance to be same as a built-in block */
    ssSetOperatingPointCompliance(S, USE_CUSTOM_OPERATING_POINT);
    ssSetOptions(S, 0);
    reset_controller_state();
}
/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, 1e-6);
    ssSetOffsetTime(S, 0, 0.0);
}
#undef MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *    You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
  static void mdlInitializeConditions(SimStruct *S)
  {
    real_T *x0 = ssGetRealDiscStates(S);
  }
#endif /* MDL_INITIALIZE_CONDITIONS */
#define MDL_START
#if defined(MDL_START) 
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
  static void mdlStart(SimStruct *S)
  {
    reset_controller_state();
  }
#endif /*  MDL_START */
/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);  
    real_T           *OutPut= ssGetOutputPortRealSignal(S,0);    
    real_T           *para1 = mxGetPr(ssGetSFcnParam(S,0));
    real_T           *para2 = mxGetPr(ssGetSFcnParam(S,1));
    real_T           *para3 = mxGetPr(ssGetSFcnParam(S,2));
    real_T           *para4 = mxGetPr(ssGetSFcnParam(S,3));

    /* Always refresh references from S-Function parameters. */
    grid_side.ref.P_active_power_ref   = para1[0];
    grid_side.ref.Q_reactive_power_ref = para2[0];
    grid_side.ref.voltage_ref          = para3[0];
    motor.ref.voltage_ref              = para4[0];

    //=====================================================================
    if(ControlTimerFlag==1)
    {
        ControlTimerFlag=0;
        Ts_control  = svpwm1.Val.PwmVecterPeriod;
        //===MOTOR============================================
        motor.Ts                       = Ts_control;
        grid_side.ref.P_active_power_ref     = para1[0];
        grid_side.ref.Q_reactive_power_ref   = para2[0];
        grid_side.ref.voltage_ref            = para3[0];
        motor.ref.voltage_ref                = para4[0];
        system_Time                          = Input(15);
        motor.ref.dvc_enable                  = (system_Time >= PRESYN_SWITCH_TIME) ? 1 : 0;
        motor.ref.active_power_ref            = vloop_slope.Out;
        
        motor.bak.Ia1                  = Input(0);
        motor.bak.Ib1                  = Input(1);
        motor.bak.Ic1                  = Input(2);
        motor.bak.Udc1                 = Input(3);   
        motor.bak.We                   = Input(4);
        motor.bak.RotorPos             = Input(5);       
        motor.control(&motor); 
        //*********SVPWM************************
		svpwm1.Ref.U_alfa   = motor.out.Us_alfa;
		svpwm1.Ref.U_beta   = motor.out.Us_beta;
		svpwm1.Par.PWMSwitchFrequency =10000.0;
		svpwm1.Ref.Udc     = motor.bak.Udc1;
		svpwm1.calc(&svpwm1);  
        
        grid_side.bak.Ia1                  = Input(6);
        grid_side.bak.Ib1                  = Input(7);
        grid_side.bak.Ic1                  = Input(8);
        grid_side.bak.pcc_uab              = Input(9);
        grid_side.bak.pcc_ubc              = Input(10);
        grid_side.bak.pcc_uca              = Input(11);
        grid_side.bak.pcc_Ia               = Input(12);
        grid_side.bak.pcc_Ib               = Input(13);
        grid_side.bak.pcc_Ic               = Input(14);
        grid_side.bak.Udc1  = motor.bak.Udc1;
       // if(system_Time >0.15)
        {
             grid_side_control(&grid_side); 
            //*********SVPWM************************
            svpwm2.Ref.U_alfa   = grid_side.out.Us_alfa;
            svpwm2.Ref.U_beta   = grid_side.out.Us_beta;
            svpwm2.Par.PWMSwitchFrequency =10000.0;
            svpwm2.Ref.Udc     = motor.bak.Udc1;
            svpwm2.calc(&svpwm2);  
        }
        
    
    }
    
    //*********************************************************************
    //=====================================================================
    //=====================================================================
	ControlTimerCounter++;
	if(ControlTimerCounter>ControlTimerPeriod) //DSP控制定时中断
	{
        ControlTimerCounter=0;
        ControlTimerFlag  =1;
        ControlTimerIntruptIndex++;
        //发同步信号,FPGA开始发脉冲
        FPGAPWMTimerCounter1       =0;
        FPGAPWMTimerIntruptIndex1  =0;
        FPGAPWMTimerCounter2       =0;
        FPGAPWMTimerIntruptIndex2  =0;
        ControlTimerPeriod = svpwm1.Val.PwmVecterPeriod*1e6;
	}
    
    //*********************************************************************
    FPGAPWMTimerCounter1++;
    FPGAPWMTimerPeriod1 = svpwm1.PwmOutTemp.Timer1[FPGAPWMTimerIntruptIndex1];//在FPGA中断进入之前给FPGA定时器赋值，进入中断之后由FPGA中断程序赋值
    PwmPulseSate1      = svpwm1.PwmOutTemp.State1[FPGAPWMTimerIntruptIndex1];
    if(FPGAPWMTimerCounter1>FPGAPWMTimerPeriod1)//FPGA计时到
    {
        FPGAPWMTimerCounter1 = 0;
        do
        {
            FPGAPWMTimerIntruptIndex1++;
            if(FPGAPWMTimerIntruptIndex1>=PWMSegmentNumber)
            {
                FPGAPWMTimerIntruptIndex1=PWMSegmentNumber-1;
                break;//防止最后一段脉冲时间为零时进入死循环
            }
            FPGAPWMTimerPeriod1 = svpwm1.PwmOutTemp.Timer1[FPGAPWMTimerIntruptIndex1];
            PwmPulseSate1      =  svpwm1.PwmOutTemp.State1[FPGAPWMTimerIntruptIndex1];
        }
        while( FPGAPWMTimerPeriod1 ==0);  
    }
    //*********************************************************************
    FPGAPWMTimerCounter2++;
    FPGAPWMTimerPeriod2 = svpwm2.PwmOutTemp.Timer1[FPGAPWMTimerIntruptIndex2];//在FPGA中断进入之前给FPGA定时器赋值，进入中断之后由FPGA中断程序赋值
    PwmPulseSate2      = svpwm2.PwmOutTemp.State1[FPGAPWMTimerIntruptIndex2];
    if(FPGAPWMTimerCounter2>FPGAPWMTimerPeriod2)//FPGA计时到
    {
        FPGAPWMTimerCounter2 = 0;
        do
        {
            FPGAPWMTimerIntruptIndex2++;
            if(FPGAPWMTimerIntruptIndex2>=PWMSegmentNumber)
            {
                FPGAPWMTimerIntruptIndex2=PWMSegmentNumber-1;
                break;//防止最后一段脉冲时间为零时进入死循环
            }
            FPGAPWMTimerPeriod2 = svpwm2.PwmOutTemp.Timer1[FPGAPWMTimerIntruptIndex2];
            PwmPulseSate2      =  svpwm2.PwmOutTemp.State1[FPGAPWMTimerIntruptIndex2];
        }
        while( FPGAPWMTimerPeriod2 ==0);  
    }    
    //*********************************************************************
    //=====================================================================
    //=====================================================================
    if(PwmPulseSate1==U0) 
    { OutPut[0]=0;OutPut[1]=1;OutPut[2]=0; OutPut[3]=1;OutPut[4]=0;OutPut[5]=1;}
    else if(PwmPulseSate1==U1) 
    { OutPut[0]=1;OutPut[1]=0;OutPut[2]=0;OutPut[3]=1;OutPut[4]=0;OutPut[5]=1;}
    else if(PwmPulseSate1==U2) 
    { OutPut[0]=1;OutPut[1]=0;OutPut[2]=1;OutPut[3]=0;OutPut[4]=0;OutPut[5]=1;}  
    else if(PwmPulseSate1==U3) 
    {OutPut[0]=0;OutPut[1]=1; OutPut[2]=1;OutPut[3]=0;OutPut[4]=0;OutPut[5]=1;}  
    else if(PwmPulseSate1==U4) 
    { OutPut[0]=0; OutPut[1]=1;OutPut[2]=1; OutPut[3]=0;OutPut[4]=1;OutPut[5]=0;}  
    else if(PwmPulseSate1==U5) 
    {  OutPut[0]=0;OutPut[1]=1; OutPut[2]=0;OutPut[3]=1; OutPut[4]=1;OutPut[5]=0; }   
    else if(PwmPulseSate1==U6) 
    {OutPut[0]=1;OutPut[1]=0; OutPut[2]=0;OutPut[3]=1; OutPut[4]=1; OutPut[5]=0; }
    else if(PwmPulseSate1==U7) 
    {OutPut[0]=1;OutPut[1]=0;OutPut[2]=1;OutPut[3]=0;OutPut[4]=1; OutPut[5]=0;}
    else if(PwmPulseSate1==U_BLANK)
    {OutPut[0]=0;OutPut[1]=0;OutPut[2]=0; OutPut[3]=0;OutPut[4]=0; OutPut[5]=0;}
    
    if(PwmPulseSate2==U0) 
    {OutPut[6]=0;OutPut[7]=1;OutPut[8]=0;OutPut[9]=1;OutPut[10]=0;OutPut[11]=1; }
    else if(PwmPulseSate2==U1) 
    {OutPut[6]=1; OutPut[7]=0; OutPut[8]=0;OutPut[9]=1;OutPut[10]=0;OutPut[11]=1; }
    else if(PwmPulseSate2==U2) 
    {OutPut[6]=1;OutPut[7]=0; OutPut[8]=1; OutPut[9]=0;OutPut[10]=0;OutPut[11]=1;}  
    else if(PwmPulseSate2==U3) 
    { OutPut[6]=0; OutPut[7]=1;OutPut[8]=1;OutPut[9]=0;OutPut[10]=0;OutPut[11]=1;}  
    else if(PwmPulseSate2==U4) 
    {OutPut[6]=0;OutPut[7]=1;OutPut[8]=1;OutPut[9]=0;OutPut[10]=1;OutPut[11]=0;}  
    else if(PwmPulseSate2==U5) 
    {  OutPut[6]=0;OutPut[7]=1; OutPut[8]=0;OutPut[9]=1;OutPut[10]=1;OutPut[11]=0;}   
    else if(PwmPulseSate2==U6) 
    { OutPut[6]=1;OutPut[7]=0;OutPut[8]=0;OutPut[9]=1;OutPut[10]=1;OutPut[11]=0;}
    else if(PwmPulseSate2==U7) 
    { OutPut[6]=1;OutPut[7]=0;OutPut[8]=1; OutPut[9]=0;OutPut[10]=1;OutPut[11]=0;}
    else if(PwmPulseSate2==U_BLANK)
    {OutPut[6]=0;OutPut[7]=0;OutPut[8]=0;OutPut[9]=0;OutPut[10]=0;OutPut[11]=0;}  

    //*********************************************************************
    //=====================================================================
    //=====================================================================
//     OutPut[12] = motor.pwm_speed_pi.Ref;
//     OutPut[13] = motor.pwm_speed_pi.Fdb;
//     OutPut[14] = motor.val.Iq_mpta;
//     OutPut[15] = motor.bak.Id1;  
//     OutPut[16] = motor.bak.Iq1;  
//     OutPut[17] = motor.bak.Id1;  
//     OutPut[18] = motor.val.Ud1_ref;
//     OutPut[19] = motor.val.Uq1_ref;
//     OutPut[20] = motor.out.Us_alfa;
//     OutPut[21] = motor.out.Us_beta;
//     OutPut[22] = svpwm1.Val.T1;
//     OutPut[23] = motor.id_pi.Out;
//     OutPut[24] = motor.iq_pi.Out;
//     OutPut[25] = motor.val.Ud_fwd;  
//     OutPut[26] = motor.val.Uq_fwd;  
    
    OutPut[12] = grid_side.ref.P_active_power_ref;
    OutPut[13] = grid_side.val.pcc_P_active_Power;
    OutPut[14] = grid_side.pf.w_ref;
    OutPut[15] = grid_side.pf.thet_ref;  
    OutPut[16] = grid_side.val.Ud1_ref ;  
    OutPut[17] = grid_side.val.Uq1_ref ;  
    OutPut[18] = grid_side.val.pcc_Q_reactive_Power;
    OutPut[19] = grid_side.ref.voltage_ref;
    OutPut[20] = grid_side.pf.U_od_ref;
    OutPut[21] = grid_side.val.pcc_u_d;
    /* MSC diagnostic mapping for DC-link tracking.
     * These outputs do not change the controller; they only expose internal
     * quantities needed to separate voltage headroom, PI saturation and
     * current-tracking errors.
     */
    OutPut[22] = motor.val.Ud_fwd;
    OutPut[23] = motor.val.Uq_fwd;
    OutPut[24] = motor.val.Ud1_ref;
    OutPut[25] = motor.val.Uq1_ref;
    OutPut[26] = motor.bak.Id1;
    OutPut[27] = motor.bak.Iq1;
    OutPut[28] = grid_side.val.Pre_syn;
    OutPut[29] = motor.val.Iq_ref;
}
#undef MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
  /* Function: mdlUpdate ======================================================
   * Abstract:
   *    This function is called once for every major integration time step.
   *    Discrete states are typically updated here, but this function is useful
   *    for performing any tasks that should only take place once per
   *    integration step.
   */
  static void mdlUpdate(SimStruct *S, int_T tid)
  {
    real_T           *x = ssGetRealDiscStates(S);
  }
#endif /* MDL_UPDATE */
#define MDL_DERIVATIVES  /* Change to #undef to remove function */
#if defined(MDL_DERIVATIVES)
  /* Function: mdlDerivatives =================================================
   * Abstract:
   *    In this function, you compute the S-function block's derivatives.
   *    The derivatives are placed in the derivative vector, ssGetdX(S).
   */
  static void mdlDerivatives(SimStruct *S)
  {
  }
#endif /* MDL_DERIVATIVES */

#define MDL_OPERATING_POINT
#if defined(MDL_OPERATING_POINT) && defined(MATLAB_MEX_FILE)
static mxArray *mdlGetOperatingPoint(SimStruct *S)
{
    const char *fields[] = {
        "FPGAPWMTimerCounter1", "FPGAPWMTimerPeriod1",
        "FPGAPWMTimerIntruptIndex1", "PwmPulseSate1",
        "FPGAPWMTimerCounter2", "FPGAPWMTimerPeriod2",
        "FPGAPWMTimerIntruptIndex2", "PwmPulseSate2",
        "ControlTimerCounter", "ControlTimerPeriod",
        "ControlTimerEnable", "ControlTimerFlag",
        "ControlTimerIntruptIndex", "Ts_control", "PWMSegmentNumber",
        "vector", "vector1", "system_Time", "svpwm1", "svpwm2", "motor",
        "vloop_slope", "d_loop_pi", "q_loop_pi", "d_voltage_loop_pi",
        "q_voltage_loop_pi", "PLL_loop_pi", "E_voltage_loop_pi",
        "power_loop_pi", "clack_trans", "park_u", "park_PLL", "grid_side",
        "lpf", "lpf1", "hpf", "bandpf", "w_vsg_state"
    };
    mxArray *snapshot = mxCreateStructMatrix(1, 1, 38, fields);
    float w_vsg_state = grid_side_get_w_vsg_state();
    UNUSED_ARG(S);

#define SNAPSHOT_FIELD(name) \
    mxSetField(snapshot, 0, #name, snapshot_bytes(&(name), sizeof(name)))
    SNAPSHOT_FIELD(FPGAPWMTimerCounter1);
    SNAPSHOT_FIELD(FPGAPWMTimerPeriod1);
    SNAPSHOT_FIELD(FPGAPWMTimerIntruptIndex1);
    SNAPSHOT_FIELD(PwmPulseSate1);
    SNAPSHOT_FIELD(FPGAPWMTimerCounter2);
    SNAPSHOT_FIELD(FPGAPWMTimerPeriod2);
    SNAPSHOT_FIELD(FPGAPWMTimerIntruptIndex2);
    SNAPSHOT_FIELD(PwmPulseSate2);
    SNAPSHOT_FIELD(ControlTimerCounter);
    SNAPSHOT_FIELD(ControlTimerPeriod);
    SNAPSHOT_FIELD(ControlTimerEnable);
    SNAPSHOT_FIELD(ControlTimerFlag);
    SNAPSHOT_FIELD(ControlTimerIntruptIndex);
    SNAPSHOT_FIELD(Ts_control);
    SNAPSHOT_FIELD(PWMSegmentNumber);
    SNAPSHOT_FIELD(vector);
    SNAPSHOT_FIELD(vector1);
    SNAPSHOT_FIELD(system_Time);
    SNAPSHOT_FIELD(svpwm1);
    SNAPSHOT_FIELD(svpwm2);
    SNAPSHOT_FIELD(motor);
    SNAPSHOT_FIELD(vloop_slope);
    SNAPSHOT_FIELD(d_loop_pi);
    SNAPSHOT_FIELD(q_loop_pi);
    SNAPSHOT_FIELD(d_voltage_loop_pi);
    SNAPSHOT_FIELD(q_voltage_loop_pi);
    SNAPSHOT_FIELD(PLL_loop_pi);
    SNAPSHOT_FIELD(E_voltage_loop_pi);
    SNAPSHOT_FIELD(power_loop_pi);
    SNAPSHOT_FIELD(clack_trans);
    SNAPSHOT_FIELD(park_u);
    SNAPSHOT_FIELD(park_PLL);
    SNAPSHOT_FIELD(grid_side);
    SNAPSHOT_FIELD(lpf);
    SNAPSHOT_FIELD(lpf1);
    SNAPSHOT_FIELD(hpf);
    SNAPSHOT_FIELD(bandpf);
    SNAPSHOT_FIELD(w_vsg_state);
#undef SNAPSHOT_FIELD
    return snapshot;
}

static void mdlSetOperatingPoint(SimStruct *S, const mxArray *snapshot)
{
    float w_vsg_state;

#define RESTORE_FIELD(name) \
    if (!restore_bytes(S, snapshot, #name, &(name), sizeof(name))) return
    RESTORE_FIELD(FPGAPWMTimerCounter1);
    RESTORE_FIELD(FPGAPWMTimerPeriod1);
    RESTORE_FIELD(FPGAPWMTimerIntruptIndex1);
    RESTORE_FIELD(PwmPulseSate1);
    RESTORE_FIELD(FPGAPWMTimerCounter2);
    RESTORE_FIELD(FPGAPWMTimerPeriod2);
    RESTORE_FIELD(FPGAPWMTimerIntruptIndex2);
    RESTORE_FIELD(PwmPulseSate2);
    RESTORE_FIELD(ControlTimerCounter);
    RESTORE_FIELD(ControlTimerPeriod);
    RESTORE_FIELD(ControlTimerEnable);
    RESTORE_FIELD(ControlTimerFlag);
    RESTORE_FIELD(ControlTimerIntruptIndex);
    RESTORE_FIELD(Ts_control);
    RESTORE_FIELD(PWMSegmentNumber);
    RESTORE_FIELD(vector);
    RESTORE_FIELD(vector1);
    RESTORE_FIELD(system_Time);
    RESTORE_FIELD(svpwm1);
    RESTORE_FIELD(svpwm2);
    RESTORE_FIELD(motor);
    RESTORE_FIELD(vloop_slope);
    RESTORE_FIELD(d_loop_pi);
    RESTORE_FIELD(q_loop_pi);
    RESTORE_FIELD(d_voltage_loop_pi);
    RESTORE_FIELD(q_voltage_loop_pi);
    RESTORE_FIELD(PLL_loop_pi);
    RESTORE_FIELD(E_voltage_loop_pi);
    RESTORE_FIELD(power_loop_pi);
    RESTORE_FIELD(clack_trans);
    RESTORE_FIELD(park_u);
    RESTORE_FIELD(park_PLL);
    RESTORE_FIELD(grid_side);
    RESTORE_FIELD(lpf);
    RESTORE_FIELD(lpf1);
    RESTORE_FIELD(hpf);
    RESTORE_FIELD(bandpf);
    RESTORE_FIELD(w_vsg_state);
#undef RESTORE_FIELD
    grid_side_set_w_vsg_state(w_vsg_state);
    rebind_controller_callbacks();
}
#endif

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
}
/*======================================================*
 * See sfuntmpl_doc.c for the optional S-function methods *
 *======================================================*/
/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif










