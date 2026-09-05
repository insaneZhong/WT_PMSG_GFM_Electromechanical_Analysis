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
#ifndef LEGACY_AD_S_FUNCTION_NAME
#define LEGACY_AD_S_FUNCTION_NAME main_legacy_ad
#endif
#define  S_FUNCTION_NAME   LEGACY_AD_S_FUNCTION_NAME  // 可复编译旧控制器+MSC主动阻尼
#define  S_FUNCTION_LEVEL  2
/* --------------------------------------------------------------------*/
/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include   "simstruc.h"
#include   <math.h>
#include   <string.h>
#include   "motorcontrol_legacy_tunable.h"
#include        "grid_forming_control.h"
#include   "svpwm.h"
#ifndef GSI_DC_ENERGY_KP_W_PER_V
#define GSI_DC_ENERGY_KP_W_PER_V 0.0f
#endif
#ifndef PWM_SWITCH_FREQUENCY_HZ
#define PWM_SWITCH_FREQUENCY_HZ 10000.0f
#endif
#ifndef GSI_DC_ENERGY_KI_W_PER_VS
#define GSI_DC_ENERGY_KI_W_PER_VS 0.0f
#endif
#ifndef GSI_DC_ENERGY_POWER_LIMIT_W
#define GSI_DC_ENERGY_POWER_LIMIT_W 0.0f
#endif
#ifndef GSI_DC_ENERGY_TARGET_V
#define GSI_DC_ENERGY_TARGET_V 1000.0f
#endif
#ifndef GSI_DC_ENERGY_INTEGRAL_ENABLE_TIME_S
#define GSI_DC_ENERGY_INTEGRAL_ENABLE_TIME_S 6.0f
#endif
#ifndef GSI_DC_ENERGY_DEADBAND_V
#define GSI_DC_ENERGY_DEADBAND_V 0.0f
#endif
#ifndef GSI_DC_ENERGY_LVRT_ONLY
#define GSI_DC_ENERGY_LVRT_ONLY 0
#endif
#ifndef MSC_TYPEC_USE_PCC_POWER_FF
#define MSC_TYPEC_USE_PCC_POWER_FF 0
#endif
#ifndef MSC_TYPEC_PCC_FF_CUTOFF_HZ
#define MSC_TYPEC_PCC_FF_CUTOFF_HZ 200.0f
#endif
#ifndef MSC_POWER_FF_A_PER_W
#define MSC_POWER_FF_A_PER_W 0.0f
#endif
#ifndef MSC_TYPEC_SPEED_NORMALIZE
#define MSC_TYPEC_SPEED_NORMALIZE 0
#endif
#ifndef MSC_TYPEC_RATED_ELEC_OMEGA_RADPS
#define MSC_TYPEC_RATED_ELEC_OMEGA_RADPS 1.0f
#endif
#ifndef MSC_TYPEC_MIN_ELEC_OMEGA_RADPS
#define MSC_TYPEC_MIN_ELEC_OMEGA_RADPS 1.0f
#endif
#ifndef GSI_PREF_DISTURBANCE_START_S
#define GSI_PREF_DISTURBANCE_START_S 1.0e9f
#endif
#ifndef GSI_PREF_DISTURBANCE_END_S
#define GSI_PREF_DISTURBANCE_END_S 1.0e9f
#endif
#ifndef GSI_PREF_DISTURBANCE_DELTA_W
#define GSI_PREF_DISTURBANCE_DELTA_W 0.0f
#endif
#ifndef GSI_DYNAMIC_PREF_INPUT_ENABLE
#define GSI_DYNAMIC_PREF_INPUT_ENABLE 0
#endif
/* Optional Vdc reference profile; Input(19) is active only in the
 * dedicated profile-enabled controller build. */
#ifndef GSI_DYNAMIC_VDC_REF_INPUT_ENABLE
#define GSI_DYNAMIC_VDC_REF_INPUT_ENABLE 0
#endif
/* Optional fifth S-function parameter.  When enabled, the MSC-DVC gains
 * can be varied without recompiling the MEX.  Keeping one controller binary
 * preserves the custom ModelOperatingPoint that stores all C globals. */
#ifndef LEGACY_RUNTIME_DVC_SCALE
#define LEGACY_RUNTIME_DVC_SCALE 0
#endif

/* The July stable MEX was built without forcing a second global-state reset
 * from mdlInitializeSizes.  Keep the current deterministic reset as the
 * default, but allow a compatibility build for operating-point snapshots to
 * reproduce that stable controller initialization exactly. */
#ifndef LEGACY_RESET_CONTROLLER_ON_INIT
#define LEGACY_RESET_CONTROLLER_ON_INIT 1
#endif
#ifndef MSC_LVRT_UDC_THRESHOLD_V
#define MSC_LVRT_UDC_THRESHOLD_V 1.0e9f
#endif
#ifndef MSC_LVRT_BLEND_RANGE_V
#define MSC_LVRT_BLEND_RANGE_V 1.0f
#endif
#ifndef MSC_LVRT_MIN_POWER_W
#define MSC_LVRT_MIN_POWER_W 0.0f
#endif
#ifndef MSC_LVRT_ENABLE_TIME_S
#define MSC_LVRT_ENABLE_TIME_S 0.0f
#endif
#ifndef MSC_LVRT_RECOVERY_UDC_LOW_V
#define MSC_LVRT_RECOVERY_UDC_LOW_V 985.0f
#endif
#ifndef MSC_LVRT_RECOVERY_UDC_HIGH_V
#define MSC_LVRT_RECOVERY_UDC_HIGH_V 1005.0f
#endif
#ifndef MSC_LVRT_RECOVERY_POWER_FRACTION
#define MSC_LVRT_RECOVERY_POWER_FRACTION 0.95f
#endif
#ifndef MSC_LVRT_RECOVERY_HOLD_S
#define MSC_LVRT_RECOVERY_HOLD_S 1.0f
#endif
#ifndef MSC_LVRT_UDC_POWER_GAIN_W_PER_V
#define MSC_LVRT_UDC_POWER_GAIN_W_PER_V 0.0f
#endif
#ifndef MSC_LVRT_PCC_VOLTAGE_THRESHOLD_V
#define MSC_LVRT_PCC_VOLTAGE_THRESHOLD_V 0.0f
#endif
#ifndef MSC_LVRT_PCC_FILTER_TAU_S
#define MSC_LVRT_PCC_FILTER_TAU_S 0.02f
#endif
#define     Input(element)       (*uPtrs[element])  /* Pointer to Input Port0 */

/*
 * The production EMT controllers export 37 signals: twelve gate commands
 * followed by 25 diagnostic values.  The averaged-converter validation copy
 * needs four additional continuous voltage commands while preserving that
 * interface exactly for every existing MEX.  This compile-time option is
 * intentionally off by default, so it cannot alter the retained switching
 * EMT controller or any of its saved operating-point snapshots.
 */
#ifndef IDEAL_AVG_OUTPUTS
#define IDEAL_AVG_OUTPUTS 0
#endif

#if IDEAL_AVG_OUTPUTS
#define LEGACY_AD_OUTPUT_WIDTH 41
#else
#define LEGACY_AD_OUTPUT_WIDTH 37
#endif

/*
 * Production switching EMT runs its S-function scheduler at 1 us so that
 * the FPGA/SVPWM pulse emulation can be represented.  An average-converter
 * analysis MEX may use a coarser scheduler, provided that the controller
 * interrupt remains at the original 100 us.  Both macros are compile-time
 * only and default to the production values, so existing MEX files retain
 * their original sample-time and timing behavior.
 */
#ifndef LEGACY_SFUNCTION_SAMPLE_TIME_S
#define LEGACY_SFUNCTION_SAMPLE_TIME_S 1e-6
#endif
#ifndef LEGACY_TIMER_TICK_US
#define LEGACY_TIMER_TICK_US 1.0
#endif

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
float       legacy_omega_rel_ad=0;
float       legacy_msc_ad_scale=1;
float       legacy_dc_energy_integral_w=0;
float       legacy_msc_iq_ff_a=0;
float       legacy_msc_pcc_ff_filter_w=0;
int         legacy_lvrt_active=0;
float       legacy_lvrt_recovery_timer_s=0;
float       legacy_lvrt_pcc_ud_filter_v=0;
float       legacy_lvrt_pcc_uq_filter_v=0;

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
extern GRID_SIDE_INV grid_side;
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
    ControlTimerPeriod = (int)(100.0 / LEGACY_TIMER_TICK_US);
    ControlTimerEnable = 0;
    ControlTimerFlag = 0;
    ControlTimerIntruptIndex = 0;
    Ts_control = 0;
    PWMSegmentNumber = 7;
    vector = 0;
    vector1 = 0;
    system_Time = 0;
    legacy_omega_rel_ad = 0;
    legacy_msc_ad_scale = 1;
    legacy_dc_energy_integral_w = 0;
    legacy_msc_iq_ff_a = 0;
    legacy_msc_pcc_ff_filter_w = 0;
    legacy_lvrt_active = 0;
    legacy_lvrt_recovery_timer_s = 0;
    legacy_lvrt_pcc_ud_filter_v = 0;
    legacy_lvrt_pcc_uq_filter_v = 0;

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
        ssSetErrorStatus(S, "Invalid or incompatible 5 MW legacy controller operating point.");
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

#if LEGACY_RUNTIME_DVC_SCALE
    ssSetNumSFcnParams(S, 5);
#else
    ssSetNumSFcnParams(S, 4);  //定义参数个数
#endif
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        return;
    }
#if LEGACY_RUNTIME_DVC_SCALE
    ssSetSFcnParamTunable(S, 4, 1);
#endif
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 20); // original 16 + shaft slip + dynamic MPPT Pref + AD scale + Vdc reference
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, LEGACY_AD_OUTPUT_WIDTH);
    ssSetNumSampleTimes(S, 1); 
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    /* Preserve all controller globals when a model operating point is saved. */
    ssSetOperatingPointCompliance(S, USE_CUSTOM_OPERATING_POINT);
    /* Make the custom operating point visible to the parent ModelOperatingPoint.
     * Without this flag Simulink may save the model-level object while
     * hiding the S-function payload, so mdlGetOperatingPoint/mdlSetOperatingPoint
     * are never available through ModelOperatingPoint.get(). */
    ssSetOperatingPointVisibility(S, 1);
    ssSetOptions(S, 0);
#if LEGACY_RESET_CONTROLLER_ON_INIT
    reset_controller_state();
#endif
}
/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, LEGACY_SFUNCTION_SAMPLE_TIME_S);
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
#undef MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
  static void mdlStart(SimStruct *S)
  {
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
#if LEGACY_RUNTIME_DVC_SCALE
    real_T           *para5 = mxGetPr(ssGetSFcnParam(S,4));
#endif
    float             dc_energy_error_v;
    float             dc_energy_control_error_v;
    float             p_energy_unsat;
    float             p_energy_correction;
    float             msc_ff_ramp;
    float             active_power_command_w;
    float             msc_ff_power_target_w;
    float             lvrt_power_w;
    float             lvrt_blend;
    float             msc_pcc_ff_input_w;
    float             msc_pcc_ff_alpha;
    float             msc_ff_speed_scale;
    float             dc_voltage_ref_v;

    //=====================================================================
    if(ControlTimerFlag==1)
    {
        ControlTimerFlag=0;
        Ts_control  = svpwm1.Val.PwmVecterPeriod;
        system_Time = Input(15);
        if (system_Time < 1.5f*(float)Ts_control)
            legacy_msc_pcc_ff_filter_w = 0.0f;
        //===MOTOR============================================
        motor.Ts                       = Ts_control;
        /* Static mask reference remains the safe fallback.  The model feeds
         * Input(19) with this same 1500-V value when the profile is disabled. */
        dc_voltage_ref_v = para4[0];
        if (GSI_DYNAMIC_VDC_REF_INPUT_ENABLE)
        {
            dc_voltage_ref_v = Input(19);
            if (dc_voltage_ref_v < 1.0f)
                dc_voltage_ref_v = para4[0];
        }
        dc_energy_error_v = Input(3) - dc_voltage_ref_v;
        dc_energy_control_error_v = 0.0f;
        if (dc_energy_error_v > GSI_DC_ENERGY_DEADBAND_V)
            dc_energy_control_error_v =
                dc_energy_error_v - GSI_DC_ENERGY_DEADBAND_V;
        else if (dc_energy_error_v < -GSI_DC_ENERGY_DEADBAND_V)
            dc_energy_control_error_v =
                dc_energy_error_v + GSI_DC_ENERGY_DEADBAND_V;
        p_energy_unsat = GSI_DC_ENERGY_KP_W_PER_V *
            dc_energy_control_error_v +
            legacy_dc_energy_integral_w;
        p_energy_correction = p_energy_unsat;
        if (p_energy_correction > GSI_DC_ENERGY_POWER_LIMIT_W)
            p_energy_correction = GSI_DC_ENERGY_POWER_LIMIT_W;
        if (p_energy_correction < -GSI_DC_ENERGY_POWER_LIMIT_W)
            p_energy_correction = -GSI_DC_ENERGY_POWER_LIMIT_W;
        if (GSI_DC_ENERGY_LVRT_ONLY && !legacy_lvrt_active)
            p_energy_correction = 0.0f;
        /*
         * Conditional-integration anti-windup.  The integral is enabled only
         * after the coordinated active-power/aerodynamic startup ramp.  When
         * saturated, integrate only if the voltage error drives the output
         * back toward the linear region.
         */
        if (Input(15) >= GSI_DC_ENERGY_INTEGRAL_ENABLE_TIME_S &&
            GSI_DC_ENERGY_KI_W_PER_VS != 0.0f && Ts_control > 0.0 &&
            ((p_energy_unsat <= GSI_DC_ENERGY_POWER_LIMIT_W &&
              p_energy_unsat >= -GSI_DC_ENERGY_POWER_LIMIT_W) ||
             (p_energy_unsat > GSI_DC_ENERGY_POWER_LIMIT_W &&
              dc_energy_control_error_v < 0.0f) ||
             (p_energy_unsat < -GSI_DC_ENERGY_POWER_LIMIT_W &&
              dc_energy_control_error_v > 0.0f)))
        {
            legacy_dc_energy_integral_w +=
                GSI_DC_ENERGY_KI_W_PER_VS *
                dc_energy_control_error_v * Ts_control;
            if (legacy_dc_energy_integral_w > GSI_DC_ENERGY_POWER_LIMIT_W)
                legacy_dc_energy_integral_w = GSI_DC_ENERGY_POWER_LIMIT_W;
            if (legacy_dc_energy_integral_w < -GSI_DC_ENERGY_POWER_LIMIT_W)
                legacy_dc_energy_integral_w = -GSI_DC_ENERGY_POWER_LIMIT_W;
        }
        active_power_command_w = para1[0];
        if (GSI_DYNAMIC_PREF_INPUT_ENABLE)
        {
            active_power_command_w = Input(17);
            if (active_power_command_w < 0.0f)
                active_power_command_w = 0.0f;
            if (active_power_command_w > para1[0])
                active_power_command_w = para1[0];
        }
        if (Input(15) >= GSI_PREF_DISTURBANCE_START_S &&
            Input(15) < GSI_PREF_DISTURBANCE_END_S)
        {
            active_power_command_w += GSI_PREF_DISTURBANCE_DELTA_W;
        }
        /*
         * Latched LVRT state.  A DC-link overvoltage enters the state after
         * cold start.  The state remains active through the subsequent DC
         * undervoltage/oscillation and exits only after both DC voltage and
         * exported active power have recovered for a continuous hold time.
         */
        if (MSC_LVRT_PCC_FILTER_TAU_S > 0.0f)
        {
            float lvrt_filter_alpha = (float)Ts_control /
                (MSC_LVRT_PCC_FILTER_TAU_S + (float)Ts_control);
            legacy_lvrt_pcc_ud_filter_v += lvrt_filter_alpha *
                (grid_side.val.pcc_u_d - legacy_lvrt_pcc_ud_filter_v);
            legacy_lvrt_pcc_uq_filter_v += lvrt_filter_alpha *
                (grid_side.val.pcc_u_q - legacy_lvrt_pcc_uq_filter_v);
        }
        else
        {
            legacy_lvrt_pcc_ud_filter_v = grid_side.val.pcc_u_d;
            legacy_lvrt_pcc_uq_filter_v = grid_side.val.pcc_u_q;
        }
        if (system_Time < MSC_LVRT_ENABLE_TIME_S)
        {
            legacy_lvrt_active = 0;
            legacy_lvrt_recovery_timer_s = 0.0f;
        }
        else if (!legacy_lvrt_active &&
                 (Input(3) > MSC_LVRT_UDC_THRESHOLD_V ||
                  (MSC_LVRT_PCC_VOLTAGE_THRESHOLD_V > 0.0f &&
                   sqrtf(legacy_lvrt_pcc_ud_filter_v*
                             legacy_lvrt_pcc_ud_filter_v +
                         legacy_lvrt_pcc_uq_filter_v*
                             legacy_lvrt_pcc_uq_filter_v) <
                       MSC_LVRT_PCC_VOLTAGE_THRESHOLD_V)))
        {
            legacy_lvrt_active = 1;
            legacy_lvrt_recovery_timer_s = 0.0f;
        }
        else if (legacy_lvrt_active)
        {
            if (Input(3) >= MSC_LVRT_RECOVERY_UDC_LOW_V &&
                Input(3) <= MSC_LVRT_RECOVERY_UDC_HIGH_V &&
                grid_side.val.pcc_P_active_Power_filter >=
                    MSC_LVRT_RECOVERY_POWER_FRACTION *
                    active_power_command_w)
            {
                legacy_lvrt_recovery_timer_s += (float)Ts_control;
                if (legacy_lvrt_recovery_timer_s >= MSC_LVRT_RECOVERY_HOLD_S)
                {
                    legacy_lvrt_active = 0;
                    legacy_lvrt_recovery_timer_s = 0.0f;
                }
            }
            else
            {
                legacy_lvrt_recovery_timer_s = 0.0f;
            }
        }
        grid_side.ref.P_active_power_ref =
            active_power_command_w + p_energy_correction;
        grid_side.ref.Q_reactive_power_ref   = para2[0];
        grid_side.ref.voltage_ref            = para3[0];   
        motor.ref.voltage_ref                = dc_voltage_ref_v;
        /*
         * Rated-power current feedforward for the MSC-DVC path.  Apply the
         * same cold-start slope as the GSC active-power command so generator
         * torque is not stepped in before aerodynamic torque is available.
         */
        msc_ff_ramp = (Input(15) - PRESYN_SWITCH_TIME) *
            GSI_PREF_RAMP_SLOPE;
        if (msc_ff_ramp < 0.0f) msc_ff_ramp = 0.0f;
        if (msc_ff_ramp > active_power_command_w)
            msc_ff_ramp = active_power_command_w;
        msc_ff_power_target_w = msc_ff_ramp;
        /* Type-c DC-link control: balance the measured grid-side output
         * power at the machine side.  The one-control-step delay is
         * intentional and avoids an algebraic loop. */
        if (MSC_TYPEC_USE_PCC_POWER_FF &&
            system_Time >= PRESYN_SWITCH_TIME)
        {
            msc_pcc_ff_input_w = grid_side.val.pcc_P_active_Power;
            if (msc_pcc_ff_input_w < 0.0f)
                msc_pcc_ff_input_w = 0.0f;
            if (msc_pcc_ff_input_w > active_power_command_w)
                msc_pcc_ff_input_w = active_power_command_w;
            msc_pcc_ff_alpha = (float)Ts_control /
                ((1.0f/(6.28318530718f*MSC_TYPEC_PCC_FF_CUTOFF_HZ)) +
                 (float)Ts_control);
            legacy_msc_pcc_ff_filter_w += msc_pcc_ff_alpha *
                (msc_pcc_ff_input_w - legacy_msc_pcc_ff_filter_w);
            msc_ff_power_target_w = legacy_msc_pcc_ff_filter_w;
            if (msc_ff_power_target_w < 0.0f)
                msc_ff_power_target_w = 0.0f;
            if (msc_ff_power_target_w > active_power_command_w)
                msc_ff_power_target_w = active_power_command_w;
        }
        if (legacy_lvrt_active)
        {
            if (MSC_LVRT_UDC_POWER_GAIN_W_PER_V > 0.0f)
            {
                lvrt_power_w = msc_ff_ramp;
                if (Input(3) > dc_voltage_ref_v)
                {
                    lvrt_power_w -= MSC_LVRT_UDC_POWER_GAIN_W_PER_V *
                        (Input(3) - dc_voltage_ref_v);
                }
            }
            else
            {
                lvrt_power_w = grid_side.val.pcc_P_active_Power_filter;
            }
            if (lvrt_power_w < MSC_LVRT_MIN_POWER_W)
                lvrt_power_w = MSC_LVRT_MIN_POWER_W;
            if (lvrt_power_w > active_power_command_w)
                lvrt_power_w = active_power_command_w;
            msc_ff_power_target_w = lvrt_power_w;
        }
        msc_ff_speed_scale = 1.0f;
        if (MSC_TYPEC_SPEED_NORMALIZE)
        {
            /* Type-c realization: P_o -> T_e*=P_o/omega_g -> iq*.
             * Input(4) is Mechanical.we from the custom PMSM mechanical
             * state bus.  Normalizing the rated A/W coefficient by the
             * rated/current mechanical speed implements P_o/omega_g. */
            float speed_abs = (float)fabs(Input(4));
            if (speed_abs < MSC_TYPEC_MIN_ELEC_OMEGA_RADPS)
                speed_abs = MSC_TYPEC_MIN_ELEC_OMEGA_RADPS;
            msc_ff_speed_scale =
                MSC_TYPEC_RATED_ELEC_OMEGA_RADPS / speed_abs;
        }
        legacy_msc_iq_ff_a = MSC_POWER_FF_A_PER_W *
            msc_ff_power_target_w * msc_ff_speed_scale;
        
        motor.bak.Ia1                  = Input(0);
        motor.bak.Ib1                  = Input(1);
        motor.bak.Ic1                  = Input(2);
        motor.bak.Udc1                 = Input(3);   
        motor.bak.We                   = Input(4);
        motor.bak.RotorPos             = Input(5);
        legacy_omega_rel_ad            = Input(16);
        legacy_msc_ad_scale            = Input(18);
        if (legacy_msc_ad_scale < 0.0f) legacy_msc_ad_scale = 0.0f;
        if (legacy_msc_ad_scale > 2.0f) legacy_msc_ad_scale = 2.0f;
#if LEGACY_RUNTIME_DVC_SCALE
        {
            float dvc_gain_scale = (float)para5[0];
            if (dvc_gain_scale < 0.05f) dvc_gain_scale = 0.05f;
            if (dvc_gain_scale > 5.0f) dvc_gain_scale = 5.0f;
            motor.pwm_speed_pi.Kp = MOTOR_PWM_SPEED_KP * dvc_gain_scale;
            motor.pwm_speed_pi.Ki = MOTOR_PWM_SPEED_KI * dvc_gain_scale;
        }
#endif
        motor.control(&motor); 
        //*********SVPWM************************
		svpwm1.Ref.U_alfa   = motor.out.Us_alfa;
		svpwm1.Ref.U_beta   = motor.out.Us_beta;
		svpwm1.Par.PWMSwitchFrequency = PWM_SWITCH_FREQUENCY_HZ;
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
            svpwm2.Par.PWMSwitchFrequency = PWM_SWITCH_FREQUENCY_HZ;
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
        ControlTimerPeriod = (int)(svpwm1.Val.PwmVecterPeriod*1e6 /
                                    LEGACY_TIMER_TICK_US);
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
    OutPut[22] = grid_side.val.Id_ref;
    OutPut[23] = grid_side.val.Id;
    OutPut[24] = grid_side.pf.U_oq_ref;
    OutPut[25] = grid_side.val.pcc_u_q;  
    OutPut[26] = grid_side.val.Iq_ref;   
    OutPut[27] = grid_side.val.grid_phase_angle;
    
    OutPut[28] = grid_side.val.Pre_syn;  
    OutPut[29] = grid_side.val.Iq;
    OutPut[30] = motor.val.Iq_ref;
    OutPut[31] = motor.pwm_speed_pi.Out;
    OutPut[32] = motor.bak.Iq1;
    OutPut[33] = motor.val.Ud1_ref;
    OutPut[34] = motor.val.Uq1_ref;
    OutPut[35] = sqrt(motor.val.Ud1_ref*motor.val.Ud1_ref +
        motor.val.Uq1_ref*motor.val.Uq1_ref);
    OutPut[36] = 1.5f*OutPut[35] /
        ((motor.bak.Udc1 > 1.0f) ? motor.bak.Udc1 : 1.0f);

#if IDEAL_AVG_OUTPUTS
    /*
     * Continuous alpha-beta voltage commands for the ideal-average EMT
     * validation copy.  They are exported before the SVPWM timing/gate
     * reconstruction and therefore isolate switching ripple without changing
     * any physical EMT MEX interface when IDEAL_AVG_OUTPUTS is disabled.
     */
    OutPut[37] = motor.out.Us_alfa;
    OutPut[38] = motor.out.Us_beta;
    OutPut[39] = grid_side.out.Us_alfa;
    OutPut[40] = grid_side.out.Us_beta;
#endif
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
        "ControlTimerCounter", "ControlTimerPeriod", "ControlTimerEnable",
        "ControlTimerFlag", "ControlTimerIntruptIndex", "Ts_control",
        "PWMSegmentNumber", "vector", "vector1", "system_Time",
        "svpwm1", "svpwm2", "motor", "legacy_omega_rel_ad",
        "legacy_msc_ad_scale", "legacy_dc_energy_integral_w",
        "legacy_msc_iq_ff_a", "legacy_msc_pcc_ff_filter_w",
        "legacy_lvrt_active", "legacy_lvrt_recovery_timer_s",
        "legacy_lvrt_pcc_ud_filter_v", "legacy_lvrt_pcc_uq_filter_v",
        "vloop_slope", "d_loop_pi", "q_loop_pi", "d_voltage_loop_pi",
        "q_voltage_loop_pi", "PLL_loop_pi", "E_voltage_loop_pi",
        "power_loop_pi", "clack_trans", "park_u", "park_PLL", "grid_side",
        "lpf", "lpf1", "hpf", "bandpf", "w_vsg_state",
        "w_vsg_sync_anchor"
    };
    mxArray *snapshot = mxCreateStructMatrix(1, 1, 48, fields);
    float w_vsg_state = grid_side_get_w_vsg_state();
    float w_vsg_sync_anchor = grid_side_get_w_vsg_sync_anchor();
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
    SNAPSHOT_FIELD(legacy_omega_rel_ad);
    SNAPSHOT_FIELD(legacy_msc_ad_scale);
    SNAPSHOT_FIELD(legacy_dc_energy_integral_w);
    SNAPSHOT_FIELD(legacy_msc_iq_ff_a);
    SNAPSHOT_FIELD(legacy_msc_pcc_ff_filter_w);
    SNAPSHOT_FIELD(legacy_lvrt_active);
    SNAPSHOT_FIELD(legacy_lvrt_recovery_timer_s);
    SNAPSHOT_FIELD(legacy_lvrt_pcc_ud_filter_v);
    SNAPSHOT_FIELD(legacy_lvrt_pcc_uq_filter_v);
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
    SNAPSHOT_FIELD(w_vsg_sync_anchor);
#undef SNAPSHOT_FIELD
    return snapshot;
}

static void mdlSetOperatingPoint(SimStruct *S, const mxArray *snapshot)
{
    float w_vsg_state;
    float w_vsg_sync_anchor;

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
    RESTORE_FIELD(legacy_omega_rel_ad);
    RESTORE_FIELD(legacy_msc_ad_scale);
    RESTORE_FIELD(legacy_dc_energy_integral_w);
    RESTORE_FIELD(legacy_msc_iq_ff_a);
    RESTORE_FIELD(legacy_msc_pcc_ff_filter_w);
    RESTORE_FIELD(legacy_lvrt_active);
    RESTORE_FIELD(legacy_lvrt_recovery_timer_s);
    RESTORE_FIELD(legacy_lvrt_pcc_ud_filter_v);
    RESTORE_FIELD(legacy_lvrt_pcc_uq_filter_v);
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
    RESTORE_FIELD(w_vsg_sync_anchor);
#undef RESTORE_FIELD
    grid_side_set_w_vsg_state(w_vsg_state);
    grid_side_set_w_vsg_sync_anchor(w_vsg_sync_anchor);
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









