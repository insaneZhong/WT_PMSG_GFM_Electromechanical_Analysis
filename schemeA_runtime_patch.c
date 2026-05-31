#include "motorcontrol.h"
#include "grid_forming_control.h"

/* Globals defined in existing source files */
extern MOTOR motor;
extern MOTOR_PI d_loop_pi;
extern MOTOR_PI q_loop_pi;
extern MOTOR_PI d_voltage_loop_pi;
extern MOTOR_PI q_voltage_loop_pi;
extern MOTOR_PI power_loop_pi;
extern GRID_SIDE_INV grid_side;

static void apply_schemeA_runtime_patch(void)
{
    /* MSC current + dc-speed loop */
    motor.id_pi.Kp = 1.7f;
    motor.id_pi.Ki = 0.0050833f;
    motor.iq_pi.Kp = 1.7f;
    motor.iq_pi.Ki = 0.0050833f;
    motor.pwm_speed_pi.Kp = 0.18f;
    motor.pwm_speed_pi.Ki = 0.0018f;

    /* GSC current loop */
    d_loop_pi.Kp = 0.23333f;
    d_loop_pi.Ki = 0.0099188f;
    q_loop_pi.Kp = 0.23333f;
    q_loop_pi.Ki = 0.0099188f;

    /* GSC voltage loop */
    d_voltage_loop_pi.Kp = 0.668f;
    d_voltage_loop_pi.Ki = 0.4008f;
    q_voltage_loop_pi.Kp = 0.668f;
    q_voltage_loop_pi.Ki = 0.4008f;

    /* Active power loop */
    power_loop_pi.Kp = 1.9792f;
    power_loop_pi.Ki = 0.00022207f;
    power_loop_pi.OutMax = 20.0f;
    power_loop_pi.OutMin = -20.0f;

    /* VSG-like defaults in existing structure */
    grid_side.pf.J_virtual = 50.0f;
    grid_side.pf.Damping_coeff = 8000.0f;
    grid_side.pf.kp_voltage = 0.02f;
    grid_side.pf.ki_voltage = 0.3f;
}

#if defined(__GNUC__)
__attribute__((constructor))
static void schemeA_ctor(void)
{
    apply_schemeA_runtime_patch();
}
#endif

