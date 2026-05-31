#ifndef SCHEMEA_TUNING_OVERRIDES_H
#define SCHEMEA_TUNING_OVERRIDES_H

/* Scheme A: map small-signal tuned parameters to nonlinear C controller.
 * This header is intentionally injected only in .c compile units so that:
 * 1) original legacy headers remain untouched;
 * 2) tuning can be rolled back by removing one include.
 */

/* ---------- motorcontrol.h overrides ---------- */
#ifdef MOTOR_ID_KP
#undef MOTOR_ID_KP
#endif
#define MOTOR_ID_KP 1.7f

#ifdef MOTOR_ID_KI
#undef MOTOR_ID_KI
#endif
#define MOTOR_ID_KI 0.0050833f

#ifdef MOTOR_IQ_KP
#undef MOTOR_IQ_KP
#endif
#define MOTOR_IQ_KP MOTOR_ID_KP

#ifdef MOTOR_IQ_KI
#undef MOTOR_IQ_KI
#endif
#define MOTOR_IQ_KI MOTOR_ID_KI

/* keep machine dc-speed loop at legacy values for no-disturbance convergence */

/* ---------- grid_forming_control.h overrides ---------- */
#ifdef CURRENT_ID_KP
#undef CURRENT_ID_KP
#endif
#define CURRENT_ID_KP 0.23333f

#ifdef CURRENT_ID_KI
#undef CURRENT_ID_KI
#endif
#define CURRENT_ID_KI 0.0099188f

#ifdef CURRENT_IQ_KP
#undef CURRENT_IQ_KP
#endif
#define CURRENT_IQ_KP CURRENT_ID_KP

#ifdef CURRENT_IQ_KI
#undef CURRENT_IQ_KI
#endif
#define CURRENT_IQ_KI CURRENT_ID_KI

/* IMPORTANT:
 * The effective grid-side d/q voltage PI in current code path uses
 * V_LOOP_KP/V_LOOP_KI (from motorcontrol.h), not GSI_V_LOOP_KP/KI.
 */
#ifdef V_LOOP_KP
#undef V_LOOP_KP
#endif
#define V_LOOP_KP 0.668f

#ifdef V_LOOP_KI
#undef V_LOOP_KI
#endif
#define V_LOOP_KI 0.4008f

/* Keep legacy symbols aligned for readability/debug logs. */
#ifdef GSI_V_LOOP_KP
#undef GSI_V_LOOP_KP
#endif
#define GSI_V_LOOP_KP V_LOOP_KP

#ifdef GSI_V_LOOP_KI
#undef GSI_V_LOOP_KI
#endif
#define GSI_V_LOOP_KI V_LOOP_KI

/* Optional strict VSG-equivalent frequency loop switch.
 * Enable after no-disturbance baseline is stable.
 */
#ifdef ENABLE_VSG_EQUIV_WREF
#undef ENABLE_VSG_EQUIV_WREF
#endif
#define ENABLE_VSG_EQUIV_WREF 1

#ifdef VSG_EQUIV_W0
#undef VSG_EQUIV_W0
#endif
#define VSG_EQUIV_W0 314.0f

#ifdef VSG_EQUIV_H
#undef VSG_EQUIV_H
#endif
#define VSG_EQUIV_H 10.0f

#ifdef VSG_EQUIV_MP
#undef VSG_EQUIV_MP
#endif
#define VSG_EQUIV_MP 1.57e-6f

/* Stage-1 Scheme A: keep legacy active-power loop and VSG shell as-is.
 * We only map current/voltage inner-loop gains first, then tune P-f/VSG
 * in a dedicated stability pass once no-disturbance steady-state is stable.
 */

#endif /* SCHEMEA_TUNING_OVERRIDES_H */
