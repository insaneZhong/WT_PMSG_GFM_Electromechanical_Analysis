# Deferred structural ideas

Do not implement these changes during the current non-structural tuning pass.

1. Add explicit anti-windup handling dedicated to the MSC DC-voltage PI loop.
   Reason: the shared PI implementation can accumulate integral action near the current-reference limit.
2. Add an optional damping-control branch driven by shaft relative speed after blind FFT/PSD identification.
   Reason: physical shaft damping is low, so control-parameter tuning alone may not remove the electromechanical mode.
3. Keep the 1200 V DC-link target and verify the plant voltage conversion ratio.
   Reason: the 690 V line-voltage object needs a DC-link voltage above the converter modulation margin. The observed 900-940 V region is a post-startup voltage drop, not a valid nominal operating point.

## 2026-06-02: Startup coordination required before further parameter scans

After restoring the capacitor initial voltage and the MSC-DVC reference to
1200 V, parameter-only ramp scans exposed a startup coordination problem:

- 2.0 MW/s: DC-link voltage drops rapidly after the 0.5 s presynchronization switch.
- 0.5 MW/s: MSC-DVC integral action accumulates before active-power export is established, causing DC overvoltage, current saturation, and power overshoot.
- 1.5 MW/s: bounded at 10 s but unstable by 30 s.

Minimal structural change for user approval:

1. Add bumpless MSC-DVC startup coordination around the existing loop.
2. During presynchronization, hold or initialize the MSC-DVC integrator.
3. After breaker closure, enable MSC torque production with a staged ramp tied to the GSC active-power ramp.
4. Keep the existing feedback path direct. Do not add a feedback filter.
5. Optionally add a torque or current feedforward term derived from active-power demand, while retaining the PI loop as the correction term.
