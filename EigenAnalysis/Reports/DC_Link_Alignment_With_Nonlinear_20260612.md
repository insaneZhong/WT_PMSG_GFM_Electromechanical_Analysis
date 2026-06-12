# DC-Link Alignment Check With Nonlinear Model

- Nonlinear model: `D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\GFM_1MW _nonlinear\Grid_FormingVSG_PMSG.mdl`
- Motor header: `D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\GFM_1MW _nonlinear\motorcontrol.h`
- GSC header: `D:\博士工作\论文工作\（1）构网型风电机组\构网型风电机组\GFM_1MW _nonlinear\grid_forming_control_vsg.h`

## Parameter Comparison

| Parameter | Meaning | Small-signal | Nonlinear | Delta | Relative Difference | Status |
|---|---|---:|---:|---:|---:|---|
| `Vdc` | DC-link operating voltage | 1500 | 1200 | 300 | 25% | mismatch |
| `C_dc` | DC-link capacitance | 0.0015 | 0.03 | -0.0285 | -95% | mismatch |
| `C_dc_motor_macro` | DC-link capacitance in motorcontrol.h | 0.0015 | 0.03 | -0.0285 | -95% | mismatch |
| `C_dc_gsc_macro` | DC-link capacitance in grid_forming_control_vsg.h | 0.0015 | 0.03 | -0.0285 | -95% | mismatch |
| `L_d` | PMSG d-axis inductance | 0.00105 | 0.00102 | 3e-05 | 2.941% | mismatch |
| `L_q` | PMSG q-axis inductance | 0.00105 | 0.00102 | 3e-05 | 2.941% | mismatch |
| `R_s` | PMSG stator resistance | 0.0122 | 0.0122 | 0 | 0% | match |
| `n_p` | PMSG pole pairs | 20 | 20 | 0 | 0% | match |
| `psi_f` | PMSG flux linkage | 8.64 | 8.64 | 0 | 0% | match |
| `J_g` | PMSG generator inertia | 183750 | 183750 | 0 | 0% | match |

## Interpretation

- The frozen small-signal baseline is reproducible, but its DC-link voltage/capacitance should not be claimed as fully aligned with the current nonlinear validation model until mismatches are resolved.
- Nonlinear `Cd` and controller macros both use the physical DC capacitance shown in the table.
- If the nonlinear object is retained, rerun the small-signal baseline with matched `Vdc` and `C_dc` before making final cross-domain damping claims.
