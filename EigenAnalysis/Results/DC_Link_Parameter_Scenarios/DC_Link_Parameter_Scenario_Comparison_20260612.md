# DC-Link Parameter Scenario Comparison

This report does not overwrite the frozen small-signal baseline. It evaluates the four topology models under alternate `Vdc` and `C_dc` values in memory.

## Scenarios

| Scenario | Vdc/V | Cdc/F | Meaning |
|---|---:|---:|---|
| `FrozenSmallSignal` | 1500 | 0.0015 | Current frozen small-signal baseline |
| `NonlinearPhysicalInit` | 1200 | 0.03 | Match nonlinear physical Cd initial voltage |
| `NonlinearValidationRef` | 1000 | 0.03 | Match current nonlinear validation VdcRef and Cd |

## Torsional Mode Metrics

| Scenario | Model | Frequency/Hz | DampingRatio | Sigma | MaxReal | Stable |
|---|---|---:|---:|---:|---:|---|
| `FrozenSmallSignal` | `GFL-WT` | 1.99762 | 0.049441 | -0.621314 | 1373.08 | 0 |
| `FrozenSmallSignal` | `GFM-GWT` | 1.99762 | 0.049441 | -0.621314 | 1298.83 | 0 |
| `FrozenSmallSignal` | `GFM-MWT` | 2.00108 | -0.0137377 | 0.172742 | 1298.83 | 0 |
| `FrozenSmallSignal` | `GFM-MWT+AD` | 1.95339 | 0.0117905 | -0.144721 | 1298.83 | 0 |
| `NonlinearPhysicalInit` | `GFL-WT` | 1.99762 | 0.049441 | -0.621314 | 1375.16 | 0 |
| `NonlinearPhysicalInit` | `GFM-GWT` | 1.99762 | 0.049441 | -0.621314 | 1298.83 | 0 |
| `NonlinearPhysicalInit` | `GFM-MWT` | 1.99993 | -0.0173169 | 0.217635 | 1298.83 | 0 |
| `NonlinearPhysicalInit` | `GFM-MWT+AD` | 1.94564 | 0.0118303 | -0.144634 | 1298.83 | 0 |
| `NonlinearValidationRef` | `GFL-WT` | 1.99762 | 0.049441 | -0.621314 | 1375.13 | 0 |
| `NonlinearValidationRef` | `GFM-GWT` | 1.99762 | 0.049441 | -0.621314 | 1298.83 | 0 |
| `NonlinearValidationRef` | `GFM-MWT` | 2.0002 | -0.0166089 | 0.208762 | 1298.83 | 0 |
| `NonlinearValidationRef` | `GFM-MWT+AD` | 1.94719 | 0.0118291 | -0.144734 | 1298.83 | 0 |

## Use

- If the relative topology ordering changes after matching nonlinear DC parameters, regenerate the frozen small-signal baseline before using it in the paper.
- If the ordering is unchanged, the old baseline can still be used as a reproducible reference, but the DC-link mismatch must be disclosed as a parameter limitation.
