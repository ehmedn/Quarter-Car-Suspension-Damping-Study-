# Quarter-Car Suspension Damping Analysis

**Mechanical Engineering | Swansea University**

A MATLAB-based investigation into how passive suspension damping and sprung mass affect a vehicle's response to a road bump.

## Project Overview

I developed a passive two-degree-of-freedom quarter-car suspension model in MATLAB to investigate how different damping coefficients influence tyre-load variation, body acceleration and suspension travel.

The study compared:

- Five damping coefficients: 500, 1,000, 1,500, 2,000 and 3,000 N·s/m
- Two vehicle speeds: 15 m/s and 20 m/s
- Two sprung masses: 250 kg and 275 kg
- A raised-cosine road bump measuring 25 mm high and 3 m long

In total, I investigated **20 simulation cases** using MATLAB's `ode45` solver.

## Main Findings

A damping coefficient of **1,000 N·s/m** produced the lowest RMS tyre-load variation across the tested mass and speed combinations while satisfying the assumed 40 mm dynamic suspension-travel limit.

### Results at 1,000 N·s/m

| Sprung Mass | Speed | Tyre-Load RMS (%) | Body Acceleration RMS (m/s²) | Peak Travel (mm) |
|---|---|---|---|---|
| 250 kg | 15 m/s | 5.566 | 0.568 | 21.19 |
| 250 kg | 20 m/s | 7.890 | 0.622 | 24.73 |
| 275 kg | 15 m/s | 5.085 | 0.517 | 21.50 |
| 275 kg | 20 m/s | 7.208 | 0.564 | 25.00 |

Increasing damping beyond 1,000 N·s/m reduced peak suspension travel but increased tyre-load variation and body acceleration. Increasing the sprung mass reduced normalised tyre-load variation and body acceleration, although suspension travel increased slightly.

These results demonstrate the trade-offs involved in selecting a suspension damping coefficient.

## MATLAB Methodology

The model consists of a sprung mass, unsprung mass, suspension spring, damper and tyre spring.

I used two coupled equations of motion to calculate the vertical response of the vehicle when travelling over the assumed bump.

The simulations included:

- Numerical integration using MATLAB's `ode45`
- RMS tyre-load variation calculations
- RMS body acceleration calculations
- Maximum dynamic suspension-travel checks
- Minimum modelled tyre-load checks
- Sprung-mass sensitivity analysis
- Numerical consistency checks using refined solver settings

## Repository Files

- `suspension_damping_study.m` — Original 250 kg simulation
- `suspension_damping_study_mass275.m` — Increased-mass sensitivity simulation
- `results_original/` — Results, plots and numerical checks for the 250 kg model
- `results_mass275/` — Results, plots and numerical checks for the 275 kg model
- `Quarter_Car_Suspension_Report_EN.pdf` — Full engineering report containing the methodology, calculations, results and conclusions

## How to Run the Simulations

1. Download or clone this repository.
2. Open MATLAB.
3. Set the current MATLAB folder to the downloaded repository.
4. Run `suspension_damping_study.m` to simulate the 250 kg model.
5. Run `suspension_damping_study_mass275.m` to simulate the 275 kg model.
6. Review the generated results folders, CSV tables and PNG graphs.

Running the scripts regenerates the saved output files.

## Limitations and Future Work

This study uses assumed vehicle and road parameters rather than data obtained from a physical vehicle.

The model assumes linear suspension and tyre behaviour and does not account for pitch, roll, aerodynamic effects or nonlinear tyre contact.

The preferred damping coefficient is therefore specific to the conditions investigated and should not be considered a universal optimum.

Further work could involve testing different road profiles, using measured vehicle parameters and validating the model against experimental suspension data.

## References

1. [MathWorks — Control Quarter-Car Suspension Dynamics Using ADMM Solver](https://www.mathworks.com/help/mpc/ug/admm-based-mpc-control-for-quarter-car-suspension.html)
2. [MathWorks — ode45 Documentation](https://www.mathworks.com/help/matlab/ref/ode45.html)
