# Mumbai 2044 Guardian Point-Defense: Predictive Turret Elevation Control

ME319 Control Systems (Autumn 2026), Assignment 2, IIT Bombay.
Predictive-feedback fire control for a laser turret elevation axis: a variable-step
6-state constant-acceleration Kalman filter, a 0.5 s position extrapolation, and a
delay-matched type-1 (P) compensator (Kp = 2000 V/rad, ramp lag = t_p) plus a firing supervisor.

## Repository layout
| Path | Contents |
|---|---|
| `src/turret_controller.m` | Final controller: Kalman filter, prediction, P compensator, firing logic |
| `src/turret_plant.m` | Turret/motor plant (assignment interface) |
| `src/target_motion_generator.m` | Standard scenarios A, B, C (assignment interface) |
| `sim/simulate_engagement.m` | RK4 closed-loop simulation and pulse/hit bookkeeping |
| `sim/mc_params.m`, `mc_kin.m`, `mc_target.m` | Random admissible-target generator for Monte Carlo |
| `sim/percentile_nearest.m` | Toolbox-free percentile helper |
| `scripts/run_analysis_and_sim.m` | Stability margins + scenarios A/B/C, saves plots |
| `scripts/monte_carlo.m` | 200-case generalization study |
| `results/` | Generated figures and Monte Carlo data |
| `report/report.tex` | IEEE-format paper source (uses figures from `results/`) |

## Reproducing the results
Requires MATLAB (Control System Toolbox for `margin`/`feedback`; optional) or GNU Octave.
```matlab
run('scripts/run_analysis_and_sim.m')   % scenarios A, B, C; writes results/scenario_*.png
run('scripts/monte_carlo.m')            % 200 random cases; writes results/mc_hist.png, mc_results.mat
```
Octave: `octave --eval "run('scripts/run_analysis_and_sim.m')"` (frequency-domain part is skipped if
the `control` package is not loaded; use `pkg load control` first).

Random seeds are fixed (seed 42 for scenarios; case index for Monte Carlo), so results are repeatable.
The turret starts at 0 deg (not aligned with the target) in all simulations.

## Design parameters (single design, never retuned per scenario)
`Kp = 2000 V/rad`, `t_p = 0.5 s`, Kalman `q = 50 m^2/s^5`, `P0 = diag(4, 75^2, 15^2)` per axis,
measurement gate `Ts = 50 ms`, fire gate 1.0 m, minimum 8 fused measurements, recharge 1.0 s.

## Reported results (Octave, seed 42)
| Scenario | Pulses | Hits | First hit [s] | Min miss [m] |
|---|---|---|---|---|
| A | 7 | 7 | 3.16 | 0.23 |
| B | 6 | 6 | 3.50 | 0.21 |
| C | 6 | 6 | 3.81 | 0.26 |

Monte Carlo (200 cases): 100 % interception, median first hit 3.65 s, median best miss 0.13 m.
Loop analysis for L(s) = 2000 Gp(s): crossover 2.00 rad/s, PM 89.6 deg, GM 14.4 dB.

## Building the paper
```
cd report && pdflatex report.tex && pdflatex report.tex
```
Instructor evaluation results are to be added once the evaluation script is released.

## License
MIT (see `LICENSE`).
