# Recovery, cleanup, and validation

## Recovered sources

The local download was identical to upstream `main` at `6ee36ed`: course starter files rather than the team's completed work. All remote branches were inspected before selecting the deliverables.

| Part | Selected upstream source |
|---|---|
| Dynamics and linearization | `niklas_6`, `6e28ff9`, `deliverable_1` / `deliverable_2` |
| Linear tracking | `Jessie`, `85c1959`, `car_project` |
| Offset-free controller and estimator | `Jessie`, `85c1959`, `deliverable_4_1` |
| Latest tube MPC | `Jessie`, `85c1959`, `deliverable_5_1` |
| Additional tube experiments and saved figures | `Mariem`, `81dcfab`, `car_project` |
| Nonlinear tracking | `niklas_6`, `6e28ff9`, `Deliverable6_1` |
| Nonlinear avoidance | `niklas_6`, `6e28ff9`, `Deliverable_6_2` |

Full commit hashes, original paths, and SHA-256 file hashes are in `source-manifest.json`. ZIP snapshots preserve each branch's tracked files, excluding macOS metadata. The initially downloaded MATLAB files remain unchanged in `archive/course-starter`.

## Corrections to recovered code

- Shared copies of `Car`, `simulate`, `visualization`, and `MpcControlBase` are consolidated in `car_project/common`.
- The dynamics example defines `Ts`, uses a column input vector, and corrects `params.mxCar.u` to `params.myCar.u`.
- The linearization example defines `Ts` and removes an unused input assignment.
- Longitudinal tracking costs use `X(2,i)` consistently, correcting the original mixed `i` / `1` time index.
- Linear controllers select Gurobi when available, with `quadprog` as a fallback.
- Matrix structures replace Control Toolbox state-space objects. The original matrices, metadata, decomposition, and exact matrix-exponential discretization are preserved.
- `project_dlqr` solves the same Riccati equation by iteration if `dlqr` is unavailable. The observer uses the analytic two-state gain for the original poles `[0.5, 0.6]`.
- A small MPT3 adapter calls modern `linprog` without its removed initial-guess argument.
- The NMPC tracking and avoidance controllers use `car.Ts` for RK4 instead of a hardcoded integration period.
- The avoidance controller allocates `N` states and `N-1` inputs, matching the dynamics loop. Its original extra terminal state/input were not connected to the vehicle dynamics. The lead projection and ellipse constraints now use the same `N` state samples.
- IPOPT bound relaxation is disabled. The simulator accepts at most `1e-7` actuator-bound roundoff and removes that roundoff before integration; larger violations still abort. Nonfinite ego controls also abort.
- Empty parameter cells in deliverable scripts are replaced with explicit structs. Template banners are removed.
- The launcher selects one controller folder at a time, avoiding same-name class conflicts.

The nonlinear objectives, 10-second prediction horizon, warm starts, constraint design, vehicle dynamics, and lead-car scenarios remain based on the team's implementation. The main replay contains results from those recovered controllers, not an independently written replacement.

## Verified environment and results

Tests were run with MATLAB R2025b Update 5 on Apple Silicon, Optimization Toolbox, CasADi 3.7.2, YALMIP commit `b06a22fe619bab2a7eeae78bf2140dbad8c2041c`, and MPT3 commit `76d28640251c0403f2c11deac9b0b2d7ee3759d0`. Control System Toolbox was not installed; the portability helpers were therefore exercised.

All six MATLAB tests pass. All four showcased scenarios reach 15 s. Nonlinear tracking reaches the 3 m / 100 km/h reference; nonlinear avoidance overtakes the lead vehicle while satisfying the sampled ellipse condition. Linear tracking reaches 3 m / 120 km/h; offset-free tracking reaches approximately 3 m / 50 km/h.

The replay harness checks finite data, time/sample consistency, sampled constraints and ellipse separation, playback, reset, seek, scenario switching, and drawing coordinates. It is not a real browser test. Static previews were rendered and visually inspected. The interactive replay is responsive by design, but a real browser UI could not be inspected because the browser host was unavailable.

## Part 5 limitations

The latest tube MPC work is preserved as experimental source. In the recovered controller, the relative state and nominal/error states do not have a complete consistent coupling. The Mariem version references unloaded gains/terminal weights and a missing saved-data file; the latest Jessie version supplies a saved-data file but still has model/feedback sign and state-linking issues. The set-generation scripts also use inconsistent saved variable interfaces. Automatically labeling this as working robust MPC would misrepresent its validation status.

The saved figures `E1.fig`, `U_tightened.fig`, and `X_tightened.fig` are retained from Mariem's branch. They are historical artifacts, not proof that the current controller is correct. The original Part 5 scripts can be investigated after selecting the folder with `setup_project('tube-mpc')`; they are not called by the main launcher.

## Scope of claims

Constraint checks concern sampled states and actuator values for the included scenarios. They do not establish continuous-time collision clearance, global solver convergence, robustness to arbitrary lead behavior, terminal-set correctness, or recursive feasibility. The nonlinear avoidance model assumes a constant-speed, constant-lane lead projection. Browser car shapes and lateral scaling are for legible playback.
