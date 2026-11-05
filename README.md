# Highway Model Predictive Control

An EPFL ME-425 team project exploring cruise control, lane changes, and overtaking using a nonlinear vehicle model and model predictive control.

![Nonlinear MPC lane change](docs/assets/nonlinear-tracking.png)

## Simulations

- **Linear MPC:** lane and speed reference tracking.
- **Offset-free MPC:** speed tracking with disturbance estimation.
- **Nonlinear MPC:** simultaneous lane changes and acceleration.
- **Collision avoidance:** overtaking a slower vehicle using an elliptical separation constraint.

Download the repository and open **[docs/replay.html](docs/replay.html)** in your browser. Choose a scenario and press **Play** to view the car animation and telemetry. This offline replay displays MATLAB simulation results.

## Run in MATLAB

Install [CasADi for MATLAB](https://web.casadi.org/get/), then run from the repository root:

```matlab
addpath('/path/to/casadi');
addpath('car_project');
result = run_project('nonlinear-tracking');
```

For overtaking, use `run_project('nonlinear-avoidance')`. Press **Start** in the MATLAB visualization to play the animation.

Linear scenarios, `run_project('tracking')` and `run_project('offset-free')`, additionally require [YALMIP](https://github.com/yalmip/YALMIP), [MPT3](https://www.mpt3.org/), and MATLAB Optimization Toolbox or Gurobi.

Tested with MATLAB R2025b and CasADi 3.7.2. Avoid adding the entire repository with `genpath`; the launcher selects the appropriate controller folder.

## Structure

```text
car_project/   Vehicle model, controllers, simulation launcher, and tests
docs/          Interactive replay, simulation data, figures, and project brief
scripts/       Static figure rendering and replay checks
```

The four demo scenarios complete successfully and the six MATLAB regression tests pass. Part 5 tube MPC remains experimental and is excluded from the demos.

## Credits

Team: [Niklas Schröder](https://github.com/niklashcs), [MeriamBouguecha](https://github.com/MeriamBouguecha), and [jessiestaudenmann](https://github.com/jessiestaudenmann).

Based on the EPFL ME-425 course material and the team's [original repository](https://github.com/niklashcs/mpc_epfl).
