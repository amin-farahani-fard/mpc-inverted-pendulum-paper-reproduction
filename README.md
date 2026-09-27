# Multirate MPC Paper Reproduction for an Inverted-Pendulum Robot

A MATLAB reproduction of the control architecture and simulation results in:

> T. Ohhira and A. Shimada, "Model Predictive Control for an
> Inverted-pendulum Robot with Time-varying Constraints," IFAC-PapersOnLine,
> vol. 50, no. 1, pp. 776-781, 2017.

[Original paper DOI: 10.1016/j.ifacol.2017.08.252](https://doi.org/10.1016/j.ifacol.2017.08.252)

This project was completed for a university Model Predictive Control (MPC)
course. It reconstructs the paper's six-state robot model, fast LQ stabilization
loop, slower constrained MPC loop, position-dependent velocity limits, and
motor-torque constraints.

## Reproduction Summary

The paper does not publish every numerical matrix, solver setting, reference
signal, or raw data point. This implementation therefore combines published
parameters with documented engineering assumptions. It reproduces the main
distance and velocity behavior closely and achieves qualitative agreement for
the short direction, tilt, and torque transients.

| Metric | Reproduced result |
| --- | ---: |
| Simulation time | 60 s |
| Target distance | 100 m |
| Final distance | 99.969 m |
| Maximum linear velocity | 3.647 m/s |
| Maximum absolute motor torque | 1.09 N m |
| Fast LQ sample time | 0.01 s (100 Hz) |
| MPC sample time | 0.08 s (12.5 Hz) |
| Prediction horizon | 10 |
| Control horizon | 5 |


## Implemented Features

- Six-state linear model:
  - displacement
  - heading angle
  - body tilt angle
  - linear velocity
  - yaw rate
  - tilt rate
- Two wheel-torque inputs
- Discrete LQ stabilization at the fast sample rate
- Exact multirate lifting from 0.01 s to 0.08 s
- Incremental-input MPC formulation
- Position-dependent velocity limits across five distance zones
- Hard bounds on the combined LQ/MPC motor torque
- Tilt and velocity constraints across the prediction horizon
- Separate state slacks for tilt and velocity
- Emergency soft-constraint fallback
- Final fallback that preserves the hard motor-torque constraint
- Reproduction plots corresponding to the paper's displacement, direction,
  tilt, velocity, and control-input figures

## Repository Structure

```text
.
|-- run_reproduction.m
|-- src/
|   |-- build_robot_model.m
|   |-- make_reference_horizon.m
|   |-- paper_parameters.m
|   |-- plot_paper_figures.m
|   |-- prediction_matrices.m
|   |-- simulate_paper_case.m
|   `-- solve_multirate_mpc.m
`-- docs/
    `-- MPC_Paper_Reproduction_Report_FA.pdf
```

Generated figures and `simulation_result.mat` are written to `output/`, which is
excluded from version control.

## Requirements

- MATLAB R2020a or newer
- Control System Toolbox
- Optimization Toolbox

The implementation uses `ss`, `c2d`, `dlqr`, `quadprog`, and
`exportgraphics`.

## Running the Reproduction

From the repository root in MATLAB:

```matlab
run_reproduction
```

The script:

1. loads the paper parameters;
2. constructs and discretizes the robot model;
3. designs the digital LQ feedback gain;
4. runs the multirate constrained MPC simulation;
5. saves the numerical result to `output/simulation_result.mat`;
6. exports publication-style PNG and MATLAB figure files.

## Time-Varying Constraints

The active velocity limit changes with the robot's distance:

| Distance interval (m) | Velocity limit (m/s) |
| --- | ---: |
| 0-8 | 1.1 |
| 8-20 | 3.3 |
| 20-35 | 2.2 |
| 35-60 | 3.3 |
| 60+ | 2.7 |

Additional limits include:

- body tilt: +/-30 degrees;
- actual wheel torque: +/-1.09 N m;
- MPC input increment: +/-2.18.

## Reproduction Limitations

- The original paper provides no raw simulation data.
- Some final controller matrices, solver tolerances, and reference details are
  not published.
- The narrow direction and tilt pulses were reconstructed from the published
  figures and are not uniquely identifiable.
- A reported inconsistency around the fourth velocity zone required an explicit
  interpretation.
- The model is linearized around the upright equilibrium and does not include
  sensor noise, wheel slip, road slope, actuator dynamics, or payload changes.
- This is a simulation reproduction, not hardware validation.

These limitations and possible robust-MPC extensions are discussed in detail in
the Persian report.

## Documentation

The complete Persian analysis and reproduction report is available at
[`docs/MPC_Paper_Reproduction_Report_FA.pdf`](docs/MPC_Paper_Reproduction_Report_FA.pdf).

The publisher's article PDF and third-party reference PDFs are intentionally not
redistributed in this repository. Use the DOI link above to access the original
publication through its official source.

## Academic Context

Course: Model Predictive Control (MPC)  
Instructor: Dr. Kamal Hosseini Sani  
Student: Amin Farahani Fard

This is an independent educational reproduction. It is not affiliated with or
endorsed by the original paper's authors, IFAC, or Elsevier.

