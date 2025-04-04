# GPC Controller for G(s) = 1/[(s+1)(s+2)]

## Overview
This folder contains a MATLAB implementation of a Generalized Predictive Control (GPC) strategy for the continuous-time system:

```
G(s) = 1 / [(s + 1)(s + 2)]
```

The system is discretized using zero-order hold (ZOH), and the GPC controller is applied to track a unit step reference signal while minimizing control effort.

## Project Includes
- A MATLAB script (`GPC.m`) implementing the predictive control algorithm.
- Simulation of the controlled system with process noise.
- Output and control signal plots for performance analysis.

## Running the File
1. Open MATLAB.
2. Navigate to the project folder.
3. Run the script `GPC.m` to simulate the system and view the results.

## Usage
The script defines the continuous-time system and converts it to discrete-time. It then computes the necessary matrices for the GPC controller and applies the control law in a simulation loop. White Gaussian noise is added to the output to simulate measurement disturbances.

You can modify the following parameters to test different controller configurations:

## Tuning Parameters
- `Np`: **Prediction horizon** – Defines how many future steps the output is predicted (default: 20).
- `Nu`: **Control horizon** – Defines how many control inputs are optimized (default: 5).
- `lambda`: **Control weighting factor** – Penalizes rapid changes in the control input (default: 1).
- `G`, `H`: Matrices computed from the step response of the discretized system used in the cost function optimization.
- `r`: Reference signal – A step signal (`r = ones(...)`) by default, can be modified for other tracking tasks.

## Example Output
The script generates two subplots:
1. **System Output vs. Reference** – Shows how accurately the system output follows the reference signal, despite the added noise.
2. **Control Effort** – Illustrates the control input over time used to achieve the tracking.

