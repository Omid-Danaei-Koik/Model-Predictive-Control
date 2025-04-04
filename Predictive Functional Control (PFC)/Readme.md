### PFC Controller 

---

## Overview
This folder contains a MATLAB implementation of a Predictive Functional Control (PFC) scheme designed for the system:

**G(s) = (-s + 1) / (s^2 + s + 2)**
Lambda=0.8
delta=1
The PFC controller is implemented to track a step reference signal while minimizing control effort and ensuring a stable system response. 

---

## The Project Includes:
- **MATLAB script for the PFC controller** (`PFC.m`): Includes the full implementation of the system and controller.
- **Plots and Results**: Simulation outputs, including system response and control effort.

---

## Running the File
1. Open MATLAB and navigate to the folder containing this project.
2. Run the script **PFC.m** to execute the controller and view simulation results.
3. Modify system parameters directly in the script to explore different behaviors and scenarios.

---

## Usage
The script **PFC.m**:
- Defines the system **G(s) = (-s+1)/(s^2 + s + 2)** and discretizes it using a sampling time (`Ts`).
- Implements a PFC control law to regulate the output and follow a reference signal (`r`).
- Plots the system response and control effort for analysis.

You can adjust control parameters such as:
- **λ (lambda)**: Regularization term to balance tracking accuracy and control effort.
- **Control input range**: Limits the amplitude of the control input.
- **Reference signal (`r`)**: Modify the step input to test different tracking scenarios.

---

## Tuning Parameters
- **λ (lambda)**: Regularization term to tune system performance. A larger value (e.g., 15) improves stability but reduces responsiveness (default: 0.8).
- **delta**: Control horizon, commonly set to 1 for PFC (default: 1).
- **Reference Signal**: Modify the step input `r` to analyze tracking accuracy under different conditions.

---

## Example Output
Two plots are generated in a single figure:
1. **System Output vs. Reference**:
   - Displays how well the system output (`y`) tracks the step reference signal (`r`).
   - Stability and tracking performance are analyzed visually.

2. **Control Effort**:
   - Shows the control signal (`u`) applied to achieve the desired tracking.
   - Constrained between defined upper and lower limits.
