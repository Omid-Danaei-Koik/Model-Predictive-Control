# MAC Controller for G(s) = 1/[(s+1)(s+2)]

## Overview
This folder contains a MATLAB implementation of a Model Algorithmic Control (MAC) scheme designed for the system:

G(s) = 1/[(s+1)(s+2)]

The MAC controller is implemented to track a reference signal while minimizing control effort and overshoot. The project includes:
- MATLAB script for the MAC controller.
- Simulation results showcasing system response.
- Adjustable parameters for controller tuning.

## Running the file
1. Open MATLAB and navigate to the project directory.
2. Run the script `MAC.m` to execute the controller and view simulation results.

## Usage
- The script `MAC.m` defines the system and applies the MAC controller.
- Modify parameters such as `H1`, `H2`, and `L` in the script to fine-tune the performance.
- The system response and control effort are plotted for analysis.
  
## Tuning Parameters
- **`H1`, `H2`**: Weight matrices influencing the prediction and control effort.
- **`L`**: Regularization term for improving stability and robustness.
- **`dUopt` constraints**: Limits the control action to prevent abrupt changes.

## Example Output
Two plots are generated:
1. **System Output vs. Reference** - Displays how well the output tracks the reference signal.
2. **Control Effort** - Shows the control input applied to achieve the desired tracking.


