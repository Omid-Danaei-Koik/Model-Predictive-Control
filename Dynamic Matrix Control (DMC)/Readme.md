DMC Controller for G(s) = 1/[(s+1)(s+2)]
##Overview
This folder contains a MATLAB implementation of a Dynamic Matrix Control (DMC) scheme designed for the system:
G(s) = 1/[(s+1)(s+2)]
The DMC controller is implemented to track a piecewise reference signal while optimizing control effort and ensuring stable system response.
#The project includes:
MATLAB script for the DMC controller.


##Running the File
Open MATLAB and navigate to the project directory.
Run the script DMC.m to execute the controller and view simulation results.

##Usage
The script DMC.m defines the system G(s) = 1/(s^2 + 3s + 2) and applies the DMC controller.
Modify parameters such as p, m, L, and the reference signal W in the script to fine-tune the performance.
The system response and control effort are plotted in two subplots for analysis.

##Tuning Parameters
p: Prediction horizon - Determines how far into the future the controller predicts the system behavior (default: 10).
m: Control horizon - Specifies the number of future control moves calculated (default: 5).
L: Lambda - Regularization term to balance tracking accuracy and control effort (default: 1).
G and H: Dynamic and free response matrices - Derived from the step response to model system behavior and past control effects.
W: Reference signal - A piecewise signal that can be adjusted to test different tracking scenarios.

##Example Output
Two plots are generated in a single figure:
System Output vs. Reference - Displays how well the output (y) tracks the reference signal (W), with a gain factor included in the title.
Control Effort - Shows the control input (u) applied to achieve the desired tracking, scaled for comparison with the output.


