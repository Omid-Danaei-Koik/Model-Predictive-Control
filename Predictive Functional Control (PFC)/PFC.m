clc; clear; close all;

%% System Definition
Ts = 0.1; % Sampling time
sys = tf([-1 1], [1 1 2]); % Transfer function of the continuous system
sysd = c2d(sys, Ts); % Convert the continuous system to discrete using zero-order hold (ZOH)

[A, B, C, D] = ssdata(ss(sysd)); % Extract state-space matrices from the discrete model
n = size(A,1); % Number of states in the system

%% Controller Parameters
lambda = 0.8 ; % Regularization parameter. Increasing lambda (e.g., 15) may improve stability.
delta = 1 ; % Control horizon (common value for PFC is 1)
Tf = 10; % Final simulation time
t = 0:Ts:Tf; % Time vector
N = length(t); % Number of time steps

%% Initialization
x = zeros(n,1); % Initial state vector
u = zeros(N,1); % Control signal (initially zero)
y = zeros(N,1); % System output (initially zero)
r = ones(N,1); % Reference signal (step input)

for k = 2:N-1
    % Predict the output at the next step
    x_pred = A*x + B*u(k-1); % Predicted state
    y_pred = C*x_pred + D*u(k-1); % Predicted output

    % Calculate the optimal control increment using the PFC rule
    num = C*A*x + D*u(k-1); % Numerator term
    den = C*B + lambda; % Denominator term
    du = (r(k) - num)/den; % Control increment

    % Apply control increment and constrain the control input
    u(k) = max(min(u(k-1) + du, 10), -10); % Constrain control signal between -10 and 10

    % Update the system state based on the new control input
    x = A*x + B*u(k);
    y(k) = C*x + D*u(k); % Update system output
end

%% Plot the Results
figure;
subplot(2,1,1);
plot(t, y, 'b', t, r, 'r--', 'LineWidth', 1.5); % Plot system output and reference signal
xlabel('Time (seconds)'); ylabel('Output');
legend('System Output','Reference');
title('PFC Output Response'); grid on;

subplot(2,1,2);
plot(t, u, 'k', 'LineWidth', 1.5); % Plot control signal
xlabel('Time (seconds)'); ylabel('Control Input');
title('PFC Control Signal'); grid on;
