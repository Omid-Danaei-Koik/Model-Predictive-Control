clc;
clear;
close all;

%% Simulation Parameters
T0 = 0; Tf = 10; Ts = 0.1;  % Shorter time for better response observation
t = (T0:Ts:Tf)';

%% Define the System
sys = tf(1, [1 3 2]); % Transfer function G(s) = 1/[(s+1)(s+2)]
sysd = c2d(sys, Ts); % Discretization of the system

[A, B, C, D] = tf2ss(sysd.num{1}, sysd.den{1}); % Convert to state-space representation
n_states = size(A,1); % Number of system states

%% GPC Parameters
Np = 20;    % Prediction horizon
Nu = 5;     % Control horizon
lambda = 1; % Control weight

%% Compute GPC Matrices
G = zeros(Np, Nu); % Step response matrix
step_resp = step(sysd, (0:Np-1)*Ts); % Step response of the discrete system

for i = 1:Np
    for j = 1:Nu
        if i >= j
            G(i,j) = step_resp(i-j+1);
        end
    end
end

H = 2*(G'*G + lambda*eye(Nu)); % Weight matrix for optimization

%% Simulation Initialization
n = length(t);
y = zeros(n,1); % System output
y(1:2) = 0; % Initial conditions
u = zeros(n,1); % Control input (renamed from 'nu' to 'u')
u(1:2) = 0; % Initial conditions
r = ones(n,1); % Step reference signal
x = zeros(n_states,1); % Initial state vector

%% GPC Control Loop
for k = 3:n-1
    % Compute Free Response
    y_free = zeros(Np,1);
    x_free = x;
    for i = 1:Np
        x_free = A*x_free + B*u(k-1);
        y_free(i) = C*x_free;
    end
    
    % Future Reference Signal
    R = r(k)*ones(Np,1);
    
    % Compute Optimal Control
    DU = H\(G'*(R - y_free)); % Solve quadratic optimization
    du = DU(1); % First control increment
    
    % Apply Control Signal
    u(k) = u(k-1) + du;
    
    % Update System State
    x = A*x + B*u(k);
    y(k) = C*x;
    
    % Add White Noise to the Output
    y(k) = y(k) + 0.01*randn;
end

%% Plot Results
figure;
subplot(2,1,1);
plot(t, y, 'b', t, r, 'r--', 'LineWidth', 1.5);
xlabel('Time (seconds)');
ylabel('Output');
legend('System Output', 'Reference');
title('System Response with GPC');
grid on;

subplot(2,1,2);
plot(t, u, 'g', 'LineWidth', 1.5);
xlabel('Time (seconds)');
ylabel('Control Input');
title('GPC Control Signal');
grid on;
