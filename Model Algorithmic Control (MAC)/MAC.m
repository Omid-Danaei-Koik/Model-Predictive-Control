% Clear the command window, workspace, and close all figures
clc;
clear;
close all;

% Define simulation parameters
T0 = 0;          % Initial time (start of simulation)
Ts = 0.1;        % Sampling time (time step)
Tf = 100;        % Final time (end of simulation)
t = T0:Ts:Tf;    % Time vector from T0 to Tf with step Ts
t = t';          % Transpose time vector to column form

% Define the continuous-time system transfer function: 1/(s^2 + 3s + 2)
system = tf(1, [1 3 2]);

% Get step response information (e.g., settling time) of the system
Info = stepinfo(system);

% Calculate the number of samples N based on settling time, adding 5 extra steps
N = floor(Info.SettlingTime / Ts) + 5;

% Define parameters for predictive control
p = 10;          % Prediction horizon
m = 5;           % Control horizon
L = 1;           % Lambda (weighting factor for control effort)

% Compute the impulse response of the system and remove the first element (t=0)
h = impulse(system, t);
h = h(2:end);

% Discretize the continuous system using sampling time Ts
system_dis = c2d(system, Ts);  % Converts to discrete-time system

% Extract denominator (A) and numerator (B) coefficients from discrete system
A = system_dis.den;            % Denominator coefficients (e.g., [1 -1.724 0.7408])
A = cell2mat(A);               % Convert cell to matrix
B = system_dis.num;            % Numerator coefficients (e.g., [0 0.004528 0.004097])
B = cell2mat(B);               % Convert cell to matrix
B = B(2:end);                  % Remove leading zero from numerator

% Initialize matrices H1 and H2 for predictive control
H1 = zeros(p, m);              % Matrix for control input contribution
H2 = zeros(p, N-1);            % Matrix for free response contribution

% Fill H1 matrix with shifted impulse response for control horizon
for i = 1:m
    H1(:, 1) = [zeros(i-1, 1); h(1:p-i+1)];  % Shifted impulse response
end

% Fill H2 matrix with reversed impulse response for free response
for i = 1:m
    H2(i, :) = [zeros(i-1, 1); h(N:-1:i+1)];  % Reversed impulse response
end

% Main simulation loop
Nt = numel(t);                 % Total number of time steps
W = [ones(floor(Nt/4), 1);    % Reference signal: step changes over time
     2*ones(floor(Nt/4), 1);
     -1*ones(floor(Nt/4), 1);
     zeros(floor(Nt/4), 1)];

y = zeros(Nt, 1);              % Initialize output vector
u = zeros(Nt, 1);              % Initialize control input vector
DegSys = numel(A) - 1;         % Degree of the system (order of denominator - 1)
Uminus = zeros(N-1, 1);        % Vector to store past control inputs

% Main loop to compute output and control input
for i = 3:Nt-1
    % Compute output using discrete system equation
    y(i) = 1.724*y(i-1) - 0.7408*y(i-2) + 0.004528*u(i-1) + 0.004097*u(i-2);
    
    % Alternative way to compute output using coefficient vectors (commented out)
    % y(i) = -A(2:end)*y(i-1:-1:i-DegSys)' + B * u(i-1:-1:i-DegSys);

    % Update past control inputs (Uminus) for free response calculation
    for j = 1:N-1
        if i-N+j < 1
            Uminus(j, 1) = 0; % Set to zero if index is before simulation start
        else
            Uminus(j, 1) = u(i-N+j);  % Use past control input
        end
    end

    % Compute free response based on past inputs
    FreeResponse = H2 * Uminus;

    % Compute optimal control increment (dUopt) using predictive control law
    dUopt = (H1'*H1 + L*eye(m,m)) \ (H1' * (W(i) - FreeResponse));
    
    % Update control input with the first element of dUopt
    u(i) = dUopt(1);
end

% Plot the results
figure(1);

% Subplot 1: Reference signal (W) and scaled output (y)
subplot(2,1,1);
plot(t(1:Nt-1), W(1:Nt-1), t(1:Nt-1), y(1:Nt-1) * W(20/Ts)/y(20/Ts), 'LineWidth', 2);
xlabel('Time (second)');       % X-axis label
ylabel('Amp y');               % Y-axis label
legend('Ref', 'Out');          % Legend for reference and output
title('Output Signal (Mac)');  % Plot title
grid on;                       % Enable grid

% Subplot 2: Scaled control effort (u)
subplot(2,1,2);
plot(t(1:Nt-1), u(1:Nt-1) * W(20/Ts)/y(20/Ts), 'Linewidth', 2);
xlabel('Time (second)');       % X-axis label
ylabel('Amp u');               % Y-axis label
title('Effort control Response (Mac)');  % Plot title
legend('U');                   % Legend for control input
grid on;                       % Enable grid
