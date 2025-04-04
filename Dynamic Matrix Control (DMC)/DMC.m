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

% Compute the step response of the system and remove the first element (t=0)
h = step(system, t);
h = h(2:end);

% Get step response information (e.g., settling time) of the system
Info = stepinfo(system);

% Calculate the number of samples N based on settling time, adding 10 extra steps
N = floor(Info.SettlingTime / Ts) + 10;

% Define parameters for Dynamic Matrix Control (DMC)
p = 10;          % Prediction horizon
m = 5;           % Control horizon
L = 1;           % Lambda (weighting factor for control effort)

% Discretize the continuous system using sampling time Ts
system_dis = c2d(system, Ts);  % Converts to discrete-time system: y(k) = 1.724y(k-1) - 0.7408y(k-2) + 0.004528u(k-1) + 0.004097u(k-2)

% Extract denominator (A) and numerator (B) coefficients from discrete system
A = system_dis.den;            % Denominator coefficients (e.g., [1 -1.724 0.7408])
A = cell2mat(A);               % Convert cell to matrix
B = system_dis.num;            % Numerator coefficients (e.g., [0 0.004528 0.004097])
B = cell2mat(B);               % Convert cell to matrix
B = B(2:end);                  % Remove leading zero from numerator

% Compute step response of the discrete system (not used directly in the loop)
sysdisi = step(system_dis);

% Initialize G matrix for control input contribution in DMC
G = zeros(p, m);               % Dynamic matrix for control increments
for i = 1:m
    G(:, 1) = [zeros(i-1, 1); h(1:p-i+1)];  % Shifted step response
end

% Initialize H matrix for free response contribution in DMC
H = zeros(p, N);               % Matrix for past control increments' effect
for i = 1:p
    for j = 1:N
        H(i, j) = h(i+j) - h(j);  % Difference in step response for free response
    end
end

% Main simulation loop
Nt = numel(t);                 % Total number of time steps
W = [2*ones(floor(Nt/4), 1);  % Reference signal: step changes over time
     1*ones(floor(Nt/4), 1);
     -1*ones(floor(Nt/4), 1);
     zeros(floor(Nt/4), 1)];

y = zeros(Nt, 1);              % Initialize output vector
u = zeros(Nt, 1);              % Initialize control input vector
dU = zeros(m, 1);              % Initialize control increment vector
Uminus = zeros(N, 1);          % Vector to store past control increments
DegSys = numel(A) - 1;         % Degree of the system (order of denominator - 1)

% Main loop to compute output and control input using DMC
for i = 3:Nt-1
    % Compute output using discrete system equation
    y(i) = 1.724*y(i-1) - 0.7408*y(i-2) + 0.004528*u(i-1) + 0.004097*u(i-2);

    % Update past control increments (Uminus) for free response calculation
    for j = 1:N-1
        if i-j < 2
            Uminus(j, 1) = 0;  % Set to zero if index is before simulation start
        else
            Uminus(j, 1) = u(i-j) - u(i-j-1);  % Compute past control increment
        end
    end

    % Compute free response based on past control increments and current output
    FreeResponse = H * Uminus + y(i);

    % Compute optimal control increment (dUopt) using DMC law
    dUopt = (G'*G + L*eye(m,m)) \ (G' * (W(i) - FreeResponse));

    % Update control input by adding the first optimal increment to the previous input
    u(i) = dUopt(1) + u(i-1);
end

% Plot the results
figure(2);

% Subplot 1: Reference signal (W) and scaled output (y)
subplot(2,1,1);
plot(t(1:Nt-1), W(1:Nt-1), t(1:Nt-1), y(1:Nt-1) * W(20/Ts)/y(20/Ts), 'LineWidth', 2);
xlabel('Time (second)');       % X-axis label
ylabel('Amp y');               % Y-axis label
legend('Ref', 'Out');          % Legend for reference and output
title(['Output Signal (DMC) , Gain = ', num2str(W(20/Ts)/y(20/Ts))]);  % Plot title with gain
grid on;                       % Enable grid

% Subplot 2: Scaled control effort (u)
subplot(2,1,2);
plot(t(1:Nt-1), u(1:Nt-1) * W(20/Ts)/y(20/Ts), 'Linewidth', 3);
xlabel('Time (second)');       % X-axis label
ylabel('Amp u');               % Y-axis label
title('Effort control Response (DMC)');  % Plot title
legend('U');                   % Legend for control input
grid on;                       % Enable grid