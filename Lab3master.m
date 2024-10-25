% master.m file for Motocross Pitch-Heave Model

% Clear workspace
clear; clc;

% Define global variables
global mcr Jcr Lcg_default Lcg_forward rg mk_front mk_rear ...

% System Parameters
vc = 10;             % Cycle forward velocity [m/s]
Lcg_default = 0.9;   % Center of gravity distance (standard config) [m]
Lcg_forward = 0.7;   % Center of gravity distance (forward config) [m]
mcr = 300;           % Mass of cycle and rider [kg]
rg = 0.5;            % Body radius of gyration [m]
ksf = 3000;          % Front suspension stiffness [N/m]
ksr = 3500;          % Rear suspension stiffness [N/m]
bsf = 400;           % Front damping coefficient [Ns/m]
bsr = 500;           % Rear damping coefficient [Ns/m]
mtf = 15;            % Front tire (unsprung) mass [kg]
mtr = 20;            % Rear tire (unsprung) mass [kg]
ktf = 30000;         % Front tire stiffness [N/m]
ktr = 40000;         % Rear tire stiffness [N/m]
Lwb = 1.6;           % Wheelbase distance [m]
g = 9.81;            % Gravitational acceleration [m/s^2]
delta_max = 0.1;     % Max suspension deflection [m]

% Calculate moments of inertia
Jcr = mcr * rg^2;    % Moment of inertia for cycle and rider

% Define the simulation time parameters
% Calculate natural frequency and timestep based on system frequency response
f_nat_heave = sqrt((ksf + ksr) / mcr) / (2 * pi); % Approx. heave natural frequency
f_nat_pitch = sqrt((ksf * Lcg_default^2 + ksr * (Lwb - Lcg_default)^2) / Jcr) / (2 * pi);

% Determine timestep and simulation length
T_heave = 1 / f_nat_heave;
T_pitch = 1 / f_nat_pitch;
dt = min(T_heave, T_pitch) / 10; % Timestep as 1/10 of minimum period
Tend = 3 * max(T_heave, T_pitch); % Simulation end time

% Set initial conditions
initial_conditions = [0; 0; 0; 0; 0; 0; 0; 0]; % Initial values for state variables

% Simulate the system
[T, Y] = ode45(@eqns, [0 Tend], initial_conditions);

% Plot results for each configuration
figure;
subplot(3, 1, 1);
plot(T, Y(:, 3), 'b', T, Y(:, 4), 'r');
title('Front and Rear Suspension Deflections');
xlabel('Time [s]');
ylabel('Deflection [m]');
legend('Front Suspension', 'Rear Suspension');

subplot(3, 1, 2);
plot(T, Y(:, 2));
title('Heave Velocity');
xlabel('Time [s]');
ylabel('Velocity [m/s]');

subplot(3, 1, 3);
plot(T, Y(:, 1));
title('Pitch Angular Velocity');
xlabel('Time [s]');
ylabel('Angular Velocity [rad/s]');

% Maximum bump height calculation
% Assuming linear system: scale inputs for bump height
