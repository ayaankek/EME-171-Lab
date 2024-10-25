% master.m 

clear;
clc;

global vc Lcg_1 Lcg_2 m_cr r_gy J_cr k_sf k_sr b_sf b_sr m_tf m_tr k_tf k_tr L_wb g A delta_max L bump_dist finish_time t_front_start t_front_apex t_front_end t_rear_start t_rear_apex t_rear_end step_size;

% System parameters
vc = 10;                    % Cycle forward velocity (m/s)
Lcg_1 = 0.9;                % CG distance (standard config) (m)
Lcg_2 = 0.7;                % CG distance (forward config) (m)
m_cr = 300;                 % Mass of cycle and rider (kg)
r_gy = 0.5;                 % Body radius of gyration (m)
J_cr = m_cr * r_gy^2;       % Moment of inertia for cycle & rider
k_sf = 3000;                % Front suspension stiffness (N/m)
k_sr = 3500;                % Rear suspension stiffness (N/m)
b_sf = 400;                 % Front damping coefficient (Ns/m)
b_sr = 500;                 % Rear damping coefficient (Ns/m)
m_tf = 15;                  % Front tire (unsprung) mass (kg)
m_tr = 20;                  % Rear tire (unsprung) mass (kg)
k_tf = 30000;               % Front tire stiffness (N/m)
k_tr = 40000;               % Rear tire stiffness (N/m)
L_wb = 1.6;                 % Wheel base distance (m)
g = 9.81;                   % Acceleration due to gravity (m/s^2)

% Bump parameters
delta_max = 0.1;            % Max suspension deflection (m)
A = 0.1;                    % Initial bump height for testing (m)
bump_dist = 0.5;            % Distance between bumps (m)

% Determine initial conditions (equilibrium setup)
initial_conditions = [0; 0; 0; 0; 0; 0; 0; 0; 0];

% Define the bump timing based on cycle forward velocity and geometry
L = vc * bump_dist;            % Horizontal distance over the bump
t_front_start = 0;             % Time front tire hits the first bump
t_front_apex = t_front_start + bump_dist / (2 * vc);
t_front_end = t_front_start + bump_dist / vc;

t_rear_start = t_front_start + L_wb / vc;
t_rear_apex = t_rear_start + bump_dist / (2 * vc);
t_rear_end = t_rear_start + bump_dist / vc;

% Define simulation time controls
natural_frequency = sqrt((k_sf + k_sr) / m_cr);  % Approximate natural frequency
vibration_period = 1 / natural_frequency;        % Vibration period

% Simulation time and step size
finish_time = t_rear_end + 3 * vibration_period;
step_size = vibration_period / 10;

% Run the simulation
[t, y] = ode45(@eqns, [0, finish_time], initial_conditions);

% Plotting results
figure;
subplot(3,1,1);
plot(t, y(:,1), t, y(:,2));
title('Front and Rear Suspension Deflections');
xlabel('Time (s)');
ylabel('Deflection (m)');
legend('Front Suspension', 'Rear Suspension');

subplot(3,1,2);
plot(t, y(:,3));
title('Heave Velocity');
xlabel('Time (s)');
ylabel('Heave Velocity (m/s)');

subplot(3,1,3);
plot(t, y(:,4));
title('Pitch Angular Velocity');
xlabel('Time (s)');
ylabel('Pitch Angular Velocity (rad/s)');
