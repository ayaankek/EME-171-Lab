clear all;
close all;
clc;

global vc Lcg_1 Lcg_2 m_cr r_gy J_cr k_sf k_sr b_sf b_sr m_tf m_tr k_tf k_tr L_wb g A delta_max L bump_dist FT t_front_start t_front_apex t_front_end t_rear_start t_rear_apex t_rear_end step_size;

% Constants and parameters
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
L = 0.5;
% Bump parameters
a = Lcg_1;
b = L_wb - Lcg_1;
delta_max = 0.1;            % Max suspension deflection (m)
A = 0.1;                    % Bump amplitude (m)
bump_dist = 0.5;            % Distance between bumps (m)
T_bump = L / vc; 

% Initial conditions
initial_p_J = 0;            % Pitch angular momentum (initially 0)
initial_p_cr = 0;           % Vertical momentum of cycle and rider (initially 0)
initial_q_sf = (m_cr * g * b)/ (k_sf * (a+b));  % Front suspension spring displacement
initial_q_sr = (m_cr * g * a)/ (k_sr * (a+b));  % Rear suspension spring displacement
initial_p_tf = 0;           % Momentum of front tire mass (initially 0)
initial_p_tr = 0;           % Momentum of rear tire mass (initially 0)
initial_q_tf = ((m_tf * g) + (m_cr * g * b / (a+b))) / (k_tf);   % Front tire deflection
initial_q_tr = ((m_tr * g) + (m_cr * g * a / (a+b))) / (k_tr);   % Rear tire deflection

initial = [initial_p_J; initial_p_cr; initial_q_sf; initial_q_sr; initial_p_tf; initial_p_tr; initial_q_tf; initial_q_tr];

% Bump definition
L = vc * bump_dist;            % Distance over the bump

% Time points for front and rear tire over two bumps
% Define time points for the front tire over the first and second bumps
t_front_start = 1;
t_front_apex = t_front_start + T_bump / 2;
t_front_end = t_front_start + T_bump;
t_rear_start = t_front_start + L_wb / vc;
t_rear_apex = t_rear_start + T_bump / 2;
t_rear_end = t_rear_start + T_bump;

% Define time points for the front tire over the second bump
t_front_start_2 = t_front_end + bump_dist / vc;
t_front_apex_2 = t_front_start_2 + T_bump / 2;
t_front_end_2 = t_front_start_2 + T_bump;
t_rear_start_2 = t_front_start_2 + L_wb / vc;
t_rear_apex_2 = t_rear_start_2 + T_bump / 2;
t_rear_end_2 = t_rear_start_2 + T_bump;

% Time control parameter
natural_frequency = sqrt((k_sf + k_sr) / m_cr);  
vibration_period = 1 / natural_frequency;        


% Simulation setup
FT = t_rear_end_2 + 25 * vibration_period;
step_size = vibration_period / 10;
time_array = linspace(0, FT, 1000);

% Road displacement function (vectorized)
road_displacement = @(t, bump_start, bump_end)(t >= bump_start & t <= bump_end) .* A .* sin(pi * (t - bump_start) / (bump_end - bump_start));

% Calculate displacements for front and rear tires for both bumps
y_road_front1 = road_displacement(time_array, t_front_start, t_front_end);
y_road_front2 = road_displacement(time_array, t_front_start_2, t_front_end_2);
y_road_rear1 = road_displacement(time_array, t_rear_start, t_rear_end);
y_road_rear2 = road_displacement(time_array, t_rear_start_2, t_rear_end_2);

% Scaling displacement to match max value
max_front1 = max(y_road_front1); % Peak for the first bump of front tire
max_rear1 = max(y_road_rear1);   % Peak for the first bump of rear tire
max_front2 = max(y_road_front2); % Peak for the second bump of front tire
max_rear2 = max(y_road_rear2);   % Peak for the second bump of rear tire

max_front = max(max_front1, max_front2);  % Max of front tire bumps
max_rear = max(max_rear1, max_rear2);     % Max of rear tire bumps
max_value = max(max_front, max_rear);

% Scale the displacement
y_road_front1 = y_road_front1 / max(max_front1) * max_value;
y_road_front2 = y_road_front2 / max(max_front2) * max_value;
y_road_rear1 = y_road_rear1 / max(max_rear1) * max_value;
y_road_rear2 = y_road_rear2 / max(max_rear2) * max_value;

% Combine displacements for both bumps for both tires
y_road_front = y_road_front1 + y_road_front2;
y_road_rear = y_road_rear1 + y_road_rear2;

% Call ODE45 solver
[t, s] = ode45(@Lab3eqns, time_array, initial);

% Plotting results

% Front and Rear Suspension Deflections
figure;
plot(t, s(:,3), t, s(:,4));
grid on;
title('Front and Rear Suspension Deflections');
xlabel('Time (s)');
ylabel('Deflection (m)');
legend('Front Suspension', 'Rear Suspension');

% Heave Velocity Plot
figure;
plot(t, s(:,2) / m_cr, 'r');
grid on;
title('Heave Velocity');
xlabel('Time (s)');
ylabel('Heave Velocity (m/s)');

% Pitch Angular Velocity Plot
figure;
plot(t, s(:,1) / m_cr, 'b');
grid on;
title('Pitch Angular Velocity');
xlabel('Time (s)');
ylabel('Pitch Angular Velocity (rad/s)');

% Combined Plot for Road Displacement 
figure;
plot(time_array, y_road_front, 'g', 'DisplayName', 'Front Tire');
hold on;
plot(time_array, y_road_rear, 'm', 'DisplayName', 'Rear Tire');
grid on;
title('Road Displacement for Front and Rear Tires');
xlabel('Time (s)');
ylabel('Road Displacement (m)');
legend;
