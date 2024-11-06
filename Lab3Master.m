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

% Bump parameters
delta_max = 0.1;            % Max suspension deflection (m)
A = 0.1;                    % Bump amplitude (m)
bump_dist = 0.5;            % Distance between bumps (m)

% Initial conditions
initial_p_J = 0;            % Pitch angular momentum (initially 0)
initial_p_cr = 0;           % Vertical momentum of cycle and rider (initially 0)
initial_q_sf = (m_tf * g - k_sf * delta_max) / k_sf;  % Front suspension spring displacement
initial_q_sr = (m_tr * g - k_sr * delta_max) / k_sr;  % Rear suspension spring displacement
initial_p_tf = 0;           % Momentum of front tire mass (initially 0)
initial_p_tr = 0;           % Momentum of rear tire mass (initially 0)
initial_q_tf = m_tf * g / k_tf;   % Front tire deflection
initial_q_tr = m_tr * g / k_tr;   % Rear tire deflection

initial = [initial_p_J; initial_p_cr; initial_q_sf; initial_q_sr; initial_p_tf; initial_p_tr; initial_q_tf; initial_q_tr];

% Bump definition
L = vc * bump_dist;            % Distance over the bump

% Front tire bump timings
t_front_start = 1;             % Time front tire hits the first bump
t_front_apex = t_front_start + bump_dist / (2 * vc);
t_front_end = t_front_start + bump_dist / vc;

% Rear tire bump timings (non-overlapping)
t_rear_start = t_front_end + bump_dist / vc;  % Rear tire starts after front tire ends
t_rear_apex = t_rear_start + bump_dist / (2 * vc);
t_rear_end = t_rear_start + bump_dist / vc;

% Second bump for front and rear tires (no overlap)
t_front_start2 = t_rear_end + bump_dist / vc;  % Front second bump starts after rear tire ends
t_front_apex2 = t_front_start2 + bump_dist / (2 * vc);
t_front_end2 = t_front_start2 + bump_dist / vc;

t_rear_start2 = t_front_end2 + bump_dist / vc;  % Rear second bump starts after front tire's second bump
t_rear_apex2 = t_rear_start2 + bump_dist / (2 * vc);
t_rear_end2 = t_rear_start2 + bump_dist / vc;

% Time control parameter
natural_frequency = sqrt((k_sf + k_sr) / m_cr);  
vibration_period = 1 / natural_frequency;        

% Simulation
FT = t_rear_end2 + 25 * vibration_period;
step_size = vibration_period / 10;

% Time array for the simulation
time_array = linspace(0, FT, 1000);

% Define road displacement function for both front and rear bumps, ensuring no negative values
road_displacement = @(t, bump_start, bump_apex, bump_end) max(0, A * (1 - abs((t - bump_start) / (bump_apex - bump_start))) .* (t >= bump_start & t <= bump_end));

% Road displacement for both front and rear tires (two instances for each)
y_road_front1 = zeros(size(time_array));
y_road_front2 = zeros(size(time_array));
y_road_rear1 = zeros(size(time_array));
y_road_rear2 = zeros(size(time_array));

% Calculate displacement for both bumps for front and rear tires
for i = 1:length(time_array)
    t = time_array(i);
    % First bump for front tire
    y_road_front1(i) = road_displacement(t, t_front_start, t_front_apex, t_front_end);
    % Second bump for front tire
    y_road_front2(i) = road_displacement(t, t_front_start2, t_front_apex2, t_front_end2);
    
    % First bump for rear tire
    y_road_rear1(i) = road_displacement(t, t_rear_start, t_rear_apex, t_rear_end);
    % Second bump for rear tire
    y_road_rear2(i) = road_displacement(t, t_rear_start2, t_rear_apex2, t_rear_end2);
end

% Get the maximum displacement for front and rear tires for both instances (before scaling)
max_front1 = max(y_road_front1); % Peak for the first bump of front tire
max_rear1 = max(y_road_rear1);   % Peak for the first bump of rear tire
max_front2 = max(y_road_front2); % Peak for the second bump of front tire
max_rear2 = max(y_road_rear2);   % Peak for the second bump of rear tire

% Find the overall maximum peak value (across both bumps for front and rear tires)
max_front = max(max_front1, max_front2);  % Max of front tire bumps
max_rear = max(max_rear1, max_rear2);     % Max of rear tire bumps

% Find the final peak value to scale to (the higher of the two)
max_value = max(max_front, max_rear);

% Scale each tire's displacement for both bumps to match the max value
y_road_front1 = y_road_front1 / max(max_front1) * max_value;
y_road_front2 = y_road_front2 / max(max_front2) * max_value;

y_road_rear1 = y_road_rear1 / max(max_rear1) * max_value;
y_road_rear2 = y_road_rear2 / max(max_rear2) * max_value;

% Combine the displacement for both bumps for both tires
y_road_front = y_road_front1 + y_road_front2;
y_road_rear = y_road_rear1 + y_road_rear2;

% Call ODE45 solver
[t, s] = ode45(@Lab3eqns, time_array, initial);

% Plotting results
figure;
plot(t, s(:,3), t, s(:,4));
grid on;
title('Front and Rear Suspension Deflections');
xlabel('Time (s)');
ylabel('Deflection (m)');
legend('Front Suspension', 'Rear Suspension');

figure;
plot(t, s(:,2) / m_cr, 'r');
grid on;
title('Heave Velocity');
xlabel('Time (s)');
ylabel('Heave Velocity (m/s)');

figure;
plot(t, s(:,1) / m_cr, 'b');
grid on;
title('Pitch Angular Velocity');
xlabel('Time (s)');
ylabel('Pitch Angular Velocity (rad/s)');

% Combined plot for road displacement with no negative values and identical max values
figure;
plot(time_array, y_road_front, 'g', 'DisplayName', 'Front Tire');
hold on;
plot(time_array, y_road_rear, 'm', 'DisplayName', 'Rear Tire');
grid on;
title('Road Displacement for Front and Rear Tires');
xlabel('Time (s)');
ylabel('Road Displacement (m)');
legend;

% Test road displacement for different times
t_test = linspace(0, FT, 1000);
y_road_test = zeros(size(t_test));
for i = 1:length(t_test)
    y_road_test(i) = road_displacement(t_test(i), t_front_start, t_front_apex, t_front_end);
end
plot(t_test, y_road_test);
title('Road Displacement over Time');
xlabel('Time (s)');
ylabel('Displacement (m)');

% Display the peak values
disp(['Peak value for front tire displacement (scaled): ', num2str(max(y_road_front)), ' meters']);
disp(['Peak value for rear tire displacement (scaled): ', num2str(max(y_road_rear)), ' meters']);
