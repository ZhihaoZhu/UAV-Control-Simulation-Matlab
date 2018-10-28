%% True plant parameters
plant_params = struct(...
    'mass',                   0.770, ...
    'gravity',                9.80665, ...
    'arm_length',             0.1103, ...
    'motor_spread_angle',     0.925, ...
    'thrust_coefficient',     8.07e-9, ...
    'moment_scale',           0.017, ...
    'motor_constant',        36.5, ...
    'rpm_min',             3000, ...
    'rpm_max',            20000, ...
    'inertia',            diag([0.0033 0.0033 0.005]),...
    'zoffset',                0.2);
%% Controller Parameters
lookahead_time = 0.01;

ctrl = struct();
ctrl.Kp = [17 17 20];
ctrl.Kv = [6.6 6.6 9];
ctrl.Kr = [190 198 80];
ctrl.Kw = [30 30 17.88];
ctrl.inertia = plant_params.inertia;
ctrl.ct = plant_params.thrust_coefficient;
ctrl.ms = plant_params.moment_scale;

%% Simulation parameters
sim_start_time = 0;
sim_dt = 0.005;
sim_end_time = 20.0;
times = sim_start_time:sim_dt:sim_end_time;
N = numel(times);

%% Set initial conditions
init_pos = [0; 0; plant_params.zoffset];
init_vel = zeros(3,1);
init_rpy = zeros(3,1);
init_avl = zeros(3,1);
init_rpm = plant_params.rpm_min*ones(4,1);


w = zeros(4,1);
Mix_Matrix = []



x=[0;0;10];
xdot = zeros(3, 1);
theta = zeros(3, 1);

for t = times
    i = input(t);
    