% Simulation times, in seconds. 
start_time = 0; 
end_time = 20; 
dt = 0.005; 
times = start_time:dt:end_time;
N = numel(times);

% load Quadcorpter parameters
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

% load control parameters
ctrl = struct();
ctrl.Kp = [17 17 20]';
ctrl.Kv = [6.6 6.6 9]';
ctrl.Kr = [190 198 80]';
ctrl.Kw = [30 30 17.88]';



ctrl.inertia = plant_params.inertia;
ctrl.ct = plant_params.thrust_coefficient;
ctrl.ms = ctrl.ct*plant_params.moment_scale;

% initialize state X
init_pos = [0; 0; 0.5];
init_vel = zeros(3,1);
init_rpm = plant_params.rpm_min*ones(4,1);

%% Initialize containers for simulation variables
% state structure:
global state;
state = struct();
state.pos = zeros(3,N-1);
state.vel = zeros(3,N-1);
state.rot = zeros(3,N-1);
state.omega = zeros(3,N-1);
state.wSpeed = zeros(4,N-1);

% desire state structure:
global des_state;
des_state = struct();
des_state.pos = zeros(3,N-1);
des_state.vel = zeros(3,N-1);
des_state.acc = zeros(3,N-1);
des_state.yaw = zeros(N-1,1);
des_state.yawdot = zeros(N-1,1);

% Initialize
state.pos(:,1) = init_pos;
state.vel(:,1) = init_vel;
state.wSpeed(:,1) = init_rpm;

time = 0; % initial time

% initial function
controlhandle = @controller;
trajhandle = @traj_generator;

%% ************************* RUN SIMULATION *************************
for iter = 1:N-1

    % generate coefficient for trajectore generator --- alpha;
%     line_traGenerator(iter, time, end_time)
%     wp_traGenerator(iter, time, end_time);
    hover(iter);

    % generate the Force and Torque
    [F_des, M_des] = controller(iter, ctrl, plant_params); % generate force, torque
    [F_act,M_act] = motor_model(iter, dt, ctrl, plant_params, F_des, M_des);
    
    % generate state vector
    s = state_vector(iter);

    % generate the derivative of state vector "s"
    sdot = physical_model(iter, F_act, M_act,s,plant_params); 
    

    % update state
    update_state(iter,sdot,dt);

    time = time + dt;
end

%% ************************ plot the trajectory *********************
disp('Initializing figures...');

x = state.pos(1,:);
y = state.pos(2,:);
z = state.pos(3,:);

des_x = des_state.pos(1,:);
des_y = des_state.pos(2,:);
des_z = des_state.pos(3,:);


plot(x);
hold on
plot(y);
hold on
plot(z);
hold on;
plot(des_state.pos(1,:));
hold on;
plot(des_state.pos(2,:));
hold on;
plot(des_state.pos(3,:));
hold on;


plot(des_state.acc(2,:)); 
plot3(x,y,z)
hold on
plot3(des_x,des_y,des_z)
view([30,30,45])
axis equal;

grid on;
grid minor;
