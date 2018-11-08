clear;
clc;
close all;

% Simulation times, in seconds. 
start_time = 0; 
end_time = 60; 
dt = 0.01; 
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
ctrl.ct = 2*plant_params.thrust_coefficient;
ctrl.ms = ctrl.ct*plant_params.moment_scale;

% initialize state X
init_pos = [0; 0; 1];
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
state.yaw = zeros(N-1,1);

% desire state structure:
global des_state;
des_state = struct();
des_state.pos = zeros(3,N-1);
des_state.vel = zeros(3,N-1);
des_state.acc = zeros(3,N-1);
des_state.rot = zeros(3,N-1);
des_state.omega = zeros(3,N-1);
des_state.yaw = zeros(N-1,1);
des_state.yawdot = zeros(N-1,1);

% Initialize
state.pos(:,1) = init_pos;
state.vel(:,1) = init_vel;
state.wSpeed(:,1) = init_rpm;

time = 0; 

current_state = 1;
ready_to_land = false;
target_iter_stamp = 0;


%% Simulation

for iter = 1:N-2

    traGenerator8(iter,time,end_time);
    
    update_state(iter,ctrl,plant_params, dt);

    time = time + dt;
end


%% ************************ plot the trajectory *********************
disp('Initializing figures...');
start_point = 1;
end_point = 5999;
disp('Initializing figures...');

x = state.pos(1,:);
y = state.pos(2,:);
z = state.pos(3,:);

figure(1)
subplot(1,3,1);
plot(x(start_point:end_point),'linewidth',1);
hold on
plot(des_state.pos(1,start_point:end_point),'linewidth',1);
xlabel('(0.01 s)','FontSize',10);
ylabel('(m)','FontSize',10);
legend('Actual X Position','Desire X Position');
hold on
title("x")
subplot(1,3,2);
plot(y(start_point:end_point), 'linewidth',1);

hold on
plot(des_state.pos(2,start_point:end_point),'linewidth',1);
xlabel('(0.01 s)','FontSize',10);
ylabel('(m)','FontSize',10);
legend('Actual Y Position','Desire Y Position');
hold on
title("y")
subplot(1,3,3);
plot(z(start_point:end_point), 'linewidth',1);
hold on
plot(des_state.pos(3,start_point:end_point),'linewidth',1);
xlabel('(0.01 s)','FontSize',10);
ylabel('(m)','FontSize',10);
legend('Actual Z Position','Desire Z Position');
hold on
title("z")


figure('Name', "error_pos")
subplot(1,3,1);
plot(des_state.pos(1,start_point:end_point)-state.pos(1,start_point:end_point),'linewidth',1);
ylim([-1 1])
xlabel('(0.01 s)','FontSize',10);
ylabel('(m)','FontSize',10);
hold on
title("error_x")
subplot(1,3,2);
plot(des_state.pos(2,start_point:end_point)-state.pos(2,start_point:end_point),'linewidth',1);
ylim([-1 1])
xlabel('(0.01 s)','FontSize',10);
ylabel('(m)','FontSize',10);
hold on
title("error_y")
subplot(1,3,3);
plot(des_state.pos(3,start_point:end_point)-state.pos(3,start_point:end_point),'linewidth',1);
xlabel('(0.01 s)','FontSize',10);
ylabel('(m)','FontSize',10);
hold on
title("error_z")


figure('Name', "error_vel")
subplot(1,3,1);
plot(des_state.vel(1,start_point:end_point)-state.vel(1,start_point:end_point),'linewidth',1);
ylim([-1 1])
xlabel('(0.01 s)','FontSize',10);
ylabel('(m/s)','FontSize',10);
hold on
title("error xvel")
subplot(1,3,2);
plot(des_state.vel(2,start_point:end_point)-state.vel(2,start_point:end_point),'linewidth',1);
ylim([-1 1])
xlabel('(0.01 s)','FontSize',10);
ylabel('(m/s)','FontSize',10);
hold on
title("error yvel")
subplot(1,3,3);
plot(des_state.vel(3,start_point:end_point)-state.vel(3,start_point:end_point),'linewidth',1);
xlabel('(0.01 s)','FontSize',10);
ylabel('(m/s)','FontSize',10);
hold on
title("error zvel")


figure('Name', "error_rot")
subplot(1,3,1);
plot(des_state.rot(1,start_point:end_point)-state.rot(1,start_point:end_point),'linewidth',1);
ylim([-1 1])
xlabel('(0.01 s)','FontSize',10);
ylabel('(rad)','FontSize',10);
hold on
title("error phi")
subplot(1,3,2);
plot(des_state.rot(2,start_point:end_point)-state.rot(2,start_point:end_point),'linewidth',1);
ylim([-1 1])
xlabel('(0.01 s)','FontSize',10);
ylabel('(rad)','FontSize',10);
hold on
title("error theta")
subplot(1,3,3);
plot(des_state.rot(3,start_point:end_point)-state.rot(3,start_point:end_point),'linewidth',1);
ylim([-1 1])
xlabel('(0.01 s)','FontSize',10);
ylabel('(rad)','FontSize',10);
hold on
title("error yaw")

figure('Name', "error_omega")
subplot(1,3,1);
plot(des_state.omega(1,start_point:end_point)-state.omega(1,start_point:end_point),'linewidth',1);
ylim([-1 1])
xlabel('(0.01 s)','FontSize',10);
ylabel('(rad/s)','FontSize',10);
hold on
title("error phi vel")
subplot(1,3,2);
plot(des_state.omega(2,start_point:end_point)-state.omega(2,start_point:end_point),'linewidth',1);
ylim([-1 1])
xlabel('(0.01 s)','FontSize',10);
ylabel('(rad/s)','FontSize',10);
hold on
title("error theta vel")
subplot(1,3,3);
plot(des_state.omega(3,start_point:end_point)-state.omega(3,start_point:end_point),'linewidth',1);
ylim([-1 1])
xlabel('(0.01 s)','FontSize',10);
ylabel('(rad/s)','FontSize',10);
hold on
title("error yaw vel")


figure(10)
plot(x(1:1999),y(1:1999),'linewidth',1);
hold on
plot(x(2000:3999),y(2000:3999),'linewidth',1);
hold on
plot(x(4000:5999),y(4000:5999),'linewidth',1);
legend('First phase','Second phase','Third phase');

xlabel('x','FontSize',10);
ylabel('y','FontSize',10);
hold on


