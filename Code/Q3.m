clear;
clc;
close all;

% Simulation times, in seconds. 
start_time = 0; 
end_time = 10; 
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
ctrl.ct = plant_params.thrust_coefficient;
ctrl.ms = ctrl.ct*plant_params.moment_scale;

% initialize state X
init_pos = [0; 0; 0];
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

time = 0; % initial time


%% tra_following simulation
for iter = 1:N-2
    if iter < N/4
        des_state.pos(3,iter) = 0.5;
    elseif iter >= N/4 && iter< N/2
        des_state.pos(3,iter) = 1;
    elseif iter >= N/2 && iter < 3*N/4
        des_state.pos(3,iter) = 0.5;
    else
        des_state.pos(3,iter) = 0;
    end
end

for iter = 1:N-2
    
%     Q3_tra(iter,time,end_time);
    update_state(iter,ctrl,plant_params, dt);

    time = time + dt;
end


%% ************************ plot the trajectory *********************
disp('Initializing figures...');

x = state.pos(1,:);
y = state.pos(2,:);
z = state.pos(3,:);

figure(1)
subplot(1,3,1);
plot(x,'linewidth',1);
ylim([-1 1])
hold on
plot(des_state.pos(1,:),'linewidth',1);
xlabel('(0.01 s)','FontSize',10);
ylabel('(m)','FontSize',10);
legend('Actual X Position','Desire X Position');
hold on
title("x")
subplot(1,3,2);
plot(y, 'linewidth',1);
ylim([-1 1])

hold on
plot(des_state.pos(2,:),'linewidth',1);
xlabel('(0.01 s)','FontSize',10);
ylabel('(m)','FontSize',10);
legend('Actual Y Position','Desire Y Position');
hold on
title("y")
subplot(1,3,3);
plot(z, 'linewidth',1);
hold on
plot(des_state.pos(3,:),'linewidth',1);
xlabel('(0.01 s)','FontSize',10);
ylabel('(m)','FontSize',10);
legend('Actual Z Position','Desire Z Position');
hold on
title("z")


figure('Name', "error_pos")
subplot(1,3,1);
plot(des_state.pos(1,:)-state.pos(1,:),'linewidth',1);
ylim([-1 1])

xlabel('(0.01 s)','FontSize',10);
ylabel('(m)','FontSize',10);
hold on
title("error_x")
subplot(1,3,2);
plot(des_state.pos(2,:)-state.pos(2,:),'linewidth',1);
ylim([-1 1])
xlabel('(0.01 s)','FontSize',10);
ylabel('(m)','FontSize',10);
hold on
title("error_y")
subplot(1,3,3);
plot(des_state.pos(3,:)-state.pos(3,:),'linewidth',1);
xlabel('(0.01 s)','FontSize',10);
ylabel('(m)','FontSize',10);
hold on
title("error_z")


