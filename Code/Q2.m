clear;
clc;
close all;

% Simulation times, in seconds. 
start_time = 0; 
end_time = 5; 
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
ctrl.Kv = [6.6 6.6 18]';

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


%% Hover simulation
des_state.pos(3,:) = 0.5;
for iter = 1:N-2

    % generate the Force and Torque
    update_state(iter,ctrl,plant_params, dt);

    time = time + dt;
end
% %% x_tra simulation
% iter_interval = round(N/5,0);
% for ii = 1:5
%     des_state.pos(1,(ii-1)*iter_interval+1:ii*iter_interval) = ii*0.1;
% end
% for iter = 1:N-2
% 
%     % generate the Force and Torque
%     update_state(iter,ctrl,plant_params, dt);
% 
%     time = time + dt;
% end


%% ************************ plot in desired range *********************
disp('Initializing figures...');

x = state.pos(1,:);
y = state.pos(2,:);
z = state.pos(3,:);

% figure(1)
% plot(z,'linewidth',1);
% hold on
% plot(des_state.pos(3,:),'linewidth',1);
% 
% xlabel('(0.01 s)','FontSize',10);
% ylabel('(m)','FontSize',10);
% legend('Actual X Position','Desire X Position');
% hold on
% title("Z Position")



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
% 
% figure('Name',"error_vel")
% subplot(1,3,1);
% plot(des_state.vel(1,:)-state.vel(1,:));
% hold on
% title("error_xvel")
% subplot(1,3,2);
% plot(des_state.vel(2,:)-state.vel(2,:));
% hold on
% title("error_yvel")
% subplot(1,3,3);
% plot(des_state.vel(3,:)-state.vel(3,:));
% hold on
% title("error_zvel")

% 
% figure('Name',"error_rot")
% subplot(1,3,1);
% plot(des_state.rot(1,:)-state.rot(1,:));
% hold on
% title("error_phi")
% 
% subplot(1,3,2);
% plot(des_state.rot(2,:)-state.rot(2,:));
% hold on
% title("error_theta")
% 
% subplot(1,3,3);
% plot(des_state.rot(3,:)-state.rot(3,:));
% hold on
% title("error_yaw")
% 
% 
% figure('Name', "error_omega")
% subplot(1,3,1);
% plot(des_state.omega(1,:)-state.omega(1,:));
% hold on
% title("error_phivel")
% 
% subplot(1,3,2);
% plot(des_state.omega(2,:)-state.omega(2,:));
% hold on
% title("error_thetavel")
% 
% subplot(1,3,3);
% plot(des_state.omega(3,:)-state.omega(3,:));
% hold on
% title("error_yawvel")
% 
% 
% figure('Name',"error_yaw")
% plot(des_state.yaw(:)-state.yaw(:));
% hold on
% title("error_yaw")


