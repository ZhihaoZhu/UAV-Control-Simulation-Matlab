clear;
clc;
close all;

% Simulation times, in seconds. 
start_time = 0; 
end_time = 20; 
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

%% tra_following simulation

for iter = 1:N-2

    if iter >= target_iter_stamp
        switch current_state
            case 1 % Take-off
                takeoff(1, iter);
                if arrived(state.pos(:, iter), des_state.pos(:, iter), 0.01) == true
                    current_state = 2;
                end

            case 2 % Hover
                % hover for 3 seconds
                hover_time = 3;
                num_iter = round(hover_time/dt, 0);
                hover(num_iter, iter);
                target_iter_stamp = iter + num_iter;    %%target_iter_stamp = 441
                tracking_start = target_iter_stamp;

                if ready_to_land
                    current_state = 4;
                    landing_start = iter*0.01+3;
                else
                    current_state = 3;
                end

            case 3 % Tracking
%                 disp("tracking")  
%                 disp(iter)
                tracking_time = 10;
                traGenerator10(iter, time, tracking_time, tracking_start);
                if iter == tracking_start + round(tracking_time/dt, 0)
                    ready_to_land = true;
                    current_state = 2;
                end
                                   
            case 4 % Land
%                 disp(iter)
                landing_time = 20-landing_start;
                landing_10(iter,time,landing_time, round(landing_start/0.01,0));

        end
    end

% 
%     x2 = 1350;
%     x3 = 1500;
%     if iter>x2 && iter<x3
%         range_x = 1;    %smaller the better 1~2
%         range_y = 5;    %larger the better  1~5
%         ctrl.Kp = [17/range_x 17/range_y 20]';
%         ctrl.Kv = [6.6/range_x 6.6/range_y 9]';
%         ctrl.Kr = [190/range_x 198/range_y 80]';
%         ctrl.Kw = [30/range_x 30/range_y 17.88]';
%     else
%         ctrl.Kp = [17 17 20]';
%         ctrl.Kv = [6.6 6.6 9]';
%         ctrl.Kr = [190 198 80]';
%         ctrl.Kw = [30 30 17.88]';
%     end
%  
    update_state(iter,ctrl,plant_params, dt);

    time = time + dt;
end

%% ************************ plot the trajectory *********************
disp('Initializing figures...');
start_point = 1;
end_point = 1990;
x = state.pos(1,:);
y = state.pos(2,:);
z = state.pos(3,:);

figure(1)
subplot(1,3,1);
plot(x(start_point:end_point));
hold on
title("x")
subplot(1,3,2);
plot(y(start_point:end_point));
hold on
title("y")
subplot(1,3,3);
plot(z(start_point:end_point));
hold on
title("z")

figure(1)
subplot(1,3,1);
plot(des_state.pos(1,start_point:end_point));
hold on
title("x")
subplot(1,3,2);
plot(des_state.pos(2,start_point:end_point));
hold on
title("y")
subplot(1,3,3);
plot(des_state.pos(3,start_point:end_point));
hold on
title("z")


figure('Name', "error_pos")
subplot(1,3,1);
plot(des_state.pos(1,start_point:end_point)-state.pos(1,start_point:end_point));
hold on
title("error_x")
subplot(1,3,2);
plot(des_state.pos(2,start_point:end_point)-state.pos(2,start_point:end_point));
hold on
title("error_y")
subplot(1,3,3);
plot(des_state.pos(3,start_point:end_point)-state.pos(3,start_point:end_point));
hold on
title("error_z")

% figure('Name',"error vel")
% subplot(1,3,1);
% plot(des_state.vel(1,start_point:end_point)-state.vel(1,start_point:end_point));
% hold on
% title("error xvel")
% subplot(1,3,2);
% plot(des_state.vel(2,start_point:end_point)-state.vel(2,start_point:end_point));
% hold on
% title("error yvel")
% subplot(1,3,3);
% plot(des_state.vel(3,start_point:end_point)-state.vel(3,start_point:end_point));
% hold on
% title("error zvel")
% 
% 
% figure('Name',"error rot")
% subplot(1,3,1);
% plot(des_state.rot(1,start_point:end_point)-state.rot(1,start_point:end_point));
% hold on
% title("error phi")
% 
% subplot(1,3,2);
% plot(des_state.rot(2,start_point:end_point)-state.rot(2,start_point:end_point));
% hold on
% title("error theta")
% 
% subplot(1,3,3);
% plot(des_state.rot(3,start_point:end_point)-state.rot(3,start_point:end_point));
% hold on
% title("error yaw")
% 
% 
% figure('Name', "error_omega")
% subplot(1,3,1);
% plot(des_state.omega(1,start_point:end_point)-state.omega(1,start_point:end_point));
% hold on
% title("error phivel")
% 
% subplot(1,3,2);
% plot(des_state.omega(2,start_point:end_point)-state.omega(2,start_point:end_point));
% hold on
% title("error thetavel")
% 
% subplot(1,3,3);
% plot(des_state.omega(3,start_point:end_point)-state.omega(3,start_point:end_point));
% hold on
% title("error yawvel")
% 
% 
% figure(10)
% plot(des_state.acc(3,start_point:end_point));
% hold on

