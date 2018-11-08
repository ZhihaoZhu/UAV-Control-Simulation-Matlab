close all;
clear;
clc;

%physical parameters
mass = 1;
I = diag([0.0033 0.0033 0.005]);
g = 9.8;
ct = 8.07e-9; %thrust_coefficient
cq = 0.017*8.07e-9; 
km = 36.5;%motor_constant
arm = 0.1103; % arm_length            
rpm_max = 20000;
rpm_min = 3000;

%controller parameters

pos_kp = [17 17 20]';
pos_kd  = [6.6 6.6 9]';
RPY_kp = [190 198 80]';
RPY_kd = [90 30 17.88]';


%simulation parameters
end_time = 5;
dt = 0.01;
tspan = linspace(0,end_time,end_time*1/dt+1);
interval_cmd = 0.3;

%placeholders
pos_xyz = zeros(3,numel(tspan));
vel_xyz = zeros(3,numel(tspan));
pos_RPY = zeros(3,numel(tspan));
vel_RPY = zeros(3,numel(tspan));
acc_xyz = zeros(3,numel(tspan));
acc_RPY = zeros(3,numel(tspan));

dd_err_xyz = zeros(3,numel(tspan));
dd_err_heading = zeros(numel(tspan));

cmd_thrust = zeros(1,numel(tspan));
net_force_inertia = zeros(3,numel(tspan));
cmd_tau = zeros(3,numel(tspan));
spin = zeros(4,numel(tspan));

dd_des_state = zeros(4,numel(tspan));
d_des_state = zeros(4,numel(tspan));
des_state = zeros(4,numel(tspan));


err_RPY = zeros(3,numel(tspan)-1);
err_xyz = zeros(3,numel(tspan)-1);

acc_spin = zeros(4,numel(tspan));
actual_spin = 3000*ones(4,numel(tspan));
actual_force_inertia = zeros(3,numel(tspan));
actual_tau = zeros(3,numel(tspan));

%compute morot spin
syms CT CQ d


A = [   CT      CT      CT      CT;
        0       d*CT    0    -d*CT;
        -d*CT   0       d*CT     0;
        -CQ     CQ     -CQ      CQ];
compute_spin(CT,CQ,d) = [A];

B = [ 1/(4*CT),           0, -1/(2*CT*d), -1/(4*CQ);
      1/(4*CT),  1/(2*CT*d),           0,  1/(4*CQ);
      1/(4*CT),           0,  1/(2*CT*d), -1/(4*CQ);
      1/(4*CT), -1/(2*CT*d),           0,  1/(4*CQ)];  
compute_inv_coeff(CT,CQ,d) = [B];

%set intial position
pos_xyz(3,1) = 0.5;
pos_xyz(1,1) = 0;

%generate commmand
des_state(3,:) = 0.5;

des_x = 0;
for time = 1:numel(tspan)
    if mod(time,interval_cmd/dt ) ==1 %&& time < numel(tspan)/2
        des_x = des_x + 0.1;
    end
    des_state(1,time) = des_x;
end


figure(1)


subplot(1,3,1);
plot(des_state(1,:));
title("desired position");
xlabel('0.01 sec'); 
ylabel('m') 
subplot(1,3,2);
plot(d_des_state(1,:));
xlabel('0.01 sec'); 
ylabel('m/s') 
title("desired velocity");   
subplot(1,3,3);
plot(dd_des_state(1,:));
xlabel('0.01 sec'); 
ylabel('m/s^2')
title("desired acceleration ");   


%% state matrices
X_dot = zeros(12,1);
X = zeros(12,1);
Q = 0.15*eye(12,12);
R = 2*eye(4,4);

%% simulation

inv_coef =compute_inv_coeff(ct, cq,arm); 
compute_spin = compute_spin(ct, cq,arm);
for time = 1:numel(tspan)-1
    
    [A,B,C] = state_matrices(mass,g,I,des_state(4,time),d_des_state(4,time));
 
    K = lqr(A,B,Q,R);
    U = (-C*((A-B*K)\B))\des_state(:,time) - K*X;
    X_dot = A*X + B*U;
    X = X + X_dot.*dt;
    
    err_xyz(1:3,time) = X(1:3) - des_state(1:3,time) ;
    dd_err_xyz(1:3,time) = X(7:9) - dd_des_state(1:3,time);
    % angular error
    [err_RPY(1:3,time), ~] = calculate_error(g,dd_err_xyz(:,time), pos_RPY(:,time) ,...
                             des_state(:,time), d_des_state(:,time),dd_des_state(:,time));
end





lengend_xyz = ["e_{x}","e_{y}","e_{z}"];
lengend_RPY = ["e_{phi}","e_{theta}","e_{yaw}"];



figure(2)
for ii = 1:3
    subplot(2,3,ii);
    plot(err_xyz(ii,:));
    hold on;
    title(lengend_xyz(1,ii));
    xlabel('0.01 sec'); 
    ylabel('m') ;
    for jj = 1:numel(tspan)-1
        if mod(jj,interval_cmd/dt) ==1
            plot(jj,err_xyz(ii,jj),'b--o')
            hold on
        end 
    end
    grid on;
end


for kk = 1:3
    subplot(2,3,3+kk);
    plot(err_RPY(kk,:));
    hold on;
    title(lengend_RPY(1,kk));
    xlabel('0.01 sec'); 
    ylabel('rad') 
    for jj = 1:numel(tspan)-1
        if mod(jj,interval_cmd/dt) ==1
            plot(jj,err_RPY(kk,jj),'b--o')
            hold on
        end 
    end
    grid on;
end




