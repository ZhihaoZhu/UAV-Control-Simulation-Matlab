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

pos_kp = [30 17 20]';
pos_kd  = [6.6 6.6 9]';
RPY_kp = [190 198 80]';
RPY_kd = [30 30 17.88]';


%simulation parameters
end_time = 10;
dt = 0.01;
tspan = linspace(0,end_time,end_time*1/dt+1);


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
err_dxyz = zeros(3,numel(tspan)-1);
err_dRPY = zeros(3,numel(tspan)-1);

acc_spin = zeros(4,numel(tspan));
actual_spin = 15487*ones(4,numel(tspan));
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
pos_xyz(3,1) = 0;
pos_xyz(1,1) = 0;

%% generate commmand
set_time = 2;
syms c0 c1 c2 c3 c4 c5 c6 c7 t

C = [c0 c1 c2 c3 c4 c5 c6 c7];
f = C(8)*t.^7+C(7)*t.^6+C(6)*t.^5+C(5)*t.^4+C(4)*t.^3+C(3)*t.^2+C(2)*t+C(1);
eqn1 = subs(f,t,0) == 0;
eqn2 = subs(f,t,set_time) == 1;
g_ = diff(f);
eqn3 = subs(g_,t,0) ==0;
eqn4 = subs(g_,t,set_time) ==0;
h = diff(g_);
eqn5 = subs(h,t,0) ==0;
eqn6 = subs(h,t,set_time) ==0;
j = diff(h);
eqn7 = subs(j,t,0) ==0;
eqn8 = subs(j,t,set_time) ==0;

[A,B] = equationsToMatrix([eqn1, eqn2, eqn3, eqn4, eqn5, eqn6, eqn7, eqn8], [c0 c1 c2 c3 c4 c5 c6 c7]);
res = linsolve(A,B);

pos = res(8)*t.^7+res(7)*t.^6+res(6)*t.^5+res(5)*t.^4+res(4)*t.^3+res(3)*t.^2+res(2)*t+res(1);
vel = diff(pos);
acc = diff(vel);

%% command heading
set_time = 2;
syms g0 g1 g2 g3 g4 g5 g6 g7 t

G = [g0 g1 g2 g3 g4 g5 g6 g7];
f = G(8)*t.^7+G(7)*t.^6+G(6)*t.^5+G(5)*t.^4+G(4)*t.^3+G(3)*t.^2+G(2)*t+G(1);
eqn1 = subs(f,t,0) == 0;
eqn2 = subs(f,t,set_time) == deg2rad(15);
g_ = diff(f);
eqn3 = subs(g_,t,0) ==0;
eqn4 = subs(g_,t,set_time) ==0;
h = diff(g_);
eqn5 = subs(h,t,0) ==0;
eqn6 = subs(h,t,set_time) ==0;
j = diff(h);
eqn7 = subs(j,t,0) ==0;
eqn8 = subs(j,t,set_time) ==0;

[A,B] = equationsToMatrix([eqn1, eqn2, eqn3, eqn4, eqn5, eqn6, eqn7, eqn8], [g0 g1 g2 g3 g4 g5 g6 g7]);
res = linsolve(A,B);

hpos = res(8)*t.^7+res(7)*t.^6+res(6)*t.^5+res(5)*t.^4+res(4)*t.^3+res(3)*t.^2+res(2)*t+res(1);
hvel = diff(hpos);
hacc = diff(hvel);



des_state(3,:) = 1;
des_state(4,:) = deg2rad(15);
for time = 1:set_time/dt    
    
    dd_des_state(3,time) = subs(acc,t,time*dt);
    dd_des_state(4,time) = subs(hacc,t,time*dt);
    d_des_state(3,time) = subs(vel,t,time*dt);
    d_des_state(4,time) = subs(hvel,t,time*dt);
    des_state(3,time) =  subs(pos,t,time*dt);
    des_state(4,time) =  subs(hpos,t,time*dt);
    
end

figure(1)

subplot(2,3,1);
plot(des_state(3,:));
subplot(2,3,2);
plot(d_des_state(3,:));
subplot(2,3,3);
plot(dd_des_state(3,:));

subplot(2,3,4);
plot(des_state(4,:));
subplot(2,3,5);
plot(d_des_state(4,:));
subplot(2,3,6);
plot(dd_des_state(4,:));



%% state matrices
X_dot = zeros(12,1);
X = zeros(12,1);
Q = 0.15*eye(12,12);
R = 2*eye(4,4);

%% simulation

for time = 1:numel(tspan)
    
    [A,B,C] = state_matrices(mass,g,I,des_state(4,time),d_des_state(4,time));
 
    K = lqr(A,B,Q,R);
    U = (-C*((A-B*K)\B))\des_state(:,time) - K*X;
    X_dot = A*X + B*U;
    X = X + X_dot.*dt;
    
    err_xyz(1:3,time) = X(1:3) - des_state(1:3,time) ;
    err_dxyz(1:3,time) = X(7:9) - d_des_state(1:3,time) ;
    dd_err_xyz(1:3,time) = X_dot(7:9) - dd_des_state(1:3,time);
    % angular error

    pos_xyz(:,time) = X(1:3);
    pos_RPY(:,time) = X(4:6);
    vel_xyz(:,time) = X(7:9);
    vel_RPY(:,time) = X(10:12);
    [err_RPY(1:3,time), err_dRPY(1:3,time)] = calculate_error(g,dd_err_xyz(:,time), pos_RPY(:,time) ,vel_RPY(:,time),...
     des_state(:,time), d_des_state(:,time),dd_des_state(:,time));
end

check_quality(pos_xyz(3,:),1,dt,"height");
check_quality(pos_RPY(3,:),deg2rad(15),dt,"heading");
check_quality(vel_xyz(3,:),0,dt,"velocity");
check_quality(vel_RPY(3,:),0,dt,"angular velocity");

lengend_xyz = ["e_{x}","e_{y}","e_{z}"];
lengend_RPY = ["e_{phi}","e_{theta}","e_{yaw}"];

figure('NumberTitle', 'off', 'Name',strcat("Kp = ",mat2str(pos_kp), " Kd = ", mat2str(pos_kd)));
for ii = 1:3
    subplot(2,3,ii);
    plot(err_xyz(ii,:));
    hold on;
    title(lengend_xyz(1,ii));   
    xlabel('0.01 sec'); 
    ylabel('m/s') ;
    grid on;
end

for kk = 1:3
    subplot(2,3,3+kk);
    plot(err_RPY(kk,:));
    hold on;
    title(lengend_RPY(1,kk));
    xlabel('0.01 sec'); 
    ylabel('rad/s') ;
    grid on;
end
hold off;

lengend_dxyz = ["e_{dot-x}","e_{dot-y}","e_{dot-z}"];
lengend_dRPY = ["e_{dot-phi}","e_{dot-theta}","e_{dot-yaw}"];

figure('NumberTitle', 'off', 'Name',strcat("Kr = ",mat2str(RPY_kp), " Kw = ", mat2str(RPY_kd)));
for ii = 1:3
    subplot(2,3,ii);
    plot(err_dxyz(ii,:));
    hold on;
    title(lengend_dxyz(1,ii));
    xlabel('0.01 sec'); 
    ylabel('m/s^2') ;
    grid on;
end

for kk = 1:3
    subplot(2,3,3+kk);
    plot(err_dRPY(kk,:));
    xlabel('0.01 sec'); 
    ylabel('rad/s^2') ;
    hold on;
    title(lengend_dRPY(1,kk));
    grid on;
end

figure('NumberTitle', 'off', 'Name',"actual velocity profile(m/s)");
plot(vel_xyz(3,:));
xlabel('0.01 sec'); 
ylabel('m/s') ;

figure('NumberTitle', 'off', 'Name',"actual angular velocity profile(rad/s)");
plot(vel_RPY(3,:));
xlabel('0.01 sec'); 
ylabel('rad/s') ;
