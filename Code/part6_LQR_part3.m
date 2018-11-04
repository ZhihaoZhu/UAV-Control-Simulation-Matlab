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


%simulation parameters
end_time = 6;
mid_time = 2;
set_time = 2*mid_time;
dt = 0.01;
tspan = linspace(0,end_time,end_time*1/dt+1);
interval_cmd = 0.01;

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

%% generate commmand

syms c0 c1 c2 c3 a0 a1 a2 a3 t

C1 = [c0 c1 c2 c3];
asd = C1(4)*t.^3+C1(3)*t.^2+C1(2)*t+C1(1);

C2 = [a0 a1 a2 a3];
dsd = C2(4)*t.^3+C2(3)*t.^2+C2(2)*t+C2(1);

v_asd  = diff(asd); a_asd = diff(v_asd);
v_dsd  = diff(dsd); a_dsd = diff(v_dsd); 

eqn1 = subs(asd,t,0) == 0;
eqn2 = subs(v_asd,t,0) == 0;
eqn3 = subs(asd,t,mid_time) == 1;
eqn4 = subs(v_dsd,t,mid_time) - subs(v_asd,t,mid_time) ==0 ;
eqn5 = subs(a_dsd,t,mid_time) - subs(a_asd,t,mid_time) ==0 ;
eqn6 = subs(dsd,t,mid_time) == 1;
eqn7 = subs(v_dsd,t,set_time) == 0;
eqn8 = subs(dsd,t,set_time) == 0;


[A,B] = equationsToMatrix([eqn1, eqn2, eqn3, eqn4, eqn5, eqn6, eqn7, eqn8], [c0 c1 c2 c3 a0 a1 a2 a3 ]);
res = linsolve(A,B);

pos_asd = res(4)*t.^3+res(3)*t.^2+res(2)*t+res(1);
vel_asd = diff(pos_asd);
acc_asd = diff(vel_asd);

pos_dsd = res(8)*t.^3+res(7)*t.^2+res(6)*t+res(5);
vel_dsd = diff(pos_dsd);
acc_dsd = diff(vel_dsd);


for time = 1:set_time/dt    
    if time <= mid_time/dt
        dd_des_state(3,time) = subs(acc_asd,t,time*dt);
        d_des_state(3,time) = subs(vel_asd,t,time*dt);
        des_state(3,time) =  subs(pos_asd,t,time*dt);
    else
        dd_des_state(3,time) = subs(acc_dsd,t,time*dt);
        d_des_state(3,time) = subs(vel_dsd,t,time*dt);
        des_state(3,time) =  subs(pos_dsd,t,time*dt);
    end
end

figure(1)

subplot(1,3,1);
plot(des_state(3,:));
subplot(1,3,2);
plot(d_des_state(3,:));
subplot(1,3,3);
plot(dd_des_state(3,:));

%% state matrices
X_dot = zeros(12,1);
X = zeros(12,1);
Q = 0.15*eye(12,12);
R = 2*eye(4,4);

%% simulation

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
    grid on;
end

for kk = 1:3
    subplot(2,3,3+kk);
    plot(err_RPY(kk,:));
    hold on;
    title(lengend_RPY(1,kk));
    grid on;
end
hold off;





