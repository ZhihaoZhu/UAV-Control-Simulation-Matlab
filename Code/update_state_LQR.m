%% This function generate the control value for each time iteration
% input: the error state: E; The desired state: X;
% output: The force and 3 torques

function update_state_LQR(iter,ctrl,plant_params, dt)
	global state;
    global des_state;
	yaw = state.rot(3,iter);
	yawdot = state.omega(3,iter);

	g = plant_params.gravity;
	I = plant_params.inertia;
	m = plant_params.mass;

	A = zeros(12,12);
	A(7,4) = g*sin(yaw);
	A(7,5) = g*cos(yaw);
	A(8,4) = -g*cos(yaw);
	A(8,5) = g*sin(yaw);
	A(10,4) = (I(2,2) - I(3,3))*yawdot^2/I(1,1);
	A(11,5) = (I(1,1) - I(3,3))*yawdot^2/I(2,2);
	A(1:6,7:12) = eye(6);
	A(11,10) = -(I(1,1) - I(3,3))*yawdot/I(2,2);
	A(10,11) = (I(2,2) - I(3,3))*yawdot/I(1,1);

	B = zeros(12,4);
	B(9:12,1:4) = [1/m,      0,  0,      0;
				   0,  1/I(1,1), 0,      0;
				   0,  0, 1/I(2,2),      0;
				   0.  0,    0,   1/I(3,3)];
    % set Q and R
	Q = 0.1* eye(12);
    Q(1,1) = 40;
    Q(2,2) = 40;
    Q(3,3) = 100;
    Q(9,9) = 0.5;
    Q(6,6) = 2;
	R = 2.4  * eye(4);
    R(1,1) = 5;
    R(2,2) = 10;
    R(3,3) = 10;
 

	K = lqr(A, B, Q, R);

	Yd = flat_des_state(iter,g);
	X = flat_state(iter);

	% generate desire state 
	V = -((A - B*K)\B) \ Yd;
    
	u = V - K*X;  % u is the control value
	acc = B*u;
    acc = acc(9:12);
    F_des = m*acc(1) + m*g;
    M_des =  I * acc(2:4);
        
    % The motor control
    d = plant_params.arm_length;
	mixMatrix = [[ctrl.ct, ctrl.ct, ctrl.ct, ctrl.ct];
				 [0, d*ctrl.ct, 0, -d*ctrl.ct];
				 [-d*ctrl.ct, 0, d*ctrl.ct, 0];
				 [-ctrl.ms, ctrl.ms, -ctrl.ms, ctrl.ms]];

	F_M = mixMatrix * state.wSpeed(:,iter).^2;
    F_act = F_M(1);
    M_act = F_M(2:4);

	% using the Midpoint Method to update speed of the 4 propellers
	des_wSpeed = round(real(sqrt((mixMatrix \ [F_des;M_des]))));  % equal to inv(A) * b
	
	% w_dot = km * (w_des - w_cur)
	wSpeed_dot = plant_params.motor_constant .* (des_wSpeed - state.wSpeed(:,iter));
	state.wSpeed(:,iter+1) = min(max(state.wSpeed(:,iter) + wSpeed_dot*dt,plant_params.rpm_min),plant_params.rpm_max);
    s = zeros(12,1);
	s(1:3) = state.pos(:,iter);
	s(4:6) = state.vel(:,iter);
	s(7:9) = state.rot(:,iter);
	s(10:12) = state.omega(:,iter);
    phi = s(7);
	theta = s(8);
	yaw = s(9);
    state.yaw(iter) = yaw; 
	p = s(10);
	q = s(11);
	r = s(12);
	R_yaw =   [[cos(yaw), -sin(yaw),          0];...
			   [sin(yaw),  cos(yaw),          0];...
			   [0,                0,          1]];
	R_theta = [[cos(theta),       0, sin(theta)];...
			   [0,                1,          0];...
			   [-sin(theta),      0, cos(theta)]];
	R_phi =   [[1,                0,          0];...
			   [0,     	   cos(phi),  -sin(phi)];...
			   [0, 	   	   sin(phi),   cos(phi)]];

	x = R_yaw*R_theta*R_phi;
	accel = 1 / plant_params.mass * (x * [0; 0; F_act] - [0; 0; plant_params.mass * plant_params.gravity]);
	omega =    [p;q;r];
	pqrdot   = plant_params.inertia \ (M_act - cross(omega, plant_params.inertia*omega));   
    R_angle = [cos(theta) 0 -cos(phi)*sin(theta);
    		   0          1             sin(phi);
    		   sin(theta) 0  cos(phi)*cos(theta)];
    state_dot = zeros(12,1);
	state_dot(1:3) = s(4:6);
	state_dot(4:6) = accel;
	state_dot(7:9) = R_angle \ omega;
	state_dot(10:12) = pqrdot;
    state.pos(:,iter+1) = state.pos(:,iter) + state_dot(1:3).*dt;
	state.vel(:,iter+1) = state.vel(:,iter) + state_dot(4:6).*dt;
	state.rot(:,iter+1) = state.rot(:,iter) + state_dot(7:9).*dt;
	state.omega(:,iter+1) = state.omega(:,iter) + state_dot(10:12).*dt;
    
end

