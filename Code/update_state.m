%% This function generate the control value for each time iteration
% input: the error state: E; The desired state: X;
% output: The force and 3 torques

function update_state(iter,ctrl,plant_params, dt)
	global state;
    global des_state;
	Exyz = ctrl.Kp.*(des_state.pos(:,iter) - state.pos(:,iter))...
		   + ctrl.Kv.*(des_state.vel(:,iter) - state.vel(:,iter));
	F_des = plant_params.mass*(plant_params.gravity + des_state.acc(3,iter) + Exyz(3));

	des_R = 1/plant_params.gravity...
			*[[sin(des_state.yaw(iter)), -cos(des_state.yaw(iter))];...
			  [cos(des_state.yaw(iter)),  sin(des_state.yaw(iter))]]...
			*[Exyz(1)+des_state.acc(1,iter);...
			  Exyz(2)+des_state.acc(2,iter)];
	 
	des_R_dot = 1/plant_params.gravity...
				*[[cos(des_state.yaw(iter)), sin(des_state.yaw(iter))];...
	  			  [-sin(des_state.yaw(iter)),cos(des_state.yaw(iter))]]...
				*([Exyz(1)+des_state.acc(1,iter);...
                   Exyz(2)+des_state.acc(2,iter)]...
                .*des_state.yawdot(iter));
	
	desire_rotation = [des_R(1),des_R(2),des_state.yaw(iter)]';
 	des_angVelocity = [des_R_dot(1),des_R_dot(2),des_state.yawdot(iter)]';
	M_des = plant_params.inertia*(ctrl.Kr.*(desire_rotation-state.rot(:,iter))... 
			+ ctrl.Kw.*(des_angVelocity - state.omega(:,iter)));
        
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

