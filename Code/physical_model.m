
%% The physical model of the Quadcopter
% Using the "Newton-Euler Equations"
% input: 4*force of each motor, X(time) state of the Quadcopter (The linear velocity)
% output: the dX(time) of the Quadcopter (The linear acceleration)

function sdot = physical_model(iter, F_act,M_act,s,plant_params)
	% Assign states
	phi = s(7);
	theta = s(8);
	yaw = s(9);

    % unit for angle --- degree
	p = s(10);
	q = s(11);
	r = s(12);

	% compute the rotation matrix
	R_yaw =   [[cos(yaw), -sin(yaw),          0];...
			   [sin(yaw),  cos(yaw),          0];...
			   [0,                0,          1]];
	R_theta = [[cos(theta),       0, sin(theta)];...
			   [0,                1,          0];...
			   [-sin(theta),      0, cos(theta)]];
	R_phi =   [[1,                0,          0];...
			   [0,     	   cos(phi),  -sin(phi)];...
			   [0, 	   	   sin(phi),   cos(phi)]];

	wRb = R_yaw*R_theta*R_phi;

	% accelaration
	accel = 1 / plant_params.mass * (wRb * [0; 0; F_act] - [0; 0; plant_params.mass * plant_params.gravity]);

	% angular 
	omega =    [p;q;r];
	pqrdot   = plant_params.inertia \ (M_act - cross(omega, plant_params.inertia*omega));  % inv(I) * (M - cross(w,Iw))
    
    % check for crazy angle
    if theta > (45/180*pi) || phi > (45/180*pi)
       disp(iter);
    end
    
    % generate rotation between pqr and rpy
    R_angle = [cos(theta) 0 -cos(phi)*sin(theta);
    		   0          1             sin(phi);
    		   sin(theta) 0  cos(phi)*cos(theta)];
    
	% Assign sdot	
	sdot = zeros(12,1);

	sdot(1:3) = s(4:6);
	sdot(4:6) = accel;
	sdot(7:9) = R_angle \ omega;
	sdot(10:12) = pqrdot;

end


