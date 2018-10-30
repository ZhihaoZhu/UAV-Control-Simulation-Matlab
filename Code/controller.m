
%% This function generate the control value for each time iteration
% input: the error state: E; The desired state: X;
% output: The force and 3 torques

function [F_des, M_des] = controller(iter,ctrl,plant_params)
	global state;
    global des_state;
	% ************************ outter loop *************************
	Exyz = ctrl.Kp.*(des_state.pos(:,iter) - state.pos(:,iter))...
		   + ctrl.Kv.*(des_state.vel(:,iter) - state.vel(:,iter));
	
	% generate the total thrust
	F_des = plant_params.mass*(plant_params.gravity + des_state.acc(3,iter) + Exyz(3));

    if des_state.yaw > (10/180*pi)
        disp("Oppp")
    end
	% ************************ inner loop *************************
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

	% generate the desire torque
	M_des = plant_params.inertia*(ctrl.Kr.*(desire_rotation-state.rot(:,iter))... 
			+ ctrl.Kw.*(des_angVelocity - state.omega(:,iter)));

end

