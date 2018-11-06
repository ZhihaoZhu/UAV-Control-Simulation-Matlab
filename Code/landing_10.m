% This function compute the coeffcient alpha for trajectory generator
% Input wpvec; wptimes; alpha;
% Output: None, but updated alpha;

function landing_10(iter,time,T,T_start)
	global des_state;
    dt = 0.01;
	persistent alpha1
    waypoint = [0  0  0.5;
                0  0  0];
    numInterval = 1;
	if iter-T_start == 0
        A = [1 0 0 0;  
			 1 1 1 1;
			 0 1 0 0;
			 0 1 2 3;];
        
		B1 = [waypoint(1,:);waypoint(2,:);zeros(1,3);[0,0,0]];

       
		alpha1 = A \ B1;

        scale = (time-T_start*dt)/(T/numInterval);
		des_state.pos(:,iter) = alpha1'*[1;scale; scale^2;   scale^3];
		des_state.vel(:,iter) = alpha1'*[0;    1; 2*scale; 3*scale^2]/(T/numInterval);
    else
        
		scale = (time-T_start*dt)/(T/numInterval);
		des_state.pos(:,iter) = alpha1'*[1;scale; scale^2;   scale^3];
		des_state.vel(:,iter) = alpha1'*[0;    1; 2*scale; 3*scale^2]/(T/numInterval);
	end
end