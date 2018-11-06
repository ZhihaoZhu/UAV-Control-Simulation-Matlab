% This function compute the coeffcient alpha for trajectory generator
% Input wpvec; wptimes; alpha;
% Output: None, but updated alpha;

function traGenerator10(iter,time,T,T_start)
	global des_state;
    dt = 0.01;
	persistent Alpha
    waypoint = [0  0  0.5;
				2  1  0.5;
				0  2  0.5;
				-2 1  0.5;
                0  0  0.5];
    numInterval = 4;
	if iter-T_start == 0
        A = [1 0 0 0;  
			 1 1 1 1;
			 0 1 0 0;
			 0 1 2 3;];
        
		B1 = [waypoint(1,:);waypoint(2,:);zeros(1,3);[0,1.4,0]];
		B2 = [waypoint(2,:);waypoint(3,:);[0,1.4,0];[-3,0,0]];
		B3 = [waypoint(3,:);waypoint(4,:);[-3,0,0];[0,-1.4,0]];
        B4 = [waypoint(4,:);waypoint(5,:);[0,-1.4,0];zeros(1,3)];
        
		alpha1 = A \ B1;
		alpha2 = A \ B2;
		alpha3 = A \ B3;
        alpha4 = A \ B4;
		Alpha = {alpha1,alpha2,alpha3,alpha4};
		
        scale = (time-T_start*dt)/(T/numInterval);
		des_state.pos(:,iter) = alpha1'*[1;scale; scale^2;   scale^3];
		des_state.vel(:,iter) = alpha1'*[0;    1; 2*scale; 3*scale^2]/(T/numInterval);
    else
        wptimes = (T_start-1)*dt:T/numInterval:T+(T_start-1)*dt;
    	intervel_index = find(wptimes >= time,1)-1;
    	coeff = cell2mat(Alpha(intervel_index));
		scale = (time-wptimes(intervel_index))/(T/numInterval);
		des_state.pos(:,iter) = coeff'*[1;scale; scale^2;   scale^3];
		des_state.vel(:,iter) = coeff'*[0;    1; 2*scale; 3*scale^2]/(T/numInterval);
	end
end