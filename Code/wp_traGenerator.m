% This function compute the coeffcient alpha for trajectory generator
% Input wpvec; wptimes; alpha;
% Output: None, but updated alpha;

function wp_traGenerator(iter,time,T)
	global des_state;
	persistent Alpha
    waypoint = [0  0  1;
				2  1  1;
				0  2  1;
				-2 1  1;
                0  0  1];
    numInterval = 4;
	if iter == 1
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
		
        scale = time/(T/numInterval);
		des_state.pos(:,iter) = alpha1'*[1;scale; scale^2;   scale^3];
		des_state.vel(:,iter) = alpha1'*[0;    1; 2*scale; 3*scale^2];
% 		des_state.acc(:,iter) = alpha1'*[0;    0;      2;  6*scale; 12*scale^2; 20*scale^3];
    else
        wptimes = 0:T/numInterval:T;
    	intervel_index = find(wptimes >= time,1)-1;
    	coeff = cell2mat(Alpha(intervel_index));
		scale = (time-wptimes(intervel_index))/(T/numInterval);
		des_state.pos(:,iter) = coeff'*[1;scale; scale^2;   scale^3];
		des_state.vel(:,iter) = coeff'*[0;    1; 2*scale; 3*scale^2];
% 		des_state.acc(:,iter) = coeff'*[0;    0;      2;  6*scale; 12*scale^2; 20*scale^3];
	end
end