
function Q3_tra(iter,time,T)
	global des_state;
	persistent Alpha
	waypoint = [0 0 0;
                0 0 1 ;
				0 0 0];
	numInterval = 2;
	if iter == 1
		A = [1 0 0 0  0  0   0   0;
			 1 1 1 1  1  1   1   1;
			 0 1 0 0  0  0   0   0;
			 0 1 2 3  4  5   6   7;
             0 0 2 0  0  0   0   0;
             0 0 2 6 12 20  30  42;
             0 0 0 6  0  0   0   0;
             0 0 0 6 24 60 120 210];

		B1 = [waypoint(1,:);waypoint(2,:);zeros(1,3);zeros(1,3);zeros(1,3);zeros(1,3);zeros(1,3);zeros(1,3)];
        B2 = [waypoint(2,:);waypoint(3,:);zeros(1,3);zeros(1,3);zeros(1,3);zeros(1,3);zeros(1,3);zeros(1,3)];
		alpha1 = A \ B1;
        alpha2 = A \ B2;
        Alpha = {alpha1,alpha2};
        
        scale = time/(T/numInterval);
        pos = alpha1'*[1;scale;scale^2;   scale^3;   scale^4;    scale^5; scale^6;       scale^7];
        vel = alpha1'*[0;    1;2*scale; 3*scale^2; 4*scale^3;  5*scale^4; 6*scale^5;   7*scale^6]/(T/2);
        acc = alpha1'*[0;    0;      2;  6*scale; 12*scale^2; 20*scale^3; 30*scale^4; 42*scale^5]/(T/2)^2;
		des_state.pos(:,iter) = pos;
		des_state.vel(:,iter) = vel;
		des_state.acc(:,iter) = acc;
    else
		wptimes = 0:T/numInterval:T+0.00001;
    	intervel_index = find(wptimes >= round(time,4),1)-1;
    	coeff = cell2mat(Alpha(intervel_index));
		scale = (time-wptimes(intervel_index))/(T/numInterval);
		pos = coeff'*[1;scale;scale^2;   scale^3;   scale^4;    scale^5; scale^6;       scale^7];
        vel = coeff'*[0;    1;2*scale; 3*scale^2; 4*scale^3;  5*scale^4; 6*scale^5;   7*scale^6]/(T/2);
        acc = coeff'*[0;    0;      2;  6*scale; 12*scale^2; 20*scale^3; 30*scale^4; 42*scale^5]/(T/2)^2;
        des_state.pos(:,iter) = pos;
		des_state.vel(:,iter) = vel;
		des_state.acc(:,iter) = acc;
	end
end