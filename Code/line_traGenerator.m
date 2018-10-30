% This function generate the trajectory for two point tracking

function line_traGenerator(iter,time,T)
	global des_state;
	persistent Alpha
	if iter == 1
		waypoint = [0 0 0.2 ;
					10 -10  10];
		% minimize the accerlation
		A = [1 0 0 0  0  0   0   0;
			 1 1 1 1  1  1   1   1;
			 0 1 0 0  0  0   0   0;
			 0 1 2 3  4  5   6   7;
             0 0 2 0  0  0   0   0;
             0 0 2 6 12 20  30  42;
             0 0 0 6  0  0   0   0;
             0 0 0 6 24 60 120 210];

		B = [waypoint(1,:);waypoint(2,:);zeros(1,3);zeros(1,3);zeros(1,3);zeros(1,3);zeros(1,3);zeros(1,3)];
		Alpha = A \ B;
        
        scale = time/T;
		des_state.pos(:,iter) = Alpha'*[1;scale;scale^2;   scale^3;   scale^4;    scale^5; scale^6;       scale^7];
		des_state.vel(:,iter) = Alpha'*[0;    1;2*scale; 3*scale^2; 4*scale^3;  5*scale^4; 6*scale^5;   7*scale^6];
		des_state.acc(:,iter) = Alpha'*[0;    0;      2;  6*scale; 12*scale^2; 20*scale^3; 30*scale^4; 42*scale^5];
    else
		scale = time/T;
		des_state.pos(:,iter) = Alpha'*[1;scale;scale^2;   scale^3;   scale^4;    scale^5; scale^6;       scale^7];
		des_state.vel(:,iter) = Alpha'*[0;    1;2*scale; 3*scale^2; 4*scale^3;  5*scale^4; 6*scale^5;   7*scale^6];
		des_state.acc(:,iter) = Alpha'*[0;    0;      2;  6*scale; 12*scale^2; 20*scale^3; 30*scale^4; 42*scale^5];
	end
end