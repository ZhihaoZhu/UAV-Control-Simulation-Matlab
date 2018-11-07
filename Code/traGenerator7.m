function traGenerator7(iter,time,T, T_start)
    global des_state;
    persistent alpha1
    dt = 0.01;
    
    waypoint = [0  0  1 0;
                0  0  10 0];

    numInterval = 1;
    if iter-T_start == 0
        A = [1 0 0 0 0  0;  
             1 1 1 1 1  1;
             0 1 0 0 0  0;
             0 1 2 3 4  5;
             0 0 2 0 0  0;
             0 0 2 6 12 20];


        B1 = [waypoint(1,:);waypoint(2,:);zeros(1,4);[0,0,0,0];zeros(1,4);[0,0,0,0]];

        alpha1 = A \ B1;
       
        scale = (time-T_start*dt)/(T/numInterval);
        pos = alpha1'*[1;scale; scale^2;   scale^3; scale^4; scale^5];
        vel = alpha1'*[0;    1; 2*scale; 3*scale^2; 4*scale^3; 5*scale^4]/(T/numInterval);
        acc = alpha1'*[0;    0;      2;  6*scale; 12*scale^2; 20*scale^3]/(T/numInterval)^2;

        des_state.pos(:,iter) = pos(1:3);
        des_state.vel(:,iter) = vel(1:3);
		des_state.acc(:,iter) = acc(1:3);
        des_state.yaw(iter) = pos(4);
    else

    	coeff = alpha1;
        scale = (time-T_start*dt)/(T/numInterval);
        
		pos = coeff'*[1;scale; scale^2;   scale^3; scale^4; scale^5];
        vel = coeff'*[0;    1; 2*scale; 3*scale^2; 4*scale^3; 5*scale^4]/(T/numInterval);
        acc = coeff'*[0;    0;      2;  6*scale; 12*scale^2; 20*scale^3]/(T/numInterval)^2;
        
        des_state.pos(:,iter) = pos(1:3);
        des_state.yaw(iter) = pos(4);
        des_state.vel(:,iter) = vel(1:3);
		des_state.acc(:,iter) = acc(1:3);
        des_state.yawdot(iter) = vel(4);

    end
end