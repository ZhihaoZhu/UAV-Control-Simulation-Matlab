% This function compute the coeffcient alpha for trajectory generator
% Input wpvec; wptimes; alpha;
% Output: None, but updated alpha;

function wp_traGenerator(iter,dt,T)
    global des_state;
    persistent Alpha
    waypoint = [0  0  0.5 0;
                2  1  0.5 0
                0  2  0.5 0;
                -2 1  0.5 0;
                0  0  0.5 0];

    numInterval = 4;
        A = [1 0 0 0 0  0;  
             1 1 1 1 1  1;
             0 1 0 0 0  0;
             0 1 2 3 4  5;
             0 0 2 0 0  0;
             0 0 2 6 12 20];

    B1 = [waypoint(1,:);waypoint(2,:);zeros(1,4);[0,2,0,0];zeros(1,4);[-8.8,0,0,0]];
	B2 = [waypoint(2,:);waypoint(3,:);[0,2,0,0];[-2.8,0,0,0];[-8.8,0,0,0];[0,-2.5,0,0]];
	B3 = [waypoint(3,:);waypoint(4,:);[-2.8,0,0,0];[0,-2,0,0];[0,-2.5,0,0];[8.8,0,0,0]];
    B4 = [waypoint(4,:);waypoint(5,:);[0,-2,0,0];zeros(1,4);[8.8,0,0,0];zeros(1,4)];

    alpha1 = A \ B1;
    alpha2 = A \ B2;
    alpha3 = A \ B3;
    alpha4 = A \ B4;
    Alpha = {alpha1,alpha2,alpha3,alpha4};
    
    true_timeList = iter*dt:dt:(iter*dt+T);
    local_timeList = dt:dt:T;
    
    for ii = 1:length(local_timeList)
        time = local_timeList(ii);
        index = round(true_timeList(ii)/dt);
        wptimes = 0:T/numInterval:T;
        intervel_index = find(wptimes >= time,1)-1;
        coeff = cell2mat(Alpha(intervel_index));
        scale = (time-wptimes(intervel_index))/(T/numInterval);
        
        pos = coeff'*[1;scale; scale^2;   scale^3; scale^4; scale^5];
        vel = coeff'*[0;    1; 2*scale; 3*scale^2; 4*scale^3; 5*scale^4]/(T/numInterval);
        acc = coeff'*[0;    0;      2;  6*scale; 12*scale^2; 20*scale^3]/(T/numInterval)^2;
        
        des_state.pos(:,index) = pos(1:3);
        des_state.yaw(index) = pos(4);
        des_state.vel(:,index) = vel(1:3);
		des_state.acc(:,index) = acc(1:3);
        des_state.yawdot(index) = vel(4);
    end

end