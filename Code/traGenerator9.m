% This function compute the coeffcient alpha for trajectory generator
% Input wpvec; wptimes; alpha;
% Output: None, but updated alpha;

function traGenerator9(iter,time,T)
    global des_state;
    persistent Alpha

    
    v1 = 2.5;
    v2 = 2.5;
    a1 = 8.8;
    a2 = 2.5;
    waypoint = [0  0  1 0;
                2  1  1 0.5*pi;
                0  2  1 pi;
                -2 1  1 1.5*pi;
                0  0  1 2*pi;
                2  1  1 2.5*pi;
                0  2  1 3*pi;
                -2 1  1 3.5*pi;
                0  0  1 4*pi;
                2  1  1 4.5*pi;
                0  2  1 5*pi;
                -2 1  1 5.5*pi;
                0  0  1 6*pi];
%     waypoint = [0  0  1 0;
%                 2  1  1 0.5*pi;
%                 0  2  1 pi;
%                 -2 1  1 1.5*pi;
%                 0  0  1 2*pi;
%                 0  0  1 2.5*pi;
%                 2  1  1 3*pi;
%                 0  2  1 1*pi;
%                 -2 1  1 1.5*pi;
%                 0  0  1 2*pi;
%                 0  0  1 0;
%                 2  1  1 0.5*pi;
%                 0  2  1 1*pi;
%                 -2 1  1 1.5*pi;
%                 0  0  1 2*pi];

    numInterval = 12;
    if iter == 1
        A = [1 0 0 0 0  0;  
             1 1 1 1 1  1;
             0 1 0 0 0  0;
             0 1 2 3 4  5;
             0 0 2 0 0  0;
             0 0 2 6 12 20];


        B1 = [waypoint(1,:);waypoint(2,:);[0,0,0,0];[0,v1,0,0];zeros(1,4);[-a1,0,0,0]];
        B2 = [waypoint(2,:);waypoint(3,:);[0,v1,0,0];[-v2,0,0,0];[-a1,0,0,0];[0,-a2,0,0]];
        B3 = [waypoint(3,:);waypoint(4,:);[-v2,0,0,0];[0,-v1,0,0];[0,-a2,0,0];[a1,0,0,0]];
        B4 = [waypoint(4,:);waypoint(5,:);[0,-v1,0,0];[v2,0,0,0];[a1,0,0,0];zeros(1,4)];
        
        B5 = [waypoint(5,:);waypoint(6,:);[v2,0,0,0];[0,v1,0,0];zeros(1,4);[-a1,0,0,0]];
        B6 = [waypoint(6,:);waypoint(7,:);[0,v1,0,0];[-v2,0,0,0];[-a1,0,0,0];[0,-a2,0,0]];
        B7 = [waypoint(7,:);waypoint(8,:);[-v2,0,0,0];[0,-v1,0,0];[0,-a2,0,0];[a1,0,0,0]];
        B8 = [waypoint(8,:);waypoint(9,:);[0,-v1,0,0];[v2,0,0,0];[a1,0,0,0];zeros(1,4)];
        
        B9 = [waypoint(9,:);waypoint(10,:);[v2,0,0,0];[0,v1,0,0];zeros(1,4);[-a1,0,0,0]];
        B10 = [waypoint(10,:);waypoint(11,:);[0,v1,0,0];[-v2,0,0,0];[-a1,0,0,0];[0,-a2,0,0]];
        B11 = [waypoint(11,:);waypoint(12,:);[-v2,0,0,0];[0,-v1,0,0];[0,-a2,0,0];[a1,0,0,0]];
        B12 = [waypoint(12,:);waypoint(13,:);[0,-v1,0,0];[0,0,0,0];[a1,0,0,0];zeros(1,4)];

% %       For trajectory 2
% 
%         B1 = [waypoint(1,:);waypoint(2,:);[v2,0,0,0];[0,v1,0,0];zeros(1,4);[-a1,0,0,0]];
%         B2 = [waypoint(2,:);waypoint(3,:);[0,v1,0,0];[-v2,0,0,0];[-a1,0,0,0];[0,-a2,0,0]];
%         B3 = [waypoint(3,:);waypoint(4,:);[-v2,0,0,0];[0,-v1,0,0];[0,-a2,0,0];[a1,0,0,0]];
%         B4 = [waypoint(4,:);waypoint(5,:);[0,-v1,0,0];[v2,0,0,0];[a1,0,0,0];zeros(1,4)];
        
%       For trajectory 3
%         B1 = [waypoint(1,:);waypoint(2,:);[v2,0,0,0];[0,v1,0,0];zeros(1,4);[-a1,0,0,0]];
%         B2 = [waypoint(2,:);waypoint(3,:);[0,v1,0,0];[-v2,0,0,0];[-a1,0,0,0];[0,-a2,0,0]];
%         B3 = [waypoint(3,:);waypoint(4,:);[-v2,0,0,0];[0,-v1,0,0];[0,-a2,0,0];[a1,0,0,0]];
%         B4 = [waypoint(4,:);waypoint(5,:);[0,-v1,0,0];[0,0,0,0];[a1,0,0,0];zeros(1,4)];

        alpha1 = A \ B1;
        alpha2 = A \ B2;
        alpha3 = A \ B3;
        alpha4 = A \ B4;
        alpha5 = A \ B5;
        alpha6 = A \ B6;
        alpha7 = A \ B7;
        alpha8 = A \ B8;
        alpha9 = A \ B9;
        alpha10 = A \ B10;
        alpha11 = A \ B11;
        alpha12 = A \ B12;
        Alpha = {alpha1,alpha2,alpha3,alpha4,alpha5,alpha6,alpha7,alpha8,alpha9,alpha10,alpha11,alpha12};
        scale = time/(T/numInterval);
        pos = alpha1'*[1;scale; scale^2;   scale^3; scale^4; scale^5];
        vel = alpha1'*[0;    1; 2*scale; 3*scale^2; 4*scale^3; 5*scale^4]/(T/numInterval);
        acc = alpha1'*[0;    0;      2;  6*scale; 12*scale^2; 20*scale^3]/(T/numInterval)^2;

        des_state.pos(:,iter) = pos(1:3);
        des_state.vel(:,iter) = vel(1:3);
		des_state.acc(:,iter) = acc(1:3);
        des_state.yaw(iter) = pos(4);
    else
        wptimes = 0:T/numInterval:T+0.00001;
    	intervel_index = find(wptimes >= round(time,4),1)-1;
    	coeff = cell2mat(Alpha(intervel_index));
		scale = (time-wptimes(intervel_index))/(T/numInterval);
        
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