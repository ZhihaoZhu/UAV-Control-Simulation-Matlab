
%% The function generate the desire state of trajectory
%  Input: X(time) The state of Quadcopter; Waypoints; Time index
%  Output: desire trajectory for each time index
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot

function  make_trajectory(iter, time)
    global des_state;
    global wptimes;
    global wpvec

    distList = wpvec(2:end,:)-wpvec(1:end-1,:);
    timeIntervalList = wptimes(2:end) - wptimes(1:end-1);
    zeroVec = zeros(1,4);

    % find which interval it is traversing
    interval_index = find( wptimes >= time,1)-1;
    timeInterval = timeIntervalList(iterval_index);

    S0Cell = (0-wptimes(interval_index))/timeInterval;
    SnCell = (wptimes(end)- wptimes(interval_index))/timeInterval;


    A=[1,   0,   0,        0,        0,        0,        0,        0;
       1,   1,   1,        1,        1,        1,        0,        0;
       0,   1,   2*S0Cell, 3*S0Cell, 4*S0Cell, 5*S0Cell, 6*S0Cell, 7*S0Cell;
       0,   1,   2*SnCell, 3*SnCell, 4*SnCell, 5*SnCell, 6*SnCell, 7*SnCell;
       0,   1,   2,        3,        4		   5,        6		   7;


    b=[waypoints0(:,t_index-1);
       waypoints0(:,t_index);
       zeroVec;zeroVec;zeroVec;zeroVec];

end

