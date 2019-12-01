%% The function generate the coefficients of a 7th order trajectory
%  Input: the position, velocity ... at waypoints; start time; end time
%       conditions = [ init_pos, end_pos,   init_vel, end_vel,
%                      init_acc, end_acc,   init_jerk, end_jerk]
%                            
%  Output: 8 coefficients of a polynomial

function [Coef] = compute_coef(conditions, ts, tf)

    A = [   ts^7        ts^6        ts^5        ts^4    ts^3    ts^2  ts    1;
            tf^7        tf^6        tf^5        tf^4    tf^3    tf^2  tf    1;
            7*ts^6      6*ts^5      5*ts^4      4*ts^3  3*ts^2  2*ts  1     0;
            7*tf^6      6*tf^5      5*tf^4      4*tf^3  3*tf^2  2*tf  1     0;
            42*ts^5     30*ts^4     20*ts^3     12*ts^2 6*ts    2     0     0;
            42*tf^5     30*tf^4     20*tf^3     12*tf^2 6*tf    2     0     0;
            210*ts^4    120*ts^3    60*ts^2     24*ts   6       0     0     0;
            210*tf^4    120*tf^3    60*tf^2     24*tf   6       0     0     0]; 
    Coef = A\conditions;    

end