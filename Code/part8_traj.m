
%% The function generate the coefficients of a 7th order trajectory
%  Input: the position, velocity ... at waypoints; start time; end time
%       conditions = [ init_pos, end_pos,   init_vel, end_vel,
%                      init_acc, end_acc,   init_jerk, end_jerk]
%  Output: 8 coefficients of a polynomial

function make_traj_ellipse(state_idx, [waypt_1,waypt_2,waypt_3,waypt_4])
    global des_state

    tracking_vel = 1;
    tracking_acc = 0.5*tracking_vel;
    t = 0:0.01:4;
    conditions_1 = [0 2 tracking_vel 0 tracking_acc 0 0 0]';
    res_1 = compute_coef(conditions_1, 0, 1);
    % traj_1 = poly2sym(sym(res_1),t);
    draw_1 = polyval(res_1,t);

    conditions_2 = [2 0 0 -tracking_vel tracking_acc 0 0 0]';
    res_2 = compute_coef(conditions_2, 1, 2);
    % traj_2 = poly2sym(sym(res_2),t);
    draw_2 = polyval(res_2,t);

    conditions_3 = [0 -2 -tracking_vel 0 tracking_acc 0 0 0]';
    res_3 = compute_coef(conditions_3, 2, 3);
    % traj_3 = poly2sym(sym(res_3),t);
    draw_3 = polyval(res_3,t);

    conditions_4 = [-2 0 0 tracking_vel tracking_acc 0 0 0]';
    res_4 = compute_coef(conditions_4, 3, 4);
    % traj_4 = poly2sym(sym(res_3),t);
    draw_4 = polyval(res_4,t);

    des_state = zeros(1,numel(t));

    for ii = 1:numel(t)
        if ii<=100
            des_state(1,ii) = draw_1(1,ii);
        elseif ii > 100 && ii <=200
            des_state(1,ii) = draw_2(1,ii);   
        elseif ii > 200 && ii <=300
            des_state(1,ii) = draw_3(1,ii);
        else
            des_state(1,ii) = draw_4(1,ii);
        end
    end
end