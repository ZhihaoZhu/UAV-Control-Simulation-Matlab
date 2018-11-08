function res = check_quality(state,goal,dt,label)
    res = zeros(1,4);
    rise_start = find(state > 0.1*goal);
    rise_end = find(state > 0.9*goal);
    res(1,1) = dt*(rise_end(1)-rise_start(1));
    
    settle = find(abs(goal-state) > 0.1*goal);
    res(1,2) = dt*settle(end);
    res(1,3) = state(end);
    res(1,4) = (max(state)-goal)/goal*100;
    disp(label);
    fprintf('rise time: %f \nsettle time: %f \nsteady state value: %f\novershoot: %f\n',res(1),res(2),res(3),res(4));
    
end