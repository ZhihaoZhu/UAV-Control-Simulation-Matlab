
%% This function update the current state of the Quadcopter
% Input: The derivative of state "sdot"; the state structure "state"
% Output: None (But update the state)

function update_state(iter,sdot,dt)
    global state;
	state.pos(:,iter+1) = state.pos(:,iter) + sdot(1:3).*dt;
	state.vel(:,iter+1) = state.vel(:,iter) + sdot(4:6).*dt;
	state.rot(:,iter+1) = state.rot(:,iter) + sdot(7:9).*dt;
	state.omega(:,iter+1) = state.omega(:,iter) + sdot(10:12).*dt;
end
