
%% The function takes 
% output the derivative of the state vector

function sdot = QuadKit()
	current_state = state_vectore(iter,state);
	
	% update the trajector
	des_state = traj_generator(time, current_state, waypoints); 
end