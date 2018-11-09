
function X = flat_state(iter)
	global state;

	X = zeros(12,1);
	
	X(1:3) = state.pos(:,iter);
	X(4:6) = state.rot(:,iter);
	X(7:9) = state.vel(:,iter);
	X(10:12) = state.omega(:,iter);
end