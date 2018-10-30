
% This function generate state vector of size 16*1 from structure state.

function s = state_vector(iter)
    global state;
	s = zeros(12,1);
	s(1:3) = state.pos(:,iter);
	s(4:6) = state.vel(:,iter);
	s(7:9) = state.rot(:,iter);
	s(10:12) = state.omega(:,iter);
end
	