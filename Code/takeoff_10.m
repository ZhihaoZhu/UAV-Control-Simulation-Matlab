
function takeoff(height, iter_stamp)
	global des_state;
	% set step in Z direction position 0.5m
	des_state.pos(3, iter_stamp:end) = height;
end

