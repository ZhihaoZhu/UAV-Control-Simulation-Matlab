% This function generate hovering trajectory common 

function hover(num_iter, iter_stamp)
	global des_state;
	% set step in Z direction position 0.5m
	des_state.pos(:, iter_stamp:iter_stamp + num_iter - 1) = repmat(des_state.pos(:, iter_stamp - 1), 1, 300);
end

