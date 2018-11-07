
function hover(num_iter, iter_stamp)
	global des_state;
	des_state.pos(:, iter_stamp:iter_stamp + num_iter - 1) = repmat(des_state.pos(:, iter_stamp - 1), 1, 300);
end

