% This function generate a line trajectory in Z-axis

function z_direction_line(numiter,ori_height)
	global des_state;
	h = 1;
	des_state.pos(3,1:round(numiter/2,0)) = h;
	des_state.pos(3,round(numiter/2,0)+1:numiter) = ori_height;
end
