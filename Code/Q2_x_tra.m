
function Q2_x_tra(numIter)
	global des_state;
	iter_interval = round(numIter/5,0);
	for ii = 1:5
		des_state.pos(1,(ii-1)*iter_interval+1:ii*iter_interval) = ii*0.1;
	end
