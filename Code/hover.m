% This function generate hovering trajectory common 

function hover(numIter)
	global des_state;
	% set step in Z direction position 0.5m
	des_state.pos(3,:) = 0.5;
	% give 5 steps in x direction
	iter_interval = round(numIter/5,0);
	for ii = 1:5
		des_state.pos(1,(ii-1)*iter_interval+1:ii*iter_interval) = ii*0.1;
	end


