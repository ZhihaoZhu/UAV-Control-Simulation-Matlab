
function Yd = flat_des_state(iter,g)
	global des_state;

	Yd = zeros(12,1);

    des_state.rot(1:2,iter) = 1/g...
							*[[sin(des_state.yaw(iter)), -cos(des_state.yaw(iter))];...
			  				  [cos(des_state.yaw(iter)),  sin(des_state.yaw(iter))]]...
							*[des_state.acc(1,iter);...
							  des_state.acc(2,iter)];
	des_state.omega(1:2,iter) = 1/g...
							  *[[cos(des_state.yaw(iter)), sin(des_state.yaw(iter))];...
				  			    [-sin(des_state.yaw(iter)),cos(des_state.yaw(iter))]]...
							  *([des_state.acc(1,iter);...
                                 des_state.acc(2,iter)]...
                			  .*des_state.yawdot(iter));
	des_state.rot(3,iter) = des_state.yaw(iter);
    des_state.omega(3,iter) = des_state.yawdot(iter);
	
	Yd(1:3) = des_state.pos(:,iter);
	Yd(4:6) = des_state.rot(:,iter);
	Yd(7:9) = des_state.vel(:,iter);
	Yd(10:12) = des_state.omega(:,iter);

end