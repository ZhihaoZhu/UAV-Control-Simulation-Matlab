%% generate s_plot for animation

function s_plot = state2Plot(iter)
    global state;
	s_plot     = zeros(13,1);
	phi   = state.rot(1,iter);
	theta = state.rot(2,iter);
	psi   = state.rot(3,iter);
	Rot   = RPYtoRot_ZXY(phi, theta, psi);
	Quat  = RotToQuat(Rot);
	s_plot(1)  = state.pos(1,iter); 		  %x
	s_plot(2)  = state.pos(2,iter);        %y
	s_plot(3)  = state.pos(3,iter);        %z
	s_plot(4)  = state.vel(1,iter);        %xdot
	s_plot(5)  = state.vel(2,iter);        %ydot
	s_plot(6)  = state.vel(3,iter);        %zdot
	s_plot(7)  = Quat(1);                  %qw
	s_plot(8)  = Quat(2);                  %qx
	s_plot(9)  = Quat(3);                  %qy
	s_plot(10) = Quat(4);                  %qz
	s_plot(11) = state.omega(1,iter);      %p
	s_plot(12) = state.omega(2,iter);      %q
	s_plot(13) = state.omega(3,iter);      %r
	
end