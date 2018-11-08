function [err_RPY, err_dRPY] = calculate_error(g,dd_err_xyz,actual_RPY, actual_dRPY,des_state, d_des_state,dd_des_state)
    dd_ex = dd_err_xyz(1);
    dd_x = dd_des_state(1);
    dd_ey = dd_err_xyz(2);
    dd_y = dd_des_state(2);
    psi = des_state(4);
    
    d_psi = d_des_state(4);
    des_RPY = [ 1/g*(sin(psi)*(dd_ex+dd_x) - cos(psi)*(dd_ey+dd_y));
                1/g*(cos(psi)*(dd_ex+dd_x) + sin(psi)*(dd_ey+dd_y));
                                psi];
    d_des_RPY  = [1/g* ( cos(psi)*( (dd_ey+dd_y)*d_psi) - sin(psi)*(dd_ey+dd_x)*d_psi );
                  1/g* ( sin(psi)*( (dd_ey+dd_y)*d_psi) + cos(psi)*(dd_ey+dd_x)*d_psi );
                                d_psi ];

    err_RPY = des_RPY - actual_RPY;
    err_dRPY = d_des_RPY - actual_dRPY;
end