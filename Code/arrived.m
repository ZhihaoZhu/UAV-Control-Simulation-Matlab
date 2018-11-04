
function isArrived = arrived(current, target, threshold)
	
    error_x = current(1) - target(1);
    error_y = current(2) - target(2);
    error_z = current(3) - target(3);
    
    if abs(error_x) <= threshold && abs(error_y) <= threshold && abs(error_z) <= threshold
        isArrived = true;
    else
        isArrived = false;
    end
end

