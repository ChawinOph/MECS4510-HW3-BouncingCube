function valid = validRobot(point_masses, springs)
%VALIDROBOT Checks whether robot is valid
%   Makes sure that all springs are connected to masses.
%   However, some masses may not be connected to springs
    springsConnect = [springs.m];
    
    if isempty(find(springsConnect>length(point_masses),1))
        valid = 1;
    else
        valid = 0;
    end

end