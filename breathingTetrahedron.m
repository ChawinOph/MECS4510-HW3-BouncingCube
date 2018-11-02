function cube = breathingTetrahedron()
    % create 8 point masses
    mass = 0.1; % m
    cube_size = 0.1; % m
    z_offset = 0.0; % m 
    k = 500;
    % v_init = [0.5, 0, 0]; % m/s
    v_init = [0, 0, 0];

    % create positions of masses in the cube
    p = ones(8, 3)/2;
    p(1:4, 3) = 0;
    p([2:3,6:7], 1) = -p([2:3,6:7], 1);
    p([3:4,7:8], 2) = -p([3:4,7:8], 2);
    p(5:8, 3) = 2*p(5:8, 3);
    p = p*cube_size;

%     R = obj.rotationAxisAngle([1 0 0], pi/6); % tile around x axis by 30 degree
    R = eye(3); % Don't tilt
    p = R*p'; % tilt all masses
    p = p' + [0 0 z_offset]; % add the offset; 

    masses = point_mass(repmat(mass, size(p,1), 1), p, repmat(v_init, 8, 1));

    % create springs based the available point masses
    comb_indcs = combnk(1:length(masses), 2);
    L_0 = zeros(size(comb_indcs, 1), 1);
    K = k*ones(size(comb_indcs, 1), 1); 
    for i = 1:length(comb_indcs)
        pair_indcs = comb_indcs(i,:);
        L_0(i) = vecnorm(masses(pair_indcs(1)).p - masses(pair_indcs(2)).p);
    end
    
    % create actuation parameters for the springs
    acts = zeros(length(L_0), 3);
    acts(1,:) = [L_0(1)/2, 4, 0];
    acts(14,:) = [L_0(14)/2, 4, 0];
    acts(23,:) = [L_0(23)/2, 4, 0];
    acts(28,:) = [L_0(28)/2, 4, 0];
    springs = spring(L_0, K, comb_indcs, acts);  
%     sprind = reshape([springs.m], 2, 28);
    cube = robot1(masses, springs);
    
end