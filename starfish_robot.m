classdef starfish_robot < handle
    %ROBOT This class represents 
    %   It's essentially a body consisting of masses connected by springs
    properties
        masses % Array of point_mass objects
        springs % Array of spring objects
    end
    
    methods
        % Constructor
        function obj = starfish_robot(masses, springs)
            %SNAKE_ROBOT Construct an instance of robot1
            %   Detailed explanation goes here
            if nargin ~= 0
                if validRobot(masses, springs)
                    obj.masses = masses;
                    obj.springs = springs;
                else
                    disp('Not a valid combination of springs and masses');
                end
            else % automically create 8 point masses and 28 springs to form a cube
                % create 8 point masses
                mass = 0.1; % m
                height = 0.2; % m (height of the robot)
                z_offset = 0.1; % m 
%                 v_init = [0.5, 0, 0]; % m/s
                v_init = [0, 0, 0];
                k = 500; % Double. N/m
                omega = pi; % (0.5 Hz of breathing);                 
                
                % create four-body octahedron
                p = zeros(17, 3);
                p(14:17, 3) = height;
                p(5:13, 3) = height/2;
                
                p(1, 1) = height/2;
                p(2, 2) = height/2;
                p(3, 1) = -height/2;
                p(4, 2) = -height/2;
                
                p(7, 1:2) = height/2*[1,1];
                p(9, 1:2) = height/2*[-1,1];
                p(11, 1:2) = height/2*[-1,-1];
                p(13, 1:2) = height/2*[1,-1];     
                
                p(6, 1) = height;
                p(8, 2) = height;
                p(10, 1) = -height;
                p(12, 2) = -height;     
                
                p(14, 1) = height/2;
                p(15, 2) = height/2;
                p(16, 1) = -height/2;
                p(17, 2) = -height/2;
                                
                spring_connect_indcs = [combnk([1 5 6 7 13 14], 2); 
                                        combnk([2 5 7 8  9 15], 2);
                                        combnk([3 5 9 10 11 16], 2);
                                        combnk([4 5 11 12 13 17],2);
                                        combnk([14 15 16 17], 2)];  
                                    
                spring_connect_indcs = unique(spring_connect_indcs,'rows');
    
                % create spring based on the spring connection indices
                L_0 = zeros(size(spring_connect_indcs, 1), 1);
                K = k*ones(size(spring_connect_indcs, 1), 1);   
                
                % spring center for finding x,y,z of the center of each
                % spring
                spring_center = zeros(size(spring_connect_indcs, 1), 3);
                acts = zeros(size(spring_connect_indcs, 1), 3);
                
                % generate lengths based on pairs of masses
                for i = 1:length(spring_connect_indcs)
                    pair_indcs = spring_connect_indcs(i,:);
                    L_0(i) = vecnorm(p(pair_indcs(1), :) - p(pair_indcs(2), :));
                    spring_center(i, :) = mean(p(pair_indcs, :)); 
                    acts(i,:) = [L_0(i)/2, omega, 0];
                end
                
                obj.springs = spring(L_0, K, spring_connect_indcs, acts);    
                
                % change the position and orientation of the robot after
                % constructing the springs
                R = obj.rotationAxisAngle([1 0 0], pi/6); % tile around x axis by 30 degree
%                 R = eye(3);
                p = R*p'; % tilt all masses
                p = p' + [0 0 z_offset]; % add the offset;  
                obj.masses = point_mass(repmat(mass, size(p), 1), p, repmat(v_init, size(p,1), 1));                
            end
        end
        
        % setters
        function obj = updateP(obj, p)
            %UPDATEP Updates the position of all masses
            %   p is an array of position vectors (each row is a mass, the
            %   columns are x,y,z)
            P = num2cell(p,2);
            [obj.masses.p] = P{:};
        end
        
        function obj = updateV(obj, v)
            %UPDATEP Updates the velocity of all masses
            %   v is an array of velocity vectors (each row is a mass, the
            %   columns are x,y,z)
            V = num2cell(v,2);
            [obj.masses.v] = V{:};
        end
        
        function obj = updateA(obj, a)
            %UPDATEP Updates the acceleration of all masses
            %   a is an array of acceleration vectors (each row is a mass, the
            %   columns are x,y,z)
            A = num2cell(a,2);
            [obj.masses.a] = A{:};
        end
        
        function forces = calcForces(obj, g, f_ext, t)
            %CALCFORCES Calculates the vector forces on each mass
            %   g is the gravitational constant (1x3 vector)
            %   f are additional forces on the nodes (num_masses x 3 array)
            my_masses = obj.masses;
            my_springs = obj.springs;
            forces = zeros(length(my_masses), 3);
            for i = 1:length(my_masses)
                % add gravitational force
                forces(i,:) = my_masses(i).mass * g + f_ext(i,:);
                % go through all springs in the robot
                for j = 1:length(my_springs)
                    % check if the current mass index is attahced to the current
                    % spring
                    if ismember(i, my_springs(j).m)
                        
                        % find the current spring length L
                        pair_indcs = my_springs(j).m;      
                        vector = my_masses(pair_indcs(1)).p - my_masses(pair_indcs(2)).p;
                        L = vecnorm(vector);
                        act = my_springs(j).act;
                        L_act = my_springs(j).L_0 + act(1)*sin(act(2)*t + act(3));
                        spring_f = my_springs(j).k*(L - L_act);
                        
                        % create the force vector with correct direction
                        if my_springs(j).m(1) == i
                            spring_v = -spring_f*vector/L;
                        elseif my_springs(j).m(2) == i
                            spring_v = spring_f*vector/L;
                        end
                        
                        % add more forces to the mass
                        forces(i,:) = forces(i,:) + spring_v;
                    end
                end
            end           
        end
        
        function [a, v, p] = calcKin(obj, f, dt, mu_s, mu_k)
            %CALCKIN Calculates the kinematics of the robot (acceleration,
            %velocity, position) based of the forces
            %   f is the array of force vectors on all the masses (not just
            %   external forces)
            %   a is the array of acceleration vectors for all masses
            %   v is the array of velocity vectors for all masses
            %   p is the array of position vectors for all masses
            my_masses = obj.masses;
            mass_pos = reshape([obj.masses.p], 3, []);
            mass_pos_z = mass_pos(3, :);
            
            a = zeros(size(f,1), size(f,2));
            v = a;
            p = v;

            % add friction
            fixed_indcs = [];
            slide_indc = [];
            if ~isempty(find(mass_pos_z < 0, 1))
                contact_indcs = find(mass_pos_z < 0); 
                % calculate the magnitude of the horizontal forces
                f_h = vecnorm(f(contact_indcs, 1:2), 2, 2);
                f_z = abs(f(contact_indcs, 3));
                fixed_indcs = find(f_h <= f_z*mu_s);
                slide_indc = find(f_h > f_z*mu_s);
            end
              
            for i = 1:length(my_masses)
                if ismember(i, fixed_indcs)
                    % not position update
                    a(i,:) = 0;
                    v(i,:) = 0;
                    p(i,:) = my_masses(i).p;
%                     disp([num2str(i) ' is fixed']);
                elseif ismember(i, slide_indc)
                    % add force from kinetic frictoin (opposite to horizontal v component)
                    f_z = abs(f(i, 3));
                    a(i,:) = (f(i,:) -  mu_k*f_z*([my_masses(i).v(1:2), 0]))/ my_masses(i).mass;
                    v(i,:) = my_masses(i).v + a(i,:)*dt;
                    p(i,:) = my_masses(i).p + v(i,:)*dt;
                else % no contact with the ground
                    a(i,:) = f(i,:) / my_masses(i).mass;
                    v(i,:) = my_masses(i).v + a(i,:)*dt;
                    p(i,:) = my_masses(i).p + v(i,:)*dt;
                end
            end
        end
        
        function pe = calcPE(obj, g)
            my_masses = obj.masses;
            my_springs = obj.springs;
            pe = 0;
            % gravitational potential energy
            for i = 1:length(my_masses)
                pe = pe + my_masses(i).mass * abs(g(3)) * my_masses(i).p(3);
            end
            % spring energy
            for i = 1:length(my_springs)
                pair_indcs = my_springs(i).m;
                vector = my_masses(pair_indcs(1)).p - my_masses(pair_indcs(2)).p;
                L = vecnorm(vector);
                pe = pe + 0.5*my_springs(i).k*(L - my_springs(i).L_0).^2;
            end
            % GRF energy will be summed in the simulation object
        end
        
        function ke = calcKE(obj)
            my_masses = obj.masses;
            ke = 0;
            for i = 1:length(my_masses)
                ke = ke + 0.5*my_masses(i).mass*(my_masses(i).v(1)^2 + ...
                    my_masses(i).v(2)^2 + my_masses(i).v(3)^2);
            end
        end
        
        function com_pos = calcCOM(obj)
            mass_pos = reshape([obj.masses.p], 3, []);
            com_pos = mean([obj.masses.mass].*mass_pos, 2);
        end
        
        function S = skew(~, v)
           % vec: 3 x 1 vector column
            S = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
        end
        
        function R = rotationAxisAngle(obj, axis, angle)
           % axis:  double 3 x 1 vector
           % angle: double
           axis = [axis(1) axis(2), axis(3)]';
           axis = axis/vecnorm(axis);
           c = cos(angle); s = sin(angle); v = 1 - c;
           R = eye(3)*c + obj.skew(axis)*s + axis*axis'*v;
        end
        
    end
end

