classdef robot1 < handle
    %ROBOT This class represents 
    %   It's essentially a body consisting of masses connected by springs
    
    properties
        masses % Array of point_mass objects
        springs % Array of spring objects
        rho % Double. Velocity damping parameter (0<p<1)
    end
    
    methods
        % Constructor
        function obj = robot1(masses,springs)
            %ROBOT1 Construct an instance of robot1
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
                cube_size = 0.1; % m
                z_offset = 0.1; % m
                k = 1000; % N/m
                
                % create positions of masses in the cube
                p = ones(8, 3)/2;
                p(1:4, 3) = 0;
                p([2:3,6:7], 1) = -p([2:3,6:7], 1);
                p([3:4,7:8], 2) = -p([3:4,7:8], 2);
                p = p*cube_size;
                p(:, 3) = 2*p(:, 3) + z_offset;
                
                obj.masses = point_mass(repmat(mass, size(p,1), 1), p);
                
                % create springs based the available point masses
                comb_indcs = combnk(1:length(obj.masses), 2);
                L_0 = zeros(size(comb_indcs, 1), 1);
                K = k*ones(size(comb_indcs, 1), 1); 
                for i = 1:length(comb_indcs)
                    pair_indcs = comb_indcs(i,:);
                    L_0(i) = vecnorm(obj.masses(pair_indcs(1)).p - obj.masses(pair_indcs(2)).p);
                end
                obj.springs = spring(L_0, K, comb_indcs);               
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
        
        function forces = calcForces(obj, g, f)
            %CALCFORCES Calculates the vector forces on each mass
            %   g is the gravitational constant (1x3 vector)
            %   f are additional forces on the nodes (num_masses x 3 array)
            my_masses = obj.mass;
            my_springs = obj.springs;
            forces = zeros(length(my_masses));
            for i = 1:length(my_masses)
                forces(i,:) = my_masses(i).mass * g + f(i,:);
                for j = 1:length(my_springs)
                    if my_springs(j).m == i
                        spring_l = sqrt(my_masses(my_springs(j).m(1)).p.^2 ...
                            + my_masses(my_springs(j).m(2)).p.^2);
                        spring_f = my_springs(j).k*(spring_l-my_springs(j).L_0);
                        spring_v = spring_f*(my_masses(my_springs(j).m(1)).p...
                            - my_masses(my_springs(j).m(2)).p)/spring_l;
                        forces(i,:) = forces(i,:) + spring_v;
                    end
                end
            end
        end
        
        function [a, v, p] = calcKin(obj, f, dt)
            %CALCKIN Calculates the kinematics of the robot (acceleration,
            %velocity, position) based of the forces
            %   f is the array of force vectors on all the masses (not just
            %   external forces)
            %   a is the array of acceleration vectors for all masses
            %   v is the array of velocity vectors for all masses
            %   p is the array of position vectors for all masses
            my_masses = obj.masses;
            a = zeros(size(f,1), size(f,2));
            v = a;
            p = v;
            for i = 1:length(my_masses)
                a(i,:) = f(i,:) / my_masses(i).mass;
                v(i,:) = my_masses(i).v + my_masses(i).a*dt;
                p(i,:) = my_masses(i).p + my_masses(i).v*dt + 0.5*my_masses(i).a*dt^2;
            end
        end
        
        function pe = calcPE(obj, g)
            my_masses = obj.masses;
            pe = 0;
            for i = 1:length(my_masses)
                pe = pe + my_masses(i).mass * g(3) * my_masses(i).p(3);
            end
        end
        
        function ke = calcKE(obj)
            my_masses = obj.masses;
            ke = 0;
            for i = 1:length(my_masses)
                ke = ke + 0.5*my_masses(i).mass*(my_masses(i).v(1)^2 + ...
                    my_masses(i).v(2)^2 + my_masses(i).v(2)^2);
            end
        end
    end
end

