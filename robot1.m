classdef robot1
    %ROBOT This class represents 
    %   It's essentially a body consisting of masses connected by springs
    
    properties
        masses % Array of point_mass objects
        springs % Array of spring objects
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
            end
        end
        % setters
        function obj = updateP(obj, p)
            %UPDATEP Updates the position of all masses
            %   p is an array of position vectors (each row is a mass, the
            %   columns are x,y,z)
            P = num2cell(p,2)
            [obj.masses.p] = P{:}
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
        
    end
end

