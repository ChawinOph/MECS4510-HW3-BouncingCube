classdef spring < handle
    %SPRING This class models a spring
    %   Detailed explanation goes here
    
    properties
        L_0 % Double. Original rest length[meters]
        act % Actuation parameters
        k % Double. Stiffness [Newtons/meter]
        m % Int Array. Index of the two masses it connects to.
    end
    
    methods
        % Constructor
        function obj = spring(L_0, k, m, act)
            %SPRING Construct an instance of the class spring
            %   Detailed explanation goes here
            if nargin~=0
                for i = length(L_0):-1:1
                    obj(i).L_0 = L_0(i,:); 
                    obj(i).k = k(i,:);
                    obj(i).m = m(i,:);
                    if nargin>3
                        obj(i).act = act(i,:);
                    end
                end
            end
        end
    end
end

