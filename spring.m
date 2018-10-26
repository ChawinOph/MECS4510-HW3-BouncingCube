classdef spring
    %SPRING This class models a spring
    %   Detailed explanation goes here
    
    properties
        L_0 %Double. Original rest length[meters]
        k %Double. Stiffness [Newtons/meter]
        m % Int Array. Index of the two masses it connects to.
        m_1
        m_2 % Int. Index of the second mass it connects to
    end
    
    methods
        % Constructor
        function obj = spring(L_0, k, m)
            %SPRING Construct an instance of the class spring
            %   Detailed explanation goes here
            if nargin~=0
                for i = length(L_0):-1:1
                    obj(i).L_0 = L_0(i,:); 
                    obj(i).k = k(i,:);
                    obj(i).m = m(i,:);
                end
%                 obj.m_1 = m_1;
%                 obj.m_2 = m_2;
            end
        end
%         % getters
%         function L_0 = getL_0(obj,inputArg)
%             %METHOD1 Summary of this method goes here
%             %   Detailed explanation goes here
%             outputArg = obj.Property1 + inputArg;
%         end
    end
end

