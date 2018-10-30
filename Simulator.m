classdef Simulator
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        bots % Array of type robot1. Represents the bodies in the system
        dt % Double. Length of one time-step (in seconds)
        g % Double array. Gravitational acceleration (m/s^2)
%       rho % Double. Global velocity damping parameter (0<p<1)
    end
    
    methods
        function obj = Simulator(bots, dt, g, rho)
            %SIMULATOR Construct an instance of simulator class
            %   Detailed explanation goes here
            if nargin>0
                obj.bots = bots;
                if nargin>1
                    obj.dt = dt;
                    if nargin >2
                        obj.g = g;
                        if nargin>3
                            obj.rho = rho;
                            if nargin>4
                                disp('Too many arguments for simulator');
                            end
                        end
                    end
                end
            end
                
        end
        
        function outputArg = step(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

