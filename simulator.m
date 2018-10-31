classdef simulator < handle
    %SIMULATOR Summary of this class goes here
    %   Detailed explanation goes here
    properties (Constant)
        g = [0, 0, -9.81]; % Double array. Gravitational acceleration (m/s^2)
        rho = 0.999; % Double. Global velocity damping parameter (0<p<1)
    end
    
    properties
        bots % Array of type robot1. Represents the bodies in the system
        dt % Double. Length of one time-step (in seconds)
    end
    
    methods
        function obj = simulator(bots, dt)
            %SIMULATOR Construct an instance of simulator class
            %   Detailed explanation goes here
            if nargin>0
                obj.bots = bots;
                if nargin>1
                    obj.dt = dt;
                    if nargin >2
                        disp('Too many arguments for simulator');
                    end
                end
            end
            
        end        
        
        function step(obj)
            % simulation step
            
        end
        
        %% VISUALIZATION       
        function drawRobot(obj)
            % clear current robot
            cla
            
            % get position of all point masses
            mass_pos = reshape([obj.bots.masses.p], 3, []);
            scat = scatter3(mass_pos(1, :), mass_pos(2, :), mass_pos(3, :));
            hold on;
            scat.MarkerEdgeColor = 'k';
            scat.MarkerFaceColor = 'b';
            
            % draw springs based on given pairs of mass indices
            pair_indcs = reshape([obj.bots.springs.m], 2, [])';
            for i = 1:size(pair_indcs, 1) 
                pair_pos = reshape([obj.bots.masses(pair_indcs(i, :)).p], 3, []);
                plot3(pair_pos(1, :), pair_pos(2, :), pair_pos(3, :), 'k')
            end
            
            % reframe the axes
            axis equal; 
            grid on; grid minor;
            xlim([-0.5 0.5]);
            ylim([-0.5 0.5]);
            zlim([-0.1 0.5]);       
        end
        
    end
end

