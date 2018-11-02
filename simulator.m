classdef simulator < handle
    %SIMULATOR Summary of this class goes here
    %   Detailed explanation goes here
    properties (Constant)
        g = [0, 0, -9.81]; % Double array. Gravitational acceleration (m/s^2)
        rho = 1; % Double. Global velocity damping parameter (0<p<1)
        k_ground = 2500; % contact force constant      
    end
    
    properties
        bots % Array of type robot1. Represents the bodies in the system
        dt % Double. Length of one time-step (in seconds)
        t = 0; % Double. Current time (sec)
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
        
        function [frames, K, V] = simulate(obj, time)
            % SIMULATE run the simulation over a period of 'time' in
            % seconds and record the animation for playback in 'frames'
            fig = figure('pos',[10 10 900 600]);
            % crate slider for playback
            
            T = 0: obj.dt : time;
            %              u = uicontrol('Style','slider','Position',[10 50 20 340],...
            %                 'Min',1,'Max',16,'Value',1);
            
            K = zeros(1, length(T)); % kinetic energy
            V = zeros(1, length(T)); % potential energy
            
            k = 0; % frame counter
            
            for i = 1:length(T)
                
                t_step = T(i);
                [ke, pe] = obj.step();
                
                V(i) = pe;
                K(i) = ke;
                
                % desired frame rate is 25 frame/s meaning meaning we need one
                % frame every other 0.04 sec (every other 0.04/dt frames)
                if mod(t_step, 0.04) == 0
                    k = k + 1;
                    clf;
                    obj.drawRobots();
                    text(0.4, 0.4, 0.4, ['t = ' num2str(t_step) ' sec']);
                    drawnow
                    frames(k) = getframe(fig);  %#ok<AGROW>
                end
            end
        end
        
        function [ke, pe] = step(obj)
            % loop through all robots
            for bot_no = 1:length(obj.bots)
                % calculate contact forces based on mass positions
                f_contact = zeros(length(obj.bots(bot_no).masses), 3);
                mass_pos = reshape([obj.bots(bot_no).masses.p], 3, []);
                mass_pos_z = mass_pos(3, :);
                
                % check if there are any masses underneath the ground
                if ~isempty(find(mass_pos_z < 0, 1))
                    contact_inds = find(mass_pos_z < 0);
                    % calculate the restoration force
                    f_contact(contact_inds, 3) = -obj.k_ground*mass_pos_z(contact_inds);
                    pe_contact = 1/2*obj.k_ground*sum(abs(mass_pos_z(contact_inds).^2));
                else
                    pe_contact = 0;
                end
                
                f_ext = f_contact;
                
                % calculate kinematic variables
                forces = obj.bots(bot_no).calcForces(obj.g, f_ext, obj.t);
                [a, v, p] = obj.bots(bot_no).calcKin(forces, obj.dt);
                
                % update all kinematic variables
                obj.bots(bot_no).updateP(p);
                obj.bots(bot_no).updateV(obj.rho*v);
                obj.bots(bot_no).updateA(a);        
                
                % get energy
                ke = obj.bots(bot_no).calcKE();
                pe = obj.bots(bot_no).calcPE(obj.g) + pe_contact;
                
                % update time
                obj.t = obj.t + obj.dt;
            end
        end
        
        %% VISUALIZATION
        function drawRobots(obj)
              
            % loop through all robots
            for bot_no = 1:length(obj.bots)
                % get position of all point masses
                mass_pos = reshape([obj.bots(bot_no).masses.p], 3, []);
                scat = scatter3(mass_pos(1, :), mass_pos(2, :), mass_pos(3, :));
                hold on;
                scat.MarkerEdgeColor = 'k';
                scat.MarkerFaceColor = 'b';
                
                % draw springs based on given pairs of mass indices
                pair_indcs = reshape([obj.bots(bot_no).springs.m], 2, [])';
                for i = 1:size(pair_indcs, 1)
                    pair_pos = reshape([obj.bots(bot_no).masses(pair_indcs(i, :)).p], 3, []);
                    plot3(pair_pos(1, :), pair_pos(2, :), pair_pos(3, :), 'k')
                end
            end
            
            % reframe the axes
            axis equal; grid on;
            xlim([-0.5 0.5]);
            ylim([-0.5 0.5]);
            zlim([-0.02 0.5]);
            xlabel('x (m)')
            ylabel('y (m)')
            zlabel('z (m)')
        end
        
    end
end

