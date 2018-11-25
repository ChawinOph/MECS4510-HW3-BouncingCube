
clc
clear
close all
%%
tic;
% create an instance of simulator with a robot object
dt = 0.0001; 
cube = robot1();
% cube = breathingCube();
sim = simulator(cube(), dt);
run_time = 3; % seconds

[frames, K, V, COM] = sim.simulate(run_time); 
close

% export to video
myVideo = VideoWriter('crawling_cube.avi');
myVideo.FrameRate = 25;  % Default 30
myVideo.Quality = 100;    % Default 75
open(myVideo);
writeVideo(myVideo, frames);
close(myVideo);

figure;
plot(0:dt:run_time, K, 0:dt:run_time, V, 'LineWidth', 1.5); hold on;
plot(0:dt:run_time, K + V, '--k', 'LineWidth', 2)
xlabel('Time (s)')
ylabel('Energy (J)')
legend('KE','PE','KE + PE');
grid on; grid minor;

figure 
plot(0:dt:run_time, COM); hold on;
xlabel('Time (s)');
ylabel('Displacement (m)');
legend('COM_x','COM_y','COM_z');
grid on; grid minor;

toc

%% tetrahedron
% tetra = breathingTetrahedron();
% dt = 0.00005; 
% sim_tetra = simulator(tetra(), dt);
% run_time = 5; % seconds
% [frames_tetra, K_tetra, V_tetra] = sim_tetra.simulate(run_time); 
% 
% % export to video
% % myVideo = VideoWriter('tetra.avi');
% % myVideo.FrameRate = 25;  % Default 30
% % myVideo.Quality = 100;    % Default 75
% % open(myVideo);
% % writeVideo(myVideo, frames_tetra);
% % close(myVideo);
% 
% figure;
% plot(0:dt:run_time, K_tetra, 0:dt:run_time, V_tetra); hold on;
% plot(0:dt:run_time, K_tetra + V_tetra)
% xlabel('Time (s)')
% ylabel('Energy (J)')
% legend('KE','PE','KE + PE');
% grid on; grid minor;