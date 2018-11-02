
clc
clear
close all
%%
tic;
% create an instance of simulator with a robot object
dt = 0.00005; 
% cube = robot1();
cube = breathingCube();
sim = simulator(cube(), dt);
run_time = 5; % seconds

[frames, K, V] = sim.simulate(run_time); 
close

% figure;
% movie(frames, 1, 25); % run 1 time at 25 fps

% export to video
myVideo = VideoWriter('bouncing_cube5.avi');
myVideo.FrameRate = 25;  % Default 30
myVideo.Quality = 100;    % Default 75
open(myVideo);
writeVideo(myVideo, frames);
close(myVideo);

figure;
plot(0:dt:run_time, K, 0:dt:run_time, V); hold on;
plot(0:dt:run_time, K + V, '--k', 'LineWidth', 2)
xlabel('Time (s)')
ylabel('Energy (J)')
legend('KE','PE','KE + PE');
grid on; grid minor;

toc

%% tetrahedron
tetra = breathingTetrahedron();
dt = 0.00005; 
sim_tetra = simulator(tetra(), dt);
run_time = 5; % seconds
[frames_tetra, K_tetra, V_tetra] = sim_tetra.simulate(run_time); 

% export to video
myVideo = VideoWriter('tetra.avi');
myVideo.FrameRate = 25;  % Default 30
myVideo.Quality = 100;    % Default 75
open(myVideo);
writeVideo(myVideo, frames_tetra);
close(myVideo);

figure;
plot(0:dt:run_time, K_tetra, 0:dt:run_time, V_tetra); hold on;
plot(0:dt:run_time, K_tetra + V_tetra)
xlabel('Time (s)')
ylabel('Energy (J)')
legend('KE','PE','KE + PE');
grid on; grid minor;