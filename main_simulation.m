%% 
clc
clear
close all

% create an instance of simulator with a robot object
dt = 0.0005; 
cube = robot1();
sim = simulator(cube(), dt);
run_time = 5; % seconds

[frames, K, V] = sim.simulate(run_time); 
close

% figure;
% movie(frames, 1, 25); % run 1 time at 25 fps

% export to video
myVideo = VideoWriter('bouncing_cube.avi');
myVideo.FrameRate = 25;  % Default 30
myVideo.Quality = 100;    % Default 75
open(myVideo);
writeVideo(myVideo, frames);
close(myVideo);

figure;
plot(0:dt:run_time, K, 0:dt:run_time, V, 0:dt:run_time, K + V)
xlabel('Time (s)')
ylabel('Energy (J)')
legend('KE','PE','KE + PE');
grid on; grid minor;

