
% clc
% clear
% close all
%%
genes = rand(15,5);
sim = simulator();

tic
% [frames1, ~, ~, ~] = sim.simulate_and_plot(starfish_robot(genes)); 
[frames1, K1, V1, COM1, fitness1] = sim.simulate_and_plot(starfish_robot([bots(10:15).gene])); 
toc
fitness1(end,:)

tic
% [frames1, ~, ~, ~] = sim.simulate_and_plot(starfish_robot(genes)); 
[K2, V2, COM2, fitness2] = sim.simulate(starfish_robot([bots(10:15).gene])); 
toc
fitness2(end,:)

% export to video
myVideo = VideoWriter('crawling_robot.avi');
myVideo.FrameRate = 25;  % Default 30
myVideo.Quality = 100;    % Default 75
open(myVideo);
writeVideo(myVideo, frames1);
close(myVideo);

% figure;
% plot(0:dt:run_time, K, 0:dt:run_time, V, 'LineWidth', 1.5); hold on;
% plot(0:dt:run_time, K + V, '--k', 'LineWidth', 2)
% xlabel('Time (s)')
% ylabel('Energy (J)')
% legend('KE','PE','KE + PE');
% grid on; grid minor;
% 
% figure 
% plot(0:dt:run_time, COM); hold on;
% xlabel('Time (s)');
% ylabel('Displacement (m)');
% legend('COM_x','COM_y','COM_z');
% grid on; grid minor;
% toc
