
% clc
% clear
% close all
%%
genes = rand(15,1);
sim = simulator();

tic


% parameters
gene = 0.5*ones(15, 1);
sorted_indcs = 1:15; % for linkage tightening (inversion)

sim = simulator(starfish_robot(gene, sorted_indcs), dt);
run_time = 5; % seconds


tic
% [frames1, ~, ~, ~] = sim.simulate_and_plot(starfish_robot(genes)); 
[K2, V2, COM2, fitness2] = sim.simulate(starfish_robot([bots(10:15).gene])); 
toc
fitness2(end,:)

% export to video
% myVideo = VideoWriter('crawling_robot.avi');
% myVideo.FrameRate = 25;  % Default 30
% myVideo.Quality = 100;    % Default 75
% open(myVideo);
% writeVideo(myVideo, frames);
% close(myVideo);

figure;
plot(V, 'LineWidth', 1.5); hold on;
plot(K, 'LineWidth', 1.5); 
plot( K + V,  'LineWidth', 1)
xlabel('Time (s)')
ylabel('Energy (J)')
legend('KE','PE','KE + PE');
grid on; grid minor;

% 
% figure 
% plot(0:dt:run_time, COM); hold on;
% xlabel('Time (s)');
% ylabel('Displacement (m)');
% legend('COM_x','COM_y','COM_z');
% grid on; grid minor;
% toc

