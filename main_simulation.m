%% 
clc
clear
close all

% create an instance of simulator with a robot object
dt = 0.0005; 
cube = robot1();
sim = simulator(cube(), dt);

frames = sim.simulate(2);

% figure;
% movie(frames, 1, 25); % 25 fps