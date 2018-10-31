%% 
clc
clear
close all

% create an instance of simulator with a robot object
dt = 0.001; 
cube = robot1();
sim = simulator(cube(), dt);

figure;
sim.drawRobot();

