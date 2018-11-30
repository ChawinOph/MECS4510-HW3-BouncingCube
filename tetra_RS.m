clc
clear
close all
%%
tic
p = 105; % population
g = 50; % number of generations

%%
genes = rand(15, p);
bots = starfish_robot(genes);
sim = simulator();
best_bots = starfish_robot.empty(0,g);

fitness_hist = zeros(g,1);
fitness_all_hist = zeros(p,g);
fitnesses = evaluate(sim, bots);
[M,I] = max(fitnesses);
% bots = bots(I);
best_bots(1) = bots(I);
fitness_hist(1:end) = M;
fitness_all_hist(:,1) = fitnesses;
%%
for i = 2:g

    % evaluate
    fitnesses = sim.evaluate(bots); 
    
    % record
    [M,I] = max(fitnesses);
    best_bots(i) = bots(I);
    if M > fitness_hist(i)
        fitness_hist(i:end) = M;
    end
    fitness_all_hist(:,i) = fitnesses;
    
    % Create new random batch
    genes = rand(15, p);
    bots = starfish_robot(genes);
end

toc;
disp('DONE!!!')
figure;
plot(fitness_hist)
best_bot = starfish_robot(best_bots(end).gene);
[~, K, V, COM, fitness] = sim.simulate_and_plot(best_bot);
save('RSrun1')