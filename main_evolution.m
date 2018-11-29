clc
clear
close all
%%
tic
p = 100; % population
g = 1000; % number of generations
m = 0.05; % mutation rate
%%
genes = rand(15, p);
bots = starfish_robot(genes);
sim = simulator();
best_bots = starfish_robot.empty(0,g)
%%
for i = 1:p
    evaluate(bots(i));
end

[fitnesses, fit_ind] = sort([bots.fit]);
best_bots(1) = bots(end);

for i = 2:g
    % crossover
    bots = bots(randperm(length(bots)));
    [children] = crossover2pt(bots);
    
    for j = 1:p
        % evaluation
        child_fit = sim.evaluate(children(j));
        % selection (deterministic crowding)
        if child_fit > bots(j).fit
            bots(j) = children(j);
        end
    end
    
    % mutation
    for j = 1: m*p
        gene = bots(j).gene;
        gene(rand(i)) = rand;
        bots(j) = starfish_robot(gene);
        sim.evaluate(bots(j));
    end
        
    % record
    [fitnesses, fit_ind] = sort([bots.fit]);
    best_bots(i) = bots(end);
end