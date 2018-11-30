clc
clear
close all
%%
tic
p = 100; % population
g = 500; % number of generations
m = 0.05; % mutation rate
%%
genes = rand(15, p);
bots = starfish_robot(genes);
sim = simulator();
best_bots = starfish_robot.empty(0,g);
%%
% for i = 1:p
%     evaluate(bots(i));
% end

% [fitnesses, fit_ind] = sort([bots.fit]);
% best_bots(1) = bots(end);

fitness_hist = zeros(g,1);
fitness_all_hist = zeros(p,g);
fitnesses = evaluate(sim, bots);
[M,I] = max(fitnesses);
% bots = bots(I);
best_bots(1) = bots(I);
fitness_hist(1) = M;
fitness_all_hist(:,1) = fitnesses;
%%
for i = 2:g
    % crossover
    shuffle_ind = randperm(length(bots));
    bots = bots(shuffle_ind);
    fitnesses = fitnesses(shuffle_ind);
    [children] = crossover2pt(bots);
    child_fits = evaluate(sim, children);
    % Can I do this next section without the loop?
    for j = 1:p
        if child_fits(j)>fitnesses(j)
            fitnesses(j) = child_fits(j);
            bots(j) = children(j);
        end
    end
    
%     for j = 1:p
%         % evaluation
%         child_fit = sim.evaluate(children(j));
%         % selection (deterministic crowding)
%         if child_fit > bots(j).fit
%             bots(j) = children(j);
%         end
%     end
    
    % mutation
    for j = 1: m*p
        gene = bots(j).gene;
        gene(randi(p)) = rand;
        bots(j) = starfish_robot(gene);
        fitnesses(j) = evaluate(sim, bots(j));
    end
        
    % record
%     fitnesses = sim.evaluate(bots); %No point to recalculate this
    [M,I] = max(fitnesses);
    best_bots(i) = bots(I);
    fitness_hist(i) = M;
    fitness_all_hist(:,i) = fitnesses;
end

toc;
disp('DONE!!!')

figure;
plot(fitness_hist)

figure;
scatter(reshape(repmat(1:g,p,1), [], 1), reshape(fitness_all_hist, [], 1))


best_bot = starfish_robot(best_bots(end).gene);
[~, K, V, COM, fitness] = sim.simulate_and_plot(best_bot);