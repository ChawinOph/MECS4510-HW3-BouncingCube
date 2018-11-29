function [children] = crossover2pt(parents)

cut = sort([randi(15, p, 1), randi(15, p, 1)],2);
children = starfish_robot.empty(0,length(parents));

for i = 1:2:length(parents)
    parA = parents(i).gene;
    parB = parents(i+1).gene;
    geneC = [parA(1:cut(1)); parB(cut(1), cut(2)); parA(cut(2),end)];
    geneD = [parB(1:cut(1)); parA(cut(1), cut(2)); parB(cut(2),end)];

    %Child is placed in same index as more similar parent
    if cut(i,2)-cut(i,1) < 8
        children(i) = starfish_robot(geneC);
        children(i+1) = starfish_robot(geneD);
    else
        children(i) = starfish_robot(geneD);
        children(i+1) = starfish_robot(geneC);
    end
end 

end