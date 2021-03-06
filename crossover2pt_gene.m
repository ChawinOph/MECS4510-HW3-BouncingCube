function [children] = crossover2pt_gene(parents) % #codegen
% This crossover function takes and returns genes, not bots
p = size(parents,2);
children = zeros(15,p);
cut = sort([randi(15, p, 1), randi(15, p, 1)],2);
% children = starfish_robot.empty(0,length(parents));

for i = 1:2:size(parents,2)-1
%     parA = parents(i).gene;
%     parB = parents(i+1).gene;
    parA = parents(:,i);
    parB = parents(:,i+1);
    geneC = [parA(1:cut(i,1)); parB(cut(i,1)+1: cut(i,2)); parA(cut(i,2)+1:end)];
    geneD = [parB(1:cut(i,1)); parA(cut(i,1)+1: cut(i,2)); parB(cut(i,2)+1:end)];

    %Child is placed in same index as more similar parent
    if cut(i,2)-cut(i,1) < 8
%         children(i) = starfish_robot(geneC);
%         children(i+1) = starfish_robot(geneD);
        children(:,i) = geneC;
        children(:,i+1) = geneD;
    else
%         children(i) = starfish_robot(geneD);
%         children(i+1) = starfish_robot(geneC);
        children(:,i) = geneD;
        children(:,i+1) = geneC;
    end
end 

end