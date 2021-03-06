function [surA, surB] = detCrowd(parA, parB)
% Do 2 point crossover
cut = sort([randi(15), randi(15)]);
[childA, childB] = crossover2pt(parA, parB, cut);

% Evaluate children's fitness
cA_fit = evaluate(childA);
cB_fit = evaluate(childB);

% Retrieve parent fitness
pA_fit = parA.fit;
pB_fit = parB.fit;

% Compare children to more similar parent. The more fit one survives to
% next round.
if cut(2)-cut(1) < 8
    if cA_fit > pA_fit
        surA = childA;
    end
    if cB_fit > pB_fit
        surB = childB;
    end
else
    if cB_fit > pA_fit
        surA = childB;
    end
    if cA_fit > pB_fit
        surB = childA;
    end
end