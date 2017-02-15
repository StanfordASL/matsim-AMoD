function [squashed] = squashFlows(Flows)

% expects an m x 2 matrix

Flows = sortrows(Flows);

uniqueflows = unique(Flows(:,1));

size = length(unique(Flows(:,1)));

squashed = zeros(size, 2);

flowind = 1; % index of flows 
squashind = 1;% index of result
for i = 1:size
    i = flowind;
    currNode = Flows(i,1);
    squashed(squashind, 1) = currNode;
    while flowind <= length(Flows) && Flows(flowind, 1) == currNode
        squashed(squashind, 2) = squashed(squashind, 2) + Flows(flowind, 2);
        flowind = flowind + 1;
    end
    squashind = squashind + 1;
end