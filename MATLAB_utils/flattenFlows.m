function [flatNodes, flatFlows] = flattenFlows(Nodes, Flows)

% turns a series of non-unit flows into several nodes with unit flows

totalsize = sum(Flows);

result = zeros(totalsize, 2);

resIdx = 1;

for i = 1:length(Nodes)
    currNode = Nodes(i);
    nodeFlow = Flows(i);
    
    result(resIdx:resIdx + nodeFlow-1, 1) = currNode;
    result(resIdx:resIdx + nodeFlow-1, 2) = 1;
    resIdx = resIdx + nodeFlow;
end

flatNodes = result(:,1);
flatFlows = result(:,2);

    