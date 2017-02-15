function [ closenode ] = findClosestNode( xycoord, NodesLocation )
% Finds closest node to xycoord. 
    closenode = 0;
    closedist = realmax;
    for i = 1:length(NodesLocation)
        X = [xycoord(1) xycoord(2); NodesLocation(i,1) NodesLocation(i,2)];
        dist = pdist(X, 'euclidean');
        if dist < closedist
            closenode = i;
            closedist = dist;
        end
    end
end

