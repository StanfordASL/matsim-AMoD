function [NodeToStation,StationToNode]=FindNodeToStation(NodesLocations,StationsLocations)

NodeToStation=zeros(length(NodesLocations),1);
    %vecdist=zeros(size(StationsLocation));
for i=1:length(NodesLocations)
    %vecdist(:,1)=NodesLocations(i,1)-StationsLocations(:,1);
    %vecdist(:,2)=NodesLocations(i,2)-StationsLocations(:,2);
    
    statdist=sqrt((NodesLocations(i,1)-StationsLocations(:,1)).^2+(NodesLocations(i,2)-StationsLocations(:,2)).^2);
    [~,NodeToStation(i)]=min(statdist);
end

StationToNode=zeros(length(StationsLocations),1);

for i=1:length(StationsLocations)
        statdist=sqrt((NodesLocations(:,1)-StationsLocations(i,1)).^2+(NodesLocations(:,2)-StationsLocations(i,2)).^2);
    [~,StationToNode(i)]=min(statdist);
    
    %StationToNode=find(StationsLocations(i,1)==NodesLocations(:,1) & StationsLocations(i,2)==NodesLocations(:,2));
end

PLOTFLAG=1
if PLOTFLAG
    figure()
    voronoi(StationsLocations(:,1),StationsLocations(:,2))
    hold all
    for i=1:length(StationsLocations)
        TempColor=rand(3,1);
        plot(NodesLocations(NodeToStation==i,1),NodesLocations(NodeToStation==i,2),'.','Color',TempColor);
        plot(NodesLocations(StationToNode(i),1),NodesLocations(StationToNode(i),2),'p','Color',TempColor,'MarkerSize',3);
    end
    axis equal
    %figure()
    %voronoi(StationsLocations(:,1),StationsLocations(:,2))
    %axis equal
end