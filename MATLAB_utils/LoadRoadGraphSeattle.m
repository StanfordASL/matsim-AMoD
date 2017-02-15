
symmetricFlag = 0;

global RoadGraph NodesLocation;
%RoadGraph=cell(N,1);

load('maps/SeattleMap')

RoadGraph=ActiveConnectivityMatrix';

N=length(ActiveConnectivityMatrix);

% define node locations
NodesLocation=zeros(N,2);
NodesLatLon = zeros(N,3);

NodesLatLon(:,1)=ActiveNodes(:,2); %Flip lon and lat
NodesLatLon(:,2)=ActiveNodes(:,1);

% link lengths
LinkLength = sparse(N,N);
NumLanes = sparse(N,N);
% define link capacities
RoadCap = sparse(N,N);
LinkNumVehicles = sparse(N,N);
LinkCapacityLeft = sparse(N,N);
LinkTime = sparse(N,N);
LinkSpeed = sparse(N,N);
CAR_LENGTH = 6.5; % meters

NominalCapacity = 0.128;


% define link freeflow speed
LinkFreeFlow = sparse(N,N);

AugRoadGraph = RoadGraph;



RefLocation = [47.6205126 -122.3492432];  %the Space Needle
tmpLocation = lla2flat(NodesLatLon, RefLocation, 0, 0);
NodesLocation(:,1) = tmpLocation(:,2); %x (longitude)
NodesLocation(:,2) = tmpLocation(:,1); %y (latitude)

% make graph symmetric
if symmetricFlag == 1
    for i = 1:N
        for j = RoadGraph{i}
            if isempty(find(RoadGraph{j}==i,1))
                RoadGraph{j} = [RoadGraph{j} i];
            end
        end
    end
end

% find link lengths
for i = 1:N
    for j = RoadGraph{i}
        LinkLength(i,j) = norm(NodesLocation(j,:) - NodesLocation(i,:));
        LinkFreeFlow(i,j) = 11; % m/s
        NumLanes(i,j) = 1;
        LinkTime(i,j) = LinkLength(i,j)/LinkFreeFlow(i,j);
        RoadCap(i,j) = NominalCapacity*NumLanes(i,j)*LinkFreeFlow(i,j)/CAR_LENGTH*3600;
    end
end

%%
% polyx = [1.8; 3.8; 3.6; 2.6; 2.75; 3; 3; 1; 0.9; -1; -1; 0.25; 1; 1.8];
% polyx = polyx*1000;
% polyy = [2.4; 5.15; 7; 8.1; 10; 12.5; 15; 17.5; 18.1; 18.1; 7; 4; 2.4; 2.4];
% polyy = polyy*1000;
%%
save('bin/zhangSeattleData.mat', 'RoadGraph', 'NodesLocation', 'RoadCap', 'LinkTime', 'RefLocation',...
      'LinkLength', 'NumLanes', 'LinkSpeed');
% 
% %plot RoadGraph
% cutoff = N;
% figure(1)
% hold on;
% plot(NodesLocation(1:cutoff,1), NodesLocation(1:cutoff,2), '.b')
% for i = 1:cutoff
%     %text(NodesLocation(i,1), NodesLocation(i,2), int2str(i));
%     for j = RoadGraph{i}
%         if j <= cutoff
%             plot([NodesLocation(i,1) NodesLocation(j,1)], [NodesLocation(i,2) NodesLocation(j,2)]);
%         end
%     end
% end
% axis equal
