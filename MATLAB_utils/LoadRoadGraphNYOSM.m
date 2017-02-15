% Forked from LoadRoadGraphLarge on Oct. 3, 2016.
% Uses OSM map


MATSIMFLAG = 1;
symmetricFlag = 0;
PLOTFLAG=0;

load('maps/ManhattanMap_OSM')

N = length(ActiveConnectivityMatrixC);
global RoadGraph NodesLocation;
RoadGraph=cell(N,1);

RoadGraph=ActiveConnectivityMatrixC';



% define node locations
NodesLocation=zeros(N,2);

NodesLonLat=zeros(N,3);
NodesLonLat(:,2)=ActiveNodesC(:,1);
NodesLonLat(:,1)=ActiveNodesC(:,2);

RoadCap=sparse(ActiveCapacityMatrixC);
LinkFreeFlow=sparse(ActiveSpeedMatrixC);

% link lengths
LinkLength = sparse(N,N);
%NumLanes = sparse(N,N);
% define link capacities
%LinkNumVehicles = sparse(N,N);
%LinkCapacityLeft = sparse(N,N);
LinkTime = sparse(N,N);
%LinkSpeed = sparse(N,N);

%NominalCapacity = 0.128;

%AugRoadGraph = RoadGraph;

RefLocation = [40.689235 -74.044385 ];
tmpLocation = lla2flat(NodesLonLat, RefLocation, 28.8, 0);%28.8


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
        if (LinkLength(i,j)==0)
            disp('WARNING: two nodes coincide!')
        end
        if (LinkFreeFlow(i,j)==0)
            disp('WARNING: zero speed between two nodes, why?')
            i
            j
        end
        LinkTime(i,j) = LinkLength(i,j) / LinkFreeFlow(i,j);
    end
end



%%
polyx = [1.8; 3.8; 3.6; 2.6; 2.75; 3; 3; 1; 0.9; -1; -1; 0.25; 1; 1.8]*1e3;

polyy = [2.4; 5.15; 7; 8.1; 10; 12.5; 15; 17.5; 18.1; 18.1; 7; 4; 2.4; 2.4]*1e3;


% P = lla2flat( LLA,  LL0,  PSI0, HREF )
% LLA = flat2lla ( P,  LL0,  PSI0, HREF )
% RefLocation, 28.8, 0)

%% Capacity adjuster
%RoadCap=0.225*RoadCap;



%%


if MATSIMFLAG
    %polyx = polyx*1000;
    %polyy = polyy*1000;
    %RoadCap = RoadCap*1000;
    %NodesLocation = NodesLocation*1000;
    
    %in MATLAB, roadcap is in vehicles per second. Here, we need a RoadCap
    %in number of vehicles. So we will compute a new RoadCap.
    
    NumLanes=ActiveCapacityMatrixC./(ActiveSpeedMatrixC./EffectiveVehicleLength);
    % deNaN
    NumLanes2=NumLanes;
    NumLanes2(isnan(NumLanes2))=0;
    NumLanes=sparse(NumLanes2);
    clear NumLanes2;
    %/deNaN
        
    LinkSpeed=LinkFreeFlow/3.6; %Converting to meters per second
    
    %RoadCap=dense(RoadCap);
    %LinkTime=dense(LinkTime);
    %LinkLength=dense(LinkLength);
    %NumLanes=dense(NumLanes);
    %LinkSpeed=dense(LinkSpeed);
    
    save('bin/zhangNYDataOSM.mat', 'RoadGraph', 'NodesLocation', 'RoadCap', ...
        'LinkTime', 'RefLocation', 'polyx', 'polyy', 'LinkLength',...
        'NumLanes', 'LinkSpeed');
    
    %RoadCap=RoadCapFlow; %Restore the flow value
end

%%
 
if PLOTFLAG
    figure()
    %plot RoadGraph
    cutoff = length(NodesLocation);
    figure()
    hold on;
    plot(NodesLocation(1:cutoff,1), NodesLocation(1:cutoff,2), '.b')
    for i = 1:cutoff
        %text(NodesLocation(i,1), NodesLocation(i,2), int2str(i));
        for j = RoadGraph{i}
            if j <= cutoff
                plot([NodesLocation(i,1) NodesLocation(j,1)], [NodesLocation(i,2) NodesLocation(j,2)]);
            end
        end
    end
    
    for i=1:cutoff
        if isempty(RoadGraph{i})
            plot(NodesLocation(i,1), NodesLocation(i,2),'dr','MarkerSize',20);
        end
    end
    kk=964;
    plot(NodesLocation(kk,1), NodesLocation(kk,2),'dm','MarkerSize',20);
    ll=1203;
        plot(NodesLocation(ll,1), NodesLocation(ll,2),'dg','MarkerSize',20);
    axis equal
    %plot(polyx,polyy,'k--')
end


%% Test: remove do not run in prod
CONNTESTFLAG=0;

if CONNTESTFLAG
    ReachFrom=zeros(N,1);
    ii=999;
    frontier=RoadGraph{ii};
    ReachFrom(frontier)=1;
    r=1;
    explored=[];
    while ~isempty(frontier)
        exploring=frontier(1);
        explored=unique([explored exploring]);
        ReachFrom(exploring)=1;
        frontier=unique([frontier RoadGraph{exploring}]);
        frontier=setdiff(frontier,explored);
        length(frontier)
        r=r+1;
        r;
    end
    fprintf('%d nodes, %d reachable from %d\n',N,sum(ReachFrom),ii)
    find(~ReachFrom)
end