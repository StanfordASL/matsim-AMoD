filename = '~/AMoD_congestion/Seattle_trips.csv';
%filename = '/Users/yhindy/drive/research/SVN/AMoD_congestion/res/November1data.csv';
MData = csvread(filename);

distortedPercentage = .3;

%this file writes TIME PICKUPNODE DROPOFFNODE for each trip in the March1
%dataset
LoadRoadGraphSeattle; %Seattle
%LoadRoadGraphLarge; %New York
load('tripDataNY100Stations.mat'); %station locations are C
load('station_node_map_seattle.mat');
LEN = length(MData);
outdata = zeros(LEN, 3);
outdata_stations = zeros(LEN, 3);
NodesLocation = NodesLocation;
Stations = C*1000;


disp('starting')
ind = 1;
for i=1:LEN
    time = MData(i,3)*60*60 + MData(i,4)*60 + MData(i,5);
    pickupxy = [MData(i,6) MData(i,7)]*1000;
    dropoffxy = [MData(i,8) MData(i,9)]*1000;
    realpickupnode = findClosestNode(pickupxy, NodesLocation);
    pickupstation = nodestostations(realpickupnode);
    %pickupstationasnode = stationstonodes(pickupstation);
    realdropoffnode = findClosestNode(dropoffxy, NodesLocation);
    if (i < 50)
        fprintf('pickup: %d, dropoff: %d \n', realpickupnode, realdropoffnode);
    end
    dropoffstation = nodestostations(realdropoffnode);
    %dropoffstationasnode = stationstonodes(dropoffstation);
    
    if dropoffstation ~= pickupstation
        chance = rand;
        if chance <= distortedPercentage % pick a random node
            realdropoffnode = ceil(rand*length(NodesLocation));
        end
        outdata(ind,:) = [time realpickupnode realdropoffnode];
        outdata_stations(ind,:) = [time pickupstation dropoffstation];
        ind = ind + 1;
        if ~mod(ind, 100)
            disp(ind);
        end
    end 
end

save('SeattleSourcesSinksNodesDistorted.mat', 'outdata');
%save('March26SourcesSinksStations.mat', 'outdata_stations');

