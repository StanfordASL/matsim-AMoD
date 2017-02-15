function [adjustedRoadCap] = adjustRoadCap(RoadCap, LinkFreeFlow, LinkLength)

load('linkMap.mat');
load('vehicledata.mat');
linkMap = linkMap';

usedRoadCap = zeros(size(RoadCap,1), size(RoadCap,1));

LinkFreeFlow = LinkFreeFlow*3600; % converting m/s to m/hr

for i = 1:size(vehlocations)
    amount = vehlocations(i);
    startlink = linkMap(i,1);
    endlink = linkMap(i,2);
    amount = amount * LinkFreeFlow(startlink, endlink) / LinkLength(startlink, endlink);
    usedRoadCap(startlink, endlink) = amount;
end

adjustedRoadCap = max(round(RoadCap - usedRoadCap),zeros(size(RoadCap)));
    

