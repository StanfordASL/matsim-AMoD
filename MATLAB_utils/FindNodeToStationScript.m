clear all; close all; clc;

LoadRoadGraphNYOSM;
load('tripDataNY100Stations.mat')
StationsLocations=C*1e3;

[nodestostations,stationstonodes]=FindNodeToStation(NodesLocation,StationsLocations);

save('station_node_map_NYOSM','nodestostations','stationstonodes')