function [vexcess, vdesired] = calcLegacyDistribution

load('vehicledata.mat');
load('station_node_map_NYOSM.mat');

% some stations don't correspond to any nodes, so we keep track of
% which ones are legit and which ones aren't
availablestations = 1:length(stationstonodes);
availablestations = ismember(availablestations, nodestostations);

% This below is WRONG and will also count unavailable stations:
% availablestations is a vector of Bools. Should use sum, not length.
numrealstations = sum(availablestations);

%This is incorrect. In road_network_sim, we compute vown as the number of
%AVAILABLE vehicles at each station, defined as
%vehdistribution-outstanding_passengers_at_the_station. So we need to get
%the number of outstanding passengers in the region and pipe it in here.
vown = vehdistribution;


totalpassengers = sum(waitingpassengers);
%If sum(vown) is not equal to sum(vdesired), the optimization will ALWAYS
%fail. this is bad.
vdesired = floor((ones(length(stationstonodes),1)*(totalvehicles - totalpassengers)/sum(availablestations)));

vdesired=vdesired.*availablestations';


vexcess = vown - waitingpassengers;


sum(abs(vexcess))
disp('Printing vown')
vown'
disp('end print')