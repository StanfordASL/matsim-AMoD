%Process requests in question:

% This file makes the A matrices
% To run, you need to comment out everything in TIBalancedMCFlow below
% the "Constraints Setup" section. 
% you will also need to set values for 
% cplex_out and numvehicles, as this script expects a return value.

%TODO: cache All A matrices (equality and inequality)

load('1820SourcesSinksStations.mat');
load('station_node_map.mat');
load('matrices/available_As.mat');
times = outdata(:,1);
Temp_Sources = outdata(:,2);
Temp_Sinks = outdata(:,3);

Sources = [];
Sinks = [];

% times = outdata(:,1);
% Sources = outdata(:,2);
% Sinks = outdata(:,3);

startind = 1;
endind = 1;

stationsflag = 1; % if this is on, the simulation runs station-wise



% filename = 'res/excessrequests.csv';
% MData = csvread(filename);
% for i = 1:length(MData)
%     Sources = [Sources; MData(i,1)];
%     Sinks = [Sinks; MData(i,2)];

t = cputime;

LoadRoadGraphLarge;

milpflag = 0;
congrexflag = 1;

N = length(RoadGraph);
M = size(Sources, 1);

RoadCap = processCompressedStructure(RoadCap, N);

TravelTimes = LinkTime;

RoadGraph = RoadGraph';

FlowsIn = ones(M, 1);

RebWeight = 0.05;

for i = 39:44
    M = i*100;
    if (~any(M == matrices))
        message = strcat('Making matrices for ', num2str(M), ' flows');
        disp(message);
        Sources = Temp_Sources(1:M);
        Sinks = Temp_Sinks(1:M);
        FlowsIn = ones(M, 1);
        Sources = nodestostations(Sources);
        Sinks = nodestostations(Sinks);
        [SourcesReb, SinksReb] = cleanUpSourcesAndSinks(Sources, Sinks, FlowsIn);
        [cplex_out, numvehicles] = TIBalancedMCFlow(RoadGraph, RoadCap, TravelTimes, RebWeight, Sources, Sinks, FlowsIn, milpflag, congrexflag, 0);
    end
end

disp('Done!');