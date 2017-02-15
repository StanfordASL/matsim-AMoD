function [] = makeMatrices(start, endind)

MATRIX_DIRECTORY = '/media/Big_Data/yhindy/reb_matrices/';

assert((start < endind), 'Invalid start and end');

load('1820SourcesSinksStations.mat');
load(strcat(MATRIX_DIRECTORY, 'available_Ms.mat'));
load('station_node_map.mat');

times = outdata(:,1);
Temp_Sources = outdata(:,2);
Temp_Sinks = outdata(:,3);

LoadRoadGraphLarge;

milpflag = 0;
congrelaxflag = 0;
sourcerelaxflag = 1;

N = length(RoadGraph);

RoadCap = processCompressedStructure(RoadCap, N);

TravelTimes = LinkTime;

RoadGraph = RoadGraph';


RebWeight = 1;

for i = start:endind
    M = i*100;
    if (~any(M == matrices))
        message = strcat('Making matrices for ', num2str(M), ' flows');
        disp(message);
        Sources = Temp_Sources(1:M);
        Sinks = Temp_Sinks(1:M);
        FlowsIn = ones(M, 1);
        Sources = nodestostations(Sources);
        Sinks = nodestostations(Sinks);
        [Aeqsparse, Aeqentry, Aineqsparse, Aineqentry] = buildMatricesReb(RoadGraph, RoadCap, TravelTimes, Sources, Sinks, FlowsIn, FlowsIn, milpflag, congrelaxflag, sourcerelaxflag);
        filename = strcat(MATRIX_DIRECTORY, 'RebCachedA', num2str(M),'.mat');
        save(filename, 'Aeqsparse', 'Aineqsparse', 'Aeqentry', 'Aineqentry', '-v7.3');
        load(strcat(MATRIX_DIRECTORY, 'available_Ms.mat'));
        matrices = [matrices M];
        save(strcat(MATRIX_DIRECTORY, 'available_Ms.mat'), 'matrices');
    end
end

disp('Done!');
        



    