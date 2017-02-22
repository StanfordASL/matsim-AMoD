function [ passpaths, rebpaths ] = runOptimization( startTime, timeHorizon, RebWeight, threshold, legacyRebalance, filename )

%for debug: filename is /home/frossi2/SVN/code/AMoD_congestion/AMoD_taxi/src/main/resources/optimizerdata/March1SourcesSinksNodes.mat

% boolean flag to enable write to disk of time to solve the optimization
logSolTimes = 1;

% Timing
timerAll=tic;
timerAllCPU=cputime;

% boolean flag to only use rebalancing routes, therefore letting passengers
% choose using dijkstra (an empty passpaths will be passed to MATSim).
onlyRebalance = 1;

% if this flag is turned on, the rebalancing routes are computed using
% Dijkstra
% This is now a parameter
%legacyRebalance = 1;

% If this is turned on, we DO NOT rebalance
donothingflag = 0;

stationsflag = 1; % if this is on, the simulation runs station-wise

%Process requests in question:
MATRIX_DIRECTORY = '/media/Big_Data/yhindy/matrices/';

load(filename);
load('station_node_map.mat');
load('station_node_map_NYOSM.mat');

% filename = 'res/excessrequests.csv';
% MData = csvread(filename);
% for i = 1:length(MData)
%     Sources = [Sources; MData(i,1)];');
if ~onlyRebalance
    availablematrices = strcat(MATRIX_DIRECTORY, 'available_As.mat');
    load(availablematrices);
    sort(matrices);
end
times = outdata(:,1);
Temp_Sources = outdata(:,2);
Temp_Sinks = outdata(:,3);

Sources = [];
Sinks = [];


t = cputime;

% Configure road map below
LoadRoadGraphNYOSM; % New York
%load('bin/zhangSeattleData.mat'); % Seattle

milpflag = 0;
congrexflag = 0;

N = length(RoadGraph);

RoadCap = processCompressedStructure(RoadCap, N);

RoadCap = threshold*RoadCap;

RoadCap = adjustRoadCap(RoadCap, LinkFreeFlow, LinkLength);

TravelTimes = LinkTime;

RoadGraph = RoadGraph';

% filename = 'res/excessrequests.csv';
% MData = csvread(filename);
% for i = 1:length(MData)
%     Sources = [Sources; MData(i,1)];
% Sinks = [Sinks; MData(i,2)];

if ~onlyRebalance & ~donothingflag
    startind = 1;
    endind = 1;

    i = startind;

    while i < length(times) && times(i) < startTime
        i = i + 1;
    end
    startind = i;

    while i < length(times) && times(i) < startTime + timeHorizon
        num = rand;
        if num < threshold
            Sources = [Sources; Temp_Sources(i)];
            Sinks = [Sinks; Temp_Sinks(i)];
        end
        i = i + 1;
    end
    endind = i;

    if (stationsflag)
        Sources = nodestostations(Sources);
        Sinks = nodestostations(Sinks);
    end

    [Sources, Sinks, FlowsIn] = consolidateSourcesAndSinks(Sources, Sinks);

    sourceSize = size(Sources, 1);
    for i = 2:length(matrices)
        if sourceSize < matrices(i)
            M = matrices(i-1);
            disp(sourceSize);
            disp(M);
            break;
        end
    end

    Sources = Sources(1:M);
    Sinks = Sinks(1:M);
    FlowsIn = FlowsIn(1:M);

    threshold = threshold*M/sourceSize;

    %Timing
    timerOpt=tic;
    timerOptCPU=cputime;
    %\Timing
    [cplex_out, numvehicles] = TIBalancedMCFlow(RoadGraph, RoadCap, TravelTimes, RebWeight, Sources, Sinks, FlowsIn, milpflag, congrexflag, 0);
    %Timing
    optimizerTimeO=toc(timerOpt);
    optimizerCPUTimeO=cputime-timerOptCPU;
    %/Timing

    %compute total number of vehicles
    %for each node, look at how many vehicles are going out
    %sum over all passengers of the indicator variables of the passengers 1 to
    %find total number of vehicles

    %save('cplex_out_ex.mat', 'cplex_out', 'Sources', 'Sinks', 'N', 'M', 'FlowsIn', '-v7.3');
    disp('Checkpoint 0');

        %Timing
    timerPax=tic;
    timerPaxCPU=cputime;
    %\Timing
    allpaths = TIPaxPathDecomposition(cplex_out, N, M, Sources, Sinks, FlowsIn);

    disp('Checkpoint 0.5');
    passpaths = TISamplePaxPaths(allpaths);

    [SinksReb, SourcesReb, SinkFlows, SourceFlows] = cleanUpSourcesAndSinks(Sources, Sinks, FlowsIn);
    [SourcesReb, SourceFlows] = flattenFlows(SourcesReb, SourceFlows);
    [SinksReb, SinkFlows] = flattenFlows(SinksReb, SinkFlows);
    SourcesReb = SourcesReb';
    SinksReb = SinksReb';
    SourceFlows = SourceFlows';
        [reb_output, numvehicles] = TIMulticommodityFlow_f(RoadGraph, RoadCap, TravelTimes, SourcesReb, SinksReb, SourceFlows, SinkFlows, milpflag, congrexflag, 1);

        [reb_output, numvehicles] = TIMulticommodityFlow_f(RoadGraph, RoadCap, TravelTimes, SourcesReb, SinksReb, SourceFlows, SinkFlows, milpflag, congrexflag, 1);

        fprintf('Congestion-aware optimizer requires rebalancing %d vehicles\n',numvehicles)

        S=length(SourcesReb);
        M=1;
        %SoRelaxkl=@(k,l) N*N*M + N*N + S*(k-1) + l;
        %SiRelaxkl=@(k,l) N*N*M + N*N + M*S + S*(k-1) + l;

        E=0; %Compute number of edges
        for i=1:length(RoadGraph)
            E=E+length(RoadGraph{i});
        end

        SoRelaxkl=@(k,l) E*M + E + S*(k-1) + l;
        SiRelaxkl=@(k,l) E*M + E + M*S + S*(k-1) + l;

        relFlowIn = SourceFlows - reb_output(SoRelaxkl(1,1):SoRelaxkl(1,S))';
        relFlowOut = SinkFlows - reb_output(SiRelaxkl(1,1):SiRelaxkl(1,S))';
        fprintf('Congestion-aware optimizer requires rebalancing %d vehicles\n',numvehicles)

        S=length(SourcesReb);
        M=1;
        %SoRelaxkl=@(k,l) N*N*M + N*N + S*(k-1) + l;
        %SiRelaxkl=@(k,l) N*N*M + N*N + M*S + S*(k-1) + l;

        E=0; %Compute number of edges
        for i=1:length(RoadGraph)
            E=E+length(RoadGraph{i});
        end

        SoRelaxkl=@(k,l) E*M + E + S*(k-1) + l;
        SiRelaxkl=@(k,l) E*M + E + M*S + S*(k-1) + l;

        relFlowIn = SourceFlows - reb_output(SoRelaxkl(1,1):SoRelaxkl(1,S))';
        relFlowOut = SinkFlows - reb_output(SiRelaxkl(1,1):SiRelaxkl(1,S))';
    SinkFlows = SinkFlows';
    disp('Checkpoint 1');

    FullRebPaths = cplex_out(N*N*M+1:N*N*(M+1));
    [FracRebSols, FracRebWeights] = TIDecomposeFracRebSol(FullRebPaths, N, M, SourcesReb, SinksReb, SourceFlows, SinkFlows);
    [Reb_solution] = TISampleRebSol(FracRebSols, FracRebWeights);

    disp('Checkpoint 2');
    [rebpaths,~] = TIRebPathDecomposition(Reb_solution, N, M, SourcesReb, SinksReb, SourceFlows, SinkFlows);
    rebpaths = rebpaths';

    %Timing
    optimizerTimeP=toc(timerPax);
    optimizerCPUTimeP=cputime-timerPaxCPU;
    %/Timing

    numvehicles = numvehicles/timeHorizon/threshold;

    save('~/SVN/code/AMoD_congestion/AMoD_taxi/src/main/resources/optimizerpaths.mat', ...
     'passpaths', 'rebpaths', 'numvehicles', 'Sources', 'Sinks');
elseif ~donothingflag

    % TODO: write logic for when only rebalancing
    passpaths = {}; % empty cell will tell route scheduler to use Dijkstra
    [SourcesReb, SourceFlows, SinksReb, SinkFlows] = calcFlowsFromDistribution(threshold);
    sourceSize = length(SourcesReb);
%     for i = 2:length(matrices)
%         if sourceSize < matrices(i)
%             M = matrices(i-1);
%             disp(sourceSize);
%             disp(M);
%             break;
%         end
%     end
%     SourcesReb = SourcesReb(1:M);
%     SourceFlows = SourceFlows(1:M);
%     SinksReb = SinksReb(1:M);
%     SinkFlows = SinkFlows(1:M);
%
%     threshold = threshold*M/sourceSize;

    if ~legacyRebalance
        %Timing
        timerOpt=tic;
        timerOptCPU=cputime;
        %\Timing

        %adjustedRoadCap = adjustRoadCap(RoadCap, currFlows);
        [reb_output, numvehicles] = TIMulticommodityFlow_f_R(RoadGraph, RoadCap, TravelTimes, SourcesReb, SinksReb, SourceFlows, SinkFlows, milpflag, congrexflag, 1);

        fprintf('Congestion-aware optimizer requires rebalancing %d vehicles\n',numvehicles)

        S_so=length(SourcesReb);
        S_si=length(SinksReb);
        M=1;
        %SoRelaxkl=@(k,l) N*N*M + N*N + S*(k-1) + l;
        %SiRelaxkl=@(k,l) N*N*M + N*N + M*S + S*(k-1) + l;

        E=0; %Compute number of edges
        for i=1:length(RoadGraph)
            E=E+length(RoadGraph{i});
        end

        SoRelaxkl=@(k,l) E*M + E + S_so*(k-1) + l;
        SiRelaxkl=@(k,l) E*M + E + M*S_so + S_si*(k-1) + l;



        relFlowIn = SourceFlows - reb_output(SoRelaxkl(1,1):SoRelaxkl(1,S_so))';
        relFlowOut = SinkFlows - reb_output(SiRelaxkl(1,1):SiRelaxkl(1,S_si))';

        fprintf('Did not rebalance %d in-flows because of congestion\n',sum(reb_output(SoRelaxkl(1,1):SoRelaxkl(1,S_so))))
        fprintf('Did not rebalance %d out-flows because of congestion\n',sum(reb_output(SiRelaxkl(1,1):SiRelaxkl(1,S_si))))

        [rebpaths, ~] = TIRebPathDecomposition_f(reb_output, RoadGraph, N, M, SourcesReb, SinksReb, relFlowIn, relFlowOut,1);

        testnode=63;
        ccounterso=0;
        fcounterso=0;
        ccountersi=0;
        fcountersi=0;
        for iii=1:length(rebpaths)
            for jjj=1:length(rebpaths{iii})
                if nodestostations(rebpaths{iii}{jjj}(1,1))==testnode
                    ccounterso=ccounterso+1;
                    fcounterso=fcounterso+rebpaths{iii}{jjj}(end,2);
                end
                if nodestostations(rebpaths{iii}{jjj}(end,1))==testnode
                    ccountersi=ccountersi+1;
                    fcountersi=fcountersi+rebpaths{iii}{jjj}(end,2);
                end
            end
        end

        rebpaths2={};
        rp2c=1;
        for iii=1:length(rebpaths)
            for jjj=1:length(rebpaths{iii})
                rebpaths2{rp2c}{1}=rebpaths{iii}{jjj};
                rp2c=rp2c+1;
            end
        end
        rebpaths=rebpaths2;


        fprintf('%d routes (flow %d) going out of station %d, %d routes (flow %d) going in\n',ccounterso,fcounterso,testnode,ccountersi,fcountersi)
        %Timing
        optimizerTimeO=toc(timerOpt);
        optimizerCPUTimeO=cputime-timerOptCPU;
        optimizerTimeP=0;
        optimizerCPUTimeP=0;
        %/Timing

        rebpaths = rebpaths';
        fprintf('Number of rebalancing sources that are active: %d\n',length(rebpaths))

        %numvehicles = length(rebpaths);
        save('~/SVN/code/AMoD_congestion/AMoD_taxi/src/main/resources/optimizerpaths.mat', ...
     'passpaths', 'rebpaths', 'numvehicles', 'Sources', 'Sinks');
    else
        disp('Legacy rebalancing')
        rebpaths = {};
        numStations = length(stationstonodes);
        rebalanceQueue = cell(numStations,1);
        % define Tij
        Tij = zeros(numStations, numStations);
        for i = 1:numStations
            for j = 1:numStations
                firstNode = stationstonodes(j);
                secondNode = stationstonodes(i);
                dist = NodesLocation(firstNode,:) - NodesLocation(secondNode,:);
                Tij(i,j) = norm(dist , 1);
            end
        end

        [vexcess, vdesired] = calcLegacyDistribution;

        %Timing
        timerOpt=tic;
        timerOptCPU=cputime;
        %\Timing
        cvx_begin quiet
            variable numij(numStations, numStations)
            minimize (sum(sum(Tij.*numij)));
            subject to
            vexcess + sum((numij' - numij)')' >= vdesired;
            numij >= 0;
        cvx_end

        if (sum(sum(isnan(numij))) || ~(strcmp(cvx_status,'Solved') || strcmp(cvx_status,'Suboptimal') || strcmp(cvx_status,'Inaccurate/Solved')))
            disp('ERROR: CVX could not find a solution')
            disp(cvx_status)
            passpaths={}
            rebpaths={}
            return
        end

        %Timing
        optimizerTimeO=toc(timerOpt);
        optimizerCPUTimeO=cputime-timerOptCPU;
        optimizerTimeP=0;
        optimizerCPUTimeP=0;
        %/Timing

        % make sure numij is integer
        numij = round(numij);
        % add rebalancing vehicles to queues
        for i = 1:numStations
            for j = 1:numStations
                for k = 1:numij(i,j)
                    rebalanceQueue{i} = [rebalanceQueue{i} j];
                end
            end
        end



        %A = LinkTime > 0; % where there is a link
        %C = LinkTime;

        %numtrips = 1;
        %for i=1:numStations
        %    destinations = unique(rebalanceQueue{i});
        %    numdests = length(destinations);
        %    for j = 1:numdests
        %        numtrips = numtrips + 1;
        %    end
        %end

        numtrips=0;
        for i=1:length(rebalanceQueue)
            numtrips=numtrips+length(rebalanceQueue{i});
        end

        for i=1:size(rebalanceQueue) % make the componenets column-wise
            rebalanceQueue{i} = rebalanceQueue{i}';
        end

        numvehicles = numtrips;

        fprintf('Number of trips: %d\n',numtrips)

        % real path: '~/SVN/code/AMoD_congestion/AMoD_taxi/src/main/resources/optimizerpairs.mat'
        save('~/SVN/code/AMoD_congestion/AMoD_taxi/src/main/resources/optimizerpairs.mat', ...
            'rebalanceQueue', 'numvehicles');
    end
elseif donothingflag
    disp('Doing nothing')
    passpaths={};
    rebpaths={};
    numvehicles=0;
    Sources=[];
    Sinks=[];
    numStations = length(stationstonodes);
    rebalanceQueue = cell(numStations,1);

    optimizerTimeO=0;
    optimizerCPUTimeO=0;
    optimizerTimeP=0;
    optimizerCPUTimeP=0;

    save('~/SVN/code/AMoD_congestion/AMoD_taxi/src/main/resources/optimizerpaths.mat', ...
     'passpaths', 'rebpaths', 'numvehicles', 'Sources', 'Sinks');
    save('~/SVN/code/AMoD_congestion/AMoD_taxi/src/main/resources/optimizerpairs.mat', ...
            'rebalanceQueue', 'numvehicles');
end

optimizerTimeAll=toc(timerAll);
optimizerCPUTimeAll=cputime-timerAllCPU;

 %Timing
 if logSolTimes
     Wallfilename=['optimizerWallTime_onlyReb',num2str(onlyRebalance),'_legacyReb',num2str(onlyRebalance)];
     Clockfilename=['optimizerClockTime_onlyReb',num2str(onlyRebalance),'_legacyReb',num2str(onlyRebalance)];
     WallfilenameO=['optimizerOnlyWallTime_onlyReb',num2str(onlyRebalance),'_legacyReb',num2str(onlyRebalance)];
     ClockfilenameO=['optimizerOnlyClockTime_onlyReb',num2str(onlyRebalance),'_legacyReb',num2str(onlyRebalance)];
     WallfilenameP=['optimizerPaxWallTime_onlyReb',num2str(onlyRebalance),'_legacyReb',num2str(onlyRebalance)];
     ClockfilenameP=['optimizerPaxClockTime_onlyReb',num2str(onlyRebalance),'_legacyReb',num2str(onlyRebalance)];
     walltimefid=fopen(Wallfilename,'a');
     clocktimefid=fopen(Clockfilename,'a');
     clocktimeOfid=fopen(ClockfilenameO,'a');
     walltimeOfid=fopen(WallfilenameO,'a');
     clocktimePfid=fopen(ClockfilenameP,'a');
     walltimePfid=fopen(WallfilenameP,'a');
     fprintf(walltimefid,'%f\n',optimizerTimeAll);
     fprintf(clocktimefid,'%f\n',optimizerCPUTimeAll);
     fprintf(walltimeOfid,'%f\n',optimizerTimeO);
     fprintf(clocktimeOfid,'%f\n',optimizerCPUTimeO);
     fprintf(walltimePfid,'%f\n',optimizerTimeP);
     fprintf(clocktimePfid,'%f\n',optimizerCPUTimeP);
     fclose(walltimefid);
     fclose(clocktimefid);
     fclose(walltimeOfid);
     fclose(clocktimeOfid);
     fclose(walltimePfid);
     fclose(clocktimePfid);
 end
 %/Timing



disp('Done!');
disp(size(rebpaths));
end
