function [SourcesReb, SourceFlows, SinksReb, SinkFlows] = calcFlowsFromDistribution(threshold)

% this function calculates rebalancing sources, sinks and flows based 
% on the vehicle distribution given by the AMoDTaxiOptimizer.java class

flagdistributedepartures=1;
if flagdistributedepartures
    disp('WARNING: randomizing start location in each cell in calcFlowsFromDistribution')
end

load('vehicledata.mat');
load('station_node_map_NYOSM.mat');

SourcesReb = [];
SourceFlows = [];
SinksReb = [];
SinkFlows = [];

% some stations don't correspond to any nodes, so we keep track of
% which ones are legit and which ones aren't
availablestations = 1:length(stationstonodes);
availablestations = ismember(availablestations, nodestostations);

% the ideal distribution vector designates how many cars should be in each 
% station to have an equilibrium. For example distribution(i) is the number
% of vehicles that should be at station i. For right now, it is uniform 
load('station_node_map_NYOSM.mat');

idealDistribution = ceil(ones(length(vehdistribution), 1)*totalvehicles/sum(availablestations));

[vexcess, vdesired] = calcLegacyDistribution;
% positive entries represent rebalancing sources, negative entries represent
% rebalancing sinks

distributionDiff = vexcess - vdesired;

%distributionDiff<-vexcess-vdesired. distributionDiff should sum to > zero;
%use calcLegacyDistribution to compute vexcess and vdesired.


for i = 1:length(distributionDiff)
    if availablestations(i)
        currDiff = distributionDiff(i);
        currNode = stationstonodes(i);
        if currDiff > 0 % source
            if flagdistributedepartures
                %rng(1);
                for j = 1:currDiff
                    %SourcesReb = [SourcesReb, currNode];
                    
                    %Modified by Federico: pick a random start node within the
                    %station's domain
                    currStartCandidates=find(nodestostations==i);
                    currStartNode=currStartCandidates(randi(length(currStartCandidates)));
                    SourcesReb = [SourcesReb, currStartNode];
                    SourceFlows = [SourceFlows, 1];
                end
            else
                % This would be way faster:
                SourcesReb = [SourcesReb, currNode];
                SourceFlows = [SourceFlows, currDiff];
            end
        elseif currDiff < 0 % sink
            if flagdistributedepartures
                %rng(1);
                for j = 1:-currDiff
                    %SinksReb = [SinksReb, currNode];
                    
                    currEndCandidates=find(nodestostations==i);
                    currEndNode=currEndCandidates(randi(length(currEndCandidates)));
                    SinksReb = [SinksReb, currEndNode];
                    SinkFlows = [SinkFlows, 1];
                end
            else
                % for j = 1:-currDiff
                %     SinksReb = [SinksReb, currNode];
                %     SinkFlows = [SinkFlows, 1];
                % end
                SinksReb = [SinksReb, currNode];
                SinkFlows = [SinkFlows, -currDiff];
            end
        end
    end
end

if isempty(SourcesReb)
    SourcesReb=[1];
    SourceFlows=[0];
end
if isempty(SinksReb)
    SinksReb=[1];
    SinkFlows=[0];
end

% % shuffle matrices so that when we chop off the bottom it won't matter
% [SourcesReb,sourceperm] = shuffle(SourcesReb);
% SourceFlows = SourceFlows(sourceperm); %Why were you shuffling SourcesReb and SourcesFlow independently? That only works if SourceFlows is a vector of ones, and then you do not need to shuffle it.
% [SinksReb,sinkperm] = shuffle(SinksReb);
% SinkFlows = SinkFlows(sinkperm);
% 
% finalSize = ceil(threshold*min(length(SinksReb),length(SourcesReb)));
% 
% size(SourcesReb)
% size(SourceFlows)
% size(SinksReb)
% size(SinkFlows)
% finalSize
% 
% pause

% SourcesReb = SourcesReb(1:finalSize);
% SourceFlows = SourceFlows(1:finalSize);
% SinksReb = SinksReb(1:finalSize);
% SinkFlows = SinkFlows(1:finalSize);

