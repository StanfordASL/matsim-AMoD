function [Sources, Sinks, Flows] = consolidateSourcesAndSinks(TempSources, TempSinks)

% this function takes in sources and sinks and consolidates duplicates 
% by just having a single source and single sink with an adjusted flow

Sources = [];
Sinks = [];
Flows = [];

for i = 1:length(TempSources)
    currSource = TempSources(i);
    currSink = TempSinks(i); 
    
    sourceIdx = find(Sources==currSource); %check if the source is in Sources
    if isempty(sourceIdx) % source isn't even in final list
        Sources = [Sources; currSource];
        Sinks = [Sinks; currSink];
        Flows = [Flows; 1];
    else 
        sinkIdx = find(Sinks == currSink); % is current sink even in the trip list ?
        tripExists = intersect(sourceIdx, sinkIdx); %does the trip already exist ?
        if isempty(sinkIdx) || isempty(tripExists) 
            Sources = [Sources; currSource]; % add the trip
            Sinks = [Sinks; currSink];
            Flows = [Flows; 1];
        else
            % fix the trip so that it has an adjusted flow
            Flows(tripExists(1)) = Flows(tripExists(1)) + 1; 
        end
    end
        % first case: sink isn't in sinks at all
        % second case: sink is in but not with currSource
        % first and second case have the same outcome (add a new trip)
        % third case: sink is in but with currSource
end