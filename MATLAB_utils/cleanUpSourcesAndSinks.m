function [SourcesReb, SinksReb, SourceFlows, SinkFlows] = cleanUpSourcesAndSinks(Sources, Sinks, Flows)

% takes in unit Sources and Sinks and replaces duplicates with a single
% source/sink pair with a non-unit flow

SourcesReb = [Sources Flows];
SourcesReb = sortrows(SourcesReb);
SinksReb = [Sinks Flows];
SinksReb = sortrows(SinksReb);

SourceFlows = squashFlows(SourcesReb);
SinkFlows = squashFlows(SinksReb);

sourceRes = [];
sinkRes = [];

usedSources = [];
usedSinks = [];

for i = 1:length(SourceFlows)
    currSource = SourceFlows(i, 1);
    sinkInd = find(currSource == SinkFlows(:,1));
    if ~isempty(sinkInd) % the node is both a source and a sink 
        SourceFlow = SourceFlows(i,2);
        SinkFlow = SinkFlows(sinkInd, 2);
        if SourceFlow > SinkFlow % there is more flow leaving than coming in
            sourceRes = [sourceRes ; currSource SourceFlow - SinkFlow];
        elseif SinkFlow > SourceFlow
            sinkRes = [sinkRes; currSource SinkFlow - SourceFlow];
        end
        usedSources = [usedSources; currSource SourceFlow];
        usedSinks = [usedSinks; currSource SinkFlow];
    end
end

unusedSources = setdiff(SourceFlows, usedSources, 'rows');
unusedSinks = setdiff(SinkFlows, usedSinks, 'rows');

sourceRes = [sourceRes; unusedSources];
sinkRes = [sinkRes; unusedSinks];

if ~isempty(sourceRes) && ~isempty(sinkRes)
    SourcesReb = sourceRes(:,1)';
    SourceFlows = sourceRes(:,2)';
    SinksReb = sinkRes(:,1)';
    SinkFlows = sinkRes(:,2)';
else
    SourcesReb= [];
    SourceFlows = [];
    SinksReb = [];
    SinkFlows = [];
end
