function [FracRebSolFlow,rho_smallestpath]=TIFindFracRebSol(FullRebPaths,N,M,SourcesReb,SinksReb,SourceFlows, SinkFlows)

% Computes a fractional rebalancing solution of from a fractional
% rebalancing flow. Assumes all nodes have the same intensity. Sources and
% Sinks are the sources and sinks of the customer flows.

%% Clean up rebalancing sources and sinks

% Clean up sources and sinks. If a node is both in sources and in
% sinks, remove it from both (exactly once).
%SourcesReb=Sinks';
%SinksReb=Sources';
%SourcesRebIdx=ones(size(SourcesReb)); %We use these to keep track of what sources and sinks are duplicated and should be removed
%SinksRebIdx=ones(size(SinksReb));


%for i=1:length(SourcesReb)
%    if sum(SinksReb==SourcesReb(i)) %If there is a duplicate
%        SinkIndex=find(SinksReb==SourcesReb(i));
%        for l=1:length(SinkIndex)
%            if (SinksRebIdx(SinkIndex(l))) %If some sink that corresponds to a source is not removed
%                SourcesRebIdx(i)=0;
%                SinksRebIdx(SinkIndex(l))=0;
%                break;
%            end
%        end
%    end
%end

%Select the surviving sources and sinks.
%SourcesReb=SourcesReb(SourcesRebIdx==1);
%SinksReb=SinksReb(SinksRebIdx==1);
%SourceFlows=Flows(SourcesRebIdx==1)';
%SinkFlows=Flows(SinksRebIdx==1)';
% 
% SourceFlows = ones(1,length(SourcesReb));
% SinkFlows = ones(1, length(SinksReb));

%[SourcesReb, SinksReb, SourceFlows, SinkFlows] = cleanupSourcesAndSinks(Sources, Sinks, Flows);

%% Compute flow decompositon of rebalancing paths
[allpaths,pathsnodeindex] = TIRebPathDecomposition(FullRebPaths,N,M,SourcesReb,SinksReb,SourceFlows, SinkFlows);

%% Select path of minimal flow
rho_smallestpath=Inf;
smallest_path_idx=[0 0];

%overallflow=0;

for s=1:length(allpaths)
    for p=1:length(allpaths{s})
        temppathflow=allpaths{s}{p}(end,2);
        %overallflow=overallflow+temppathflow;
        if temppathflow<=rho_smallestpath
            rho_smallestpath=temppathflow;
            smallest_path_idx=[s p];
        end
        
%         if sum (pathsnodeindex{s}{p}(2) == [98 99 122 149])
%             fprintf('Path s: %d p: %d has weight %f, source %d, sink %d\n',s,p,temppathflow,pathsnodeindex{s}{p}(1),pathsnodeindex{s}{p}(2))
%         end
%         
    end
end

%% Let's do some diagnostics on the path decomposition, shall we?
% sourcepathflow=zeros(length(allpaths),1);
% sinkpathflow=zeros(length(allpaths),1);
% for s=1:length(allpaths)
%     sourcepathflow(s)=0;
%     for p=1:length(allpaths{s})
%         temppathflow=allpaths{s}{p}(end,2);
%         sourcepathflow(s)=sourcepathflow(s)+temppathflow;
%         sinkpathflow(pathsnodeindex{s}{p}(2))=sinkpathflow(pathsnodeindex{s}{p}(2))+temppathflow;
%     end
%     if (abs(sourcepathflow(s)-SourceFlows(s))>1e-14)
%         fprintf('Weird: source %d has outgoing path flow %f and source flow %f\n',s,sourcepathflow(s),SourceFlows(s))
%     end
% end
% 
% for s=1:length(allpaths)
%     if (abs(sinkpathflow(s)-SinkFlows(s))>1e-14)
%         fprintf('Weird: sink %d has outgoing path flow %f and sink flow %f\n',s,sinkpathflow(s),SinkFlows(s))
%     end 
% end

%% We retrieve the source and sink of the smallest path
Smallest_path_source_idx=pathsnodeindex{smallest_path_idx(1)}{smallest_path_idx(2)}(1);
Smallest_path_sink_idx=pathsnodeindex{smallest_path_idx(1)}{smallest_path_idx(2)}(2);

%Just checking
assert(SourcesReb(Smallest_path_source_idx)==allpaths{smallest_path_idx(1)}{smallest_path_idx(2)}(1,1),'Assertion failed! The source of the smallest path was incorrectly identified')
assert(SinksReb(Smallest_path_sink_idx)==allpaths{smallest_path_idx(1)}{smallest_path_idx(2)}(end,1),'Assertion failed! The sink of the smallest path was incorrectly identified')


%% Build graph with the remaining origins and destinations

%In the graph, sources come first, then sinks. Thus, the i-th source node
%is node i, the j-th sink node is node length(SourcesReb)+j

MatchingNeighbors=cell(length(SourcesReb) + length(SinksReb),1);
LinkWeights=zeros(length(SourcesReb)+length(SinksReb),length(SourcesReb)+length(SinksReb));

%countPaths=0;
%cpsource=zeros(length(allpaths),1);
for s=1:length(allpaths) %Go through all paths
    for p=1:length(allpaths{s})
        %Check the paths do not touch the edge of smallest weight
        if (pathsnodeindex{s}{p}(1)~=Smallest_path_source_idx & pathsnodeindex{s}{p}(2)~=Smallest_path_sink_idx)
            MatchingNeighbors{pathsnodeindex{s}{p}(1)}=[MatchingNeighbors{pathsnodeindex{s}{p}(1)}, length(SourcesReb)+ pathsnodeindex{s}{p}(2)]; %Add a link between the source and the sink of path (s,p)
            LinkWeights(pathsnodeindex{s}{p}(1),length(SourcesReb)+pathsnodeindex{s}{p}(2))=allpaths{s}{p}(end,2);
            %countPaths=countPaths+1;
            %cpsource(s)=cpsource(s)+1;
        end
    end
end
%% Compute a matching of weight rho_smallestpath

% We just use the multicommodity flow solver with no relaxation whatsoever.
% We look for a feasible link, so traveltimes=1 for everyone.
% Sources and Sinks can be used to isolate the min. cost edge, declaring it
% not a source or sink (in addition to having cap at zero on all incident
% edges). Then SourceFlow and SinkFlow are straightforward, all at rho.
% However, careful: you need to move from a description where sources and
% sinks are separate to a description where they go together

MatchTravelTimes=ones(length(SourcesReb)+length(SinksReb));

MatchSources=1:1:length(SourcesReb);
MatchSources=MatchSources(MatchSources~=Smallest_path_source_idx);
%Check: if MatchSources is empty after this, then the remaining matching
%only has one, integral flow. Then you should just output that and skip the
%part where you compute a matching with a LP.
if (length(MatchSources)==0)
    rho_matching_flow_sol=zeros(1,N*N);
else
    MatchSinks=length(SourcesReb)+1:1:(length(SourcesReb)+length(SinksReb));
    MatchSinks=MatchSinks(MatchSinks~=(Smallest_path_sink_idx)+length(SourcesReb));
    
    MatchFlowsIn=rho_smallestpath*ones(1,length(MatchSources));
    MatchFlowsOut=rho_smallestpath*ones(1,length(MatchSinks));
    fprintf('Computing matching of weight %f (remaining flow %f)\n',rho_smallestpath,SourceFlows(1))
    [rho_matching_flow_sol] = TIMulticommodityFlow(MatchingNeighbors,LinkWeights,MatchTravelTimes,MatchSources,MatchSinks,MatchFlowsIn,MatchFlowsOut,0,0,0,0);
    
    assert(~isempty(rho_matching_flow_sol),'Assertion failed! Could not find a fractional rebalancing solution. This is bad.')
end
%% Now we have the matching. We need to translate this back, though.
% We go through the solution. For each edge (corresponding to a path), we
% find the relevant path by going through pathsnodeindex. This is kind of
% messy. Then, we pull out the path (in allpaths) and add flow
% rho_smallestpath to it.

FindMatchFlowij=@(i,j) (length(SourcesReb)+length(SinksReb))*(i-1)+length(SourcesReb)+j;

FracRebSolFlowMat=zeros(N,N);
FracRebSolFlow=zeros(N*N,1);

FindFracRebFlowij=@(i,j) N*(i-1)+j;

% We find an edge that's nonzero. If its weight is not rho, we throw an
% exception. If it is rho, we look for its origin and destination. We have
% the origin id oid and the destination id did (identifying origin and
% destination node by their position in SourcesReb and SinksReb
% respectively). We look in pathsnodeindex{oid} until we find that
% pathsnodeindex{oid}{j}=[oid did]. Then the relevant path is
% allpaths{oid}{j}. 

for i=1:length(SourcesReb)
    for j=1:length(SinksReb)
        assert(rho_matching_flow_sol(FindMatchFlowij(i,j))==0 | rho_matching_flow_sol(FindMatchFlowij(i,j))==rho_smallestpath,'Assertio failed! The matching is not integral')
        
        if (rho_matching_flow_sol(FindMatchFlowij(i,j))>0)
            pathids=[i -1]; %These identify the path in allpaths
            for k=1:length(pathsnodeindex{i})
                if (pathsnodeindex{i}{k}(1)==i & pathsnodeindex{i}{k}(2)==j)
                    pathids(2)=k;
                end
            end
            assert(pathids(2)~=-1,'Assertion failed! Did not find path')
            % Now we have our path allpaths{pathids(1)}{pathids(2)}. We add
            % the right flow in FracRebSolFlow
            
            for k=2:size(allpaths{pathids(1)}{pathids(2)},1) %Go through the path
                FracRebSolFlowMat(allpaths{pathids(1)}{pathids(2)}(k-1,1),allpaths{pathids(1)}{pathids(2)}(k,1))=FracRebSolFlowMat(allpaths{pathids(1)}{pathids(2)}(k-1,1),allpaths{pathids(1)}{pathids(2)}(k,1))+rho_smallestpath;
            end
            
        end
    end
end

for k=2:size(allpaths{smallest_path_idx(1)}{smallest_path_idx(2)},1) %Go through the smallest path
    FracRebSolFlowMat(allpaths{smallest_path_idx(1)}{smallest_path_idx(2)}(k-1,1),allpaths{smallest_path_idx(1)}{smallest_path_idx(2)}(k,1))=FracRebSolFlowMat(allpaths{smallest_path_idx(1)}{smallest_path_idx(2)}(k-1,1),allpaths{smallest_path_idx(1)}{smallest_path_idx(2)}(k,1))+rho_smallestpath;
end

for i=1:N
    for j=1:N
        FracRebSolFlow(FindFracRebFlowij(i,j))=FracRebSolFlowMat(i,j);
    end
end