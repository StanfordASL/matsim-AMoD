function [allpaths] = TIPaxPathDecomposition(MCFlowOutput,N,M,Sources,Sinks,Flows)

% Computes path decomposition based on fractional flow output stored in
% MCFlowOutput.

Flowkij=@(k,i,j) (k-1)*N*N + (i-1)*N + j;

pathcounter=zeros(M,1);
allpaths=cell(M,1);

% For each passenger
% Set the first node as that passenger's source
% Search for flows exiting the first node, pick the largest and ID that as
% the second node
% Set the first node as the second node, iterate
% Until you reach the sink
% Remove the relevant flow


for k=1:M %For each passenger
    k
    
    if size(allpaths{k})==0
        allpaths{k}={};
    end
    paths={};
    
    
    while (sum(sum( MCFlowOutput(Flowkij(k,1,1):Flowkij(k,N,N)) ))>1e-8) %While there is flow left. Hack to avoid numerical issues
        
        %sum(sum( MCFlowOutput(Flowkij(k,1,1):Flowkij(k,N,N)) ))
        
        pathcounter(k)=pathcounter(k)+1;
        currpos=Sources(k);
        nextpos=currpos;
        myflow=Inf;
        
        % Go through and greedily build path
        paths{pathcounter(k)}=[currpos 1];
        while (myflow>0 & nextpos ~= Sinks(k))
            tempout = MCFlowOutput(Flowkij(k,currpos,1):Flowkij(k,currpos,N));
            [tempflow,nextpos]=max(tempout); %index and next position, picking which way to go, node with highest weight that is adjacent
            myflow=min(myflow,tempflow);
            
            paths{pathcounter(k)}=[paths{pathcounter(k)};[nextpos myflow]];
            currpos=nextpos;
        end
        
        % Remove weight from path
        for l=2:size(paths{pathcounter(k)},1)
            tempflow=paths{pathcounter(k)}(end,2);
            MCFlowOutput(Flowkij(k,paths{pathcounter(k)}(l-1,1),paths{pathcounter(k)}(l,1)))=MCFlowOutput(Flowkij(k,paths{pathcounter(k)}(l-1,1),paths{pathcounter(k)}(l,1)))-tempflow;
        end

    end
    allpaths{k}=paths;
end