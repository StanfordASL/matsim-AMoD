function [cplex_out, numvehicles] = TIBalancedMCFlow(RoadGraph,RoadCap,TravelTimes,RebWeight,Sources,Sinks,FlowsIn,milpflag,congrelaxflag,sourcerelaxflag)

% Computes a balanced multicommodity flow on the capacitated network
% described by RoadGraph between given sources s and sinks t with a given
% flow betweek each s-t pair.
%
% The flow is (re)balanced: we compute flows for the m source-sink pairs
% and also a flow from all the sinks to all the sources (all using the same
% commodity). The m source-sink pairs represent passenger demands; the
% rebalancing flow represents empty vehicles moving from passengers'
% destinations to sources.
%
% Inputs:
% - RoadGraph, a nx1 cell structure. RoadGraph{i} contains the neighbors of
%   i
% - RoadCap, a nxn matrix. RoadCap(i,j) is the capacity of the i-j link.
% - TravelTimes, a nxn matrix. TravelTimes(i,j) is the travel time along
%   the i-j link.
% - RebWeight, the rlative importance of the rebalancing cost wrt the
%   customer cost
% - Sources, a m-x-1 vector. Sources(i) is the source node of the i-th flow
% - Sinks, same as sources
% - FlowsIn, a m-by-1 vector. FlowsIn(i) is the amount of flow entering the
%   source of flow i.
% - milpflag (default: 1). If 1, the problem is solved as a MILP. If 0, the
%   problem is solved as a linear relaxation.
% - congrelaxflag (default: 0). If this flag is on, then flows are allowed
%   to violate the  congestion constraint for a cost.
%   A slack variable is defined for each edge. The cost is defined in the
%   main code.
% - sourcerelaxflag (default: 0). %If this flag is on, each flow is allowed
%   to reduce its sources (and sinks) for a cost. This is especially useful
%   when it is not possible to compute a satisfying rebalancing flow because
%   of timing constraints but, if the congestion constraints are tight, it
%   can also preserve feasibility if not all flows can be realized.
%   A slack variable is defined for each source and sink.
%
% Forked from TIMultiCommodityFlow on Jan. 24, 2016 (Rev. 10958 on SVN)

if nargin<10
    sourcerelaxflag=0; 
end
if nargin<9
congrelaxflag=0; 
end
if nargin<8
    milpflag=1;
end

CongestionCost=5*1e7; %The cost of violating the congestion constraint
SourceCost=1e9;     % The cost of dropping a source or sink altogether

debugflag=1; %Makes output verbose
progressflag=1; 
cachedAeqflag=1;    % If true, does not compile Aeq and Beq but reads them
                    % from file. Does NOT attempt to validate cache against
                    % current data: ensure the cache file is the right one!
relaxCcostflag=0;   % If 0, the congestion relaxation cost is large and
                    % constant. If 1, the congestion relaxation cost on an
                    % edge equals the traversal time on that edge.

MATRIX_DIRECTORY = '/media/Big_Data/yhindy/matrices/';
N=length(RoadGraph);

M=size(Sources,1);
%S=size(Sources,2); % Assumption: each flow has the same number of sources 
                   %  and sinks 
                   %  i.e., for each flow k, |sources(k)|=|sinks(k)| 

StateSize=N*N*(M+1) + N*N + 2*M;

%% Utility functions
Flowkij=@(k,i,j) (k-1)*N*N + (i-1)*N + j;
Flowijr=@(i,j) M*N*N + (i-1)*N + j;
CRelaxij=@(i,j) N*N*(M+1) + (i-1) * N + j;
SoRelaxk=@(k) N*N*(M+1) + N*N + k;
SiRelaxk=@(k) N*N*(M+1) + N*N + M + k;

%% Cost function
if (debugflag)
    disp('Building cost function')
end

% auxiliary
NumEdges=zeros(N,1);
for i=1:N
    NumEdges(i)=length(RoadGraph{i});
end
EdgeNo=sum(NumEdges);
% end aux

%f_cost=spalloc(StateSize,1,(M+1)*EdgeNo+N^2+2*M);
f_cost=zeros(StateSize,1);

f_cost_numvehicles=f_cost;

if (progressflag)
    fprintf('n (out of %d):',N)
end
for i=1:N
    %for j=RoadGraph{i}
        for k=1:M
            f_cost(Flowkij(k,i,1):Flowkij(k,i,N))=TravelTimes(i,:);
            %f_cost(Flowkij(k,i,j))=TravelTimes(i,j);
        end
        f_cost(Flowijr(i,1):Flowijr(i,N))=RebWeight.*TravelTimes(i,:);
        %f_cost(Flowijr(i,j))=RebWeight.*TravelTimes(i,j);
    %end
    if progressflag & ~mod(i,50)
        fprintf('%d ',i);
    end
end
f_cost_numvehicles=f_cost;
%Number of vehicles=f_cost_numvehicles*cplex_out'
if (progressflag)
    fprintf('\n',i);
end

if ~relaxCcostflag
    f_cost(N*N*(M+1)+1 : N*N*(M+1)+N*N)=CongestionCost;
else
    for i=1:N
        f_cost(CRelaxij(i,1):CRelaxij(i,N))=TravelTimes(i,:);
    end
end

f_cost(N*N*(M+1)+N*N+1 : N*N*(M+1)+N*N+2*M)=SourceCost;

%% Constraints setup
if (debugflag)
    disp('Initializing constraints')
end

% N*M equality constraints, one per node and per flow. Each constraint has
% fewer than 2(N-1) entries (every node is connected to at most N-1
% out-nodes and N-1 in-nodes). In addition, there are 2*S*M entries to
% relax sources and sinks
% 
% N*N inequality constraints, one per edge. Each inequality constraint has
% M+1 entries (one per flow and one for the relaxation).

n_eq_constr = N*(M+1);
n_eq_entries = n_eq_constr*2*(N-1) + 4*M; %Upper bound

Aeqsparse=zeros(n_eq_entries,3);
Beq=zeros(n_eq_constr,1);
Aeqrow=1;
Aeqentry=1;

n_ineq_constr = N*N;
n_ineq_entries = n_ineq_constr*(M+2);


Aineqsparse=zeros(n_ineq_entries,3);
Bineq=zeros(n_ineq_constr,1);
Aineqrow=1;
Aineqentry=1;

%% Equality constraints: for each flow, flow conservation
if (debugflag)
    disp('Building equality constraints in sparse form.')

end

availmatrices = strcat(MATRIX_DIRECTORY, 'available_As.mat');
load(availmatrices);
sort(matrices);

if cachedAeqflag %load A and build B
    if debugflag
        disp('Loading precompiled equality Aeqsparse')
    end
    filename = strcat(MATRIX_DIRECTORY, 'CachedA',num2str(M),'.mat');
    load(filename);
    
    Aeqrow = 1;
    if debugflag
        fprintf('Building Beq');
    end
    
    if progressflag
        fprintf('\nm (out of %d):', M);
    end
    
    for m = 1:M
        if progressflag && ~mod(m, 25)
            fprintf('%d ', m)
        end
        
        for i = 1:N
            Beq(Aeqrow) = 0;
            
            if (sum(Sources(m,:) == i) > 0)
                if Sources(m) == i
                    Beq(Aeqrow) = Beq(Aeqrow) + FlowsIn(m);
                end
            end
            if (sum(Sinks(m,:) == i) > 0)
                if Sinks(m) == i
                    Beq(Aeqrow) = Beq(Aeqrow) - FlowsIn(m);
                end
            end
            
            Aeqrow = Aeqrow + 1;
        end
    end
    if debugflag
        fprintf('\nBeq for rebalancing');
    end
    
    if progressflag
        fprintf('\nn (out of %d):', N);
    end
    %Rebalancing
    
    for i = 1:N
        if progressflag && ~mod(i,10)
            fprintf('%d ', i);
        end
        
        Beq(Aeqrow) = 0;
        
        for m = 1:M
            if Sinks(m) == i
                Beq(Aeqrow) = Beq(Aeqrow) + FlowsIn(m);
            end
            if Sources(m) == i
                Beq(Aeqrow) = Beq(Aeqrow) - FlowsIn(m);
            end
        end
        
        Aeqrow = Aeqrow + 1;
    end
    
    if progressflag
        fprintf('\n')
    end
                    
else % build it all from scratch
    if progressflag
        fprintf('m: (out of %d) ',M)
    end
    for m=1:M
        if progressflag && ~mod(m,25)
            fprintf('%d ',m)
            if ~mod(m,300)
                fprintf('\n')
            end
        end
        for i=1:N
            for j=1:N
                %if j~=i
                if sum(RoadGraph{i}==j)
                    Aeqsparse(Aeqentry,:)=[Aeqrow,Flowkij(m,i,j), 1];
                    Aeqentry=Aeqentry+1;
                end
                if sum(RoadGraph{j}==i)
                    Aeqsparse(Aeqentry,:)=[Aeqrow,Flowkij(m,j,i),-1];
                    Aeqentry=Aeqentry+1;
                end
                %end
            end
            
            Beq(Aeqrow) = 0;
            
            if (sum(Sources(m,:) == i)>0)
                if Sources(m) == i
                    Beq(Aeqrow) = Beq(Aeqrow)+FlowsIn(m);
                    
                    Aeqsparse(Aeqentry,:)=[Aeqrow,SoRelaxk(m),sourcerelaxflag];
                    Aeqentry=Aeqentry+1;
                end
                
            end
            if (sum(Sinks(m,:) == i)>0)
                if Sinks(m) == i
                    Beq(Aeqrow) = Beq(Aeqrow)-FlowsIn(m);
                    
                    Aeqsparse(Aeqentry,:)=[Aeqrow,SiRelaxk(m),-sourcerelaxflag];
                    Aeqentry=Aeqentry+1;
                end
            end
            
            Aeqrow=Aeqrow+1;
        end
    end
    if (progressflag)
        fprintf('\nn (out of %d):',N)
    end
    % Rebalancing
    for i=1:N
        if progressflag && ~mod(i,10)
            fprintf('%d ',i);
        end
        for j=1:N
            if sum(RoadGraph{i}==j)
                Aeqsparse(Aeqentry,:)=[Aeqrow,Flowijr(i,j), 1];
                Aeqentry=Aeqentry+1;
            end
            if sum(RoadGraph{j}==i)
                Aeqsparse(Aeqentry,:)=[Aeqrow,Flowijr(j,i),-1];
                Aeqentry=Aeqentry+1;
            end
        end
        
        Beq(Aeqrow) = 0;
        
        for m=1:M
            if Sinks(m) == i
                Beq(Aeqrow) = Beq(Aeqrow)+FlowsIn(m);
                
                Aeqsparse(Aeqentry,:)=[Aeqrow,SiRelaxk(m),sourcerelaxflag];
                Aeqentry=Aeqentry+1;
            end
            if Sources(m) == i
                Beq(Aeqrow) = Beq(Aeqrow)-FlowsIn(m);
                
                Aeqsparse(Aeqentry,:)=[Aeqrow,SoRelaxk(m),-sourcerelaxflag];
                Aeqentry=Aeqentry+1;
            end
        end
        
        Aeqrow=Aeqrow+1;
    end
    if (progressflag)
        fprintf('\n')
    end
    
    availmatrices = strcat(MATRIX_DIRECTORY, 'available_As.mat');
    load(availmatrices);
    matrices = [matrices M];
    save(availmatrices, 'matrices');
    cachedFile = strcat(MATRIX_DIRECTORY, 'CachedABeq', num2str(M));
    save(cachedFile,'Aeqsparse','Aeqrow','Aeqentry','-v7.3')
    
end

%% Inequality constraint: capacity

if cachedAeqflag
    % Aineq should already be imported from above
    % Need to build b
    fprintf('Loading Aineq from File.');
    Aineqrow=1;
    Aineqentry= M*N^2 + 2*N^2 + 1;
    for i = 1:N
        for j = 1:N
            Bineq(Aineqrow) = RoadCap(i,j);
            Aineqrow = Aineqrow + 1;
        end
    end
else %Build it all from scratch
    if (debugflag)
        disp('Building inequality constraints in sparse form')
    end
    if progressflag
        fprintf('n (out of %d): ',N)
    end
    for i=1:N
        if progressflag && ~mod(i,50)
            fprintf('%d ',i)
        end
        for j=1:N
            for m=1:M
                Aineqsparse(Aineqentry,:)=[Aineqrow,Flowkij(m,i,j),1];
                Aineqentry=Aineqentry+1;
            end
            Aineqsparse(Aineqentry,:)=[Aineqrow,Flowijr(i,j),1];
            Aineqentry=Aineqentry+1;
            Aineqsparse(Aineqentry,:)=[Aineqrow,CRelaxij(i,j),-congrelaxflag];
            Aineqentry=Aineqentry+1;
            Bineq(Aineqrow)=RoadCap(i,j);
            Aineqrow=Aineqrow+1;
        end
    end
end

%% Saving inequality matrices
% filename = strcat('/media/Big_Data/yhindy/matrices/CachedABeq',num2str(M),'.mat');
% load(filename);
% 
% newfilename = strcat('/media/Big_Data/yhindy/matrices/CachedA',num2str(M),'.mat');
% save(newfilename, 'Aeqsparse', 'Aineqsparse', 'Aeqentry', 'Aineqentry', '-v7.3');
% load('/media/Big_Data/yhindy/matrices/available_As.mat');
% matrices = [matrices M];
% save('/media/Big_Data/yhindy/matrices/available_As.mat', 'matrices');
if (progressflag)
    fprintf('\n')
end

%% Assembling matrices

if Aeqrow-1~=n_eq_constr
    fprintf('ERROR: unexpected number of equality constraints (expected: %d, actual: %d)\n',n_eq_constr,Aeqrow-1)
end
if Aineqrow-1~=n_ineq_constr
    disp('ERROR: unexpected number of inequality constraints')
end

if Aineqentry-1~=n_ineq_entries
    fprintf('ERROR: unexpected number of inequality entries (expected: %d, actual: %d)\n',n_ineq_entries,Aineqentry-1)
end

if (debugflag)
    disp('Building matrices from sparse representation')
end

Aeqsparse=Aeqsparse(1:Aeqentry-1,:);

Aeq=sparse(Aeqsparse(:,1),Aeqsparse(:,2),Aeqsparse(:,3), Aeqrow-1, StateSize);

Aineq=sparse(Aineqsparse(:,1),Aineqsparse(:,2),Aineqsparse(:,3), Aineqrow-1, StateSize);

%% Upper and lower bounds
% Upper and lower bounds
if (debugflag)
    disp('Building upper and lower bounds')
end

lb=zeros(StateSize,1); %Nothing can be negative

ub=Inf*ones(StateSize,1); %Why not? We enforce capacity separately
if progressflag
    fprintf('n (out of %d): ',N)
end

for i=1:N
    if progressflag && ~mod(i,50)
        fprintf('%d ',i)
    end
    for j=1:N
        if sum(RoadGraph{i}==j)==0 %If i and j are not neighbors
            for m=1:M
                ub(Flowkij(m,i,j)) = 0;
            end
        end
    end
end

if progressflag
    fprintf('\n')
end

% Clarification: if two nodes are neighbors, we enforce a soft capacity
% constraint. If two nodes are not neighbors, we enforce a hard no-flow
% constraint through the upper bound above.

for k=1:M
    ub(SoRelaxk(k))=FlowsIn(k);
    ub(SiRelaxk(k))=FlowsIn(k);
end

disp('Done building UBs and LBs')

%% Variable type

if (debugflag)
    disp('Building constraint type')
end

% Building block
ConstrType=char(zeros(1,StateSize));

% Continuous-probability version
if (~milpflag)
    ConstrType(1:end)='C';
else
    % Actual MILP formulation
    ConstrType(1:end)='I';
end

%% Call optimizer

if (debugflag)
    disp('Calling optimizer')
end

sostype=[];
sosind=[];
soswt=[];


if (milpflag)
    MyOptions=cplexoptimset('cplex');
    %MyOptions.parallel=1;
    %MyOptions.threads=8;
    MyOptions.mip.tolerances.mipgap=0.01;
    %MyOptions.Display = 'iter';
    tic
    [cplex_out,fval,exitflag,output]=cplexmilp(f_cost,Aineq,Bineq,Aeq,Beq,sostype,sosind,soswt,lb,ub,ConstrType,[],MyOptions);
    toc
else
    tic
    [cplex_out,fval,exitflag,output]=cplexlp(f_cost,Aineq,Bineq,Aeq,Beq,lb,ub) ;
    toc
end
if (debugflag)
    fprintf('Solved! fval: %f\n', fval)
    disp(output)
    
f_cost_numvehicles = f_cost_numvehicles';
numvehicles = f_cost_numvehicles*cplex_out;
end