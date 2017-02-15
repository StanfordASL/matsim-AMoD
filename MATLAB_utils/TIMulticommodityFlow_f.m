function [cplex_out,numvehicles] = TIMulticommodityFlow_f(RoadGraph,RoadCap,TravelTimes,Sources,Sinks,FlowsIn,FlowsOut,milpflag,congrelaxflag,sourcerelaxflag,debugflag)

% Forked from TIMulticommodityFlow on Oct. 4
% Computes a multicommodity flow on the capacitated network described by
% RoadGraph between given sources s and sinks t with a given flow betweek
% each s-t pair.

% Inputs:
% - RoadGraph, a nx1 cell structure. RoadGraph{i} contains the neighbors of
%   i
% - RoadCap, a nxn matrix. RoadCap(i,j) is the capacity of the i-j link.
% - TravelTimes, a nxn matrix. TravelTimes(i,j) is the travel time on the i-j link.
% - Sources, a m-by-k vector. Sources(i) is the source node (or collection 
%   of source nodes) of the i-th flow
% - Sinks, same as sources
% - FlowsIn, a m-by-k vector. FlowsIn(i,k) is the amount of flow entering 
%   source k of flow i.
% - FlowsOut, a m-by-k vector. FlowsOut(i,k) is the amount of flow exiting
%   sink k of flow i.
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

if nargin<=10
   debugflag=1; %Makes output verbose
end
if nargin<=9
    sourcerelaxflag=0; 
end
if nargin<=8
congrelaxflag=0; 
end
if nargin<=7
    milpflag=1;
end




CongestionCost=1e6; %The cost of violating the congestion constraint
SourceCost=1e8;     % The cost of dropping a source or sink altogether

if debugflag
    progressflag=1;
else
    progressflag=0;
end
cachedAeqflag=0;
relaxCcostflag=1;

for i=1:length(RoadGraph)
    RoadGraph{i}=sort(unique(RoadGraph{i}));
end

%Nodes in ReverseGraph{i} are such that RoadGraph{ReverseGraph{i}} contains
%i
ReverseGraph=cell(size(RoadGraph));
for i=1:length(RoadGraph)
    for j=RoadGraph{i}
        ReverseGraph{j}=[ReverseGraph{j} i];
    end
end

for i=1:length(ReverseGraph)
    ReverseGraph{i}=sort(unique(ReverseGraph{i}));
end

N=length(RoadGraph);
M=size(Sources,1);
S=size(Sources,2); % Assumption: each flow has the same number of sources 
                   %  and sinks 
                   %  i.e., for each flow k, |sources(k)|=|sinks(k)| 

E=0;
NumEdges=zeros(N,1);
for i=1:N
    NumEdges(i)=length(RoadGraph{i});
    E=E+length(RoadGraph{i});
end
                   
StateSize=E*M + E + 2*M*S;

%% Utility functions

cumNeighbors=cumsum(NumEdges);
cumNeighbors=[0;cumNeighbors(1:end-1)];

%Build a matrix for neighborhood. This will be useful in a moment
neighborCounterS=sparse([],[],[],N,N,E);

TempNeighVec=zeros(N,1);
for i=1:N
    for j=RoadGraph{i}
        TempNeighVec(j)=1;
    end
    NeighCounterLine=cumsum(TempNeighVec);
    for j=RoadGraph{i}
        neighborCounterS(i,j)=NeighCounterLine(j);
    end
    TempNeighVec=zeros(N,1);
end

neighborCounter=neighborCounterS;

Flowkij=@(k,i,j) (k-1)*E + cumNeighbors(i) + neighborCounter(i,j);
CRelaxij=@(i,j) E*M + cumNeighbors(i) + neighborCounter(i,j);
SoRelaxkl=@(k,l) E*M + E + S*(k-1) + l;
SiRelaxkl=@(k,l) E*M + E + M*S + S*(k-1) + l;

%% Cost function
if (debugflag)
    disp('Building cost function')
end
f_cost=zeros(StateSize,1);
if (progressflag)
    fprintf('n (out of %d):',N)
end
for i=1:N
    if ~isempty(RoadGraph{i})
        for k=1:M
            f_cost(Flowkij(k,i,RoadGraph{i}(1)):Flowkij(k,i,RoadGraph{i}(end)))=TravelTimes(i,RoadGraph{i});
            %f_cost(Flowkij(k,i,j))=TravelTimes(i,j);
        end
    end
    if progressflag & ~mod(i,10)
        fprintf('%d ',i);
    end
end
if (progressflag)
    fprintf('\n');
end

f_cost_numvehicles = f_cost;

if ~relaxCcostflag
    f_cost(E*M+1 : E*M+E)=CongestionCost;
else
    for i=1:N
        if ~isempty(RoadGraph{i})
            f_cost(CRelaxij(i,RoadGraph{i}(1)):CRelaxij(i,RoadGraph{i}(end)))=TravelTimes(i,RoadGraph{i});
        end
    end
end

f_cost(SoRelaxkl(1,1) : SiRelaxkl(M,S))=SourceCost;

%% Constraints setup
if (debugflag)
    disp('Initializing constraints')
end

% N*M equality constraints, one per node and per flow. Each constraint has
% fewer than 2(N-1) entries (every node is connected to at most N-1
% out-nodes and N-1 in-nodes). In addition, there are 2*S*M entries to
% relax sources and sinks

% N*N inequality constraints, one per edge. Each inequality constraint has
% M+1 entries (one per flow and one for the relaxation).

n_eq_constr = N*M;
%n_eq_entries = n_eq_constr*2*(N-1) + 2*S*M; %Upper bound
n_eq_entries = 2*E*(M)+ 2*S*M;

Aeqsparse=zeros(n_eq_entries,3);
Beq=zeros(n_eq_constr,1);
Aeqrow=1;
Aeqentry=1;

n_ineq_constr = E;
n_ineq_entries = n_ineq_constr*(M+1);


Aineqsparse=zeros(n_ineq_entries,3);
Bineq=zeros(n_ineq_constr,1);
Aineqrow=1;
Aineqentry=1;

%% Equality constraints: for each flow, flow conservation
if (debugflag)
    disp('Building equality constraints in sparse form')
end


if cachedAeqflag
    if debugflag
        disp('Loading precompiled equality Aeqsparse and Beq')
    end
    load('PaxCachedABeq')
else
    
    if progressflag
        fprintf('m: (out of %d) ',M)
    end
    for m=1:M
        if progressflag & ~mod(m,10)
            fprintf('%d ',m)
            if ~mod(m,300)
                fprintf('\n')
            end
        end
        for i=1:N
%             for j=1:N
%                 %if j~=i
%                 if sum(RoadGraph{i}==j)
%                     Aeqsparse(Aeqentry,:)=[Aeqrow,Flowkij(m,i,j), 1];
%                     Aeqentry=Aeqentry+1;
%                 end
%                 if sum(RoadGraph{j}==i)
%                     Aeqsparse(Aeqentry,:)=[Aeqrow,Flowkij(m,j,i),-1];
%                     Aeqentry=Aeqentry+1;
%                 end
%                 %end
%             end
            
            if ~isempty(RoadGraph{i})
                for j=RoadGraph{i} %Out-flows
                    Aeqsparse(Aeqentry,:)=[Aeqrow,Flowkij(m,i,j), 1];
                    Aeqentry=Aeqentry+1;
                end
            end
            if ~isempty(ReverseGraph{i})
                for j=ReverseGraph{i} %In-flows
                    Aeqsparse(Aeqentry,:)=[Aeqrow,Flowkij(m,j,i),-1];
                    Aeqentry=Aeqentry+1;
                end
            end
            
            
            Beq(Aeqrow) = 0;
            
            if (sum(Sources(m,:) == i)>0)
                tempsources=Sources(m,:);
                for ll=1:length(tempsources)
                    if Sources(m,ll) == i
                        Beq(Aeqrow) = Beq(Aeqrow)+FlowsIn(m,ll);
                        
                        Aeqsparse(Aeqentry,:)=[Aeqrow,SoRelaxkl(m,ll),sourcerelaxflag];
                        Aeqentry=Aeqentry+1;
                    end
                end
            end
            if (sum(Sinks(m,:) == i)>0)
                tempsinks=Sinks(m,:);
                for ll=1:length(tempsinks)
                    if Sinks(m,ll) == i
                        Beq(Aeqrow) = Beq(Aeqrow)-FlowsOut(m,ll);
                        
                        Aeqsparse(Aeqentry,:)=[Aeqrow,SiRelaxkl(m,ll),-sourcerelaxflag];
                        Aeqentry=Aeqentry+1;
                    end
                end
            end
            
            Aeqrow=Aeqrow+1;
        end
    end
    if (progressflag)
        fprintf('\n')
    end
    
    save('PaxCachedABeq','Aeqsparse','Beq','Aeqrow','Aeqentry','-v7.3')
    
end

%% Inequality constraint: capacity
if (debugflag)
    disp('Building inequality constraints in sparse form')
end

if progressflag
    fprintf('n (out of %d): ',N)
end

for i=1:N
    if progressflag & ~mod(i,10)
        fprintf('%d ',i)
    end
    for j=RoadGraph{i}
        for m=1:M
            Aineqsparse(Aineqentry,:)=[Aineqrow,Flowkij(m,i,j),1];
            Aineqentry=Aineqentry+1;
        end
        Aineqsparse(Aineqentry,:)=[Aineqrow,CRelaxij(i,j),-congrelaxflag];
        Aineqentry=Aineqentry+1;
        Bineq(Aineqrow)=RoadCap(i,j);
        Aineqrow=Aineqrow+1;
    end
end

if (progressflag)
    fprintf('\n')
end

%% Assembling matrices

if Aeqrow-1~=n_eq_constr
    disp('ERROR: unexpected number of equality constraints')
end
if Aineqrow-1~=n_ineq_constr
    disp('ERROR: unexpected number of inequality constraints')
end

if Aineqentry-1~=n_ineq_entries
    disp('ERROR: unexpected number of inequality entries')
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

% for i=1:N
%     for j=1:N
%         if sum(RoadGraph{i}==j)==0 %If i and j are not neighbors
%             for m=1:M
%                 ub(Flowkij(m,i,j)) = 0;
%             end
%         end
%     end
% end

% Clarification: if two nodes are neighbors, we enforce a soft capacity
% constraint. If two nodes are not neighbors, we enforce a hard no-flow
% constraint through the upper bound above.

for k=1:M
    ub(SoRelaxkl(k,1):SoRelaxkl(k,S))=FlowsIn(k,:)';
    ub(SiRelaxkl(k,1):SiRelaxkl(k,S))=FlowsOut(k,:)';
end
if (debugflag)
    disp('Done building UBs and LBs')
end
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
    if (debugflag)
        tic
    end
    [cplex_out,fval,exitflag,output]=cplexmilp(f_cost,Aineq,Bineq,Aeq,Beq,sostype,sosind,soswt,lb,ub,ConstrType,[],MyOptions);
    
    if (debugflag)
        toc
    end
else
    if (debugflag)
        tic
    end
    [cplex_out,fval,exitflag,output]=cplexlp(f_cost,Aineq,Bineq,Aeq,Beq,lb,ub) ;
    if (debugflag)
        toc
    end
end
if (debugflag)
    fprintf('Solved! fval: %f\n',fval)
    disp(output)
end

numvehicles = f_cost_numvehicles' * cplex_out;