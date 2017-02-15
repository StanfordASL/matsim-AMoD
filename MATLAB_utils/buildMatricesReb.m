function [Aeqsparse, Aeqentry, Aineqsparse, Aineqentry] = buildMatricesReb(RoadGraph, RoadCap, TravelTimes, Sources, Sinks, FlowsIn, FlowsOut, milpflag, congrelaxflag, sourcerelaxflag)



CongestionCost=1e6; %The cost of violating the congestion constraint
SourceCost=1e8;     % The cost of dropping a source or sink altogether

debugflag = 1;

cachedAeqflag = 0;

if debugflag
    progressflag=1;
else
    progressflag=0;
end
relaxCcostflag=1;

N=length(RoadGraph);
M=size(Sources,1);
S=size(Sources,2); % Assumption: each flow has the same number of sources 
                   %  and sinks 
                   %  i.e., for each flow k, |sources(k)|=|sinks(k)| 

StateSize=N*N*M + N*N + 2*M*S;

%% Utility functions
Flowkij=@(k,i,j) (k-1)*N*N + (i-1)*N + j;
CRelaxij=@(i,j) N*N*M + (i-1) * N + j;
SoRelaxkl=@(k,l) N*N*M + N*N + S*(k-1) + l;
SiRelaxkl=@(k,l) N*N*M + N*N + M*S + S*(k-1) + l;
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
n_eq_entries = n_eq_constr*2*(N-1) + 2*S*M; %Upper bound

Aeqsparse=zeros(n_eq_entries,3);
Beq=zeros(n_eq_constr,1);
Aeqrow=1;
Aeqentry=1;

n_ineq_constr = N*N;
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
            
            if (sum(Sources(m,:) == i)>0)
                tempsources=Sources(m,:);
                for ll=1:length(tempsources)
                    if Sources(m,ll) == i
                        Aeqsparse(Aeqentry,:)=[Aeqrow,SoRelaxkl(m,ll),sourcerelaxflag];
                        Aeqentry=Aeqentry+1;
                    end
                end
            end
            if (sum(Sinks(m,:) == i)>0)
                tempsinks=Sinks(m,:);
                for ll=1:length(tempsinks)
                    if Sinks(m,ll) == i
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
    for j=1:N
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
