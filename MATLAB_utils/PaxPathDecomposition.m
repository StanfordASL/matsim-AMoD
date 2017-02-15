% Assume you have Xtrij_out, Xtijv_out, Vti_out,Utr_out,Etr_out, T

function [allpaths]=PaxPathDecomposition(Xtrij_out,Xtijv_out, Vti_out,Utr_out,Etr_out, T,R,Sr,Dr)

passindex=1:1:R;

plotpathsflag=1;

pathcounter=zeros(R,1);
allpaths=cell(R,1);

% Data structure: paths{r} contains the path decomposition for passenger r.
% paths{r}{i} is the i-th path. It is a Tx3 matrix. The first column is
% time. The second colun is node. The third column is flow.

%Back up data
Xtrij_out_c=Xtrij_out;
Xtijv_out_c=Xtijv_out;
Vti_out_c=Vti_out;
Etr_out_c= Etr_out;
Utr_out_c=Utr_out;


%%
mypathflow=1;

for passnumber=1:R
    if size(allpaths{passnumber})==0
        allpaths{passnumber}={};
    end
    paths={};
    while sum(Utr_out(:,passnumber) & mypathflow)
        pathcounter(passnumber)=pathcounter(passnumber)+1;
        
        %sum(Utr_out(:,passnumber))
        paths{pathcounter(passnumber)}=[];
        [myflow,mytime]=max(Utr_out(:,passnumber));
        currentpos = Sr(passnumber);
        nextpos = currentpos;
        
        paths{pathcounter(passnumber)} = [mytime, nextpos, myflow];
        
        while (mytime<=T & myflow>0 & nextpos ~= Dr(passnumber))
            
            
            [maxrflow,maxrflowidx]=max(Xtrij_out(mytime,passnumber,currentpos,:));
            myflow=min(myflow,maxrflow);
            if maxrflowidx == Dr(passnumber) && Etr_out(mytime,passnumber) %Next stop is the last
                myflow=min(myflow,Etr_out(mytime,passnumber));
                onestoptrip=1; %Next time we drop the guy off
            end
            nextpos=maxrflowidx;
            mytime = mytime+1;
            paths{pathcounter(passnumber)}=[paths{pathcounter(passnumber)};[mytime nextpos myflow]];
            currentpos = nextpos;
        end
        
         
        %paths{pathcounter(passnumber)}(1,1)
        %paths{pathcounter(passnumber)}(end,1)
        %mypathflow
        
        for t=paths{pathcounter(passnumber)}(1,1):paths{pathcounter(passnumber)}(end,1)-1
            startnode = paths{pathcounter(passnumber)}(paths{pathcounter(passnumber)}(:,1)==t,2);
            endnode = paths{pathcounter(passnumber)}(paths{pathcounter(passnumber)}(:,1)==t+1,2);
            Xtrij_out(t,passnumber,startnode,endnode) = Xtrij_out(t,passnumber,startnode,endnode) - mypathflow;
        end

        Utr_out(paths{pathcounter(passnumber)}(1,1),passnumber) = Utr_out(paths{pathcounter(passnumber)}(1,1),passnumber) - mypathflow;
        Etr_out(paths{pathcounter(passnumber)}(end,1)-1,passnumber) =     Etr_out(paths{pathcounter(passnumber)}(end,1)-1,passnumber) - mypathflow;

        
%             % If the next step has a different pax on board, reduce Etr.
%             if paths{pathcounter}(t,4)~=paths{pathcounter}(t+1,4)
%                 Etr_out(t,paths{pathcounter}(t,4)) = Etr_out(t,paths{pathcounter}(t,4)) - mypathflow;
%             end
%             %If the previous step had no pax on board, reduce Utr
%             if paths{pathcounter}(t-1,4)~=paths{pathcounter}(t,4)
%                 Utr_out(t,paths{pathcounter}(t,4)) = Utr_out(t,paths{pathcounter}(t,4)) - mypathflow;
%             end
           
        
        
        
    end
    
    allpaths{passnumber} = paths;
    
end



% while sum(Vti_out(1,:) & mypathflow) %Stop when either we're done or we get a zero flow. The latter should never happen
%     
%     pathcounter=pathcounter+1
%     sum(Vti_out(1,:))
%     
%     paths{pathcounter}=[];
%     
%     [myflow,tempidx]=max(Vti_out(1,:));
%     
%     passnumber=0;
%     onestoptrip=0;
%     
%     currentpos=tempidx;
%     nextpos=-1;
%      
%     paths{pathcounter}=[1 currentpos myflow 0];
%     t=2;
%     
%     while t<=T
%         
%         %tempstate=-1; %1 if we remove flow from V, 2 if we remove flow from Xv, 3 if we remove flow from Xr
%         
%         if (~passnumber | onestoptrip) %If I'm not carrying a passenger at this time or it's time to drop her off
%             onestoptrip=0; 
%             passnumber=0;
%             
%             %Look for the max flow among Vti(tempt,currentpos),
%             %Xtijv_out(tempt,currentpos,:) and Xtrij_out(tempt, those routes with Utr_out>0 and start at currentpos,currentpos,:)
%             
%             %For each pax route that starts at time t from currentpos, 
%             startingrroutes = passindex(Utr_out(t,:)>0 & (Sr == currentpos)');
%             %we compute the out-flow and the out-node
%             outroutes=zeros(length(startingrroutes),3);
%             for i=1:length(startingrroutes)
%                 [tempmaxrflow,maxrflowidx]=max(Xtrij_out(t,startingrroutes(i),currentpos,:));
%                 outroutes(i,:)=[startingrroutes(i) tempmaxrflow maxrflowidx];
%             end
%             if (size(outroutes,1))
%                 [maxrflow,bestpaxidx]=max(outroutes(:,2));
%             else
%                 maxrflow=0;
%                 bestpaxidx=0;
%             end
%             
%             
%             % Max rebalancing flow
%             [maxvflow,maxvflowidx]=max(Xtijv_out(t,currentpos,:));
%             
%             % Max non-passenger flow (either stay or rebalance)
%             maxnrflow=max(maxvflow,Vti_out(t,currentpos));
%             
%             % Now decide which is the best flow overall
%             if (maxnrflow>=maxrflow || size(outroutes,1)==0) %We are not picking up anyone
%                 if (Vti_out(t,currentpos)>=maxvflow)    %Stay in place
%                     nextpos=currentpos;
%                     myflow=min(myflow,Vti_out(t,currentpos));
%                 else    %Rebalance
%                     nextpos=maxvflowidx;
%                     myflow=min(myflow,maxvflow);
%                 end
%             else %Let's take a passenger on board!
%                 passnumber=outroutes(bestpaxidx,1);
%                 nextpos=outroutes(bestpaxidx,3);
%                 myflow=min(myflow,outroutes(bestpaxidx,2));
%                 
%                 if (nextpos == Dr(passnumber) && Etr_out(t,passnumber)) %if we arrive at the next time step
%                     onestoptrip=1;
%                     myflow=min(myflow,Etr_out(t,passnumber));
%                 end
%             end
%             
%         
%         else %If we're carrying a pax, we only look for that route
%             
%             % Look for max among Xtrij_out(tempt, passnumber,currentpos,:)
%             %If the result is the passenger's destination and
%             %Etr_out(tempt+1,passnumber) , then reset passnumber=0
%             
%             % Remember: Etr represents the arrivals at the NEXT time step
%             
%             
%             
%             [maxrflow,maxrflowidx]=max(Xtrij_out(t,passnumber,currentpos,:));
%             myflow=min(myflow,maxrflow);
%             if maxrflowidx == Dr(passnumber) && Etr_out(t,passnumber) %Next stop is the last
%                 myflow=min(myflow,Etr_out(t,passnumber));
%                 onestoptrip=1; %Next time we drop the guy off
%             end
%             nextpos=maxrflowidx;
%             
%         end
%         
%         paths{pathcounter}=[paths{pathcounter};[t nextpos myflow passnumber]];
%         currentpos = nextpos;
%         t=t+1;
%     end
%     
%     %Remove flows from Xij, V, etc
% 
%     
%     % TODO this does not work. Reduce start and end flows too!
%     mypathflow=paths{pathcounter}(end,3);
%     Vti_out(1,paths{pathcounter}(1,2)) = Vti_out(1,paths{pathcounter}(1,2))-mypathflow;
%     
%     for t=2:T
%         if paths{pathcounter}(t,4)>0 %Pax on board
%             Xtrij_out(t,paths{pathcounter}(t,4),paths{pathcounter}(t-1,2),paths{pathcounter}(t,2)) = Xtrij_out(t,paths{pathcounter}(t,4),paths{pathcounter}(t-1,2),paths{pathcounter}(t,2)) - mypathflow;
%             % If the next step has a different pax on board, reduce Etr.
%             if paths{pathcounter}(t,4)~=paths{pathcounter}(t+1,4)
%                 Etr_out(t,paths{pathcounter}(t,4)) = Etr_out(t,paths{pathcounter}(t,4)) - mypathflow;
%             end
%             %If the previous step had no pax on board, reduce Utr
%             if paths{pathcounter}(t-1,4)~=paths{pathcounter}(t,4)
%                 Utr_out(t,paths{pathcounter}(t,4)) = Utr_out(t,paths{pathcounter}(t,4)) - mypathflow;
%             end
%             
% 
%         elseif paths{pathcounter}(t,2)==paths{pathcounter}(t-1,2) %We were stationary
%             Vti_out(t,paths{pathcounter}(t,2))=Vti_out(t,paths{pathcounter}(t,2)) - mypathflow;
%         else %We were rebalancing
%             Xtijv_out(t,paths{pathcounter}(t-1,2),paths{pathcounter}(t,2)) = Xtijv_out(t,paths{pathcounter}(t-1,2),paths{pathcounter}(t,2))-mypathflow;
%         end
%     end
%     
% 
% end
% 
% %Restoring stuff
% sum(sum(Vti_out))
% sum(sum(sum(sum(Xtrij_out))))
% sum(sum(sum(Xtijv_out)))
% 
% 
% %%
% Xtrij_out=Xtrij_out_c;
% Xtijv_out=Xtijv_out_c;
% Vti_out=Vti_out_c;
% Etr_out=Etr_out_c;
% Utr_out=Utr_out_c;
% 
% %%
% if plotpathsflag
%     figure()
%     pathsplotX=ceil(sqrt(16/9*pathcounter));    %We plot all requests in the same plot
%     pathsplotY=ceil(pathsplotX*9/16);
%     while (pathsplotX*(pathsplotY-1))>=pathcounter      %If we can get away with one less row
%         pathsplotY=pathsplotY-1;
%     end
%     while (pathsplotY-1)<=(pathsplotX-mod(pathcounter,pathsplotX)-1) && mod(pathcounter,pathsplotX)~=0 %If we can remove the last column and stuff it in the last row
%         pathsplotX=pathsplotX-1;
%     end
%     
%     for p=1:pathcounter
%         subplot(pathsplotY,pathsplotX,p)
%         plotRoadGraph(RoadGraph,NodesLocation,5);
%         hold all
% 
%         for t=2:T
%             if (paths{p}(t-1,2)~=paths{p}(t,2))   %If we do move at this timestep
%                 if (paths{p}(t,4)) %Carrying a pax
%                     %plot([FindX(paths{p}(t-1,2)) FindX(paths{p}(t,2))],[FindY(paths{p}(t-1,2)) FindY(paths{p}(t,2))],'r','LineWidth',3)
%                     quiver(FindX(paths{p}(t-1,2)), FindY(paths{p}(t-1,2)),FindX(paths{p}(t,2))- FindX(paths{p}(t-1,2)) , FindY(paths{p}(t,2))-FindY(paths{p}(t-1,2)),0,'m','LineWidth',3)
%                 else
%                     plot([FindX(paths{p}(t-1,2)) FindX(paths{p}(t,2))],[FindY(paths{p}(t-1,2)) FindY(paths{p}(t,2))],'b','LineWidth',3)
%                 end
%             end
%         end
%         plot(FindX(paths{p}(1,2)),FindY(paths{p}(1,2)),'.g','MarkerSize',10)
%         plot(FindX(paths{p}(end,2)),FindY(paths{p}(end,2)),'.r','MarkerSize',10)
%         
%     end
% end

%%
Xtrij_out=Xtrij_out_c;
Xtijv_out=Xtijv_out_c;
Vti_out=Vti_out_c;
Etr_out=Etr_out_c;
Utr_out=Utr_out_c;