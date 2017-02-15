function [Reb_solution]=TISampleRebSol(FracRebSols,FracRebWeights)

pp=rand()*sum(FracRebWeights); %We scale to the actual amount of weight we have

cweights=cumsum(FracRebWeights);
sampledpath=1;
while cweights(sampledpath)<pp
    sampledpath=sampledpath+1;
end

Reb_solution=FracRebSols{sampledpath}/FracRebWeights(sampledpath);