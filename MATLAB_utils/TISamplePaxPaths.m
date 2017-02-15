function [ipaths] = TISamplePaxPaths(allpaths)

% Samples passenger paths from allpaths structure, where allpaths{k}{j} is
% the j-th path of passenger k, an m by 2 matrix where the first column
% represents nodes and the last element of the second column is the actual
% flow.

M=size(allpaths,1);

ipaths=cell(M,1);

for passnumber=1:M
    paths=allpaths{passnumber};
    pathsno=length(paths);
    
    
    weights=zeros(pathsno,1);
    for l=1:pathsno
        weights(l)=paths{l}(end,2);
    end
    
    sweights=sum(weights); %We wish to sample exactly sweights routes
    
    samplesno=round(sweights); %Round, just in case
    
    if sweights~=samplesno
        fprintf('WARNING!! Weights of passenger %d do not add up to an integer (sum: %f)\n',passnumber,sweights);
        ipaths{passnumber}=[];
        if sweights<=0 %% If there is no path at all, skip. Otherwise, sample what you can
            fprintf('ERROR!! Weights of passenger %d add up to zero(sum: %f), skipping\n',passnumber,sweights);
            continue
        end
        
    end    %paths should sum to one
    
    ipathspax=cell(samplesno,1);
    for samplecounter=1:samplesno
        pp=rand()*sweights; %We scale to the actual amount of weight we have
        cweights=cumsum(weights);
        sampledpath=1;
        while cweights(sampledpath)<pp
            sampledpath=sampledpath+1;
        end
        ipathspax{samplecounter}=paths{sampledpath}(:,1);
    end
    ipaths{passnumber}=ipathspax;
end