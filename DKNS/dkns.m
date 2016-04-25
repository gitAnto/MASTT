%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Multi Agent Simulator for Target Tracking (MASTT)
%
%  dkns.m
%
%  Run all the phases of the distributed estimation algorithm for the
%  robotic network
%
%-------------------------------------------------------------------------%
%
%   (c) 2009-2013
%
%   A. Petitti
%   D. Di Paola
%   S. Giannini
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function agents = dkns(agents, Nt, filterFlag)
%
%  INPUTS:
%  agents          = agents' structure
%  Nt              = number of targets
%  filterFlag     = filter flag 
%
%  OUTPUTS:
%  agents          = updated agents' structure 
%


%% Phase 1 : Estimation
if strcmp(filterFlag,'kf')
    
    for i = 1 : length(agents)
        for j = 1 : Nt
            
            [x_estim, P_estim] = kalmanFilter(agents(i).dkns.KF(j), agents(i).sens.isSensing(j));
            
            agents(i).dkns.KF(j).x = x_estim;
            agents(i).dkns.KF(j).P = P_estim;
        end
    end
    
else
    error('MASF/_CORE :: CDMTT.m - No valid Filter Flag value');
end


%% Phase 2 : Node Selection

netDiameter = length(agents)-1;%getNetDiameter(); % Todo decentralized function
agents = maxConsensus(agents, Nt, netDiameter);


return