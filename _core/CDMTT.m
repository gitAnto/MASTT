%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Multi Agent Simulator for Target Tracking (MASTT)
%
%  CDMTT.m
%
%  Run each phase of the distributed estimation algorithm for the
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

function agents = CDMTT(agents, targets, params, dkns_params)
%
%  INPUTS:
%  agents      = agents' structure 
%  targets     = targets' structure 
%  params      =  Global Parameters structure
%  dkns_params =  dkns parameters
%
%  OUTPUTS:
%  agents      = agents' structure 
%

%% Sensing Phase
if strcmp(params.SENS_MODEL,'rb')
    
    agents = sensingRB(agents,targets,dkns_params);
elseif strcmp(params.SENS_MODEL,'standard')
    
    agents = sensingStandard(agents,targets,dkns_params);
else
    
    error('MASF/_CORE :: CDMTT.m - No valid Sensing Model [SENS_MODEL] value');
end

%% DKNS Phase

agents = dkns(agents, dkns_params.Nt, 'kf');

return