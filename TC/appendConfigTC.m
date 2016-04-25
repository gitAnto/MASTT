%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Multi Agent Simulator for Target Tracking (MASTT)
%
%  -----------------------------------------------------------------------
%  
%
%  TC/appendConfigTC.m
%
%  % Comment
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

function agents_tc = appendConfigTC(agents)%, tc_params)
%
%  INPUTS: 
%  agents     = Agents structure
%  tc_params  = TC Parameters of the agent network structure
%
%  OUTPUTS:
%  agents_tc  = updated agents structure
%


%-------------------------------------------------%
% Update Agent structure
%

% Copy the agents structures
agents_tc = agents;


for i = 1 : length(agents_tc)

    agents_tc(i).tc.G = zeros(1,length(agents_tc));       % k-th row of the adjacency matrix
    agents_tc(i).tc.W = ones(1,length(agents_tc))*inf;    % k-th row of the proximity graph's matrix
    agents_tc(i).tc.path = ones(length(agents_tc),2)*inf; % local tree
    agents_tc(i).tc.pos = ones(length(agents_tc),1)*inf;  % neighbours' location
    agents_tc(i).tc.link = zeros(length(agents_tc));      % link to be preserved
    agents_tc(i).tc.neig.num = 0;                         % number of neighbours
    
end


for k = 1:length(agents_tc)
    agents_tc = createGraph(agents_tc,k);
    agents_tc = topologyControl(agents_tc,k);
end




return




