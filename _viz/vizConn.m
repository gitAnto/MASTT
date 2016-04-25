%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Multi Agent Simulator for Target Tracking (MASTT)
%
%  -----------------------------------------------------------------------
%  Visualization (_viz) Toolbox
%
%
%  vizConn.m
%
%  Visualization agents connections given the connection graph
%
%-------------------------------------------------------------------------%
%
%   (c) 2009-2013
%
%   A. Petitti
%   D. Di Paola
%   S. Giannini
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function viz = vizConn(viz, agents)
%
%  INPUTS:
%  viz    = visualition structure
%  agents = Agents structure
%
%  OUTPUTS:
%  viz    = visualition structure
%

%% Parameters initialization
%
n = length(agents);

%% Connections
%
for i = 1 : n
    for k = 1 : n
        if (i~=k) && (agents(i).tc.G(k) == 1)
            plot(viz.axs,[agents(i).state.x,agents(k).state.x],...
                [agents(i).state.y,agents(k).state.y],'color',[0 1 0]','linewidth',1)
        end
    end
end

return

