%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Multi Agent Simulator for Target Tracking (MASTT)
%
%  -----------------------------------------------------------------------
%  
%
%  getNetDiameter.m
%
%  % Computing the graph diameter's length.
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

function longestShortestPath = getNetDiameter(G)
%
%  INPUTS: 
%  G                   = communication graph
%
%  OUTPUTS:
%  longestShortestPath = longest shortest path length
%

N = size(G,2);

% Convert PN Graph
for i = 1:N
    for j = 1:N
        if G(i,j)==0;
            G(i,j)=Inf;
        end
        if ( i == j)
            G(i,j) = 1;
        end
    end
end

longestShortestPath = 0;
for i = 1:N
    for j = 1:N
        if(i>=j) 
            continue;
        end
        [sp, spcost] = dijkstra(i, j, G);
        temp = size(sp,2) - 1;
        if(temp > longestShortestPath)
            longestShortestPath = temp;
        end
    end
end

