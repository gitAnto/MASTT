%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Multi Agent Simulator for Target Tracking (MASTT)
%
%  -----------------------------------------------------------------------
%  Visualization (_viz) Toolbox
%
%  bearingControl.m
%
%  Setting the orientation of the agents according to the position
%  estimation of the target
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

function agents = bearingControl(agents)
%
%  INPUTS: 
%  agents = Agents structure
%
%  OUTPUTS:
%  agents = Agents structure
%

for i = 1:length(agents)
    agents(i).state.theta = atan2(agents(i).dkns.KF(1).x(3) - agents(i).state.y, agents(i).dkns.KF(1).x(1) - agents(i).state.x);
end