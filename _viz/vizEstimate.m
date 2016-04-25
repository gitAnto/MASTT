%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Multi Agent Simulator for Target Tracking (MASTT)
%
%  -----------------------------------------------------------------------
%  Visualization (_viz) Toolbox
%
%
%  vizEstimate.m
%
%  Visualization of the estimation of the target's position.
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

function viz = vizEstimate(viz, agents, env)
%
%  INPUTS:
%  viz = visualition structure
%  agents = Agents structure
%
%  OUTPUTS:
%  viz = visualition structure
%


%% Estimates as asterisks
%

if (abs(agents(1).dkns.KF(1).x(1)) < env.W/2 && abs(agents(1).dkns.KF(1).x(3)) < env.H/2)
    hnew =  plot(viz.axs,agents(1).dkns.KF(1).x(1),agents(1).dkns.KF(1).x(3),'*', 'LineWidth', 1, 'MarkerSize', 10);
else
    disp('');
end

%    legend(hnew, 'DKNS', 'Location', 'NorthOutside');
return

