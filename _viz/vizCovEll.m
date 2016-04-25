%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Multi Agent Simulator for Target Tracking (MASTT)
%
%  -----------------------------------------------------------------------
%  Visualization (_viz) Toolbox
%
%
%  vizCovEll.m
%
%  Visualization of the covariance ellipse.
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

function viz = vizCovEll(viz, x, P)
%
%  INPUTS:
%  viz = visualition structure
%  x   = state estimation
%  P   = covariance matrix
%
%  OUTPUTS:
%  viz = visualition structure
%

%% Parameters initialization
%
%n = length(agents);
acc = 100;   % plot accuracy of circles

mu = [x(1); x(3)];
Sigma = [P(1,1) P(1,3); P(3,1) P(3,3)];

%% Estimates as circles
%
%for k = 1 : n
%hnew =  plot(agents(1).cdtt.KF(1).x(1),agents(1).cdtt.KF(1).x(3),'*', 'LineWidth', 1, 'MarkerSize', 10);
%end
h = plotgauss2d(mu, Sigma);

%legend(hnew, 'DENS', 'Location', 'NorthOutside');
return

