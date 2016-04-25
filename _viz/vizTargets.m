%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Multi Agent Simulator for Target Tracking (MASTT)
%
%  -----------------------------------------------------------------------
%  Visualization (_viz) Toolbox
%
%
%  vizTargets.m
%
%  Visualization of targets in the environment.
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

function viz = vizTargets(viz, targets)
%
%  INPUTS:
%  viz     = visualition structure
%  targets = Targets structure
%
%  OUTPUTS:
%  viz     = visualition structure
%

%% Parameters initialization
%
Nt = length(targets);
acc = 100;   % plot accuracy of circles



%% Targets as circles
%

for k= 1 : Nt
    
    x = targets(k).state.x;
    y = targets(k).state.y;
    
    q = x + 1i * y;
    r = targets(k).state.d;
    h = q + r * exp(1i*(0:pi/acc:2*pi));
    patch(real(h), imag(h), targets(k).draw.color);
end

return

