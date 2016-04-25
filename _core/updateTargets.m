%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Multi Agent Simulator for Target Tracking (MASTT)
%
%  updateTargets.m
%
%  Update the targets structure
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

function targets = updateTargets(params, dkns_parameters, targets)
%
%  INPUTS:
%  params       = Global Parameters structure
%  dkns_params  = dkns parameters
%  targets      = targets structure to update
%
%  OUTPUTS:
%  targets      = updated targets structure
%


%% MASF Updateting targets structure
%

for k = 1:dkns_parameters.Nt
    
    
    w = dkns_parameters.sigma_0*randn(2,1);
    
    x = [targets(k).state.x;  targets(k).state.vel_x; targets(k).state.y;  targets(k).state.vel_y;];
    
    M = [mu(x(1), dkns_parameters.a) 0; 0 mu(x(3), dkns_parameters.a)];
    F1 = [1 params.dt; 0 1];
    F2 = [1 params.dt; -params.dt*dkns_parameters.c1 1-params.dt*dkns_parameters.c2];
    
    Gi = [ (params.dt^2*dkns_parameters.sigma_0)/2; params.dt*dkns_parameters.sigma_0];
    zero = [0; 0];
    
    A = [M(1,1)*F1 M(1,2)*F1; M(2,1)*F1 M(2,2)*F1] + [(1-M(1,1))*F2  (-M(1,2))*F2; (-M(2,1))*F2 (1-M(2,2))*F2];
    B = [Gi zero; zero Gi];
    
    x = A*x + B*w;
    
    targets(k).state.x       = x(1);
    targets(k).state.y       = x(3);
    targets(k).state.vel_x   = x(2);
    targets(k).state.vel_y   = x(4);
    targets(k).state.vel     = sqrt(x(2)^2 + x(4)^2);
    targets(k).state.theta   = atan2(targets(k).state.vel_y, targets(k).state.vel_x);
    
end

return



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Olfati-Saber Trajectory Functions
%  [from "Distributed Tracking in Sensor Networks with Limited Sensing Range"]
%

function y =  mu(z,a)

y = (sigma(a+z) + sigma(a-z)) / 2;

return

function y = sigma(z)

if z >= 0
    
    y = 1;
    
else
    
    y = -1;
    
end

return