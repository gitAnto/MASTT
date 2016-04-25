%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Multi Agent Simulator for Target Tracking (MASTT)
%
%  createTargets.m
%
%  Create the basic targets' structure
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

function targets = createTargets(env, dkns_params)
%
%  INPUTS:
%  env         = Enviroment structure
%  dkns_params = dkns structure
%
%  OUTPUTS:
%  targets     = Targets structure
%


%% MASF Populating targets structure
%
if dkns_params.Nt>0
    for i = 1:dkns_params.Nt
        
        targets(i) = target('id',i);  % Create a new target with an identifier
        targets(i).status = 'ACTIVE';
        
        targets(i).draw.color = [rand rand rand];
        
        %-----------------------------------%
        % Physical Parameter
        %
        
        [state_x, state_y] = targetPositions(env);
        
        targets(i).state.x = state_x;           % target x coordinate
        targets(i).state.y = state_y;           % target y coordinate
        targets(i).state.theta = pi*rand();     % target orientation (radiant)
        
        targets(i).state.vel = 10;              % target velocity magnitude
        targets(i).state.vel_x = 7;             % target velocity x component
        targets(i).state.vel_y = 20;            % target velocity y component
        
        
        targets(i).state.d = .5;                % target occupied space
    end
else
    targets = [];
end

return



function [x_coord,y_coord] = targetPositions(env)

%% Targets positioning function
%
x_coord = -env.W/4+(env.W)/2.*rand;
y_coord = -env.H/4+(env.H)/2.*rand;


return