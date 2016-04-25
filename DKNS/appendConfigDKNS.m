%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Multi Agent Simulator for Target Tracking (MASTT)
%
%  appendConfigDKNS.m
%
%  Update the agents structure
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

function agents_dkns = appendConfigDKNS(agents, params, dkns_params)
%
%  INPUTS: 
%  agents      = Agents structure
%  params      = Global Parameters of the agent network structure
%  dkns_params = DKNS Parameters of the agent network structure
%
%  OUTPUTS:
%  agents_dkns = updated agents structure
%


%-------------------------------------------------%
% Update Agent structure
%

% Copy the agents structures
agents_dkns = agents;


for i = 1 : length(agents_dkns)
    
    %-------------------------------------------------%
    % Update Agent structure
    %
    agents_dkns(i).dkns.zone.x = agents_dkns(i).state.x;
    agents_dkns(i).dkns.zone.y = agents_dkns(i).state.y;
    
    agents_dkns(i).sens.isSensing = 0;

    
    %------------------------------------------------%
    % Kalman Filter (KF) Parameters 
    %
    
    sigma_w = params.dt*dkns_params.sigma_0^2;
    Qi = [(params.dt^2/4)*sigma_w^2 (params.dt/2)*sigma_w^2; (params.dt/2)*sigma_w^2 sigma_w^2];
    zero = zeros(2);
    Q = [Qi zero; zero Qi];

    %Q = dkns_params.sigma_0^2 * eye(4);
    
    A = [1 params.dt 0 0;0 1 0 0;0 0 1 params.dt;0 0 0 1];
    
    H = [1 0 0 0;0 0 1 0];

    agents_dkns(i).dkns.KF.A = A;
    agents_dkns(i).dkns.KF.H = H;

    agents_dkns(i).dkns.KF.Q = Q;
    agents_dkns(i).dkns.KF.R = (agents_dkns(i).id)^(1/2)*eye(2);
    agents_dkns(i).dkns.KF.P = 10*(dkns_params.kw)^2*eye(4);

    agents_dkns(i).dkns.KF.x = [0.1 0 0.1 0]';%[randn() 0 randn() 0]';
    agents_dkns(i).dkns.KF.z = [0 0]';

    agents_dkns(i).dkns.KF.x_fusion = [0 0 0 0]';
    
    
end

return