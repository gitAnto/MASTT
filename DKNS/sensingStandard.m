%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Multi Agent Simulator for Target Tracking (MASTT)
%
%  sensingStandard.m
%
%  Sensing phase with gaussian sensing error
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

function agents = sensingStandard(agents,targets,dkns_params)
%
%  INPUTS:
%  agents          = agents' structure
%  targets         = targets' structure
%  dkns_params     = DKNS parameters of the agents' network structure 
%
%  OUTPUTS:
%  agents          = updated agents' structure 
%

for i = 1:length(agents)
    
    theta =         agents(i).state.theta;
    orientation =   agents(i).sens.orientation;
    angle =         agents(i).sens.angle;
    
    for j = 1 : length(targets)
        
        beta = atan2(targets(j).state.y-agents(i).state.y, targets(j).state.x-agents(i).state.x);
        dist = sqrt( (targets(j).state.x-agents(i).state.x)^2 + (targets(j).state.y-agents(i).state.y)^2 );
        
        if sign(beta)>sign(theta)
            theta = theta + 2*pi;
        elseif sign(beta)<sign(theta)
            beta = beta + 2*pi;
        end
        
        if dist <= agents(i).sens.range
            
            if ( beta <= (theta + orientation + angle/2) ) && ( beta >= (theta + orientation - angle/2) )
                
                
                
                % Update isSensing variable
                agents(i).sens.isSensing(j) = true;
                
                % Update the measurement
                agents(i).dkns.KF(j).z(1)  = targets(j).state.x + dkns_params.ST.sigma*(-.5+rand);
                agents(i).dkns.KF(j).z(2)  = targets(j).state.y + dkns_params.ST.sigma*(-.5+rand);
                
                % Update the covariance measurement matrix
                agents(i).dkns.KF(j).R(1,1) = dkns_params.ST.sigma^2;
                agents(i).dkns.KF(j).R(2,2) = dkns_params.ST.sigma^2;
                agents(i).dkns.KF(j).R(2,1) = 0;
                agents(i).dkns.KF(j).R(1, 2) = 0;
                
            else
                % Update isSensing variable
                agents(i).sens.isSensing(j) = false;
            end
            
        else
            % Update isSensing variable
            agents(i).sens.isSensing(j) = false;
        end
        
    end
end