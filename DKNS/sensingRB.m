%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Multi Agent Simulator for Target Tracking (MASTT)
%
%  sensingRB.m
%
%  Sensing phase with Range Bearing Sensor model
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

function agents = sensingRB(agents,targets,dkns_params)
%
%  INPUTS:
%  agents          = agents' structure
%  targets         = targets' structure
%  dkns_params     = DKNS parameters of the agents' network structure 
%
%  OUTPUTS:
%  agents          = updated agents' structure 
%

for i = 1 : length(agents)
    
    theta =         agents(i).state.theta;
    orientation =   agents(i).sens.orientation;
    angle =         agents(i).sens.angle;
    
    for j = 1 : length(targets)
        
        % Questo IF ? da RIVEDERE!!!
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
                
                % Update variance value
                sigma_ro = dkns_params.RB.k_d*( 1 + ( exp(dkns_params.RB.k_rho*( dist-agents(i).sens.range)/agents(i).sens.range )));
                sigma_beta = dkns_params.RB.k_theta*dist/agents(i).sens.range;
                
                % RB measures
                dist = dist + sigma_ro*(-.5+rand);
                beta = beta + sigma_beta*(-.5+rand);
                
                
                % Update the measurement
                agents(i).dkns.KF(j).z(1) = dist*cos(beta) + agents(i).state.x;
                agents(i).dkns.KF(j).z(2) = dist*sin(beta) + agents(i).state.y;
                
                
                % Update the covariance measurement matrix
                T = [cos(beta) -sin(beta); sin(beta) cos(beta)];
                agents(i).dkns.KF(j).R = T*[sigma_ro^2 0; 0 dist^2*sigma_beta^2]*T';
                
                
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