%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Multi Agent Simulator for Target Tracking (MASTT)
%
%  createAgents.m
%
%  Create the basic agents' structure.
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

function agents = createAgents(n, env, params)
%
%  INPUTS:
%  n      = Number of agents
%  env    = Enviroment structure
%  params  = Global Parameters structure
%
%  OUTPUTS:
%  agents = Agents structure
%

n_ag = n;

if ( strcmp(params.POS_MODE,'grid') && mod(n,round(sqrt(n))*round(sqrt(n))) ) % for visualization purposes
    n_ag = round(sqrt(n))*round(sqrt(n));
    disp('');
    fprintf(2,'  params.POS_MODE == grid --> number of agents set to %d \n', n_ag); 

end

%% MASF Populating agents structure
%
for i = 1 : n_ag
    
    agents(i) = agent('id',i);             % Create a new agent with a unique identifier
    agents(i).status = 'FIXED';
    
    %-----------------------------------%
    % DraW Parameter
    %
    agents(i).draw.color = [1 1 0];
    
    %-----------------------------------%
    % Physical Parameter
    %
    agents(i).state.vel =   0;             % agent velocity
    agents(i).state.vel_x = 0;             % agent velocity x component
    agents(i).state.vel_y = 0;             % agent velocity y component
    
    agents(i).state.theta = pi*rand();     % agent orientation (radiant)
    agents(i).state.theta_t = 0;           % number of iteration with the same theta
    
    agents(i).state.r = 0;                 % agent radius (only for circular agents)
    agents(i).state.d = 2;                 % agent occupied space
    
    agents(i).state.x = 0;                 % agent x coordinate
    agents(i).state.y = 0;                 % agent y coordinate
    agents(i).state.dr = 0;
    
    % Agents Positioning Function
    if( strcmp(params.POS_MODE,'random') ||  strcmp(params.POS_MODE,'grid') )
        [agents(i).state.x, agents(i).state.y] = agentPositions(n, env, agents, params.POS_MODE);
    else
        error('MASF/_CORE :: createAgents.m - No valid Positioning Mode [POS_MODE] value');
    end
    
    
    %-----------------------------------%
    % Communication System
    %
    if ( strcmp(params.COMM_MODE,'default') || strcmp(params.COMM_MODE,'os'))
        agents(i).comm.range = (env.L + 12*agents(i).state.d)/(round(sqrt(n))+1) + 2;
    else
        error('MASF/_CORE :: createAgents.m - No valid Communication Mode [COMM_MODE] value');
    end
    
    
    %-----------------------------------%
    % Sensing System
    %
    
    % Common Parameters for omogeneous robots
    agents(i).sens.range = agents(i).state.d*4;
    agents(i).sens.angle = pi;                  % Sensor's field of view
    agents(i).sens.orientation = 0;             % FOV = [orientation - angle/2; orientation + angle/2] w.r.t. agent's frame
    
    
    if( strcmp(params.SENS_MODE,'iso') )
        agents(i).sens.angle = 2*pi;
    elseif( strcmp(params.SENS_MODE,'sector') )
        % do nothing - maintain common parameters
    else
        error('MASF/_CORE :: createAgents.m - No valid Sensing Mode [SENS_MODE] value');
    end
    
    if( strcmp(params.SENS_TYPE,'homo') )
        % do nothing
    elseif( strcmp(params.SENS_TYPE,'hetero') )
        agents(i).sens.range = agents(i).sens.range + agents(i).sens.range*.5*rand;
        if( ~strcmp(params.SENS_MODE,'iso') )
            agents(i).sens.angle = agents(i).sens.angle + pi/4*randn;
        end
        agents(i).sens.orientation = 0; %agents(i).sens.orientation + pi/4*randn;
    else
        error('MASF/_CORE :: createAgents.m - No valid Sensing Type [SENS_TYPE] value');
    end
    
end

return


function [x_coord, y_coord] = agentPositions(n, env, agents, mode)
%% Agents positioning function
%
agent_id = numel(agents);
agent_diam = agents(agent_id).state.d;
area_perc_red = 12;                     % percentage of reduction of the positioning area

L_min = -(env.L/2) + (env.L/100*area_perc_red);
L_max = env.L/2 - (env.L/100*area_perc_red);

if( strcmp(mode,'random') )
    % Random positioning
    x_coord = L_min + (L_max - L_min)*rand;
    y_coord = L_min + (L_max - L_min)*rand;
    
    if( agent_id > 1 )
        free_position = false;
        while(free_position == false);
            x_coord = L_min + (L_max - L_min)*rand;
            y_coord = L_min + (L_max - L_min)*rand;
            
            for i = 1 : (agent_id-1)
                dist = sqrt((x_coord - agents(i).state.x)^2 + (y_coord - agents(i).state.y)^2);
                if dist > 7 * agent_diam
                    free_position = true;
                else
                    free_position = false;
                    break;
                end
            end
        end
    end
elseif( strcmp(mode,'grid') )
    % Grid-like positioning
    envL_augm = env.L + 10*agent_diam;
    n_L = round(sqrt(n));
    agent_l = envL_augm/(n_L +1);
    
    if( mod(agent_id, n_L) == 0 )
        x_row=n_L;
        y_row=agent_id/n_L;
    else
        x_row=mod(agent_id, n_L);
        y_row=ceil(agent_id/n_L);
    end
    
    x_coord = x_row*agent_l - envL_augm/2;
    y_coord = y_row*agent_l - envL_augm/2;
    
    %     vet_pos(k).lim_inf_x = (i-1)*x_g;
    %     vet_pos(k).lim_sup_x = (i+1)*x_g;
    
    
    %     vet_pos(k).x_centroid = i*x_g - W/2;
    %     vet_pos(k).y_centroid = j*y_g - H/2;%
end

return
