%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Multi Agent Simulator for Target Tracking (MASTT)
%
%  -----------------------------------------------------------------------
%  Visualization (_viz) Toolbox
%
%
%  vizAgents.m
%
%  Visualization of agents in the environment.
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

function viz = vizAgents(viz, agents, param, sensorFlag)
%
%  INPUTS:
%  viz        = visualition structure
%  agents     = Agents structure
%  param      = Global Parameters of the agent network structure
%  sensorFlag = sensor flag: on/off
%
%  OUTPUTS:
%  viz        = visualition structure
%

%% Parameters initialization
%
n = length(agents);
acc = 100;   % plot accuracy of circles


%% Sensing Areas
%
if strcmp(sensorFlag,'on')
    sens_area_color= [.8 .8 .8];
    
    if( strcmp(param.SENS_MODE,'iso') )
        for k = 1 : n
            q = agents(k).state.x + 1i * agents(k).state.y;
            r = agents(k).sens.range;
            h = q + r * exp(1i*(0:pi/acc:2*pi));
            
            plot(viz.axs,real(h), imag(h), 'color',sens_area_color,'linewidth',1);
        end
    elseif( strcmp(param.SENS_MODE,'sector') )
        
        for k = 1 : n
            
            theta_s = agents(k).state.theta - (agents(k).sens.angle/2) + agents(k).sens.orientation;
            
            % Define the sector polygon
            th = 0: 2*pi/acc : agents(k).sens.angle;
            xunit = agents(k).sens.range * cos(th) + agents(k).state.x;
            yunit = agents(k).sens.range * sin(th) + agents(k).state.y;
            P = [agents(k).state.x xunit; agents(k).state.y yunit];
            
            % Rotation Matrix
            R = [ cos(theta_s) -sin(theta_s); ...
                sin(theta_s) cos(theta_s)];
            % Apply the rotation to the polygon
            P = R*P;
            
            % Traslation Matrix
            T = [ones(1, size(P,2)) * agents(k).state.x - P(1,1); ...
                ones(1, size(P,2)) * agents(k).state.y - P(2,1)];
            % Apply the traslation to the polygon
            P = P + T;
            
            p = patch(P(1,:), P(2,:),'w');
            set(p,'FaceColor', [.95 .95 .95]);
            set(p,'FaceAlpha', .3);
            set(p,'EdgeColor',sens_area_color);
            
        end
    else
        error('MASF/_VIZ :: vizAgents.m - No valid Sensing Type [SENS_MODE] value');
    end
end

%% Agents as triangular vehicles
%
for k = 1 : n
    
    % Define the vehicle polygon
    a = [-0.3536 -0.3536]';
    b = [.5 0]';
    c = [-0.3536 0.3536]';
    d = [-0.2 0]';
    
    % Scale the polygon
    P = agents(k).state.d*[a b c d];
    % Rotation Matrix
    R = [ cos(agents(k).state.theta) -sin(agents(k).state.theta); ...
        sin(agents(k).state.theta) cos(agents(k).state.theta)];
    % Traslation Matrix
    T = [ones(1,size(P,2))*agents(k).state.x; ones(1,size(P,2))*agents(k).state.y];
    
    % Apply the roto-traslation to the polygon
    P = R*P + T;
    
    patch(P(1,:), P(2,:), agents(k).draw.color);
end

return

