%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Multi Agent Simulator for Target Tracking (MASTT)
%
%  -----------------------------------------------------------------------
%
%   (c) 2009-2013
%
%   A. Petitti
%   D. Di Paola
%   S. Giannini
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function handles = simulate(handles)

global sim

while true
    
    if sim
        %% Distributed Estimation Algorithm
        handles.agents = CDMTT(handles.agents, handles.targets, handles.params, handles.dkns_params);
        
        %% Update target positions
        handles.targets = updateTargets(handles.params, handles.dkns_params, handles.targets);
        
        handles.agents = bearingControl(handles.agents);
        
        %% Clean Axes
        cla(handles.axes1);
        
        %% Visualize Targets
        Nt = length(handles.targets);
        acc = 100;   % plot accuracy of circles
        %
        % Targets as circles
        %
        for k= 1 : Nt
            x = handles.targets(k).state.x;
            y = handles.targets(k).state.y;
            
            q = x + 1i * y;
            r = handles.targets(k).state.d;
            h = q + r * exp(1i*(0:pi/acc:2*pi));
            patch(real(h), imag(h), handles.targets(k).draw.color);
        end
        
        %% Visualize Connections
        n = length(handles.agents);
        % Connections
        %
        for i = 1 : n
            for k = 1 : n
                if (i~=k) && (handles.agents(i).tc.G(k) == 1)
                    plot(handles.axes1,[handles.agents(i).state.x,handles.agents(k).state.x],...
                        [handles.agents(i).state.y,handles.agents(k).state.y],'color',[0 1 0]','linewidth',1)
                end
            end
        end
        
        %% Visualize Agents
        n = length(handles.agents);
        acc = 100;   % plot accuracy of circles
        %
        % Sensing Areas
        %
        if strcmp(handles.viz_params.plotSensors,'on')
            sens_area_color= [.8 .8 .8];
            
            if( strcmp(handles.params.SENS_MODE,'iso') )
                for k = 1 : n
                    q = handles.agents(k).state.x + 1i * handles.agents(k).state.y;
                    r = handles.agents(k).sens.range;
                    h = q + r * exp(1i*(0:pi/acc:2*pi));
                    
                    plot(handles.axes1,real(h), imag(h), 'color',sens_area_color,'linewidth',1);
                end
            elseif( strcmp(handles.params.SENS_MODE,'sector') )
                
                for k = 1 : n
                    
                    theta_s = handles.agents(k).state.theta - (handles.agents(k).sens.angle/2) + handles.agents(k).sens.orientation;
                    
                    % Define the sector polygon
                    th = 0: 2*pi/acc : handles.agents(k).sens.angle;
                    xunit = handles.agents(k).sens.range * cos(th) + handles.agents(k).state.x;
                    yunit = handles.agents(k).sens.range * sin(th) + handles.agents(k).state.y;
                    P = [handles.agents(k).state.x xunit; handles.agents(k).state.y yunit];
                    
                    % Rotation Matrix
                    R = [ cos(theta_s) -sin(theta_s); ...
                        sin(theta_s) cos(theta_s)];
                    % Apply the rotation to the polygon
                    P = R*P;
                    
                    % Traslation Matrix
                    T = [ones(1, size(P,2)) * handles.agents(k).state.x - P(1,1); ...
                        ones(1, size(P,2)) * handles.agents(k).state.y - P(2,1)];
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
        %
        % Agents as triangular vehicles
        %
        for k = 1 : n
            
            % Define the vehicle polygon
            a = [-0.3536 -0.3536]';
            b = [.5 0]';
            c = [-0.3536 0.3536]';
            d = [-0.2 0]';
            
            % Scale the polygon
            P = handles.agents(k).state.d*[a b c d];
            % Rotation Matrix
            R = [ cos(handles.agents(k).state.theta) -sin(handles.agents(k).state.theta); ...
                sin(handles.agents(k).state.theta) cos(handles.agents(k).state.theta)];
            % Traslation Matrix
            T = [ones(1,size(P,2))*handles.agents(k).state.x; ones(1,size(P,2))*handles.agents(k).state.y];
            
            % Apply the roto-traslation to the polygon
            P = R*P + T;
            
            patch(P(1,:), P(2,:), handles.agents(k).draw.color);
        end
        
        %% Visualize Estimate
        plot(handles.axes1,handles.agents(1).dkns.KF(1).x(1),handles.agents(1).dkns.KF(1).x(3),'*', 'LineWidth', 1, 'MarkerSize', 10);
        
        %% Visualize Covariance
        mu = [handles.agents(1).dkns.KF(1).x(1); handles.agents(1).dkns.KF(1).x(3)];
        Sigma = [handles.agents(1).dkns.KF(1).P(1,1) handles.agents(1).dkns.KF(1).P(1,3); handles.agents(1).dkns.KF(1).P(3,1) handles.agents(1).dkns.KF(1).P(3,3)];
        plotgauss2d(mu, Sigma);     
        
        pause(.1);
    else
        break;
    end
end
return