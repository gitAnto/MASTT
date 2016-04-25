%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Multi Agent Simulator for Target Tracking (MASTT)
%
%  -----------------------------------------------------------------------
%
%
%  DKNS/maxConsensus.m
%
%  MAx-consensus protocol
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

function agents = maxConsensus(agents, Nt, diam)
%
%  INPUTS:
%  agents          = agents' structure
%  Nt              = number of targets
%  diam            = length of the diameter of the network
%
%  OUTPUTS:
%  agents          = updated agents' structure 
%

for t = 1 : diam
    
    % Communication 
    [gamma_, x_, P_] = communication(agents, Nt);
    
    % Consesus
    for i = 1 : length(agents)
        %
        % MAX
       
        % for each Target
        for k=1:Nt
            
            gammaMax = gamma_(i,k);
            indMax   = agents(i).id;
            
            % for each neighbour
            for j = 1:length(agents)
                if agents(i).tc.G(j) 
                    
                    if gamma_(j,k) >= gammaMax
                        
                        if gamma_(j,k) == gammaMax
                            if agents(j).id > indMax
                                indMax = agents(j).id;
                                agents(i).dkns.KF(k).x = x_{j,k};
                                agents(i).dkns.KF(k).P = P_{j,k};
                            end
                        else
                            gammaMax = gamma_(j,k);
                            indMax   = agents(j).id;
                            agents(i).dkns.KF(k).x = x_{j,k};
                            agents(i).dkns.KF(k).P = P_{j,k};                            
                        end
                        
                    end
                end
            end
        end
        
    end
end

return

%% Communication function
function [gamma_, x_, P_] = communication(A, Nt)

Na = length(A);

% BUILD gamma_ from all agents
gamma_ = zeros(Na,Nt);
for i = 1 : Na 
    for j = 1 : Nt 
        gamma_(i,j)  = 1/trace(A(i).dkns.KF(j).P);
    end
end


% BUILD x_ from all agents
x_ = cell(Na,Nt);
for i = 1 : Na 
    for j = 1 : Nt 
        x_{i,j}  = A(i).dkns.KF(j).x;
    end
end


% BUILD P_ from all pNode
P_ =  cell(Na,Nt);
for i = 1 : Na 
    for j = 1 : Nt 
        P_{i,j}  = A(i).dkns.KF(j).P;
    end
end


return





