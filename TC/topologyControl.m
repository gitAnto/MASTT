%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Multi Agent Simulator for Target Tracking (MASTT)
%
%  -----------------------------------------------------------------------
%  
%
%  TC/topologyControl.m
%
%  % Control of network connectivity.
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

function Ag = topologyControl(Ag,u)
%
%  INPUTS: 
%  Ag = Agents structure
%  u  = agent's id
%
%  OUTPUTS:
%  Ag = Agents structure
%
Nu = size(Ag(u).tc.G,2);
in = zeros(Nu,1); %1-> i link che partono dall'agente in(i,1) sono stati aggiunti all'insieme degli archi
in(u,1)=1;
edges=[];
m=5;


%-----------------------------------%
% save all the neighbours of the root node
%
l=0;
for i = 1:Nu
    if(Ag(u).tc.G(1,i))
        
        l=l+1;

        if(u<i)
            edges(1,l)=u;
            edges(2,l)=i;
        else
            edges(1,l)=i;
            edges(2,l)=u;
        end
        edges(3,l)=Ag(u).tc.W(1,i);
        
        edges(4,l)=1;        % 1 -> edge to be visited
        edges(5,l)=inf;      % 0 -> edge in a cycle
    end
end

if(size(edges,2)>0)
    
    tree=1;
    while(tree)
        
        linked=[];
        linked=lmst(edges);
        
        z = size(linked,2);
        q = 0;
        
        if(z~=0)
            
            p=linked(1,1);
            s=linked(2,1);
            %-----------------------------------%
            % if there's a new node's neighbour,
            % links of the new node's neighbour are added
            %
            if(in(p,1)==0)
                in(p,1)=1;
                
                for i =1:Nu
                    if(Ag(u).tc.G(1,i))
                        if(i~=p)
                            if(in(i,1)==0)
                                dp = Ag(u).tc.pos(p,1) - Ag(u).tc.pos(i,1);
                                
                                dist = sqrt(dp .* conj(dp));
                                if(dist<=Ag(u).comm.range)
                                    q=q+1;
                                    if(p<i)
                                        linked(1,z+q)=p;
                                        linked(2,z+q)=i;
                                        linked(5,z+q)= 1;
                                    else
                                        linked(1,z+q)=i;
                                        linked(2,z+q)=p;
                                        linked(5,z+q)= -1;
                                    end
                                    linked(3,z+q)= dist/(Ag(u).comm.range-dist)^m;
                                    linked(4,z+q)= 1;
                                    
                                end
                            end
                        end
                    end
                end
            end

            z = size(linked,2);
            q = 0;
            if(in(s,1)==0)
                in(s,1)=1;
                
                for i =1:Nu
                    if(Ag(u).tc.G(1,i))
                       
                        if(i~=s)
                            if(in(i,1)==0)
                                dp = Ag(u).tc.pos(s,1) - Ag(u).tc.pos(i,1);
                              
                                dist = sqrt(dp .* conj(dp));
                                if(dist<=Ag(u).comm.range)%si pu? usare Ag(u).range se i raggi di comunicazione sono omogenei
                                    q=q+1;
                                    if(s<i)
                                        linked(1,z+q)=s;
                                        linked(2,z+q)=i;
                                        linked(5,z+q)= 1;
                                    else
                                        linked(1,z+q)=i;
                                        linked(2,z+q)=s;
                                        linked(5,z+q)= -1;
                                    end
                                    linked(3,z+q)= dist/(Ag(u).comm.range-dist)^m;%si pu? usare Ag(u).range se i raggi di comunicazione sono omogenei
                                    linked(4,z+q)= 1;
                                    
                                end
                            end
                        end
                    end
                end
            end
            
            %-----------------------------------%
            % first edge of lmst vector added 
            %
            linked(4,1)=0;
            linked(5,1)=0;

            for i=2:size(linked,2)
                if(linked(1,i)==p || linked(1,i)==s)
                    if(linked(5,i)==inf)
                        linked(5,i)=1;
                    else
                        if(linked(5,i)==-1)
                            linked(5,i)=linked(5,i)+1;
                        end
                    end
                end
                if(linked(2,i)==p || linked(2,i)==s)
                    if(linked(5,i)==inf)
                        linked(5,i)=-1;
                    else
                        if(linked(5,i)==1)
                            linked(5,i)=linked(5,i)-1;
                        end
                    end
                end
            end
            
            %-----------------------------------%
            % path update
            %
            if(linked(1,1)==u)
                Ag(u).tc.path(linked(2,1),1)=u;
                Ag(u).tc.path(linked(2,1),2)=linked(3,1);
               
            else
                if(linked(2,1)==u)
                    Ag(u).tc.path(linked(1,1),1)=u;
                    Ag(u).tc.path(linked(1,1),2)=linked(3,1);
                  
                else
                    if(Ag(u).tc.path(linked(1,1),1)==inf)
                        Ag(u).tc.path(linked(1,1),1)=linked(2,1);
                        Ag(u).tc.path(linked(1,1),2)=linked(3,1);
                      
                    else
                        Ag(u).tc.path(linked(2,1),1)=linked(1,1);
                        Ag(u).tc.path(linked(2,1),2)=linked(3,1);
                    
                    end
                    
                end
            end
            
            
            
            edges=[];
            edges=linked;
        else
            tree=0;
        end
    end
    
else
    disp('');
    disp('Disconnected network!');
    disp('');    
end



Ag = bidirectionalLink(Ag,u);

return
