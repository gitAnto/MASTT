%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Multi Agent Simulator for Target Tracking (MASTT)
%
%  -----------------------------------------------------------------------
%  
%
%  TC/lmst.m
%
%  % Finding the candidate link to be added to the local minimum spanning tree.
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

function linked = lmst(edges)
%
%  INPUTS: 
%  edges  = edges' structure
%
%  OUTPUTS:
%  linked = candidate edge
%

%-----------------------------------%
% edges sorted by cost
%
[costs,I] = sort(edges(3,:));
linked=[];
k=0;
for i = 1:size(I,2)
    if(edges(4,I(i))==1 && edges(5,I(i))~=0)
        k=k+1;
        linked(1,k)=edges(1,I(i));
        linked(2,k)=edges(2,I(i));
        linked(3,k)=edges(3,I(i));
        linked(4,k)=edges(4,I(i));
        linked(5,k)=edges(5,I(i));
    end
end

if(size(linked,2)~=0)
    
    costo_min = linked(3,1);
    %-----------------------------------%
    % edges with the same cost, sorted by index
    %
    k=0;
    while(k<size(linked,2) && linked(3,k+1)==costo_min)
        k=k+1;
    end
    if(k>1)
        [ind_max,II] = sort(linked(2,1:k));
        link1(2,1:k)=ind_max;
        for i = 1:k
            link1(1,i)=linked(1,II(i));
            link1(3,i)=linked(3,II(i));
            link1(4,i)=linked(4,II(i));
            link1(5,i)=linked(5,II(i));
        end
        linked(:,1:k)=link1;%
        i_max = linked(2,1);
        m=0;
        while(m<size(link1,2) && link1(2,m+1)==i_max)
            m=m+1;
        end
        if(m>1)
            [ind_min,III] = sort(link1(1,1:m));
            link2(1,1:m)=ind_min;
            for i = 1:m
                link2(2,i)=link1(2,III(i));
                link2(3,i)=link1(3,III(i));
                link2(4,i)=link1(4,III(i));
                link2(5,i)=link1(5,III(i));
            end
            linked(:,1:m)=link2;
            link1=[];
            link2=[];

         end

    end
    
    
end
    