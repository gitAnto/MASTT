%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Multi Agent Simulator for Target Tracking (MASTT)
%
%  -----------------------------------------------------------------------
%  
%
%  TC/createGraph.m
%
%  % Communication graph's creation.
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

function Ag = createGraph(Ag,a)
%
%  INPUTS: 
%  Ag = Agents structure
%  a  = agent's id
%
%  OUTPUTS:
%  Ag = Agents structure
%

%-----------------------------------%
% Adjacency matrix
%
Nu = size(Ag(a).tc.G,2);

p = zeros(1,Nu);
temp = zeros(Nu,1);
m = 5;

for k = 1 : Nu
    p(k) = Ag(k).state.x + 1i * Ag(k).state.y;
end


dp = p - p(a); 
dist = sqrt(dp .* conj(dp));
vedo = dist <= Ag(a).comm.range; % if 1, it can communicate with agent a 

for k = 1 : Nu
    if(vedo(k))
        temp(k) = p(k);          % who are my neighbours?
    end
end

root=a;

Ag(a).tc.G(1,:) = vedo;
Ag(a).tc.G(1,a) = 0;
Ag(a).tc.neig.num= sum((Ag(a).tc.G));


%-----------------------------------%
% r-disk proximity graph computation
% weight function: f(p)=||p||*(r-||p||)^(-m)
%
for i=1:Nu
    if(Ag(a).tc.G(1,i)||i==a)
        Ag(a).tc.W(1,i) = dist(1,i)/(Ag(a).sens.range-dist(1,i))^m;
    end
end

%-----------------------------------%
% a is the root of its local tree
%
Ag(a).tc.path(root,1) = 0;
Ag(a).tc.path(root,2) = 0;
Ag(a).tc.pos(:,1) = temp(:,1);    % neighbours' location

return
