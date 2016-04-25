%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Multi Agent Simulator for Target Tracking (MASTT)
%
%  -----------------------------------------------------------------------
%  
%
%  TC/bidirectionalLink.m
%
%  % Noticing to my neighbours who's in my lmst
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


function  Ag = bidirectionalLink(Ag,u)
%
%  INPUTS: 
%  Ag = Agents structure
%  u  = agent's id
%
%  OUTPUTS:
%  Ag = Agents structure
%

Nu = size(Ag(u).tc.path,1);
for i = 1:Nu
    if(Ag(u).tc.G(1,i) && Ag(u).tc.path(i,1)~=inf && Ag(u).tc.path(i,1)~=0)
        Ag(i).tc.link(Ag(u).tc.path(i,1),i)=1;
        Ag(i).tc.link(i,Ag(u).tc.path(i,1))=1;
        Ag(Ag(u).tc.path(i,1)).link(Ag(u).tc.path(i,1),i)=1;
        Ag(Ag(u).tc.path(i,1)).link(i,Ag(u).tc.path(i,1))=1;
    end
end
