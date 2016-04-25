%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Multi Agent Simulator for Target Tracking (MASTT)
%
%  Agent.m
%
%  Target struct declaration  
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

function t = target(varargin)
%
%  INPUTS: 
%  varargin = Inizialization pairs (field, value) 
%
%  OUTPUTS:
%  t        = target structure
%

%% Target struct
%
t = struct('id'                 , -1,...        % Parameters    
           'type'               , 'n/a',...
           'status'             , 'IDLE',...    
           'state'              , [],...        % Physical Parameters          
           'draw'               , []);   % Draw variables
    
       
%% Inputs Handler
%
   if mod(nargin,2) == 0
       % even # of inputs, given as ('param','value') pairs
       % e.g. 'name', '14QB', 'id', 11 ...
       ii = 1;
       while ii < nargin
           try
               t = setfield(t,varargin{ii},varargin{ii+1});
           catch
               error(['MASF/_CORE :: target.m - Invalid pair: (' varargin{ii} ', ' varargin{ii+1} ')']);
           end
           ii = ii + 2;
       end
   else
       error('MASF/_CORE :: target.m - Inputs must be paired, e.g. t = target(''name'', ''apparel'', ''id'', 42)');
   end

return
