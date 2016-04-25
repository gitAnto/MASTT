%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Multi Agent Simulator for Target Tracking (MASTT)
%
%  Agent.m
%
%  Agent struct declaration  
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

function a = agent(varargin)
%
%  INPUTS: 
%  varargin = Inizialization pairs (field, value) 
%
%  OUTPUTS:
%  a        = Agent structure
%

%% Agent struct
%
a = struct('id'                 , -1,...        % Parameters    
           'type'               , 'n/a',...
           'status'             , 'IDLE',...    
           'state'              , [],...        % Physical Parameters          
           'comm'               , [],...
           'sens'               , [],... 
           'draw'               , []);          % Draw variables
       
%% Inputs Handler
%
   if mod(nargin,2) == 0
       % even # of inputs, given as ('param','value') pairs
       % e.g. 'name', '14QB', 'id', 11 ...
       ii = 1;
       while ii < nargin
           try
               a = setfield(a,varargin{ii},varargin{ii+1});
           catch
               error(['MASF/_CORE :: agent.m - Invalid pair: (' varargin{ii} ', ' varargin{ii+1} ')']);
           end
           ii = ii + 2;
       end
   else
       error('MASF/_CORE :: agent.m - Inputs must be paired, e.g. a = agent(''name'', ''James'', ''id'', 007)');
   end

return
