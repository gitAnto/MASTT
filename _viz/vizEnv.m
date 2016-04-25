%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Multi Agent Simulator for Target Tracking (MASTT)
%
%  -----------------------------------------------------------------------
%  Visualization (_viz) Toolbox
%
%
%  vizEnv.m
%
%  Visualization of the environment (main viz object).
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

function viz = vizEnv(env)
%
%  INPUTS:
%  env    = Enviroment structure
%
%  OUTPUTS:
%  viz    = visualition structure
%

%% Parameters initialization
%

winPosX  = 200;
winPosY  = 100;

winSizeX = 700;
winSizeY = 700;

xmin = -env.L/2;
xmax = env.L/2;
ymin = -env.L/2;
ymax = env.L/2;

viz.f = figure(); clf;
set(viz.f,'doublebuffer','on','position',[winPosX winPosY winSizeX winSizeY],'color',[1 1 1]);

viz.axs = axes('position',[.1 .1 .8 .8]);
set(viz.axs,'box','on','nextplot','add','xlim',[xmin xmax],'ylim',[ymin ymax],...
    'xtick',[],'ytick',[],'plotboxaspectratio',[(xmax-xmin) (ymax-ymin) 1]);

return

