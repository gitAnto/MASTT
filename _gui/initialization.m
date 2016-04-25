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

function handles = initialization(handles)

%-----------------------------------%
% Setup Paths
%
addpath('./_tools');
addpath('./DKNS');
addpath('./TC');

%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MASF-Parameters initialization
%


%-----------------------------------%
% VISUALIZATIONS
%

addpath('./_viz');


%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MASF-Core setup
%

addpath('./_core');


%-----------------------------------%
% Init Structures
%

ENV = [];
OBS = [];
PARAMS = [];
AGENTS = [];
VIZ_PARAMS = [];


%-----------------------------------%
% ENVIROMENT PARAMETERS
%
ENV.W = 90;          % Environment dimension
ENV.H = 90;
ENV.L = max([ENV.W ENV.H]);
ENV.mode = 'closed'; % closed / toroid


%-----------------------------------%
% AGENTS PARAMETERS
%

PARAMS.Nu = handles.Nu;      % Number of Sensing Units

% Agents Positioning Mode
% random    = random free position in the environment
% grid      = grid-like pattern position in the environment
%
PARAMS.POS_MODE = handles.pos;

% Communication System Mode
% default   = isotropic communication area defined by a range
% os        = Olfati-Saber paper range definition
%
PARAMS.COMM_MODE = 'default';

% Sensing System Mode
% iso       = isotropic sensing area (FOV = 360??)
% sector    = sector sensing area (FOV defined by range and angle)
%
PARAMS.SENS_MODE = handles.sens.type;

% Sensing System Type
% homo      = homogeneous angles and ranges
% hetero    = heterogeneous angles and ranges
%
PARAMS.SENS_TYPE = handles.sens.mode;

% Sensing System Model
% rb        = range bearing sensor model
% standard  = standard KF-model
%
PARAMS.SENS_MODEL = handles.sens.model;


AGENTS = createAgents(PARAMS.Nu,ENV,PARAMS);


%-----------------------------------%
% GENERAL PARAMETERS
%

% Simulation Params
PARAMS.Nt = 1;          % Number of Targets

PARAMS.n_iters = 300;   % Iterations for each experiment
PARAMS.dt = 0.04;       % Time Step


%-----------------------------------%
% VISUALIZATION PARAMETERS
%
VIZ_PARAMS.plotSensors = 'on';

%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DKNS-Specific setup
%


%-----------------------------------%
% DKNS Params
%
DKNS_PARAMS.Nt = PARAMS.Nt;

% Target Moving Model Parameters
DKNS_PARAMS.c1 = 0.75;
DKNS_PARAMS.c2 = 1;
DKNS_PARAMS.a = 33;
% Kalman Filter Parameters
DKNS_PARAMS.kw = 5;
DKNS_PARAMS.kv = 3;
DKNS_PARAMS.sigma_0 = 5;
% Sensing Parameters
DKNS_PARAMS.RB.k_theta = 1/10;
DKNS_PARAMS.RB.k_d = 1.056;
DKNS_PARAMS.RB.k_rho = 10.07;
DKNS_PARAMS.ST.sigma = 5;

%-----------------------------------%
% TC Params
%
 TC_PARAMS.m = 5;

%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DKNS-MainLoop
%

%-----------------------------------%
% Append Configurations to Agents strucutures
%
AGENTS = appendConfigTC(AGENTS);

%-----------------------------------%
% Visualization parameters initialization
%
winPosX  = 200;
winPosY  = 100;

winSizeX = 700;
winSizeY = 700;

xmin = -ENV.L/2;
xmax = ENV.L/2;
ymin = -ENV.L/2;
ymax = ENV.L/2;

%viz.f = figure(); clf;
%set(viz.f,'doublebuffer','on','position',[winPosX winPosY winSizeX winSizeY],'color',[1 1 1]);

%viz.axs = axes('position',[.1 .1 .8 .8]);
set(handles.axes1,'box','on','nextplot','add','xlim',[xmin xmax],'ylim',[ymin ymax],...
    'xtick',[],'ytick',[],'plotboxaspectratio',[(xmax-xmin) (ymax-ymin) 1]);


% Setup network for this experiment
AGENTS_DKNS = appendConfigDKNS(AGENTS, PARAMS, DKNS_PARAMS);

% Setup targets for this experiment
TARGETS = createTargets(ENV, DKNS_PARAMS);

% Add structures to handles
handles.agents       = AGENTS_DKNS;
handles.targets      = TARGETS;
handles.env          = ENV;
handles.params       = PARAMS;
handles.viz_params   = VIZ_PARAMS;
handles.dkns_params  = DKNS_PARAMS;
handles.tc_params    = TC_PARAMS;