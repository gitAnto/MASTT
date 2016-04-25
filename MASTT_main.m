%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Multi Agent Simulator for Target Tracking
%   
%   Part of the code is related to:
%      D.Di Paola, A.Petitti, A.Rizzo, Distributed Kalman Filtering via Node Selection in Heterogeneous Sensor Networks,
%      International Journal of Systems Science, 2014.
%
%
%   (c) 2009-2013
%
%   A. Petitti
%   D. Di Paola
%   S. Giannini
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;
clc;

disp('--------------------------------------------------------------');
disp('|      Multi Agent Simulator for Target Tracking(MASTT)       |');
disp('--------------------------------------------------------------');
disp(' ');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DKNS-User Parameters Initialization
%

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

disp('- Initialization core parameters ... ');


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

PARAMS.Nu = 18;      % Number of Sensing Units

% Agents Positioning Mode
% random    = random free position in the environment
% grid      = grid-like pattern position in the environment
%
PARAMS.POS_MODE = 'random';

% Communication System Mode
% default   = isotropic communication area defined by a range
% os        = Olfati-Saber paper range definition
%
PARAMS.COMM_MODE = 'default';

% Sensing System Mode
% iso       = isotropic sensing area (FOV = 360??)
% sector    = sector sensing area (FOV defined by range and angle)
%
PARAMS.SENS_MODE = 'sector';

% Sensing System Type
% homo      = homogeneous angles and ranges
% hetero    = heterogeneous angles and ranges
%
PARAMS.SENS_TYPE = 'hetero';

% Sensing System Model
% rb        = range bearing sensor model
% standard  = standard KF-model
%
PARAMS.SENS_MODEL = 'rb';

disp('- Create agent structures ... ');

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

disp('- Initialization specific DKNS parameters ... ');

%-----------------------------------%
% DKNS Params
%
DKNS_PARAMS.Nt = 1;%PARAMS.Nt;

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
% Simulation Core
%
vizMain = vizEnv(ENV);

% Setup network for this experiment
AGENTS_DKNS = appendConfigDKNS(AGENTS, PARAMS, DKNS_PARAMS);

% Setup targets for this experiment
TARGETS = createTargets(ENV, DKNS_PARAMS);

reverseStr = '';
fprintf('\n');
disp('* Start Experiment *');
fprintf('\n');
for k_iters = 1 : PARAMS.n_iters   % Single Experiment
    
    % Display the progress
    percentDone = 100 * k_iters / PARAMS.n_iters;
    msg = sprintf('Completed ... %3.1f', percentDone);
    fprintf([reverseStr, msg]);
    reverseStr = repmat(sprintf('\b'), 1, length(msg));
    
    %% Distributed Estimation Algorithm
    AGENTS_DKNS = CDMTT(AGENTS_DKNS, TARGETS, PARAMS, DKNS_PARAMS);
    
    %% Update target positions
    TARGETS = updateTargets(PARAMS, DKNS_PARAMS, TARGETS);
    
    AGENTS_DKNS = bearingControl(AGENTS_DKNS);
    vizMain = vizClean(vizMain);
    vizMain = vizTargets(vizMain, TARGETS);
    vizMain = vizConn(vizMain,AGENTS_DKNS);
    vizMain = vizAgents(vizMain,AGENTS_DKNS,PARAMS,VIZ_PARAMS.plotSensors);
    vizMain = vizEstimate(vizMain, AGENTS_DKNS, ENV);
    vizMain = vizCovEll(vizMain, AGENTS_DKNS(1).dkns.KF(1).x, AGENTS_DKNS(1).dkns.KF(1).P);
    pause(.1);
     
end

fprintf('\n\n');
disp('* Stop Experiment *');



%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%