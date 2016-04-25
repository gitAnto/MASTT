%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Multi Agent Simulator for Target Tracking (MASTT)
%
%  kalmanFilter.m
%
%  Kalman Filter (with sensing/no sensing option)
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

function [x_estim, P_estim] = kalmanFilter(kf, isSensing)
%
%  INPUTS:
%  kf           = Kalman Filter Structure
%  isSensing    = Sensing Flag 
%
%  OUTPUTS:
%  x_estim      = estimation of the process state 
%  P_estim      = error covariance matrix   
%

%Predicted (a priori) state estimate
x_estim = kf.A * kf.x; 

%Predicted (a priori) estimate covariance
P_estim = kf.A * kf.P * kf.A' + kf.Q; 


if isSensing
    % Optimal Kalman gain
    K = P_estim * kf.H' * (kf.H * P_estim * kf.H' + kf.R)^-1;
    
    % Updated (a posteriori) state estimate
    x_estim = x_estim + K * (kf.z - kf.H * x_estim);
         
    % Updated (a posteriori) estimate covariance
    I = eye(size(K,1), size(kf.H,2));
    P_estim = (I - K * kf.H) * P_estim; 
end

return