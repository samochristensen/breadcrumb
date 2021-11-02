function [ x ] = truth2nav( x_t, simpar )
%truth2nav maps the truth state vector to the navigation state vector
%
% Inputs:
%   x_t = truth state (mixed units)
%
% Outputs
%   x = navigation state (mixed units)
%
% Example Usage
% [ xhat ] = truth2nav( x )

% Author: Randy Christensen
% Date: 21-May-2019 14:17:45
% Reference: 
% Copyright 2019 Utah State University
indx = simpar.states.ix;
m
x = x_t([indx.position indx.velocity indx.attitude indx.accel_bias,...
     indx.gyro_bias indx.crumb_pos]);
end
