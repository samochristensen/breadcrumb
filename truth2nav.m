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
% x = x_t([indx.position, ...
%          indx.velocity, ...
%          indx.heading, ...
%          indx.steer_ang, ...
%          indx.accl_bias, ...
%          indx.gyro_bias, ...
%          indx.crumb_pos]);

% Plane model? Assumes that 2D plane

true_position = [x_t([indx.position])', 0]';
true_velocity = [x_t([indx.velocity])', 0, 0]';
true_orientation = eul2quat([0, 0, x_t([indx.heading])], "XYZ")';
true_accl_bias = x_t([indx.accl_bias]);
true_gyro_bias = x_t([indx.gyro_bias]);
true_crumb_pos = x_t([indx.crumb_pos]);

x = [true_position;...
     true_velocity;...
     true_orientation;...
     true_accl_bias;...
     true_gyro_bias;...
     true_crumb_pos];

end
