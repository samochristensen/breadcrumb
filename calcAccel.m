function [acceleration] = calcAccel(x, acceleration_x, simpar)
%calcAccel synthesizes the acceleration that would be measured given the
%steering angle, velocity, length of the vehicle, and DESIRED acceleration
%
% Inputs:
%   Input1 = description (units)
%   Input2 = description (units)
%
% Outputs
%   Output1 = description (units)
%   Output2 = description (units)
%
% Example Usage
% [ output_args ] = contInertialMeas( input_args )

% Author: 
% Date: 31-Aug-2020 15:46:59
% Reference: 
% Copyright 2020 Utah State University

input_acceleration = acceleration_x;
true_velocity_xb = x(simpar.states.ix.velocity);
L = simpar.general.L;
true_steer_ang = x(simpar.states.ix.steer_ang);

% my original: acceleration = [input_acceleration, ((true_velocity_xb^2)/L)*tan(true_steer_ang), -1*simpar.general.g]';
acceleration = [-1*true_velocity_xb^2 * tan(true_steer_ang)/L; input_acceleration; 0];
