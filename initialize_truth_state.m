function [ x ] = initialize_truth_state(simpar)
%initialize_truth_state initialize the truth state vector consistent with
%the initial covariance matrix
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
% [ output_args ] = initialize_truth_state( input_args )

% Author: 
% Date: 31-Aug-2020 15:46:59

% In the initialization of the truth and navigation states, you need only
% ensure that the estimation error is consistent with the initial
% covariance matrix.  One realistic way to do this is to set the true 
% vehicle states to the same thing every time, and randomize any sensor 
% parameters.
lengthTruthVector = simpar.states.nx;
ixindex = simpar.states.ix;

x = zeros(lengthTruthVector,1); % initialize the truth state vector with 0s
x(ixindex.pos_E) = simpar.general.ic.r_Ei; % init pos, inert frame, east dim
x(ixindex.pos_N) = simpar.general.ic.r_Ni; % init pos, inert frame, north dim
x(ixindex.vel_yb) = simpar.general.ic.v_yb; % init vel, body frame, north dim
x(ixindex.head_angle) = simpar.general.ic.head_angle; % init heading angle
x(ixindex.st_angle) = simpar.general.ic.st_angle; % init steering angle
x(ixindex.abias) = [simpar.general.ic.abias_x; ... % init accel bias x
                    simpar.general.ic.abias_y; ... % init accel bias y
                    simpar.general.ic.abias_z;];   % init accel bias z
x(ixindex.gbias) = [simpar.general.ic.gbias_x; ... % init gyro bias x
                    simpar.general.ic.gbias_y; ... % init gyro bias y
                    simpar.general.ic.gbias_z;];   % init gyro bias z
x(ixindex.cpos) = [simpar.general.ic.rc_x; ...     % init IBC pos, inert frame, east?
                   simpar.general.ic.rc_y; ...     % init IBC pos, inert frame, nort?
                   simpar.general.ic.rc_z;];       % init IBC pos, inert frame, up?
end
