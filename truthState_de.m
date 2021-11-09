function xdot = truthState_de( x, input)
%truthState_de computes the derivative of the truth state
%
% Inputs:
%   x = truth state (mixed units)
%   u = input (mixed units)
%
% Outputs
%   xdot = truth state derivative (mixed units)
%
% Example Usage
% xdot = truthState_de( x, input)

% Author: Randy Christensen
% Date: 21-May-2019 10:40:10
% Reference: none
% Copyright 2019 Utah State University

%% Unpack the inputs
simpar = input.simpar;
%inputs
true_acceleration_x = input.acceleration_x;
true_steer_ang_rate = input.steer_ang_rate;
%states
true_velocity_xb = x(simpar.states.ix.velocity);
true_heading = x(simpar.states.ix.heading);
true_steer_ang = x(simpar.states.ix.steer_ang);
L = 1;

true_accl_bias = x(simpar.states.ix.accl_bias);
true_gyro_bias = x(simpar.states.ix.gyro_bias);
accl_time_constant = simpar.general.tau_accel;
gyro_time_constant = simpar.general.tau_gyro;


%TODO: Implement Noise
true_accl_noise = zeros(3,1);
true_gyro_noise = zeros(3,1);

%% Compute individual elements of x_dot
dot_true_position_x = true_velocity_xb * cos(true_heading);
dot_true_position_y = true_velocity_xb * sin(true_heading);
dot_true_velocity_x = true_acceleration_x;
dot_true_heading = (true_velocity_xb/L)*tan(true_steer_ang);
dot_true_steer_ang = true_steer_ang_rate;
dot_accl_bias = (-1/accl_time_constant)*true_accl_bias + true_accl_noise;
dot_gyro_bias = (-1/gyro_time_constant)*true_gyro_bias + true_gyro_noise;
dot_crumb_pos = zeros(3,1);


%% Assign to output
xdot = [dot_true_position_x;...
        dot_true_position_y;...
        dot_true_velocity_x;...
        dot_true_heading;...
        dot_true_steer_ang;...
        dot_accl_bias;...
        dot_gyro_bias;...
        dot_crumb_pos;];
end