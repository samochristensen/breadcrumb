function xhatdot = navState_de(xhat,input)
%navState_de computes the derivative of the nav state
%
% Inputs:
%   xhat = nav state (mixed units)
%   input = input (mixed units)
%
% Outputs
%   xhatdot = nav state derivative (mixed units)
%
% Example Usage
% xhatdot = navState_de(xhat,input)

% Author: Randy Christensen
% Date: 21-May-2019 10:40:10
% Reference: none
% Copyright 2019 Utah State University

%% Unpack the inputs
simpar = input.simpar;
meas_accl = input.measured_accl;
meas_ang_accl = input.measured_ang_accl;

est_velocity = xhat(simpar.states.ixf.velocity);
est_attitude_i2b = xhat(simpar.states.ixf.attitude);
est_T_b2i = q2tmat(est_attitude_i2b)';
 
est_accl_bias = xhat(simpar.states.ixf.accl_bias);
gravity_i = [0; 0; -1*simpar.general.g];

est_gyro_bias = xhat(simpar.states.ixf.gyro_bias);
accl_time_constant = simpar.general.tau_accel;
gyro_time_constant = simpar.general.tau_gyro;

%% Compute individual elements of x_dot
dot_position = est_velocity;
dot_velocity = est_T_b2i*(meas_accl - est_accl_bias) + gravity_i;
dot_attitude = 0.5 * qmult([0; meas_ang_accl - est_gyro_bias], est_attitude_i2b); 
dot_accl_bias = (-1/accl_time_constant)*est_accl_bias;
dot_gyro_bias = (-1/gyro_time_constant)*est_gyro_bias;
dot_crumb_pos = zeros(3,1);

%% Assign to output
xhatdot = [dot_position;
           dot_velocity;
           dot_attitude;
           dot_accl_bias;
           dot_gyro_bias;
           dot_crumb_pos;];
end
