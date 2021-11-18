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

accl_time_constant = simpar.general.tau_accel;
gyro_time_constant = simpar.general.tau_gyro;
 
est_accl_bias = xhat(simpar.states.ixf.accl_bias);
est_gyro_bias = xhat(simpar.states.ixf.gyro_bias);
gravity_i = [0; 0; simpar.general.g];  % NED, Coordinates

est_velocity = xhat(simpar.states.ixf.velocity);
est_attitude_i2b = xhat(simpar.states.ixf.attitude);
est_T_b2i = q2tmat(est_attitude_i2b)';


%% Compute individual elements of x_dot
% time derivative of position
dot_position = est_velocity;
% derivative of velocity - w/ Isaac's Revisions
conj_est_attitude = qConjugate(est_attitude_i2b);
accel_quat = [0; meas_accl - est_accl_bias;];
dot_vel_pre = qmult(accel_quat, qmult(accel_quat, conj_est_attitude)); 
dot_velocity = dot_vel_pre([2 3 4]) + gravity_i;

% derivative of attitude
attitude_quat = [0; meas_ang_accl - est_gyro_bias];
dot_attitude = 0.5 * qmult(attitude_quat, est_attitude_i2b); 

% derivative of accl bias
dot_accl_bias = (-1/accl_time_constant)*est_accl_bias;
% derovatove of gyro bias
dot_gyro_bias = (-1/gyro_time_constant)*est_gyro_bias;
% derivative of crumb pos
dot_crumb_pos = zeros(3,1);

%% Assign to output
xhatdot = [dot_position;
           dot_velocity;
           dot_attitude;
           dot_accl_bias;
           dot_gyro_bias;
           dot_crumb_pos;];
end
