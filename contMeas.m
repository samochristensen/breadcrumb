function [ y_tilde ] = contMeas(x, acceleration_x, simpar)
%contInertialMeas synthesizes noise measurements used to propagate the
%navigation state
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

accl_bias = x(simpar.states.ix.accl_bias);
gyro_bias = x(simpar.states.ix.gyro_bias);
grav_body = [0; 0; -simpar.general.g];

%TODO: Implement accelerometer noise here
accel_noise = zeros(3,1);
%TODO: Implement gyroscope noise here
gyro_noise = zeros(3,1);  

acceleration = calcAccel(x, acceleration_x, simpar);
angular_accel = calcAngAccel(x, simpar);

measured_acceleration = acceleration - grav_body + accl_bias + accel_noise;
measured_angular_accel = angular_accel + gyro_bias + gyro_noise;

y_tilde = [measured_acceleration; measured_angular_accel];      

end
