function [ z_tilde ] = synthesize_measurement(x, simpar, R_ibc)
%synthesize_measurement_example synthesizes the discrete measurement
%corrupted by noise
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
% [ output_args ] = synthesize_measurement_example( input_args )
%
% See also FUNC1, FUNC2

% Author: Randy Christensen
% Date: 31-Aug-2020 16:00:48
% Reference: 
% Copyright 2020 Utah State University

% Unpack variables
r1_b = [simpar.general.r1_x; simpar.general.r1_y; simpar.general.r1_z]; ...
    % Sensing coil 1 in body frame
r2_b = [simpar.general.r2_x; simpar.general.r2_y; simpar.general.r2_z]; ...
    % Sensing coil 2 in body frame
r_c = x(simpar.states.ixf.cpos); % Ground circuit position in inertial frame
r_v = x(simpar.states.ixf.pos); % Vehicle position in inertial frame
q = x(simpar.states.ixf.att); % Attitude quaternion of vehicle wrt inertial frame
q_conj = qConjugate(q);
f = simpar.general.f; % Carrier frequency [hz]
c = simpar.general.c; % Speed of light in air
var = R_ibc;


% Calculate d_1 and d_2 (distances from sensing coils to ground circuit)
r1_quat = [0; r1_b];
r1_pre = qmult(q_conj, qmult(r1_quat, q));
d_1 = norm(r_c - r_v - r1_pre([2 3 4]));

r2_quat = [0; r2_b];
r2_pre = qmult(q_conj, qmult(r2_quat, q));
d_2 = norm(r_c - r_v - r2_pre([2 3 4]));

%Synthesize noise
v_pdoa = sqrt(var)*randn(1,1);

% Calculate z_tilde based on measurement model
z_tilde = (2*pi*f/c)*(d_2-d_1) + v_pdoa;
end
