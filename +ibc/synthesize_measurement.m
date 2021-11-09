function [ synthetic_measurement ] = synthesize_measurement( x_true, simpar )
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
    carrierFreq = simpar.general.f;
    speedOfLight = simpar.general.c;
    k = 2*pi*carrierFreq/speedOfLight;

    Inertial2Body_i = x_true(simpar.states.ix.position);
    theta_b2i = x_true(simpar.states.ix.heading);
    T_b2i = [cos(theta_b2i) -sin(theta_b2i) 0; sin(theta_b2i) cos(theta_b2i) 0; 0 0 1];
    Inertial2Crumb_i = x_true(simpar.states.ix.crumb_pos);
    Body2Antenna1_b = [simpar.general.r_body2antenna1_x;
                       simpar.general.r_body2antenna1_y;
                       simpar.general.r_body2antenna1_z;];
    Body2Antenna2_b = [simpar.general.r_body2antenna2_x;
                       simpar.general.r_body2antenna2_y;
                       simpar.general.r_body2antenna2_z;];
    
    d1 = norm([Inertial2Body_i; 0] + T_b2i*Body2Antenna1_b - Inertial2Crumb_i);
    d2 = norm([Inertial2Body_i; 0] + T_b2i*Body2Antenna2_b - Inertial2Crumb_i);
    
    phase1 = -k * d1;
    phase2 = -k * d2;
    nu = 0;

    synthetic_measurement = phase1 - phase2 + nu;

end
