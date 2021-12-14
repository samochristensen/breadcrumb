function [ P ] = update_covariance( P_pre, K, H, R, G, simpar )
%update_covariance updates the covariance matrix
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
% [ output_args ] = update_covariance( input_args )
%
% See also FUNC1, FUNC2

% Author: Randy Christensen
% Date: 31-Aug-2020 16:02:09
% Reference: 
% Copyright 2020 Utah State University

%Don't forget to perform numerical checking and conditioning of covariance
%matrix
I = eye(simpar.states.nxfe);

P = (I - K*H)*P_pre*(I - K*H)' + K*G*R*G'*K';
end
