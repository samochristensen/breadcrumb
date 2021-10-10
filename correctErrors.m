function [ x_hat_c ] = correctErrors( x_hat, dele, simpar)
%correctState corrects the state vector given an estimated error state
%vector
%
% Inputs:
%   x_hat = estimated state vector (mixed units)
%   delx = error state vector (mixed units)
%
% Outputs
%   x = description (units)
%
% Example Usage
% [ x ] = correctState( x_hat, delx )

% Author: Randy Christensen
% Date: 10-Dec-2018 11:44:28
% Reference:
% Copyright 2018 Utah State University

%Get size of input and verify that it is a single vector
[~,m_x] = size(x_hat);
[~, m_delx] = size(dele);
assert(m_x == m_delx);
x_hat_c = nan(simpar.states.nxf,m_x);

%index
ind_t = simpar.states.ixf;
ind_e = simpar.states.ixfe;

%Correct errors
for i=1:m_x
    x_hat_c(ind_t.position) = x_hat(ind_t.position) + dele(ind_e.position);
    x_hat_c(ind_t.velocity) = x_hat(ind_t.velocity) + dele(ind_e.velocity);
    x_hat_c(ind_t.attitude) = qmult([1; dele(ind_e.attitude)./2], x_hat(ind_t.attitude));
    x_hat_c(ind_t.accel_bias) = x_hat(ind_t.accel_bias) + dele(ind_e.accel_bias);
    x_hat_c(ind_t.gyro_bias) = x_hat(ind_t.gyro_bias) + dele(ind_e.gyro_bias);
    x_hat_c(ind_t.crumb_pos) = x_hat(ind_t.crumb_pos) + dele(ind_e.crumb_pos);
end
end
