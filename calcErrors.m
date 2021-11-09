function [ dele ] = calcErrors( xhat, x, simpar )
%calcErrors computes estimation errors
%
% Inputs:
%   x_hat = estimated state vector(mixed units)
%   x = state vector (mixed units)
%
% Outputs
%   dele = estimation error state vector (mixed units)
%
% Example Usage
% [ dele ] = calcErrors( x_hat, x )

% Author: Randy Christensen
% Date: 21-May-2019 13:43:16
% Reference:
% Copyright 2019 Utah State University

%Get size of input and verify that it is a single vector
[~,m_x] = size(x);
[~, m_xhat] = size(xhat);
assert(m_x == m_xhat);
xhat_t = truth2nav(x, simpar);
dele = nan(simpar.states.nxfe,m_x);

%index
ind_t = simpar.states.ixf;
ind_e = simpar.states.ixfe;

%Calculate errors
for i=1:m_x
    %dele = x - xhat
    dele(ind_e.position) = xhat_t(ind_t.position) - xhat(ind_t.position);
    dele(ind_e.velocity) = xhat_t(ind_t.velocity) - xhat(ind_t.velocity);
    q1 = xhat_t(ind_t.attitude);
    q2 = xhat(ind_t.attitude);
    dele(ind_e.attitude) = quat2eul(qmult(q1,qConjugate(q2))')';
    dele(ind_e.accl_bias) = xhat_t(ind_t.accl_bias) - xhat(ind_t.accl_bias);
    dele(ind_e.gyro_bias) = xhat_t(ind_t.gyro_bias) - xhat(ind_t.gyro_bias);
    dele(ind_e.crumb_pos) = xhat_t(ind_t.crumb_pos) - xhat(ind_t.crumb_pos);
end
end
