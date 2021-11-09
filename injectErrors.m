function [ xhat_err ] = injectErrors( xhat_true, dele, simpar )
%injectErrors injects errors into the state estimates
%
% Inputs:
%   xhat_true = true navigation state vector (mixed units)
%   delx = error state vector (mixed units)
%
% Outputs
%   xhat_err = estimated state vector(units)
%
% Example Usage
% [ xhat_err ] = injectErrors( xhat_true, delx )

% Author: Randy Christensen
% Date: 10-Dec-2018 11:54:42
% Reference: 
% Copyright 2018 Utah State University

%Get size of inputs
[~,m_x] = size(xhat_true);
[~, m_delx] = size(dele);
assert(m_x == m_delx);
%Inject errors
xhat_err = zeros(simpar.states.nxf,m_x);
%Indeces
ind_e = simpar.states.ixfe;
ind_n = simpar.states.ixf;

for i=1:m_x
    xhat_err(ind_n.position) = xhat_true(ind_n.position) - dele(ind_e.position);
    xhat_err(ind_n.velocity) = xhat_true(ind_n.velocity) - dele(ind_e.velocity);
    xhat_err(ind_n.attitude) = qmult([1; dele(ind_e.attitude)./-2], xhat_true(ind_n.attitude));
    xhat_err(ind_n.accl_bias) = xhat_true(ind_n.accl_bias) - dele(ind_e.accl_bias);
    xhat_err(ind_n.gyro_bias) = xhat_true(ind_n.gyro_bias) - dele(ind_e.gyro_bias);
    xhat_err(ind_n.crumb_pos) = xhat_true(ind_n.crumb_pos) - dele(ind_e.crumb_pos);
end
end
