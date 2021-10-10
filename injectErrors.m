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
for i=1:m_x
    xhat_err(1:6,i) = [xhat_true(1:6,i) - dele(1:6,i)];
    xhat_err(7:10,i) = qmult(xhat_true(7:10,i),[1; dele(7:9,i)]);
    xhat_err(11:end,i) = [xhat_err(11:end,i) - dele(10:end,i)];
end
end
