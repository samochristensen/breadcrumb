function [ B ] = calc_B(xhat, simpar )

%% Unpack the inputs
q_hat_i2b = xhat(simpar.states.ixf.att);
T_hat_b2i = q2tmat(q_hat_i2b)';

%% Compute B
B = zeros(simpar.states.nxfe, 4);
B([4:6],[1:3]) = -T_hat_b2i;
B([7:9],[4:6]) = -eye(3);
B([10:12],[7:9]) = eye(3);
B([13:15],[10:12]) = eye(3);
end
