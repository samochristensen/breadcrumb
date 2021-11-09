function [acceleration] = calcAccel(x, acceleration_x, simpar)

input_acceleration = acceleration_x;
true_velocity_xb = x(simpar.states.ix.velocity);
L = simpar.general.L;
true_steer_ang = x(simpar.states.ix.steer_ang);

acceleration = [input_acceleration, ((true_velocity_xb^2)/L)*tan(true_steer_ang), -1*simpar.general.g]';
