function [angular_acceleration] = calcAngAccel(x, simpar)

true_steer_ang = x(simpar.states.ix.steer_ang);
true_velocity_xb = x(simpar.states.ix.velocity);

angular_acceleration = [0, 0, (true_velocity_xb/simpar.general.L) * tan(true_steer_ang)]';
