function [omega] = calc_omega(x, simpar)

phi = x(simpar.states.ix.st_angle); % index of steering angle, to find current true steering angle
vy = x(simpar.states.ix.vel_yb); % index of vel_N in the body frame
L = simpar.general.L; % length of the bicycle model
omega =[0; 0; vy/L*tan(phi)]; % rotation of the vehicle, in one dimension, based on steer_angle (etc.)
end
