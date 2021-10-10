function [quat] = euler2quat(tsai, theta, phi)
%EULER2QUAT Summary of this function goes here
%   Detailed explanation goes here

quat = [cos(phi/2)*cos(theta/2)*cos(tsai/2) + sin(phi/2)*sin(theta/2)*sin(tsai/2);...
        sin(phi/2)*cos(theta/2)*cos(tsai/2) - cos(phi/2)*sin(theta/2)*sin(tsai/2);...
        cos(phi/2)*sin(theta/2)*cos(tsai/2) + sin(phi/2)*cos(theta/2)*sin(tsai/2);...
        cos(phi/2)*cos(theta/2)*sin(tsai/2) - sin(phi/2)*sin(theta/2)*cos(tsai/2);];

end

