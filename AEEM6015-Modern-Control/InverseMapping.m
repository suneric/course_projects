% find desired [T;phi;theta;yaw_rate] with desired input [up;upsi]
% and the measured pitch rate : q
function out=InverseMapping(in,P)
u = in(1:4);
x = in(5:10);
psi = x(3);
q = x(5);
% inverse mapping to find T, phi, theta, yaw_rate
u_p = u(1:3);
u_r = u(4);
m = P.m;
% desired Thrust
T_d = m*sqrt(u_p'*u_p);
% body frame to vechile 1 frame
R_yaw = [cos(psi), sin(psi), 0;...
          -sin(psi), cos(psi), 0;...
          0, 0, 1];
% define z for calculating desired phi, theta and yaw rate
z = R_yaw*u_p*(m/(-T_d));
phi_d = asin(-z(2));
theta_d = atan(z(1)/z(3));
r_d = u_r*cos(theta_d)*cos(phi_d)-q*sin(phi_d); % yaw_rate

out = [T_d;phi_d;theta_d;r_d];
end