%--------------------------------------------------------------------------
% AEEM6036 Assignment 2-2
% Description:
% Integrate yaw, pitch, roll angles over a simulation time of 1 minutes
% with give a initial yaw pitch and roll angles of a vehicle are (40,30,80)
% degree and the angular velocity vector of the vehicle in body frame is
% omega = 20 (sin(0.1t),0.01, cos(0.1t)) deg/s
% Author: Yufeng Sun
% Date: Sep 30, 2020
%--------------------------------------------------------------------------

%% execute integration 
init_eas = (pi/180)*[40;30;80];
[t, eas] = Simulation(60,init_eas);
print_euler_angles(t,(180/pi)*eas)

%--------------------------------------------------------------------------
% print the euler angles in one figure
function print_euler_angles(t,eas)
hold on
figure(1)
title('Euler Angles');
xlabel('time (s)');
ylabel('angle (degree)');
plot(t,eas(:,1));
plot(t,eas(:,2));
plot(t,eas(:,3));
legend('\psi','\theta','\phi');
hold off
end

% simulation with ode45 in time span [0,T] with 
% an initial value of euler angles
function [t, eas] = Simulation(T,init)
[t,eas] = ode45(@EAVelocity,[0,T],init);
end

% compute the euler angle velocities with the 
% angular velocity define in body frame 
function dydt = EAVelocity(t,y)
omega = (pi/180)*20*[sin(0.1*t);0.01;cos(0.1*t)];
theta = y(2);
phi = y(3);
mat = [0 sin(phi) cos(phi);
       0 cos(phi)*cos(theta) -sin(phi)*cos(theta);
       cos(theta) sin(phi)*sin(theta) cos(phi)*sin(theta)];
dydt = (1/cos(theta))*mat*omega;
end