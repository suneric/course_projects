%--------------------------------------------------------------------------
% AEEM6036 Assignment 3-3
% Description:
% Give a initial eular angle of a vehicle and the angular velocity in body
% frame, intergrate the euler parameter over a simulation time 1 minutes
% Author: Yufeng Sun
% Date: Oct 10, 2020
%--------------------------------------------------------------------------


%% execute integration 
init_eas = (pi/180)*[40 30 80]';
[t,eps,epsc,eas] = Simulation(60,init_eas);
print_eps(t,eps);
print_eps_constraint(t,epsc);
print_eas(t,(180/pi)*eas);

%--------------------------------------------------------------------------
% print the euler prameters 
function print_eps(t,eps)
hold on
figure(1)
title('Euler Parameters');
xlabel('time (s)');
ylabel('value');
plot(t,eps(:,1));
plot(t,eps(:,2));
plot(t,eps(:,3));
plot(t,eps(:,4));
legend('\beta_0','\beta_1','\beta_2','\beta_3');
hold off
end

%--------------------------------------------------------------------------
% print the euler prameters constraint 
function print_eps_constraint(t,epsc)
hold on
figure(2)
title('Euler Parameters Constraint');
xlabel('time (s)');
ylabel('value');
plot(t,epsc);
hold off
end


%--------------------------------------------------------------------------
% print the euler angles 
function print_eas(t,eas)
hold on
figure(3)
title('Euler Angles');
xlabel('time (s)');
ylabel('angle (degrees)');
plot(t,eas(1,:));
plot(t,eas(2,:));
plot(t,eas(3,:));
legend('\psi','\theta','\phi');
hold off
end

% simulation with ode45 in time span [0,T] with 
% an initial value of euler angles
function [t,eps,epsc,eas] = Simulation(T,init_eas)
init_eps = EAS2EPV(init_eas);
[t,eps] = ode45(@EulerParamKDE,[0,T],init_eps);
epsc = zeros(1,length(t));
eas = zeros(3,length(t));
for i=1:length(t)
    epsc(1,i) = eps(i,1)^2+eps(i,2)^2+eps(i,3)^2+eps(i,4)^2;
    eas(1:3,i) = EPS2EAS(eps(i,1:4));
end
end

% compute the euler angle velocities with the 
% angular velocity define in body frame 
function dydt = EulerParamKDE(t,y)
omega = (pi/180)*20*[sin(0.1*t);0.01;cos(0.1*t)];
b0 = y(1);
b1 = y(2);
b2 = y(3);
b3 = y(4);
mat = [-b1 -b2 -b3;
       b0 -b3 b2;
       b3 b0 -b1;
       -b2 b1 b0];
dydt = 0.5*mat*omega;
end

% Euler Angles from Euler Parameters
function eas = EPS2EAS(epv)
eas = [0;0;0];
dcm = EP2DCM(epv);
eas(1) = atan2(dcm(1,2),dcm(1,1));
eas(2) = asin(dcm(1,3));
eas(3) = atan2(dcm(2,3),dcm(3,3));
end

% Direct Cosine Matrix from Euler parameter
function mat = EP2DCM(epv)
b0 = epv(1);
b1 = epv(2);
b2 = epv(3);
b3 = epv(4);
mat = [b0^2+b1^2-b2^2-b3^2 2*(b1*b2+b0*b3) 2*(b1*b3-b0*b2);
       2*(b1*b2-b0*b3) b0^2-b1^2+b2^2-b3^2 2*(b2*b3+b0*b1);
       2*(b1*b3+b0*b2) 2*(b2*b3-b0*b1) b0^2-b1^2-b2^2+b3^2]; 
end

% euler parameters from euler angles
function epv = EAS2EPV(eas)
dcm = DCM321(eas);
[phi, axis] = DCM2PRE(dcm);
epv = PRE2EPV(phi,axis);
end

% euler parameter vector from principle rotation elements
function epv = PRE2EPV(phi,axis)
epv = zeros(4,1);
epv(1,1) = cos(phi/2);
epv(2:4,1) = sin(phi/2)*axis;
end

% priciple rotation elements from direct cosine angles
function [phi, axis] = DCM2PRE(dcm)
phi = acos(0.5*(trace(dcm)-1));
e1 = dcm(2,3)-dcm(3,2); 
e2 = dcm(3,1)-dcm(1,3);
e3 = dcm(1,2)-dcm(2,1);
axis = (1/(2*sin(phi)))*[e1 e2 e3]';
end

% Direct Cosine Matrix from euler angles
function mat = DCM321(eas)
mat = R1(eas(3))*R2(eas(2))*R3(eas(1));
end

% rotation matrix about axis 1
function mat = R1(ea)
mat = [1 0 0;
       0 cos(ea) sin(ea);
       0 -sin(ea) cos(ea)];
end
% rotation matrix about axis 2
function mat = R2(ea)
mat = [cos(ea) 0 -sin(ea);
       0 1 0;
       sin(ea) 0 cos(ea)];
end
% rotation matrix about axis 3
function mat = R3(ea)
mat = [cos(ea) sin(ea) 0;
       -sin(ea) cos(ea) 0;
       0 0 1]; 
end