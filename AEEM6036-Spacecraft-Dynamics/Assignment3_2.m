%--------------------------------------------------------------------------
% AEEM6036 Assignment 3-2
% Description:
% evaluate attitude of spacecraft relative to space station given the 
% transforms of spacecraft and space sation relative to an inertial frame
% Author: Yufeng Sun
% Date: Oct 10, 2020
%--------------------------------------------------------------------------

% DCM of spacecraft in the fixed inertial frame
eas = (pi/180)*[230 70 103];
C1 = DCM321(eas);
disp('direct cosine matrix of spacecraft in the inertial frame:');
disp(C1);

% DCM of space station in the fixed inertial frame
epv = [0.328474 -0.437966 0.801059 -0.242062];
C2 = EP2DCM(epv);
disp('direct cosine matrix of space station in the inertial frame:');
disp(C2);

% DCM of spacecraft in the space station frame
C = C1*C2^(-1);
disp('direct cosine matrix of space station in the space station frame:');
disp(C2);

[phi,axis] = PRE(C);
disp('principle rotation elements: (phi (degree), axis)');
disp(phi*180/pi);
disp(axis);


% priciple rotation elements from direct cosine angles
function [phi, axis] = PRE(dcm)
phi = acos(0.5*(trace(dcm)-1));
e1 = dcm(2,3)-dcm(3,2); 
e2 = dcm(3,1)-dcm(1,3);
e3 = dcm(1,2)-dcm(2,1);
axis = (1/(2*sin(phi)))*[e1 e2 e3]';
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