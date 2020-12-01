%--------------------------------------------------------------------------
% AEEM6036 Assignment 3-1
% Description:
% give a 3-1-3 euler angles, find
% 1) principle rotation elements
% 2) euler parameters
% 3) classical rodrigues parameters
% 4) modified rodrigues parameters
% Author: Yufeng Sun
% Date: Oct 10, 2020
%--------------------------------------------------------------------------

eas = (pi/180)*[-30 40 20];
disp('3-1-3 euler angles: (radians)');
disp(eas);

dcm = DCM313(eas);
disp('direct cosine matrix:');
disp(dcm);

[phi, axis] = DCM2PRE(dcm);
disp('principle rotation elements: (phi or phi-2*pi, degree), axis)');
disp('choose the one in [0,180] for less maneuver');
disp(phi*180/pi);
disp(phi*180/pi-360);

disp(axis);

epv = PRE2EPV(phi,axis);
disp('euler parameter vector: (beta0,beta1,beta2,beta3)');
disp(epv);

rps = EPV2RPS(epv);
disp('rodrigues parameter vector: (rough1,rough2,rough3)');
disp(rps);

mrps = RPS2MRPS(epv);
disp('modified rodrigues parameter vector: (sigma1,sigma2,sigma3)');
disp(rps);

% modified rodrigues parameters from euler parameters
function mrps = RPS2MRPS(epv)
mrps = zeros(3,1);
mrps(:,1) = epv(2:4,1)/(1+epv(1,1));
end

% rodrigues parameters from euler parameter
function rps = EPV2RPS(epv)
rps = zeros(3,1);
rps(:,1) = epv(2:4,1)/epv(1,1);
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

% Direct Cosine Matrix
function mat = DCM313(eas)
mat = R3(eas(3))*R1(eas(2))*R3(eas(1));
end

% rotation matrix about axis 1
function mat = R1(ea)
mat = [1 0 0;
       0 cos(ea) sin(ea);
       0 -sin(ea) cos(ea)];
end

% rotation matrix about axis 3
function mat = R3(ea)
mat = [cos(ea) sin(ea) 0;
       -sin(ea) cos(ea) 0;
       0 0 1]; 
end