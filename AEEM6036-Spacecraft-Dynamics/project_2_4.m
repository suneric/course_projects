%--------------------------------------------------------------------------
% AEEM6036 Spacecraft Dynamics 
% Project 2 Satellite Attitude Control with Reaction Wheel System
% Author: Yufeng Sun
% Date: Nov 19, 2020
% problem 4: find the attitude matrices with respact to inertial frame
% by using TRIAD method and QUEST method
%--------------------------------------------------------------------------

%% sensor observation
bv_sun = [0.8 -0.5 0.2]';
bv_star = [-0.3 -0.1 0.9]';
nv_sun = [1 0 0]';
nv_star = [0 0 1]';
% using star track sensor as prime axis as it is more accurate
bv1 = bv_star / norm(bv_star);
bv2 = bv_sun / norm(bv_sun);
nv1 = nv_star / norm(nv_star);
nv2 = nv_sun / norm(nv_sun);

%% attitude matrix using TRIAD
% intermediate t frame respective to body frame
bt1 = bv1;
bt2 = cross(bv1, bv2) / norm(cross(bv1, bv2));
bt3 = cross(bt1, bt2);
C_BT = [bt1 bt2 bt3];
% intermediate t frame respective to inertial frame
nt1 = nv1;
nt2 = cross(nv1, nv2) / norm(cross(nv1, nv2));
nt3 = cross(nt1, nt2);
C_NT = [nt1 nt2 nt3];

C_BN_TRIAD = C_BT*C_NT';
disp('Estimated Attitude by TRIAD-method:');
disp(C_BN_TRIAD);

%% attitude matrix using QUEST
B = 2*bv1*nv1'+bv2*nv2';
S = B + B';
z = [B(2,3)-B(3,2) B(3,1)-B(1,3) B(1,2)-B(2,1)]';
sigma = trace(B);
global K
K = [sigma z';
    z S-sigma*eye(3)];
s = fsolve(@character,3);
rou = ((s+sigma)*eye(3)-S)^-1*z;

C_BN_QUEST = RP2DCM(rou);
disp('Estimated Attitude by QUEST-method:');
disp(C_BN_QUEST);

%% attitude difference 
C_BB = C_BN_TRIAD*C_BN_QUEST';
[phi,axis] = DCM2PRE(C_BB);
err = phi*(180/pi);
disp('Error difference with principal rotation angle:');
disp(err)

%% supporting functions

% QUEST method characteristic equation
function f = character(s)
global K
f = det(K-s*eye(4));
end

% priciple rotation elements from direct cosine angles
function [phi,axis] = DCM2PRE(dcm)
phi = acos(0.5*(trace(dcm)-1));
e1 = dcm(2,3)-dcm(3,2); 
e2 = dcm(3,1)-dcm(1,3);
e3 = dcm(1,2)-dcm(2,1);
axis = (1/(2*sin(phi)))*[e1 e2 e3]';
end

% Rodrigues Parameter to Direction cosine matrix
function C = RP2DCM(rpv)
r1 = rpv(1);
r2 = rpv(2);
r3 = rpv(3);
C = (1/(1+rpv'*rpv))*[1+r1^2-r2^2-r3^2 2*(r1*r2+r3) 2*(r1*r3-r2);
                        2*(r1*r2-r3) 1-r1^2+r2^2-r3^2 2*(r2*r3+r1);
                        2*(r1*r3+r2) 2*(r2*r3-r1) 1-r1^2-r2^2+r3^2];
end
