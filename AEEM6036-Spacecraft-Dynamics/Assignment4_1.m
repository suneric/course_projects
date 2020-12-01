%--------------------------------------------------------------------------
% AEEM6036 Assignment 4
% Description:
% Find the attitude control matrix between body frame and inertial frame
% with multiple sensing vector
% Author: Yufeng Sun
% Date: Oct 29, 2020
%--------------------------------------------------------------------------

%% sensored 4 unit vector in body frame and inertial frame
bv1 = [0.8273 0.5541 -0.0920];
bv2 = [-0.8285 0.5522 -0.0955];
bv3 = [0.2155 0.5522 0.8022];
bv4 = [0.5570 -0.7442 -0.2884];
nv1 = [-0.1517 -0.9669 0.2050];
nv2 = [-0.8393 0.4494 -0.3044];
nv3 = [-0.0886 -0.5856 -0.8000];
nv4 = [0.8814 -0.0303 0.5202];

%% 4-1 TRIAD Method
C_BN12 = TRIAD(bv1,bv2,nv1,nv2);
C_BN13 = TRIAD(bv1,bv3,nv1,nv3);
C_BN14 = TRIAD(bv1,bv4,nv1,nv4);
C_Ref = (C_BN12+C_BN13+C_BN14)/3;
err_1 = Accuracy(C_BN12, C_Ref);
err_2 = Accuracy(C_BN13, C_Ref);
err_3 = Accuracy(C_BN14, C_Ref);
disp('Estimated Attitude by TRIAD method using v1 and v2:');
disp(C_BN12);
disp('Error Principal Angle:');
disp(err_1)
disp('Estimated Attitude by TRIAD method using v1 and v3:');
disp(C_BN13);
disp('Error Principal Angle:');
disp(err_2)
disp('Estimated Attitude by TRIAD method using v1 and v4:');
disp(C_BN14);
disp('Error Principal Angle:');
disp(err_3)
disp('The least accurate measurement is k=4 as the error principle angle is the largest');
%% 4-2 Davenport's q-method
B = 2*bv1'*nv1+bv2'*nv2+bv3'*nv3;
S = B + B';
Z = [B(2,3)-B(3,2) B(3,1)-B(1,3), B(1,2)-B(2,1)]';
sigma = trace(B);
K = [sigma Z';
    Z S-sigma*eye(3)];
[V,D] = eigs(K,1); % the lagest eigen value of K and corresponding vector
C_BN_q = EP2DCM(V);
err_q = Accuracy(C_BN_q, C_BN12);
disp('Estimated Attitude by q-method:');
disp(C_BN_q);
disp('Error Principal Angle:');
disp(err_q)

%% 4-2 QUEST method
s0 = 2+1+1;
s = fsolve(@CharacterEqn,s0);
rou_quest = ((s+sigma)*eye(3)-S)^-1*Z;
C_BN_quest = RP2DCM(rou_quest);
err_quest = Accuracy(C_BN_quest, C_BN12);
disp('Estimated Attitude by QUEST-method:');
disp(C_BN_quest);
disp('Error Principal Angle:');
disp(err_quest)


%% 4-2 OLAE method
d1 = bv1-nv1;
d2 = bv2-nv2;
d3 = bv3-nv3;
D = [d1';d2';d3'];
s1 = bv1+nv1;
s2 = bv2+nv2;
s3 = bv3+nv3;
SK = [Skew(s1);Skew(s2);Skew(s3)];
W = eye(9);
for i=1:3
    W(i,i)=2;
end
rou_olae = (SK'*W*SK)^-1*SK'*W*D;
C_BN_olae = RP2DCM(rou_olae);
err_olae = Accuracy(C_BN_olae, C_BN12);
disp('Estimated Attitude by OLAE-method:');
disp(C_BN_olae);
disp('Error Principal Angle:');
disp(err_olae)

%% support functions

% skew matrix of a vector
function mat = Skew(vec)
x = vec(1);
y = vec(2);
z = vec(3);
mat = [0 -z y;
       z 0 -x;
       -y x 0];
end

%
% TRIAD method for computing the matrix between body frame and inertial
% frame with given 2 pairs of unit vector measured by sensors
%
function C_BN = TRIAD( bv1, bv2, nv1, nv2)
% intermediate t frame respective to body frame
bt1 = bv1 / norm( bv1 );
bt2 = cross( bv1, bv2 ) / norm( cross( bv1, bv2 ) );
bt3 = cross( bt1, bt2 ) / norm( cross( bt1, bt2 ) );
C_BT = [ bt1' bt2' bt3' ];

% intermediate t frame respective to inertial frame
nt1 = nv1 / norm( nv1 );
nt2 = cross( nv1, nv2 ) / norm( cross( nv1, nv2 ) );
nt3 = cross( nt1, nt2 ) / norm( cross( nt1, nt2 ) );
C_NT = [ nt1' nt2' nt3' ];

C_BN = C_BT*C_NT';
end

% characteristic equation used in QUEST method
function f = CharacterEqn(s)
bv1 = [0.8273 0.5541 -0.0920];
bv2 = [-0.8285 0.5522 -0.0955];
bv3 = [0.2155 0.5522 0.8022];
nv1 = [-0.1517 -0.9669 0.2050];
nv2 = [-0.8393 0.4494 -0.3044];
nv3 = [-0.0886 -0.5856 -0.8000];
B = 2*bv1'*nv1+bv2'*nv2+bv3'*nv3;
S = B + B';
Z = [B(2,3)-B(3,2) B(3,1)-B(1,3), B(1,2)-B(2,1)]';
sigma = trace(B);
K = [sigma Z';
    Z S-sigma*eye(3)];
f = det(K-s*eye(4));
end

% Measure the attitude matrix accuracy
function err = Accuracy(C_BN, C_BN_Ref)
C_BB = C_BN*C_BN_Ref';
[phi,axis] = DCM2PRE(C_BB);
err = phi;
end

% priciple rotation elements from direct cosine angles
function [phi,axis] = DCM2PRE(dcm)
phi = acos(0.5*(trace(dcm)-1));
e1 = dcm(2,3)-dcm(3,2); 
e2 = dcm(3,1)-dcm(1,3);
e3 = dcm(1,2)-dcm(2,1);
axis = (1/(2*sin(phi)))*[e1 e2 e3]';
end

function mat = EP2DCM(epv)
b0 = epv(1);
b1 = epv(2);
b2 = epv(3);
b3 = epv(4);
mat = [b0^2+b1^2-b2^2-b3^2 2*(b1*b2+b0*b3) 2*(b1*b3-b0*b2);
       2*(b1*b2-b0*b3) b0^2-b1^2+b2^2-b3^2 2*(b2*b3+b0*b1);
       2*(b1*b3+b0*b2) 2*(b2*b3-b0*b1) b0^2-b1^2-b2^2+b3^2]; 
end

function mat = RP2DCM(rpv)
r1 = rpv(1);
r2 = rpv(2);
r3 = rpv(3);
mat = (1/(1+rpv'*rpv))*[1+r1^2-r2^2-r3^2 2*(r1*r2+r3) 2*(r1*r3-r2);
                        2*(r1*r2-r3) 1-r1^2+r2^2-r3^2 2*(r2*r3+r1);
                        2*(r1*r3+r2) 2*(r2*r3-r1) 1-r1^2-r2^2+r3^2];
end
