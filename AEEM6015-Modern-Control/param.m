%==========================================================================
%parameter file
%
%01/16/2015: Last modified by Rajikant Sharma
%==========================================================================
close all;
clc;
  

%% Quadrotr Inertial Parameters
P.Jx = 0.114700;
P.Jy = 0.057600;
P.Jz = 0.171200;
% gravity (m/s^2)

%% mass (kg)
P.m = 1.56;
P.g = 9.8;
%% Initial Conditions: Use these inital values to initialize your s-function
P.x0=[...
    0;...pn0
    0;...pe0
    0;...pd0
    0;...u0
    0;...v0
    0;...w0
    0;...phi0
    0;...theta0
    0;...psi0
    0;...p0
    0;...q0
    0];  %r0

P.N = 10;
P.Xf=randn(2,P.N)*10; % landmark for range sensor
P.Q=eye(7,7);
P.Q(1,1)=0.1^2;
P.Q(2,2)=0.1^2;
P.Q(3,3)=0.1^2;
P.Q(4,4)=0.1^2;
P.Q(5,5)=0.1^2;
P.Q(6,6)=0.1^2;
P.Q(7,7)=0.1^2;

P.Ts = 0.1; %s
P.R = 0.1^2;