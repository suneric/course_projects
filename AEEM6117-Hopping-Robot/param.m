close all
clc

P.m = 100; %kg
P.g = 9.81/6; % N/kg

% swift springs metric coilover springs
% https://www.kamispeed.com/products/swift-65mm-id-4-length-metric-coilover-springs?variant=14182995099692
% diameters = 60mm - 70mm
% spring length = 127 mm - 280 mm 
% spring rates from 3kg/mm - 34kg/mm

% energy, 
% 
P.k = 24000*9.81; % kg/m * N/kg = N/m  [7000lb]
P.l = 0.34; % m
P.l0 = 0.24; % m initial state of the spring
P.th = (45/180)*pi; %[0~pi/2]
P.x0 = [P.l0*sin(P.th);0;P.l0*cos(P.th);0]; %x xdot y ydot
P.sc = 2; % number of spring 
P.th2 = (0/180)*pi;