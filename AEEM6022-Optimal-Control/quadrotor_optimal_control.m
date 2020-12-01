% project
% optimal control of uav for performing inspection
% with simplifed model by assuming small rolling and pitch angles
clc, clear all

% constants
global m g Ix Iy Iz
m = 1.56; % kg
g = 9.81; % kg m/s^2
Ix = 0.114700; % kg m^2
Iy = 0.057600;
Iz = 0.171200;

% boundary conditions
global x0 xT T num
x0 = [0;0;0;0;0;0;0;0;0;0;0;0];
xT = [0;0;-5;0;0;0;0;0;0;0;0;0];
T = 0.1;
num = 200;
[x,n,u,t] = bvp_fsolve();
plot_states(x,n,u,t);
plot_trajectory(x);

%--------------------------------------------------------------------------
% initial guess of costate
function l = costate_initial_guess()
lx = 1;
ly = 1;
lz = 1;
lphi = 0;
ltheta = 0;
lpsi = 0;
lxd = 0;
lyd = 0;
lzd = 0;
lphid = 1;
lthetad = 1;
lpsid = 1;
l = [lx;ly;lz;lphi;ltheta;lpsi;lxd;lyd;lzd;lphid;lthetad;lpsid];
end

%------------------------------------------------------------------------
function [x,n,u,t] = bvp_fsolve()
global x0 T num
tspan = linspace(0,T*num,num);
l0 = costate_initial_guess();
options = optimoptions('fsolve','Display','iter');
l0 = fsolve(@trajectory_err,l0,options)
s0 = [x0;l0];
[t,s] = ode45(@bvp_ode,tspan,s0);
for i=1:length(t)
    x(:,i) = s(i,1:12);
    n(:,i) = s(i,13:24);
    u(:,i) = optimal_control(s(i,:));
end
end

%--------------------------------------------------------------------------
% trajectory error
function err = trajectory_err(l0)
global x0 xT T num
tspan = linspace(0,T*num,num);
s0 = [x0;l0];
[t,S] = ode45(@bvp_ode,tspan,s0);
sT = S(length(t),1:12);
err = sT'-xT
end

%-------------------------------------------------------------------------------------------------------------------------------------------------------------
% ode functions for quadrotor dynamics with states and cotates
% s = [states;costates]
function sd = bvp_ode(t,s)
x = s(1:12);
n = s(13:24);
u = optimal_control(x,n);
xd = motion_state(x,u);
nd = motion_costate(x,n,u);
sd = [xd;nd];
end

% optimal control according to necessary condition of Hamiltonian equation
function u = optimal_control(x,n)
global m Ix Iy Iz
u1 = (1/m)*( n(7)*(cos(x(4))*sin(x(5))*cos(x(6))+sin(x(4))*sin(x(6))) + n(8)*(cos(x(4))*sin(x(5))*sin(x(6))-sin(x(4))*cos(x(6))) + n(9)*cos(x(4))*cos(x(5)) );
u2 = -n(10)/Ix;
u3 = -n(11)/Iy;
u4 = -n(12)/Iz;
% the input is saturated
u = [u1;u2;u3;u4];
end

function xd = motion_state(x,u)
global m g Ix Iy Iz
x1_dot = x(7);
x2_dot = x(8);
x3_dot = x(9);
x4_dot = x(10);
x5_dot = x(11);
x6_dot = x(12);
x7_dot = -(u(1)/m)*( cos(x(4))*sin(x(5))*cos(x(6))+sin(x(4))*sin(x(6)) );
x8_dot = -(u(1)/m)*( cos(x(4))*sin(x(5))*sin(x(6))-sin(x(4))*cos(x(6)) );
x9_dot = g - (u(1)/m)*cos(x(4))*cos(x(5));
x10_dot = u(2)/Ix;
x11_dot = u(3)/Iy;
x12_dot = u(4)/Iz;
xd = [x1_dot;x2_dot;x3_dot;x4_dot;x5_dot;x6_dot;x7_dot;x8_dot;x9_dot;x10_dot;x11_dot;x12_dot];
end

function nd = motion_costate(x,n,u)
global m
n1_dot = 0;
n2_dot = 0;
n3_dot = 0;
n4_dot = (u(1)/m)*( n(7)*(-sin(x(4))*sin(x(5))*cos(x(6))+cos(x(4))*sin(x(6))) + n(8)*(-sin(x(4))*sin(x(5))*sin(x(6))-cos(x(4))*cos(x(6)))+ n(9)*( -sin(x(4))*cos(x(5))) );
n5_dot = (u(1)/m)*( n(7)*cos(x(4))*cos(x(5))*cos(x(6)) + n(8)*cos(x(4))*cos(x(5))*sin(x(6)) - n(9)*cos(x(4))*sin(x(5)) );
n6_dot = (u(1)/m)*( n(7)*(-cos(x(4))*sin(x(5))*sin(x(6))+sin(x(4))*cos(x(6))) + n(8)*(cos(x(4))*sin(x(5))*cos(x(6))+sin(x(4))*sin(x(6))) );
n7_dot = -n(1);
n8_dot = -n(2);
n9_dot = -n(3);
n10_dot = -n(4);
n11_dot = -n(5);
n12_dot = -n(6);
nd = [n1_dot;n2_dot;n3_dot;n4_dot;n5_dot;n6_dot;n7_dot;n8_dot;n9_dot;n10_dot;n11_dot;n12_dot];
end
%-------------------------------------------------------------------------------------------------------------------------------------------------------------

%-------------------------------------------------------------------------------------------------------------------------------------------------------------
% plot functions
function plot_states(x,n,u,t)
figure(1)
hold on
plot(t,x(1,:));
plot(t,x(2,:));
plot(t,-x(3,:));
plot(t,x(4,:));
plot(t,x(5,:));
plot(t,x(6,:));
plot(t,x(7,:));
plot(t,x(8,:));
plot(t,x(9,:));
plot(t,x(10,:));
plot(t,x(11,:));
plot(t,x(12,:));
legend('x_1(t)','x_2(t)','x_3(t)','x_4(t)','x_5(t)','x_6(t)','x_7(t)','x_8(t)','x_9(t)','x_1_0(t)','x_1_1(t)','x_1_2(t)');
xlabel('time');
ylabel('state');
title('states');
hold off

figure(2)
hold on
plot(t,n(1,:));
plot(t,n(2,:));
plot(t,n(3,:));
plot(t,n(4,:));
plot(t,n(5,:));
plot(t,n(6,:));
plot(t,n(7,:));
plot(t,n(8,:));
plot(t,n(9,:));
plot(t,n(10,:));
plot(t,n(11,:));
plot(t,n(12,:));
legend('lamda_1(t)','lamda_2(t)','lamda_3(t)','lamda_4(t)','lamda_5(t)','lamda_6(t)','lamda_7(t)','lamda_8(t)','lamda_9(t)','lamda_1_0(t)','lamda_1_1(t)','lamda_1_2(t)');
title('co-states');
xlabel('time');
ylabel('costate');
hold off

figure(3)
hold on
plot(t,u(1,:));
plot(t,u(2,:));
plot(t,u(3,:));
plot(t,u(4,:));
legend('F(t)','T_p_h_i(t)','T_t_h_e_t_a(t)','T_p_s_i(t)');
title('control inputs');
xlabel('time');
ylabel('control');
hold off

end

function plot_trajectory(x)
figure(4)
hold on
axis([-10,10,-10,10,0,10])
view(-44,22);
grid on
plot3(x(1,:),x(2,:),-x(3,:))
xlabel('x');
ylabel('y');
zlabel('z');
hold off
end
%-------------------------------------------------------------------------------------------------------------------------------------------------------------
