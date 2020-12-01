% project
% optimal control of uav for performing inspection
% 
clc, clear all

% Quadrotr Inertial Parameters
P.Jx = 0.114700;
P.Jy = 0.057600;
P.Jz = 0.171200;
P.m = 1.56;
P.g = 9.8;
% quadtrotor parameters
P.boxlength = 0.2;
P.armlength = 0.4;
P.armwidth = 0.01;
P.boxwidth = 0.2;
P.boxheight = 0.2;
P.ystart =0;
P.zstart =0;
P.armposition = 0;
P.perimeter = 0.05;
P.periwidth = 0.02;
%

% constants
global m g Ix Iy Iz uMat
m = 1.56; % kg
g = 9.81; % kg m/s^2
Ix = 0.114700; % kg m^2
Iy = 0.057600;
Iz = 0.171200;

kb = 3.8305e-6;
kd = 2.2518e-8;
l = 0.2;
uMat = [...
    kb,kb,kb,kb;
    0,-l*kb,0,l*kb;
    l*kb,0,-l*kb,0;
    -kd,kd,-kd,kd;
    ];

% boundary conditions
global x0 xT T num
x0 = [-2;2;-5;0;0;0;0;0;0;0;0;0];
xT = [2;-1;-2;0;0;0.5*pi;0;0;0;0;0;0];
T = 0.1;
num = 100;
xM = [0;2;-5;0;0;0;1;-1;1;0;0;0];

[x,n,u,o,t] = bypass(x0,xM,xT);
%l0 = costate_initial_guess();
%[x,n,u,o,t] = bvp_solve(x0,xT,0,100,l0);
plot_trajectory(t,x,P);
% plot_states(x,n,u,o,t);


function [x,n,u,o,t] = bypass(x0,xM,xT)
global T
l0 = [0;0;-1;0;0;0;1;1;0;0;0;0];
[x1,n1,u1,o1,t1] = bvp_solve(x0,xM,0,20,l0);
l1 = [0;0;-1;1;1;0;0.1;0.1;0;0;0;0];
[x2,n2,u2,o2,t2] = bvp_solve(xM,xT,20*T,50,l1);
x = [x1,x2];
n = [n1,n2];
u = [u1,u2];
o = [o1,o2];
t = [t1,t2];
end

%--------------------------------------------------------------------------
% initial guess of costate
function l = costate_initial_guess()
lx = 0;
ly = 0;
lz = -1;
lphi = 0;
ltheta = 0;
lpsi = 0;
lxd = 0.5;
lyd = 0.5;
lzd = 0;
lphid = 0;
lthetad = 0;
lpsid = 0;
l = [lx;ly;lz;lphi;ltheta;lpsi;lxd;lyd;lzd;lphid;lthetad;lpsid];
end

% initial guess for (T=10s)
% x0 = [0;0;0;0;0;0;0;0;0;0;0;0];
% xT = [0;0;-5;0;0;0.5*pi;0;0;0;0;0;0];
% lx = 1;
% ly = 1;
% lz = 1;
% lphi = 0.1;
% ltheta = 0.1;
% lpsi = 1;
% lxd = 1;
% lyd = 1;
% lzd = 10;
% lphid = 0;
% lthetad = 0;
% lpsid = 0;

% initial guess for (T=10s)
% x0 = [0;0;0;0;0;0;0;0;0;0;0;0];
% xT = [0;0;-5;0;0;0;0;0;0;0;0;0];
% lx = 1;
% ly = 1;
% lz = 1;
% lphi = 1;
% ltheta = 1;
% lpsi = 0;
% lxd = 0;
% lyd = 0;
% lzd = 1;
% lphid = 0;
% lthetad = 0;
% lpsid = 0;

%-------------------------------------------------------------------------
% solve by fsolve
% function [x,n,u,t] = bvp_fsolve()
% global x0 T num
% l0 = costate_initial_guess();
% options = optimoptions('fsolve','Display','iter','StepTolerance',1e-10);
% l0 = fsolve(@trajectory_err,l0,options)
% s0 = [x0;l0];
% tspan = linspace(0,T*num,num);
% [t,s] = ode45(@bvp_ode,tspan,s0);
% for i=1:length(t)
%     x(:,i) = s(i,1:12);
%     n(:,i) = s(i,13:24);
%     u(:,i) = optimal_control(s(i,:));
% end
% end
% 
% function err = trajectory_err(l0)
% global x0 xT T num
% tspan = linspace(0,T*num,num);
% s0 = [x0;l0];
% [t,S] = ode45(@bvp_ode,tspan,s0);
% sT = S(length(t),1:12);
% err = sT'-xT
% end

%-------------------------------------------------------------------------
% solve by bvp4c
% start inspection point
% end inspection point
% ts: starting time
% nT: number of time T
% l0: initial guess
function [x,n,u,o,t] = bvp_solve(xa,xb,ts,nT,l0)
global x0 xT T num uMat
x0 = xa;
xT = xb;
num = nT;
solinit.x = linspace(0,T*num,num);
solinit.y = [
    x0(1)*ones(1,num);
    x0(2)*ones(1,num);
    x0(3)*ones(1,num);
    x0(4)*ones(1,num);
    x0(5)*ones(1,num);
    x0(6)*ones(1,num);
    x0(7)*ones(1,num);
    x0(8)*ones(1,num);
    x0(9)*ones(1,num);
    x0(10)*ones(1,num);
    x0(11)*ones(1,num);
    x0(12)*ones(1,num);
    l0(1)*ones(1,num);
    l0(2)*ones(1,num);
    l0(3)*ones(1,num);
    l0(4)*ones(1,num);
    l0(5)*ones(1,num);
    l0(6)*ones(1,num);
    l0(7)*ones(1,num);
    l0(8)*ones(1,num);
    l0(9)*ones(1,num);
    l0(10)*ones(1,num);
    l0(11)*ones(1,num);
    l0(12)*ones(1,num)
    ];
options = bvpset('Stats','on','RelTol',1e-3,'Nmax',2000);
sol = bvp5c(@bvp_ode,@bvp_bc,solinit,options);
t = sol.x;
for i=1:length(t)
    t(i) = t(i)+ts;
    x(:,i) = sol.y(1:12,i);
    n(:,i) = sol.y(13:24,i);
    u(:,i) = optimal_control(sol.y(:,i));
    c = uMat^(-1)*u(:,i);
    o(1,i) = sqrt(c(1));
    o(2,i) = sqrt(c(2));
    o(3,i) = sqrt(c(3));
    o(4,i) = sqrt(c(4));
end
end

function res = bvp_bc(ya,yb)
global x0 xT
b1 = ya(1:12)-x0;
b2 = yb(1:12)-xT;
res = [b1;b2];
end

%-------------------------------------------------------------------------------------------------------------------------------------------------------------
% ode functions for quadrotor dynamics with states and cotates
% s = [states;costates]
function sd = bvp_ode(t,s)
u = optimal_control(s);
xd = motion_state(s,u);
ld = motion_costate(s,u);
sd = [xd;ld];
end

% optimal control according to necessary condition of Hamiltonian equation
function u = optimal_control(s)
global m Ix Iy Iz
lv = s(21);
lp = s(22);
lq = s(23);
lr = s(24);
u1 = lv/m;
u2 = -lp/Ix;
u3 = -lq/Iy;
u4 = -lr/Iz;
u = [u1;u2;u3;u4];
end

function xd = motion_state(s,F)
global m g Ix Iy Iz
x = s(1);
y = s(2);
z = s(3);
phi = s(4);
theta = s(5);
psi = s(6);
u = s(7);
v = s(8);
w = s(9);
p = s(10);
q = s(11);
r = s(12);

% transformation
R_roll = [...
         1, 0, 0;...
         0, cos(phi), sin(phi);...
         0, -sin(phi), cos(phi)
         ];
R_pitch = [...
          cos(theta), 0, -sin(theta);...
          0, 1, 0;...
          sin(theta), 0, cos(theta)];
R_yaw = [...
          cos(psi), sin(psi), 0;...
          -sin(psi), cos(psi), 0;...
          0, 0, 1];
R = R_roll*R_pitch*R_yaw;  
R = R';
% differential equations
xyzdot = R*[u;v;w];
angdot = [...
    1 sin(phi)*tan(theta) cos(phi)*tan(theta);...
    0 cos(phi) -sin(phi);...
    0 sin(phi)/cos(theta) cos(phi)/cos(theta);...
    ]*[p;q;r];
uvwdot = [r*v-q*w;p*w-r*u;q*u-p*v]+g*[-sin(theta); cos(theta)*sin(phi); cos(theta)*cos(phi)]+[0; 0; -F(1)/m];
pqrdot = [...
    ((Iy-Iz)/Ix)*q*r;...
    ((Iz-Ix)/Iy)*p*r;...
    ((Ix-Iy)/Iz)*p*q;...
    ]+[F(2)/Ix;F(3)/Iy;F(4)/Iz];
xd = [xyzdot;angdot;uvwdot;pqrdot];

% x_d = u*cos(theta)*cos(psi)+v*(sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))+w*(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi));
% y_d = u*cos(theta)*sin(psi)+v*(sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))+w*(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi));
% z_d = -u*sin(theta)+v*sin(phi)*cos(theta)+w*cos(phi)*cos(theta);
% phi_d = p+q*sin(phi)*tan(theta)+r*cos(phi)*tan(theta);
% theta_d = q*cos(phi)-r*sin(phi);
% psi_d = q*sin(phi)/cos(theta)+r*cos(phi)/cos(theta);
% u_d = r*v-q*w-g*sin(theta);
% v_d = p*w-r*u+g*cos(theta)*sin(phi);
% w_d = q*u-p*v-F(1)/m+g*cos(theta)*cos(phi);
% p_d = (F(2)+q*r*(Iy-Iz))/Ix;
% q_d = (F(3)+p*r*(Iz-Ix))/Iy;
% r_d = (F(4)+p*q*(Ix-Iy))/Iz;
% xd = [x_d;y_d;z_d;phi_d;theta_d;psi_d;u_d;v_d;w_d;p_d;q_d;r_d];
end

function ld = motion_costate(s,F)
global g Ix Iy Iz
% state
x = s(1);
y = s(2);
z = s(3);
phi = s(4);
theta = s(5);
psi = s(6);
u = s(7);
v = s(8);
w = s(9);
p = s(10);
q = s(11);
r = s(12);
% costate
lx = s(13);
ly = s(14);
lz = s(15);
lphi = s(16);
ltheta = s(17);
lpsi = s(18);
lu = s(19);
lv = s(20);
lw = s(21);
lp = s(22);
lq = s(23);
lr = s(24);

lx_d = 0;
ly_d = 0;
lz_d = 0;
lphi_d = -lx*(v*(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))+w*(-sin(phi)*sin(theta)*cos(psi)+cos(phi)*sin(psi)))...
         -ly*(v*(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))+w*(-sin(phi)*sin(theta)*sin(psi)-cos(phi)*cos(psi)))...
         -lz*(v*cos(phi)*cos(theta)-w*sin(phi)*cos(theta))...
         -lphi*(q*cos(phi)*tan(theta)-r*sin(phi)*tan(theta))...
         -ltheta*(-q*sin(phi)-r*cos(phi))...
         -lpsi*(q*cos(phi)/cos(theta)-r*sin(phi)/cos(theta))...
         -lv*g*cos(theta)*cos(phi)...
         +lw*g*cos(theta)*sin(phi);
ltheta_d = -lx*(-u*sin(theta)*cos(psi)+v*sin(phi)*cos(theta)*cos(psi)+w*cos(phi)*cos(theta)*cos(psi))...
           -ly*(-u*sin(theta)*sin(psi)+v*sin(phi)*cos(theta)*sin(psi)+w*cos(phi)*cos(theta)*sin(psi))...
           -lz*(-u*cos(theta)-v*sin(phi)*sin(theta)-w*cos(phi)*sin(theta))...
           -lphi*(q*sin(phi)/cos(theta)^2+r*cos(phi)/cos(theta)^2)...
           -lpsi*(q*sin(phi)*sin(theta)/cos(theta)^2+r*cos(phi)*sin(theta)/cos(theta)^2)...
           +lu*g*cos(theta)...
           +lv*g*sin(theta)*sin(phi)...
           +lw*g*sin(theta)*cos(phi);
lpsi_d = -lx*(-u*cos(theta)*sin(psi)-v*(sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))-w*(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)))...
          -ly*(u*cos(theta)*cos(psi)+v*(sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))+w*(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)));
lu_d = -lx*cos(theta)*cos(psi)-ly*cos(theta)*sin(psi)+lz*sin(theta)+lv*r-lw*q;
lv_d = -lx*(sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))...
       -ly*(sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))...
       -lz*sin(phi)*cos(theta)...
       -lu*r...
       +lw*p;
lw_d = -lx*(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))...
       -ly*(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))...
       -lz*cos(phi)*cos(theta)...
       +lu*q...
       -lv*p;
lp_d = -lphi-lv*w+lw*v-lq*(r*(Iz-Ix)/Iy)-lr*(q*(Ix-Iy)/Iz);
lq_d = -lphi*sin(phi)*tan(theta)-ltheta*cos(phi)-lpsi*sin(phi)/cos(theta)+lu*w-lw*u-lp*(r*(Iz-Ix)/Iy)-lr*(p*(Ix-Iy)/Iz);
lr_d = -lphi*cos(phi)*tan(theta)+ltheta*sin(phi)-lpsi*cos(phi)/cos(theta)-lu*v+lv*u-lp*(q*(Iz-Ix)/Iy)-lq*(p*(Ix-Iy)/Iz);
ld = [lx_d;ly_d;lz_d;lphi_d;ltheta_d;lpsi_d;lu_d;lv_d;lw_d;lp_d;lq_d;lr_d];
end
%-------------------------------------------------------------------------------------------------------------------------------------------------------------

%-------------------------------------------------------------------------------------------------------------------------------------------------------------
% plot functions
function plot_states(x,n,u,o,t)
figure(2)
hold on
plot(t,x(1,:),'LineWidth',2);
plot(t,x(2,:),'LineWidth',2);
plot(t,-x(3,:),'LineWidth',2);
plot(t,x(4,:),'LineWidth',2);
plot(t,x(5,:),'LineWidth',2);
plot(t,x(6,:),'LineWidth',2);
plot(t,x(7,:),'LineWidth',2);
plot(t,x(8,:),'LineWidth',2);
plot(t,x(9,:),'LineWidth',2);
plot(t,x(10,:),'LineWidth',2);
plot(t,x(11,:),'LineWidth',2);
plot(t,x(12,:),'LineWidth',2);
legend('x(t)','y(t)','z(t)','phi(t)','theta(t)','psi(t)','u(t)','v(t)','w(t)','p(t)','q(t)','r(t)');
xlabel('time');
ylabel('state');
title('states');
hold off

figure(3)
hold on
plot(t,n(1,:),'LineWidth',2);
plot(t,n(2,:),'LineWidth',2);
plot(t,n(3,:),'LineWidth',2);
plot(t,n(4,:),'LineWidth',2);
plot(t,n(5,:),'LineWidth',2);
plot(t,n(6,:),'LineWidth',2);
plot(t,n(7,:),'LineWidth',2);
plot(t,n(8,:),'LineWidth',2);
plot(t,n(9,:),'LineWidth',2);
plot(t,n(10,:),'LineWidth',2);
plot(t,n(11,:),'LineWidth',2);
plot(t,n(12,:),'LineWidth',2);
legend('lamda_x(t)','lamda_y(t)','lamda_z(t)','lamda_p_h_i(t)','lamda_t_h_e_t_a(t)','lamda_p_s_i(t)','lamda_u(t)','lamda_v(t)','lamda_w(t)','lamda_p(t)','lamda_q(t)','lamda_r(t)');
title('co-states');
xlabel('time');
ylabel('costate');
hold off

figure(4)
hold on
plot(t,u(1,:),'LineWidth',2);
plot(t,u(2,:),'LineWidth',2);
plot(t,u(3,:),'LineWidth',2);
plot(t,u(4,:),'LineWidth',2);
legend('F_t(t)','T_p_h_i(t)','T_t_h_e_t_a(t)','T_p_s_i(t)');
title('control inputs');
xlabel('time');
ylabel('control');
hold off

figure(5)
hold on
plot(t,o(1,:),'LineWidth',2);
plot(t,o(2,:),'LineWidth',2);
plot(t,o(3,:),'LineWidth',2);
plot(t,o(4,:),'LineWidth',2);
legend('omega_1(t)','omega_2(t)','omega_3(t)','omega_4(t)');
title('motor inputs');
xlabel('time');
ylabel('motor speed');
hold off

figure(6)
hold on
J = zeros(1,length(t));
J(1) = 0.5*(u(1,1)^2+u(2,1)^2+u(3,1)^2+u(4,1)^2);
for i=2:length(t)
    J(i) = J(i-1)+0.5*(u(1,i)^2+u(2,i)^2+u(3,i)^2+u(4,i)^2);
end
plot(t,J,'LineWidth',2);
xlabel('time');
ylabel('peformance Index J_0');
hold off

end

function plot_trajectory(t,x,P)
drawQuad(zeros(1,13),P)
for i=1:length(t)
    s = [x(:,i);i];
    drawQuad(s,P);
    pause(0.05);
end
% figure(4)
% hold on
% axis([-10,10,-10,10,0,10])
% view(-44,22);
% grid on
% plot3(x(1,:),x(2,:),-x(3,:))
% xlabel('x');
% ylabel('y');
% zlabel('z');
% hold off
end
%-------------------------------------------------------------------------------------------------------------------------------------------------------------



