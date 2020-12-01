%--------------------------------------------------------------------------
% AEEM6036 Spacecraft Dynamics 
% Assignment 5 Reproduce the simulation in paper "Quaternion Feedback
% Regulator for Spacecraft Eigenaxis Rotations"
% Author: Yufeng Sun
% Date: Nov 19, 2020
%--------------------------------------------------------------------------

%% parameters
% spacecraft inertial matrix
global J mu D
J = [1200 100 -200;
    100 2200 300;
    -200 300 3100];
% initial quaternion
q0 = [0.57 0.57 0.57 0.159]';
% initial angular velocity of spacecraft
w0 = [0.01 0.01 0.01]';

mu = 0.9;

% gain matrix
D = 0.316*[1200 0 0;
     0 2200 0;
     0 0 3100];
K1 = [201 0 0;
      0 110 0;
      0 0 78];
K2 = [110 0 0;
      0 110 0;
      0 0 110];
K3 = [72 0 0;
      0 110 0;
      0 0 204];
K4 = [60 0 0;
      0 110 0;
      0 0 155];
  
%% Quaternion Feeback Regulator
T = 100; % time 
init_y = zeros(7,1); % init value of angular velocity and quaternion
init_y(1:3) = w0;
init_y(4:7) = q0;
qd = [0 0 0 1]'; % desired quaternion
[t1,ws1,qs1,us1,es1] = Simulation([0,T],init_y, K1);
[t2,ws2,qs2,us2,es2] = Simulation([0,T],init_y, K2);
[t3,ws3,qs3,us3,es3] = Simulation([0,T],init_y, K3);
[t4,ws4,qs4,us4,es4] = Simulation([0,T],init_y, K4);
draw_history_q_and_u(t1,t2,t3,t4,qs1,qs2,qs3,qs4,us1,us2,us3,us4);
draw_history_w_and_qi(t1,t2,t3,t4,qs1,qs2,qs3,qs4,ws1,ws2,ws3,ws4,es1,es2,es3,es4);


%% supporting functions
function draw_history_w_and_qi(t1,t2,t3,t4,q1,q2,q3,q4,w1,w2,w3,w4,es1,es2,es3,es4)
hold on
figure(2)
h1 = subplot(4,2,1);
draw_subplot(h1,t1,t2,t3,t4,w1(:,1),w2(:,1),w3(:,1),w4(:,1),"W1 RAD/SEC");
h2 = subplot(4,2,2);
draw_subplot(h2,t1,t2,t3,t4,w1(:,2),w2(:,2),w3(:,2),w4(:,2),"W2 RAD/SEC");
h3 = subplot(4,2,3);
draw_subplot(h3,t1,t2,t3,t4,w1(:,3),w2(:,3),w3(:,3),w4(:,3),"W3 RAD/SEC");
h4 = subplot(4,2,4);
draw_subplot(h4,t1,t2,t3,t4,es1,es2,es3,es4,"EIGEN ANGLE DEG");
h5 = subplot(4,2,5);
draw_qi_plot(h5,q1,q2,q3,q4,1,2,"Q1","Q2");
h6 = subplot(4,2,6);
draw_qi_plot(h6,q1,q2,q3,q4,2,3,"Q2","Q3");
h7 = subplot(4,2,7);
draw_qi_plot(h7,q1,q2,q3,q4,1,3,"Q1","Q3");
legend("Case 1: K=kJ^-1","Case 2: K=kI","Case 3: K=(\alpha J+\beta I)^-1","Case 4: K=kJ");
hold off
end


% plot angular velocity
function draw_history_q_and_u(t1,t2,t3,t4,q1,q2,q3,q4,u1,u2,u3,u4)
hold on
figure(1)
h1 = subplot(4,2,1);
draw_subplot(h1,t1,t2,t3,t4,q1(:,1),q2(:,1),q3(:,1),q4(:,1),"Q1");
h2 = subplot(4,2,2);
draw_subplot(h2,t1,t2,t3,t4,q1(:,2),q2(:,2),q3(:,2),q4(:,2),"Q2");
h3 = subplot(4,2,3);
draw_subplot(h3,t1,t2,t3,t4,q1(:,3),q2(:,3),q3(:,3),q4(:,3),"Q3");
h4 = subplot(4,2,4);
draw_subplot(h4,t1,t2,t3,t4,q1(:,4),q2(:,4),q3(:,4),q4(:,4),"Q4");
h5 = subplot(4,2,5);
draw_subplot(h5,t1,t2,t3,t4,u1(:,1),u2(:,1),u3(:,1),u4(:,1),"U1 Nm");
h6 = subplot(4,2,6);
draw_subplot(h6,t1,t2,t3,t4,u1(:,2),u2(:,2),u3(:,2),u4(:,2),"U2 Nm");
h7 = subplot(4,2,7);
draw_subplot(h7,t1,t2,t3,t4,u1(:,3),u2(:,3),u3(:,3),u4(:,3),"U3 Nm");
legend("Case 1: K=kJ^-1","Case 2: K=kI","Case 3: K=(\alpha J+\beta I)^-1","Case 4: K=kJ");
hold off
end


function draw_subplot(h,t1,t2,t3,t4,v1,v2,v3,v4,y_label)
hold on
plot(t1, v1);
plot(t2, v2);
plot(t3, v3);
plot(t4, v4);
xlabel(h, "TIME (SEC)");
ylabel(h, y_label);
hold off
end

function draw_qi_plot(h,q1,q2,q3,q4,xi,yi,x_label,y_label)
hold on
plot(q1(:,xi), q1(:,yi));
plot(q2(:,xi), q2(:,yi));
plot(q3(:,xi), q3(:,yi));
plot(q4(:,xi), q4(:,yi));
xlabel(h, x_label);
ylabel(h, y_label);
hold off
end

% simulation with a specificed K gain
function [t,ws,qs,us,es] = Simulation(t_span,init_y,K_gain)
global K J D mu
K = K_gain;
[t,y] = ode45(@SpacecraftDynamics,t_span,init_y);
ws = zeros(length(t),3);
qs = zeros(length(t),4);
us = zeros(length(t),3);
es = zeros(length(t),1);
for i = 1:length(t)
    w = y(i,1:3)';
    ws(i,1:3) = w(1:3);
    q = y(i,4:7)';
    qs(i,1:4) = q(1:4);
    u = -mu*(-skew(w))*J*w-D*w-K*q(1:3);
    us(i,1:3) = u(1:3);
    es(i,1) = (2*acos(q(4)))*(180/pi);
end
end

function dy = SpacecraftDynamics(t, y)
w = y(1:3);
q = y(4:6);
q4 = y(7);

global J mu D K
% dynamics with a closed-loop control
% where the control torque is 
% u = -mu*skew(w)*J*w-D*w-K*q;
dw = J^-1*((1-mu)*(-skew(w))*J*w-D*w-K*q);
dq = 0.5*(-skew(w))*q+0.5*q4*w;
dq4 = -0.5*w'*q;

dy = zeros(7,1);
dy(1:3)=dw(1:3);
dy(4:6)=dq(1:3);
dy(7) = dq4;
end

