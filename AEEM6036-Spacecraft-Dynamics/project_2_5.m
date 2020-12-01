%--------------------------------------------------------------------------
% AEEM6036 Spacecraft Dynamics 
% Project 2 Satellite Attitude Control with Reaction Wheel System
% Author: Yufeng Sun
% Date: Nov 19, 2020
% problem 5: Design a MRP-based control law to control the satellite
% attitude to zeros attitudes and angular velocity in 50 s
%--------------------------------------------------------------------------

%% parameters
% spacecraft inertial matrix
global J Jw D K
J = [500 0 0;
    0 400 0;
    0 0 440];
Jw = [0.1 0 0;
      0 0.1 0;
      0 0 0.1];
% initial satellite attitude
C = [0.8273 0.4674 -0.3115;
    -0.5193 0.8479 -0.1067;
    0.2142 0.25 0.9442];
s0 = DCM2MRP(C); % converted to MRP

% initial angular velocity of spacecraft
w0 = [0.031 -0.0259 0.4678]';
% initial wheel speed
hw0 = [0 0 550]'; 

d = 0.5526;
k = 0.3054;

% gain matrix
%D = d*J;
%K = k*J;

D = [276 0 0;
    0 221 0;
    0 0 243];
K = [153 0 0;
    0 122 0;
    0 0 134];

%% Quaternion Feeback Regulator
T = 50; % time 
init_y = zeros(9,1); % init value of angular velocity and quaternion
init_y(1:3) = w0;
init_y(4:6) = hw0;
init_y(7:9) = s0;
[t,ws,hws,ss,us] = Simulation([0,T],init_y);
draw_history(t,ws,hws,ss,us);

final_ws = ws(length(t),:);
norm(final_ws)
final_ss = ss(length(t),:);
norm(final_ss)

%% supporting functions
function draw_history(t,ws,hws,ss,us)
hold on
figure(1)
h1 = subplot(4,3,1);
plot(t,ss(:,1))
xlabel(h1, "TIME (SEC)");
ylabel(h1, "\sigma_1");
h2 = subplot(4,3,2);
plot(t,ss(:,2))
xlabel(h2, "TIME (SEC)");
ylabel(h2, "\sigma_2");
h3 = subplot(4,3,3);
plot(t,ss(:,3))
xlabel(h3, "TIME (SEC)");
ylabel(h3, "\sigma_3");

h4 = subplot(4,3,4);
plot(t,ws(:,1))
xlabel(h4, "TIME (SEC)");
ylabel(h4, "\omega_1 (RAD/SEC)");
h5 = subplot(4,3,5);
plot(t,ws(:,2))
xlabel(h5, "TIME (SEC)");
ylabel(h5, "\omega_2 (RAD/SEC)");
h6 = subplot(4,3,6);
plot(t,ws(:,3))
xlabel(h6, "TIME (SEC)");
ylabel(h6, "\omega_3 (RAD/SEC)");

h7 = subplot(4,3,7);
plot(t,hws(:,1))
xlabel(h7, "TIME (SEC)");
ylabel(h7, "\omega_w_1 (RPM)");
h8 = subplot(4,3,8);
plot(t,hws(:,2))
xlabel(h8, "TIME (SEC)");
ylabel(h8, "\omega_w_2 (RPM)");
h9 = subplot(4,3,9);
plot(t,hws(:,3))
xlabel(h9, "TIME (SEC)");
ylabel(h9, "\omega_w_3 (RPM)");

h10 = subplot(4,3,10);
plot(t,us(:,1))
xlabel(h10, "TIME (SEC)");
ylabel(h10, "U1 (Nm)");
h11= subplot(4,3,11);
plot(t,us(:,2))
xlabel(h11, "TIME (SEC)");
ylabel(h11, "U2 (Nm)");
h12 = subplot(4,3,12);
plot(t,us(:,3))
xlabel(h12, "TIME (SEC)");
ylabel(h12, "U3 (Nm)");

hold off
end


% simulation with a specificed K gain
function [t,ws,hws,ss,us] = Simulation(t_span,init_y)
global J Jw D K 
options = odeset('RelTol',1e-5);
[t,y] = ode45(@SpacecraftDynamics,t_span,init_y,options);
ws = zeros(length(t),3);
hws = zeros(length(t),3);
ss = zeros(length(t),3);
us = zeros(length(t),3);
for i = 1:length(t)
    w = y(i,1:3)';
    ws(i,1:3) = w;
    hw = y(i,4:6)';
    hws(i,1:3) = 9.5493*hw;
    s = y(i,7:9)';
    ss(i,1:3)=s;
    u = skew(w)*(J*w+Jw*hw)-D*w-K*s;
    us(i,1:3) = u;
end
end

function dy = SpacecraftDynamics(t, y)
w = y(1:3);
hw = y(4:6);
s = y(7:9);
global J Jw D K
% dynamics with a closed-loop control
u = skew(w)*(J*w+Jw*hw)-D*w-K*s;
dhw = Jw^-1*u;
dw = J^-1*(-skew(w)*(J*w+Jw*hw)+u);
ds = (1/4)*((1-s'*s)*eye(3)+2*skew(s)+2*(s*s'))*w;

dy = zeros(9,1);
dy(1:3)=dw;
dy(4:6)=dhw;
dy(7:9)=ds;
end

function s = DCM2MRP(C)
a = sqrt(1+trace(C));
b =  1/(a*(a+2));
s = [0 0 0]';
s(1) = b*(C(2,3)-C(3,2));
s(2) = b*(C(3,1)-C(1,3));
s(3) = b*(C(1,2)-C(2,1));
end
