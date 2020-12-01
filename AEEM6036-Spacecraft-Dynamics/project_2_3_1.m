%--------------------------------------------------------------------------
% AEEM6036 Spacecraft Dynamics 
% Project 2 Satellite Attitude Control with Reaction Wheel System
% Author: Yufeng Sun
% Date: Nov 19, 2020
% problem 3-1: plot the time response of the angular velocity and the 
% nutation angle with 
% J=[500 0 0;
%    0 400 -7;
%    0 -7 440]
%--------------------------------------------------------------------------


%% parameters
% initial satellite angular velocity
omega0 = [0.523598775 0 0]'; % rad/s (5 rpm)
% initial wheel momentum
hw0 = [0 0 0]'; 
% teminal time
T = 5000;
% nominal wheel momentum
hn = 55;
% const wheel control
global d_hw
d_hw = [0 0 hn/T]';

%% 3-1
[t,omegas,betas]=Simulation([0,T],[omega0;hw0]);
draw_time_response(t,omegas,betas);
draw_3D(omegas);
omegas(length(t),:)

%%
% assuming no external torque, the system angular momentum conserved
function [t,omegas,betas] = Simulation(t_span,init_y)
[t,y] = ode45(@SpacecraftDynamics,t_span,init_y);
omegas = zeros(length(t),3);
betas = zeros(length(t),1);
for i = 1:length(t)
    omega = y(i,1:3);
    omegas(i,1:3)=omega;
    betas(i,1) = nutation_angle(omega,[0 0 1]);
end
end

% nutation angle
function beta = nutation_angle(v1,v2)
beta = (180/pi)*acos(dot(v1,v2)/(norm(v1)*norm(v2)));
end

% plot angular velocity
function draw_time_response(t,omegas,betas)
hold on
figure(1)
h1 = subplot(4,1,1);
plot(t, omegas(:,1));
xlabel(h1, "TIME (SEC)");
ylabel(h1, "W1 (RAD/SEC)");

h2 = subplot(4,1,2);
plot(t, omegas(:,2));
xlabel(h2, "TIME (SEC)");
ylabel(h2, "W2 (RAD/SEC)");

h3 = subplot(4,1,3);
plot(t, omegas(:,3));
xlabel(h3, "TIME (SEC)");
ylabel(h3, "W3 (RAD/SEC)");

h4 = subplot(4,1,4);
plot(t,betas);
xlabel(h4, "TIME (SEC)");
ylabel(h4, "\beta (DEG)");
hold off
end

function draw_3D(omegas)
hold on
figure(2)
axis([-10,10,-10,10,-10,10]);
plot3(omegas(:,1),omegas(:,2),omegas(:,3));
xlabel("W1");
ylabel("W2");
zlabel("W3");
title("Angular Velocity");
hold off
end

% according to euler's rotational equation of motion
% assuming no external toque, the system angular momentum conserved
function dy = SpacecraftDynamics(t,y)
% spacecraft inertia matrix
J = [500 0 0;
    0 400 -7;
    0 -7 440];
omega = y(1:3);
hw = y(4:6);
% const wheel control
global d_hw
% dh = J*d_omega + d_hw + skew(omega)*(J*omega+hw) = 0
d_omega = (J^-1)*(-skew(omega)*(J*omega+hw)-d_hw);
dy = zeros(6,1);
dy(1:3) = d_omega;
dy(4:6) = d_hw;
end


 
