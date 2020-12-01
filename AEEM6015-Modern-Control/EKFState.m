function out=EKFState(in,P)
% sensor data with noise
phi = in(1);
theta = in(2);
p = in(3);
q = in(4);
r = in(5);
T = in(6);
rho = in(7:6+P.N);
t = in(7+P.N);
% estimate pn,pe,pd,vn,ve,vd and psi

persistent xhat; % esitimate states
persistent Phat; % variance estimated

R=P.R;
Q=P.Q;

if t<P.Ts
    xhat=zeros(7,1); % 1x7
    Phat=eye(7,7); %7x7
end

Tout=P.Ts/5;

% Prediction
N=10; %
for i=1:N
    pndot=xhat(4);
    pedot=xhat(5);
    pddot=xhat(6);
    psihat=xhat(7);
    vndot=(-T/P.m)*(cos(phi)*sin(theta)*cos(psihat)+sin(phi)*sin(psihat));
    vedot=(-T/P.m)*(cos(phi)*sin(theta)*sin(psihat)-sin(phi)*cos(psihat));
    vddot=(-T/P.m)*cos(phi)*cos(theta)+P.g;
    psidot=p*(sin(phi)/cos(theta))+r*(cos(phi)/cos(theta));
    xdot=[pndot;pedot;pddot;vndot;vedot;vddot;psidot];
    xhat=xhat+(Tout/N)*xdot;
    % Jocobian A=pf(x,u)/px
    A=[...
        0 0 0 1 0 0 0;...
        0 0 0 0 1 0 0;...
        0 0 0 0 0 1 0;...
        0 0 0 0 0 0 (-T/P.m)*(-cos(phi)*sin(theta)*sin(psihat)+sin(phi)*cos(psihat));...
        0 0 0 0 0 0 (-T/P.m)*(cos(phi)*sin(theta)*cos(psihat)+sin(phi)*sin(psihat));...
        0 0 0 0 0 0 0;...
        0 0 0 0 0 0 0];
    Phat=Phat+(Tout/N)*(A*Phat+Phat*A'+Q);
end
% update with measurment
for j=1:P.N
    pnhat=xhat(1);
    pehat=xhat(2);
    pdhat=xhat(3);
    pn1 = P.Xf(1,j);
    pe1 = P.Xf(2,j);
    pd1 = 0;
    % estimate range
    rho_e=((pnhat-pn1)^2+(pehat-pe1)^2+(pdhat-pd1)^2)^0.5;
    Crho = [(pnhat-pn1)/rho_e (pehat-pe1)/rho_e (pdhat-pd1)/rho_e 0 0 0 0];
    L=Phat*Crho'*(R+Crho*Phat*Crho')^-1;
    Phat=(eye(7)-L*Crho)*Phat;
    xhat=xhat+L*(rho(j)-rho_e);
end
out=[xhat(1:6);phi;theta;xhat(7);p;q;r];
end

% function out=RotationMatrix(phi,theta,psi)
% % inertial frame to body frame transformation
% R_roll = [...
%           1, 0, 0;...
%           0, cos(phi), sin(phi);...
%           0, -sin(phi), cos(phi)];
% R_pitch = [...
%           cos(theta), 0, -sin(theta);...
%           0, 1, 0;...
%           sin(theta), 0, cos(theta)];
% R_yaw = [...
%           cos(psi), sin(psi), 0;...
%           -sin(psi), cos(psi), 0;...
%           0, 0, 1];
% R = R_roll*R_pitch*R_yaw;  
% % body frame to inertial frame
% out = R';
% end