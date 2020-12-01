% control attitude by following
% a desired input of Thrust, phi, theta and yaw rate
function out=AttitudeController(in,P)
% desired input for attitude control [T;Phi;Theta;r]
err = in(1:4);
% 6 quad states
x = in(5:10);
% attitude control 
tau = PID(err(2:4),x);
%tau = LQR(err,P);
out = [err(1);tau];
end

function out=PID(err,x)
p = x(4);
q = x(5);
Kp = [90 85 55];
Kd = [3.0 2.0];
tau_phi = Kp(1)*err(1)-Kd(1)*p;
tau_theta = Kp(2)*err(2)-Kd(2)*q;
tau_psi = Kp(3)*err(3);
out = [tau_phi;tau_theta;tau_psi]; % force, torque (phi, theta, psi)
end

% function out = LQR(err,P)
% % state space description of the system
% A=[0 0 0 1 0 0;...
%    0 0 0 0 1 0;...
%    0 0 0 0 0 1;...
%    0 0 0 0 0 0;...
%    0 0 0 0 0 0;...
%    0 0 0 0 0 0];
% B = [0 0 0;...
%      0 0 0;...
%      0 0 0;...
%      1/P.Jx 0 0;...
%      0 1/P.Jy 0;...
%      0 0 1/P.Jz];
% % determin Q matrix 4by4
% Q=zeros(6); 
% Q(1,1) = 10;
% Q(2,2) = 10;
% Q(3,3) = 1;
% Q(4,4) = 0;
% Q(5,5) = 0;
% Q(6,6) = 10;
% R = zeros(3); 
% R(1,1) = 1/10;
% R(2,2) = 1/10;
% R(3,3) = 1/10;
% [K,~]=lqr(A,B,Q,R);
% out = K*[err(1);err(2);0;0;0;err(3)];
% end