function out=Trajectory(in)
t = in;

%T = 5; % for trajectory 1;
T = 10; % trajectory 2 and 3;
w = 2*pi/T;

% trajectory 1
%out = generate_trajectory(t, 1.5, 0.75, 0, -0.75, w, 0.5*w, w);
% trajectory 2
%out = generate_trajectory(t, 1.5, 0.75, 0.5, -0.75, w, 0.5*w, w);
% trajectory 3
%out = generate_trajectory(t, 0.75, 0.75, 0, -0.75, w, w, w);
% trajectory 4
out = generate_trajectory(t,1.5,1.5,0.5,-0.75,w,w,w);
end

function out=generate_trajectory(t, a, b, c, n, w1, w2, w3)
%position
pn = a*cos(w1*t);
pe = b*sin(w2*t);
pd = n+c*sin(w3*t);
psi = 0.0;
% desired trajectory with flat output
% velocity
pnd = -a*w1*sin(w1*t);
ped = b*w2*cos(w2*t);
pdd = c*w3*cos(w3*t);
psid = 0.0;
% accelaration
pndd = -a*w1*w1*cos(w1*t);
pedd = -b*w2*w2*sin(w2*t);
pddd = -c*w3*w3*sin(w3*t);
psidd = 0.0;

% output trajectory: y,yd,ydd
out=[pn;pe;pd;psi;pnd;ped;pdd;psid;pndd;pedd;pddd;psidd];
end
